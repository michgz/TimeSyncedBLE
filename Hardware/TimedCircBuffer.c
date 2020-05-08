/**
 * A circular buffer implementation that maintains exact timing
 */

#include "TimedCircBuffer.h"
#include "app_error.h"
#include "Config.h"

#include "amt.h"
#include "time_sync.h"
#include "app_fifo.h"

#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdbool.h>


// Must do detection against a longer-term average?
static const inline bool doDetect(void) {return true;}

// Number of samples over which to calculate the longer-term average
#define DETECT_N   (1000)

// Square of the magnitude difference from the longer-term average to trigger a detection
#define DETECT_SQ_MAG    (100*100)

/* Macro for declaring a circular buffer.   */
#define DEF_CIRC_BUFFER(_name, _size)       \
typedef struct _name ## _tag                \
{                                           \
    const size_t        size;               \
    const size_t        halfLockSize;           \
    const size_t        recordSize;         \
    bool                is_locked;          \
    unsigned int        ptr;                \
    bool                has_wrapped; \
    XYZ_T               vals [(_size)];     \
    int32_t             xyz_sum[3];         \
    uint32_t            xyz_sum_n;          \
    XYZ_T               average;            \
    bool                have_average;       \
    uint32_t            time_base;          \
    uint32_t            lock_time;          \
    bool is_sending; \
    unsigned lock_start; \
    unsigned lock_end;   \
    bool is_lock_overflowed; \
    unsigned int read_ptr ;/* in bytes not records */    \
    unsigned int read_cnt_remaining; /*in bytes*/  \
} _name ## _T   ;                           \
static _name ## _T  _name = {(_size),(_size)/4,sizeof(XYZ_T),false,0U,false};  \


/* Declare the buffer by name and size.   */
DEF_CIRC_BUFFER(buf, 2500);


static bool lock_buffer_at_time_point(buf_T * p_buf, uint32_t time_point);

void TimedCircBuffer_Init(void)
{
    buf.ptr = 0U;
    buf.xyz_sum_n = 0U;
    buf.have_average = false;
}




//////// ERROR HANDLING ////////////////////////////////////////////////////


static uint32_t err_val[3] = {0};

/* Error which will be placed on the upload queue.   */
#define PERIPHERAL_ERROR__LOCK_FAILED 0x51

static void PERIPHERAL_ERROR(uint32_t err, uint32_t v1, uint32_t v2)
{
    if (err_val[0] != 0) return;
    err_val[0] = err; err_val[1] = v1; err_val[2] = v2;
}


////////////////////////////////////////////////////////////////////////
// Declare a few unit test cases for locking. These are used to confirm
// the calculations.
////////////////////////////////////////////////////////////////////////

static void unitTest_1(void)
{
    buf.ptr = 200;
    buf.has_wrapped = true;
    buf.time_base = 13000;

    lock_buffer_at_time_point(&buf, 13400);
}

static void unitTest_2(void)
{
    buf.ptr = 200;
    buf.has_wrapped = true;
    buf.time_base = 13000;

    lock_buffer_at_time_point(&buf, 12600);
}

void TimedCircBuffer_UnitTests(void)
{
    unitTest_1();
    unitTest_2();
    buf.ptr = 0U; buf.is_locked = false; buf. has_wrapped = false; buf.is_sending = false;
}



enum
{
    INSTRUCTION_CODE__LOCK = 0x0A,
    INSTRUCTION_CODE__IS_LOCKED = 0x0B,
    INSTRUCTION_CODE__READ_OUT = 0x0E,
    INSTRUCTION_CODE__QUERY_IS_SYNCED = 0x11,
    INSTRUCTION_CODE__QUERY_SYNC_DEBUGS = 0x13,
    INSTRUCTION_CODE__QUERY_CURRENT_TIME = 0x16,

    TRANSMIT_CODE__DATA = 0x80,

};


// Add a value to the longer-term average
static void AddToAverage(buf_T * p_buf, const XYZ_T * xyz)
{
    p_buf->xyz_sum[0] += xyz->xyz[0];
    p_buf->xyz_sum[1] += xyz->xyz[1];
    p_buf->xyz_sum[2] += xyz->xyz[2];

    p_buf->xyz_sum_n ++;
    if (p_buf->xyz_sum_n >= DETECT_N)
    {
        p_buf->average.xyz[0] = (p_buf->xyz_sum[0] / DETECT_N);
        p_buf->average.xyz[1] = (p_buf->xyz_sum[1] / DETECT_N);
        p_buf->average.xyz[2] = (p_buf->xyz_sum[2] / DETECT_N);

        p_buf->have_average = true;

        p_buf->xyz_sum[0] = 0; p_buf->xyz_sum[1] = 0; p_buf->xyz_sum[2] = 0;
        p_buf->xyz_sum_n = 0;
    }

}

// Calculate a square difference from the long term average
static uint32_t sqMag(buf_T * p_buf, const XYZ_T * xyz)
{
    if (!p_buf->have_average)
    {
        return 0U;
    }

    uint32_t sq = 0U;
    int32_t diff;
    diff = (xyz->xyz[0] - p_buf->average.xyz[0]);
    sq += (uint32_t) (diff*diff);
    diff = (xyz->xyz[1] - p_buf->average.xyz[1]);
    sq += (uint32_t) (diff*diff);
    diff = (xyz->xyz[2] - p_buf->average.xyz[2]);
    sq += (uint32_t) (diff*diff);

    return sq;
}

// A detection has been triggered
static inline void trigger(void)
{
}


void TimedCircBuffer_Add(const XYZ_T * xyz)
{
    if (buf.is_lock_overflowed)
    {
        return;
    }

    if (buf.ptr >= buf.size)
    {
        buf.ptr = 0U;
        buf.has_wrapped = true;

        // Get the current accurate time. The function returns values in 16MHz accuracy, so this is changing to 
        // 16M / 32000 = 500Hz.
        buf.time_base = (uint32_t) (ts_timestamp_get_ticks_u64(6) / 32000ULL);
    }

    // Handle the long-term average calculation if configured to do so.
    if (doDetect())
    {
        AddToAverage(&buf, xyz);
    }

    if (buf.is_locked && buf.ptr == buf.lock_start)
    {
        // The lock has overflowed. Can't write to it.
        buf.is_lock_overflowed = true;
        return;
    }

    if (doDetect())
    {
        if (sqMag(&buf, xyz) > DETECT_SQ_MAG)
        {
            trigger();
        }
    }

    memcpy((void *)&buf.vals[buf.ptr ++], (void *) xyz, sizeof(XYZ_T));

    if (buf.is_locked && buf.ptr == buf.lock_end)
    {
        // The lock is complete! Trigger the sending.
        buf.is_sending = true;
    }
}

static void release_lock(buf_T * p_buf)
{
    if (p_buf->is_locked)
    {
        p_buf->is_sending = false;
        p_buf->lock_start = 0;
        p_buf->lock_end = 0;
        p_buf->read_ptr = 0;
        p_buf->read_cnt_remaining = 0;
        if (p_buf->is_lock_overflowed)
        {
            // Has overflowed. Need to reset everything
            p_buf->is_lock_overflowed = false;
            p_buf->has_wrapped = false;
            p_buf->ptr = 0;
        }
        p_buf->is_locked = false;
    }
}

static unsigned int TestSendRemainingBytes = 0;

static void TimedCircBuffer_StartSending(void)
{
    if (isUseSyncTimer() || !isTestDevice())
    {
        // In this case, we require the sync timer to have worked, i.e. lock has been achieved.
        if (! buf.is_locked)
        {
            PERIPHERAL_ERROR(0x1B, 0, 0);
            return;
        }
    }

    TestSendRemainingBytes = 10000;  // Should be calculated from the size of the lock area.
    if (isTestDevice())
    {
        srand( *((unsigned int *)0x10000080 ) );   // FICR "Encryption Root"
    }
}

static bool Error_FifoFill(app_fifo_t * const p_fifo)
{
    uint32_t length = 0;
    app_fifo_write(p_fifo, NULL, &length);

    if (length > 16)
    {
        length = 16;
    }

    if (length > 0)
    {
        uint32_t err_ind = 0x1F;
        uint32_t size = 4;
        app_fifo_write(p_fifo, (uint8_t *)&err_ind, &size);
        length -= size;
    }

    if (length > 0)
    {
        app_fifo_write(p_fifo, (uint8_t *)err_val, &length);
    }

    err_val[0] = 0;
    TestSendRemainingBytes = 0;

    release_lock(&buf);

    return (length > 0);
}

static bool TestDevice_FifoFill(app_fifo_t * const p_fifo)
{
    uint32_t length = 0;
    app_fifo_write(p_fifo, NULL, &length);

    if (length > TestSendRemainingBytes)
    {
        length = TestSendRemainingBytes;
    }

    if (length > 0)
    {
        uint32_t i;
        for(i = 0; i < length; i ++)
        {
            app_fifo_put(p_fifo, (uint8_t)rand());
        }
        TestSendRemainingBytes -= length;
    }

    if (TestSendRemainingBytes == 0)
    {
        release_lock(&buf);
    }

    return (length > 0);
}


// There's space to add more to the FIFO.
bool TimedCircBuffer_FifoFill(app_fifo_t * const p_fifo)
{
    if (err_val[0] != 0)
    {
        return Error_FifoFill(p_fifo);
    }

    if (isTestDevice())
    {
        return TestDevice_FifoFill(p_fifo);
    }

    if (!buf.is_sending)
    {
        return false;
    }

    uint32_t rem_length = 0;
    rem_length =  buf.size * buf.recordSize - buf.read_ptr;
    if (rem_length == 0)
    {
        // Error! Shouldn't get here.
    }

    uint32_t length = 0;
    app_fifo_write(p_fifo, NULL, &length);

    if (length > buf.read_cnt_remaining)
    {
        length = buf.read_cnt_remaining;
    }
    if (length > rem_length)
    {
        length = rem_length;
    }

    if (length > 0)
    {
        app_fifo_write(p_fifo, &((uint8_t *)buf.vals)[buf.read_ptr], &length);
        buf.read_ptr += length;
        if (buf.read_ptr >= buf.size * buf.recordSize)
        {
            buf.read_ptr = 0;
        }
        buf.read_cnt_remaining -= length;
    }

    if (buf.read_cnt_remaining == 0)
    {
        release_lock(&buf);
    }

    return (length > 0);
}

static uint32_t length_of_buffer_in_time_steps(void)
{
    // Time to fill the buffer (125Hz samples) in units of 500Hz.

    return ((buf.size) / 125) * 500;
}

static bool lock_buffer_at_time_point(buf_T * p_buf, uint32_t time_point)
{

    uint32_t time_point_ptr;

    uint32_t start = 0, end = 0;

    if (!p_buf->has_wrapped)
    {
        // If not wrapped, then we don't know the latest time base. Cannot use.
        return false;
    }

    if (p_buf->is_locked)
    {
        // Already locked. Can't lock again
        return false;
    }

    if (time_point >= p_buf->time_base)
    {
        time_point_ptr = (time_point - p_buf->time_base)/4UL;
        if (time_point_ptr >= p_buf->ptr)
        {
            // The requested time is in the future, and that is disallowed here. This
            // should be changed to allow times slightly into the future, to account for
            // slight clock offsets.
            return false;
        }

        end = time_point_ptr + p_buf->halfLockSize;
        if (end >= p_buf->size)
        {
            // Wrapped around
            end -= p_buf->size;
        }

        if (time_point_ptr > p_buf->halfLockSize)
        {
            start = time_point_ptr - p_buf->halfLockSize;
        }
        else
        {
            start = time_point_ptr + p_buf->size - p_buf->halfLockSize;
        }

    }
    else
    {

        time_point_ptr =  p_buf->size - (p_buf->time_base - time_point)/4UL;

        if (time_point_ptr < p_buf->ptr)
        {
            // The requested time has already been overwritten. Can not handle.
            return false;
        }

        // TODO: check the time requested is at least half a lock size before the overwrite.

        end = time_point_ptr + p_buf->halfLockSize;
        if (end >= p_buf->size)
        {
            // Wrapped around
            end -= p_buf->size;
        }

        if (time_point_ptr > p_buf->halfLockSize)
        {
            start = time_point_ptr - p_buf->halfLockSize;
        }
        else
        {
            start = time_point_ptr + p_buf->size - p_buf->halfLockSize;
        }

    }

    // All is good. Now start the lock.


    p_buf->is_lock_overflowed = false;
    p_buf->lock_start = start;
    p_buf->lock_end = end;
    p_buf->read_ptr = p_buf->lock_start * p_buf->recordSize;
    p_buf->read_cnt_remaining = 2 * p_buf->halfLockSize * p_buf->recordSize;
    p_buf->is_locked = true;
    p_buf->lock_time = time_point;
    p_buf->is_sending = false;

    return true;
}


bool TimedCircBuffer_RxOperation_NoResponse(uint32_t code, uint32_t data)
{
    switch (code)
    {
        case INSTRUCTION_CODE__LOCK:
            {
                if (buf.is_locked && buf.lock_time == data)
                {
                    ; // ignore repeat requests
                }
                else
                {
                    bool res = lock_buffer_at_time_point(&buf, data);
                    if (!res)
                    {
                        PERIPHERAL_ERROR(PERIPHERAL_ERROR__LOCK_FAILED, data, buf.time_base);
                    }
                }
            }
            break;

        case INSTRUCTION_CODE__READ_OUT:
            {
                TimedCircBuffer_StartSending();
            }
            break;

        default:
            break;
    }

    return true;
}

bool TimedCircBuffer_RxOperation(uint32_t code, uint32_t data)
{
    switch (code)
    {
        case INSTRUCTION_CODE__QUERY_IS_SYNCED:
            {
                (void) data;

                uint32_t resp [2] = {INSTRUCTION_CODE__QUERY_IS_SYNCED + 0x80000000UL, ts_is_synced()};
                amts_queue_tx_data((uint8_t *) resp, 2*sizeof(uint32_t));
            }
            break;
        case INSTRUCTION_CODE__QUERY_SYNC_DEBUGS:
            {
                (void) data;

                uint32_t resp [1] = {INSTRUCTION_CODE__QUERY_SYNC_DEBUGS + 0x80000000UL};
                amts_queue_tx_data((uint8_t *) resp, 1*sizeof(uint32_t));

                ts_flash_out_debug();
            }
            break;
        case INSTRUCTION_CODE__QUERY_CURRENT_TIME:
            {
                (void) data;

                uint32_t resp [2] = {INSTRUCTION_CODE__QUERY_CURRENT_TIME + 0x80000000UL, (uint32_t) (ts_timestamp_get_ticks_u64(6) / 32000ULL) };
                amts_queue_tx_data((uint8_t *) resp, 2*sizeof(uint32_t));
            }
            break;
        case INSTRUCTION_CODE__LOCK:
            {
                bool res = lock_buffer_at_time_point(&buf, data/*      (uint32_t) (ts_timestamp_get_ticks_u64(6) / 32000ULL)   */         );

                uint32_t resp [2] = {INSTRUCTION_CODE__LOCK + 0x80000000UL, (uint32_t) res};
                amts_queue_tx_data((uint8_t *) resp, 2*sizeof(uint32_t));
            }
            break;
        case INSTRUCTION_CODE__IS_LOCKED:
            {
                (void) data;

                uint32_t resp [2] = {INSTRUCTION_CODE__IS_LOCKED + 0x80000000UL, (uint32_t) buf.is_locked};
                amts_queue_tx_data((uint8_t *) resp, 2*sizeof(uint32_t));
            }
            break;
        case INSTRUCTION_CODE__READ_OUT:
            // Start reading out the lock zone. This must be a different instruction because we expect the
            // lock instruction to come from a different connection handle than the read out instruction.
            {
            }
            break;
        default:
            break;
    }

    return true;
}
