/**
 * A circular buffer implementation that maintains exact timing
 */

#include "TimedCircBuffer.h"
#include "app_error.h"
#include "Config.h"
#include "HwTimers.h"

#include "amt.h"
#include "time_sync.h"
#include "app_fifo.h"
#include "app_timer.h"

#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdbool.h>

#include "nrf_log.h"

APP_TIMER_DEF(m_lock_timeout_timer);

extern bool publicIsCentral(void);

// Must do detection against a longer-term average?
static const inline bool doDetect(void) {return publicIsCentral();}

// Number of samples over which to calculate the longer-term average
#define DETECT_N   (1000)

// Square of the magnitude difference from the longer-term average to trigger a detection
//  Units of g/1000 ^2.
static uint32_t DETECT_SQ_MAG = (1000*1000);

/* A circular buffer.   */
typedef struct buf_tag
{
    size_t        size;
    size_t        halfLockSize;
    size_t        recordSize;
    bool                is_locked;
    unsigned int        ptr;
    bool                has_wrapped;
    uint32_t            time_base;
    uint32_t            lock_time;
    bool is_sending;
    unsigned lock_start;
    unsigned lock_end;
    bool is_lock_overflowed;
    unsigned int read_ptr ;/* in bytes not records */
    unsigned int read_cnt_remaining; /*in bytes*/
    XYZ_T               vals [1];
} buf_T;

//static _name ## _T  _name = {(_size),(_size)/4,sizeof(XYZ_T),false,0U,false};

buf_T * p_Buf;


/* Declare the buffer by name and size.   */
//DEF_CIRC_BUFFER(buf, 250);


typedef struct avg_buf_tag
{
    int32_t             xyz_sum[3];
    uint32_t            xyz_sum_n;
    XYZ_T               average;
    bool                have_average;
    uint32_t            trigger_delay;  // Delay between passing the threshold and actually performing the trigger. Allows largest_so_far to really measure a peak value
    uint32_t            trigger_countdown;
    uint32_t            trigger_holdoff;
    uint32_t            trigger_holdoff_countdown;  // Hold-off between completing a trigger and receiving a new one
    uint32_t            largest_so_far; // Square magnitude of the largest offset so far.

} avg_buf_T;


avg_buf_T * p_Avg;

static void AvgBuffer_Init(avg_buf_T * p_avg_buf)
{
    p_avg_buf->have_average = false;
    p_avg_buf->xyz_sum_n = 0;

    // A delay between getting a reading and doing a detection. Units of samples.
    p_avg_buf->trigger_delay = 125;

    // Holdoff after a detection before the next one.
    p_avg_buf->trigger_holdoff = 125*2;

    p_avg_buf->trigger_countdown = 0;

    // Start with a small holdoff.
    p_avg_buf->trigger_holdoff_countdown = 625;
}

static void AvgBuffer_Clear(avg_buf_T * p_avg_buf)
{
    p_avg_buf->have_average = false;
    p_avg_buf->xyz_sum_n = 0;

    p_avg_buf->trigger_countdown = 0;

    // Start with a small holdoff.
    p_avg_buf->trigger_holdoff_countdown = 625;
}

static bool lock_buffer_at_time_point(buf_T * p_buf, uint32_t time_point);
static void release_lock(buf_T * p_buf);

static void lock_timeout_handler(void * p_context)
{
    // The locked data has not been read out. Assume something has gone wrong and just
    // release it anyway.

    NRF_LOG_ERROR("LOCK timed out");

    if (!!p_Buf)
    {
        release_lock(p_Buf);
    }
}

void TimedCircBuffer_Init(size_t size_of_buffer)
{
    p_Avg = malloc(sizeof(avg_buf_T));
    if (!p_Avg)
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }
    AvgBuffer_Init(p_Avg);

    if (size_of_buffer > 0)
    {
        p_Buf = malloc(sizeof(buf_T) + size_of_buffer*sizeof(XYZ_T));
        if (!p_Buf)
        {
            APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
        }
        memset((void *)p_Buf, 0, sizeof(buf_T));
        p_Buf->size = size_of_buffer;
        p_Buf->halfLockSize = size_of_buffer/4;
        p_Buf->recordSize = sizeof(XYZ_T);
    }
    else
    {
        p_Buf = NULL;
    }

    ret_code_t err_code;

    err_code = app_timer_create(&m_lock_timeout_timer, APP_TIMER_MODE_SINGLE_SHOT, &lock_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

void TimedCircBuffer_SetThreshold(uint32_t new_threshold)
{
    // Reject the default value which is 0.
    if (new_threshold > 0 && new_threshold < 8192)
    {
        DETECT_SQ_MAG = (new_threshold * new_threshold);
    }
}

void TimedCircBuffer_Clear(void)
{
    if (!! p_Avg)
    {
        AvgBuffer_Clear(p_Avg);
    }

    p_Buf->is_locked = false;
    p_Buf->is_lock_overflowed = false;
    p_Buf->has_wrapped = false;
    p_Buf->is_sending = false;
    p_Buf->ptr = 0;
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

#if 0

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

#endif

void TimedCircBuffer_UnitTests(void)
{
    //unitTest_1();
    //unitTest_2();
    //buf.ptr = 0U; buf.is_locked = false; buf. has_wrapped = false; buf.is_sending = false;
}


enum
{
    INSTRUCTION_CODE__LOCK = 0x0A,
    INSTRUCTION_CODE__IS_LOCKED = 0x0B,
    INSTRUCTION_CODE__READ_OUT = 0x0E,
    INSTRUCTION_CODE__TEST = 0x21,
    // 0x22 reserved for trigger uploads
    INSTRUCTION_CODE__IDENTIFY_SELF = 0x23,
    INSTRUCTION_CODE__QUERY_IS_SYNCED = 0x11,
    INSTRUCTION_CODE__QUERY_SYNC_DEBUGS = 0x13,
    INSTRUCTION_CODE__QUERY_CURRENT_TIME = 0x16,

    INSTRUCTION_CODE__TRIGGER = 0x27,

    TRANSMIT_CODE__DATA = 0x80,

};


// Add a value to the longer-term average
static void AddToAverage(avg_buf_T * p_buf, const XYZ_T * xyz)
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
static uint32_t sqMag(avg_buf_T * p_buf, const XYZ_T * xyz)
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
extern void trigger(uint32_t size);


static void processTrigger(avg_buf_T * const p_avg, uint32_t sq_mag, const uint32_t sq_mag_thresh)
{
    if (p_avg->trigger_countdown == 0)
    {
        // Not yet triggered. Handle a new value here.

        // First, check if held off.
        if (p_avg->trigger_holdoff_countdown > 0)
        {
            p_avg->trigger_holdoff_countdown --;
            return;
        }

        // Now do the trigger detection.
        if (sq_mag > sq_mag_thresh)
        {
            p_avg->trigger_countdown = p_avg->trigger_delay;
            p_avg->largest_so_far = sq_mag;
        }
    }
    else
    {
        // Has already triggered. Must handle differently.

        if (sq_mag > p_avg->largest_so_far)
        {
            p_avg->largest_so_far = sq_mag;
        }

        p_avg->trigger_countdown --;
        if (p_avg->trigger_countdown == 0)
        {
            // Triggered!
            trigger(p_avg->largest_so_far);
            p_avg->trigger_holdoff_countdown = p_avg->trigger_holdoff;
        }
    }

}


static void Add(buf_T * const p_buf, const XYZ_T * xyz)
{
    if (p_buf->is_lock_overflowed)
    {
        return;
    }

    if (p_buf->ptr >= p_buf->size)
    {
        p_buf->ptr = 0U;
        p_buf->has_wrapped = true;

        // Get the current accurate time. The function returns values in 16MHz accuracy, so this is changing to 
        // 16M / 32000 = 500Hz.
        p_buf->time_base = (uint32_t) (ts_timestamp_get_ticks_u64(6) / 32000ULL);
    }


    if (p_buf->is_locked && p_buf->ptr == p_buf->lock_start)
    {
        // The lock has overflowed. Can't write to it.
        p_buf->is_lock_overflowed = true;
        return;
    }

    memcpy((void *)&p_buf->vals[p_buf->ptr ++], (void *) xyz, sizeof(XYZ_T));

    if (p_buf->is_locked && p_buf->ptr == p_buf->lock_end)
    {
        // The lock is complete! Trigger the sending.
        p_buf->is_sending = true;

        app_timer_start(m_lock_timeout_timer, APP_TIMER_TICKS(4*60*1000), (void *) 0);
    }
}


void TimedCircBuffer_Add(const XYZ_T * xyz)
{
    // Add to the buffer
    if (!!p_Buf)
    {
        Add(p_Buf, xyz);
    }

    // Handle the long-term average calculation if configured to do so.
    if (doDetect() && !!p_Avg)
    {
        AddToAverage(p_Avg, xyz);

        uint32_t sq_mag = sqMag(p_Avg, xyz);
        processTrigger(p_Avg, sq_mag, DETECT_SQ_MAG);
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

static bool testLock = false;   // True if pretending to be locked, while sending debug values.

static void TimedCircBuffer_StartSending(void)
{
    if (!testLock && (isUseSyncTimer() || !isTestDevice()))
    {
        // In this case, we require the sync timer to have worked, i.e. lock has been achieved.
        if (!p_Buf || ! p_Buf->is_locked)
        {
            PERIPHERAL_ERROR(0x1B, 0, 0);
            return;
        }
    }

    TestSendRemainingBytes = 10000;  // Should be calculated from the size of the lock area.
    if (testLock)
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

    NRF_LOG_ERROR("LOCK error, discarding");

    if (!!p_Buf)
    {
        release_lock(p_Buf);
    }

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
        if (isTestDeviceDebug())
        {
            uint32_t length_2 = length;
            while (length_2 >= 4)
            {
                uint32_t val = 10000 - TestSendRemainingBytes;
                uint32_t length_3 = 4;
                app_fifo_write(p_fifo, (const uint8_t *)&val, &length_3);

                length_2 -= 4;
                TestSendRemainingBytes -= 4;
                if (TestSendRemainingBytes < 4)
                {
                    break;
                }
            }

        }
        else
        {
            uint32_t i;
            for(i = 0; i < length; i ++)
            {
                app_fifo_put(p_fifo, (uint8_t)rand());
            }
            TestSendRemainingBytes -= length;
        }
    }

    if (TestSendRemainingBytes == 0)
    {
        NRF_LOG_INFO("LOCK completed (test)");

        if (!!p_Buf)
        {
            release_lock(p_Buf);
        }
    }

    return (length > 0);
}


static bool Buf_FifoFill(buf_T * const p_buf, app_fifo_t * const p_fifo)
{
    if (!p_buf->is_sending)
    {
        return false;
    }

    uint32_t rem_length = 0;
    rem_length =  p_buf->size * p_buf->recordSize - p_buf->read_ptr;
    if (rem_length == 0)
    {
        // Error! Shouldn't get here.
    }

    uint32_t length = 0;
    app_fifo_write(p_fifo, NULL, &length);

    if (length > p_buf->read_cnt_remaining)
    {
        length = p_buf->read_cnt_remaining;
    }
    if (length > rem_length)
    {
        length = rem_length;
    }

    if (length > 0)
    {
        app_fifo_write(p_fifo, &((uint8_t *)p_buf->vals)[p_buf->read_ptr], &length);
        p_buf->read_ptr += length;
        if (p_buf->read_ptr >= p_buf->size * p_buf->recordSize)
        {
            p_buf->read_ptr = 0;
        }
        p_buf->read_cnt_remaining -= length;
    }

    if (p_buf->read_cnt_remaining == 0)
    {
        NRF_LOG_INFO("LOCK completed");

        if (!!p_Buf)
        {
            release_lock(p_Buf);
        }
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

    if (testLock)
    {
        bool res = TestDevice_FifoFill(p_fifo);
        if (!res)
        {
            testLock = false; // fininshed
        }
        return res;
    }

    if (!!p_Buf)
    {
        return Buf_FifoFill(p_Buf, p_fifo);
    }

    return false;
}

#if 0
static uint32_t length_of_buffer_in_time_steps(buf_T * p_buf)
{
    // Time to fill the buffer (125Hz samples) in units of 500Hz.

    return ((p_buf->size) / 125) * 500;
}
#endif

static bool lock_buffer_at_time_point(buf_T * p_buf, uint32_t time_point)
{

    uint32_t time_point_ptr;

    uint32_t start = 0, end = 0;

    if (!p_buf->has_wrapped && time_point != 0xFFFFFFFFUL)
    {
        // If not wrapped, then we don't know the latest time base. Cannot use.

        NRF_LOG_ERROR("lock_buffer_at_time_point: not wrapped");
        return false;
    }

    if (p_buf->is_locked)
    {
        // Already locked. Can't lock again

        //NRF_LOG_ERROR("lock_buffer_at_time_point: already locked");
        return false;
    }

    if (time_point == 0xFFFFFFFFUL)
    {
        // For debugging, reserve this specific value to mean "current time".
        if (p_buf->ptr == 0)
        {
            // wrap-around
            time_point_ptr = p_buf->size - 1;
        }
        else
        {
            time_point_ptr = p_buf->ptr - 1;
        }

        time_point = p_buf->time_base + 4UL*p_buf->ptr;

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

        NRF_LOG_INFO("lock_buffer_at_time_point: requested 0xFFFFFFFF locked 0x%x", time_point);

    }
    else if (time_point >= p_buf->time_base)
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

        NRF_LOG_INFO("lock_buffer_at_time_point: requested 0x%x (1)", time_point);

    }
    else
    {

        time_point_ptr =  p_buf->size - (p_buf->time_base - time_point)/4UL;

        if (time_point_ptr < p_buf->ptr)
        {
            // The requested time has already been overwritten. Can not handle.
            NRF_LOG_ERROR("lock_buffer_at_time_point: requested 0x%x time-base 0x%x", time_point, p_buf->time_base);
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

        NRF_LOG_INFO("lock_buffer_at_time_point: requested 0x%x (2)", time_point);
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

static bool is_buffer_locked(buf_T * const p_buf, uint32_t data)
{
    return (p_buf->is_locked && p_buf->lock_time == data);
}


bool TimedCircBuffer_RxOperation_NoResponse(uint32_t code, uint32_t data)
{
    switch (code)
    {
        case INSTRUCTION_CODE__READ_OUT:
            {
                TimedCircBuffer_StartSending();
            }
            break;

        case INSTRUCTION_CODE__LOCK:
            {
                if (!!p_Buf)
                {
                    // Discard error indicator -- we can't do anything with it
                    (void) lock_buffer_at_time_point(p_Buf, data);
                }
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

        case INSTRUCTION_CODE__TEST:
            {
                if (!!p_Buf && is_buffer_locked(p_Buf, data))
                {
                    ; // the buffer is locked for real, don't do a test lock
                }
                else
                {
                    // Don't worry if p_Buf is zero. That may simply indicate a central device --
                    // they should also be responsive to test reads.
                    testLock = true;
                }
            }
            break;

        case INSTRUCTION_CODE__LOCK:
            {
                if (!p_Buf)
                {
                    PERIPHERAL_ERROR(PERIPHERAL_ERROR__LOCK_FAILED, data, 0xFEFEFEFE);
                }
                else
                {
                    if (is_buffer_locked(p_Buf, data))
                    {
                        ; // ignore repeat requests
                    }
                    else if (testLock)
                    {
                        ; // test lock in progress
                    }
                    else
                    {
                        bool res = lock_buffer_at_time_point(p_Buf, data);
                        if (!res)
                        {
                            PERIPHERAL_ERROR(PERIPHERAL_ERROR__LOCK_FAILED, data, p_Buf->time_base);
                        }
                        if (isTestDevice())
                        {
                            testLock = true;
                        }
                    }
                }
            }
            break;

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
#if 0   // What was this??
        case INSTRUCTION_CODE__LOCK:
            {
                bool res = false;
                if (!!p_Buf)
                {
                    res = lock_buffer_at_time_point(p_Buf, data/*      (uint32_t) (ts_timestamp_get_ticks_u64(6) / 32000ULL)   */         );
                }

                uint32_t resp [2] = {INSTRUCTION_CODE__LOCK + 0x80000000UL, (uint32_t) res};
                amts_queue_tx_data((uint8_t *) resp, 2*sizeof(uint32_t));
            }
            break;
#endif // 0
        case INSTRUCTION_CODE__IS_LOCKED:
            {
                (void) data;
                uint32_t is_locked = 0;
                if (!!p_Buf)
                {
                    is_locked = (uint32_t)p_Buf->is_locked;
                }


                uint32_t resp [2] = {INSTRUCTION_CODE__IS_LOCKED + 0x80000000UL, is_locked};
                amts_queue_tx_data((uint8_t *) resp, 2*sizeof(uint32_t));
            }
            break;
        case INSTRUCTION_CODE__IDENTIFY_SELF:
            {
                led_identification_seq_init();
            }
            break;
        case INSTRUCTION_CODE__TRIGGER:
            {
                // Only react if this is a central device
                if (publicIsCentral())
                {
                    trigger(0x1000);
                }
                uint32_t resp[2] = {INSTRUCTION_CODE__TRIGGER + 0x80000000UL, 0x00000000};
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
