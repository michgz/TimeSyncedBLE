/**
 * A circular buffer implementation that maintains exact timing
 */

#ifndef __H_TIMED_CIRC_BUFFER
#define __H_TIMED_CIRC_BUFFER

#include <stdbool.h>
#include <stdint.h>

#include "app_fifo.h"

/* Define the data type for axis. High-precision accelerometers (e.g. ADXL355) may need uint32_t here.  */
typedef int16_t ACCEL_AXIS_DATA_T;

typedef struct XYZ_tag
{
    ACCEL_AXIS_DATA_T   xyz[3];   // X, Y, Z accelerations in 1/16384 g.
    uint16_t            n;         // number of readings that were averaged to give this reading.

} XYZ_T;

/* Initialise the storage   */
extern void TimedCircBuffer_Init(size_t size_of_buffer);

/* Set the threshold for the triggering (Central only)   */
extern void TimedCircBuffer_SetThreshold(uint32_t new_threshold);

/* Clear an existing storage */
extern void TimedCircBuffer_Clear(void);

/* Add an item to the circular buffer   */
extern void TimedCircBuffer_Add(const XYZ_T *);

/* An instruction has been received for this buffer. 32-bit instruction code and optional 32-bit
 * data.     */
extern bool TimedCircBuffer_RxOperation(uint32_t code, uint32_t data);

/* Same as above, but no reponse expected.   */
extern bool TimedCircBuffer_RxOperation_NoResponse(uint32_t code, uint32_t data);

extern bool TimedCircBuffer_FifoFill(app_fifo_t * const);

extern void TimedCircBuffer_UnitTests(void);


#endif /* __H_TIMED_CIRC_BUFFER */
