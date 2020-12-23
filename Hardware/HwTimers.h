/**
 * Setup of the timers used by the application
 */

#ifndef __H_HW_TIMERS
#define __H_HW_TIMERS

#include <stdbool.h>
#include <stdint.h>

extern void hw_timers_init(void);
extern void led_flash_seq_init(void);
extern void led_flash_seq_start(void);
extern void led_flash_seq_stop(void);

extern void hw_timers_stop(void);
extern void led_identification_seq_init(void);  // flash the LED in a "self-identification" sequence, for the human user to view. Once complete,
                                                // return to normal flash sequence.

#endif /* __H_HW_TIMERS */
