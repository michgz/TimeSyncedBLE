/**
 * Setup of the timers used by the application
 */

#ifndef __H_HW_TIMERS
#define __H_HW_TIMERS

#include <stdbool.h>
#include <stdint.h>

extern void hw_timers_init(void);
extern void led_flash_seq_init(void);

extern void hw_timers_stop(void);

#endif /* __H_HW_TIMERS */
