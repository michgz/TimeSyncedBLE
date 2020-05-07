/**
 * Setup of the sensor (accelerometer) used by the application
 */

#ifndef __H_HW_SENSOR
#define __H_HW_SENSOR

#include <stdbool.h>
#include <stdint.h>

#include "nrf_drv_timer.h"

extern void gpio_init(void);
extern void sensor_init(void);
extern void timer_led_event_handler(nrf_timer_event_t event_type, void* p_context);


extern void sensor_off(void);

#endif /* __H_HW_SENSOR */
