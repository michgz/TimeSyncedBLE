#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

// Custom board design is based around the Telit BlueMod+S42 module
#include "bluemod_s42.h"
#include "nrf_clock.h"


// One LED is defined
#define LEDS_NUMBER    1

#define LED_1          GPIO6

#define LEDS_LIST    { LED_1 }

#define BSP_LED_0      LED_1

#define LEDS_ACTIVE_STATE 0

// No buttons are defined
#define BUTTONS_NUMBER 0

#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_LIST { }

#define BUTTONS_ACTIVE_STATE 0

// I2C lines
#define TWI_SCL_PIN    GPIO0
#define TWI_SDA_PIN    GPIO1

// Accelerometer interrupt pin (input)
#define INT_ACCEL_1    GPIO2
#define INT_ACCEL_1_ACTIVE_STATE   0

// HW revision pins
#define HWREV_PIN_0    GPIO14
#define HWREV_PIN_1    GPIO3

// LED self-test pin
#define LED_1_MON      GPIO4

// Sensor power switch control (output)
#define SEN_ENABLE     GPIO7
#define SEN_ENABLE_ACTIVE_STATE 0


// Low frequency clock source to be used by the SoftDevice. Current hardware design
//  has no 32kHz crystal, so RC is the only option here.
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LFCLK_RC,               \
                                 .rc_ctiv       = 20,                               \
                                 .rc_temp_ctiv  = 20,                               \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_500_PPM}

#endif // CUSTOM_BOARD_H
