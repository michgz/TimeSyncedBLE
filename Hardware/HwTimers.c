/**
 * Setup of the timers used by the application
 */

#include "HwTimers.h"
#include "HwSensor.h"   // timer_led_event_handler()
#include "app_error.h"

#include "app_timer.h"
#include "bsp.h"
#include "nrf_drv_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_timer.h"
#include "nrf_drv_pwm.h"
#include "nrf_ppi.h"

#include "time_sync.h"

#include <string.h>
#include <stdbool.h>

const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(1);

static bool borrow_led = false;

void hw_timers_init(void)
{

    const uint32_t time_ms = 8; //Time(in milliseconds) between consecutive compare events. 125Hz
    uint32_t time_ticks;

    ret_code_t err_code;

    //Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&TIMER_LED, &timer_cfg, timer_led_event_handler);
    APP_ERROR_CHECK(err_code);

#if 1
    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_LED, time_ms);

    nrf_drv_timer_extended_compare(
         &TIMER_LED, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&TIMER_LED);
#endif
}

void hw_timers_stop(void)
{
    if (nrf_drv_timer_is_enabled(&TIMER_LED))
        nrf_drv_timer_disable(&TIMER_LED);
}


#if 0
static volatile uint64_t tt[32];
static volatile uint32_t ty[32];
static unsigned int tt_ptr = 0U;

static void add_tt(uint64_t x, uint32_t y)
{
    if (tt_ptr >= 32) tt_ptr = 0U;
    tt[tt_ptr] = x;
    ty[tt_ptr ++] = y;
}
#endif // 0

extern bool IsAwake(void);


///////////////// PWM CODE ////////////////////////////////////////////////////

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

void TIMER4_IRQHandler(void)
{

    if (nrf_timer_event_check(NRF_TIMER4, NRF_TIMER_EVENT_COMPARE0))
    {

        // Clear the event
        nrf_timer_event_clear(NRF_TIMER4, NRF_TIMER_EVENT_COMPARE0);

        // This should be a one-off interrupt. Cancel any more.
    //    NVIC_DisableIRQ(TIMER4_IRQn);

        if (!borrow_led)
        {
            uint64_t time = ts_timestamp_get_ticks_u64(6);   // time in units of 1/16MHz.




            uint64_t  modulo = time % (16000000ULL*10);  // Where in the LED flash sequence we should be

            modulo /= ((16000000*10)/(31250*10));   // Translate into timer ticks.
            modulo = (31250*10-1) - modulo;

            //add_tt(0x8000000000000000ULL | time, modulo);

            nrf_ppi_channel_disable(NRF_PPI_CHANNEL7);
            NRF_TIMER4->CC[1] = modulo;
            if (IsAwake())
            {
                nrf_ppi_channel_enable(NRF_PPI_CHANNEL7);
            }
        }

    }

    if (nrf_timer_event_check(NRF_TIMER4, NRF_TIMER_EVENT_COMPARE1))
    {

        // Clear the event
        nrf_timer_event_clear(NRF_TIMER4, NRF_TIMER_EVENT_COMPARE1);

#if 0
#ifdef PERIPHERAL
        uint64_t time = ts_timestamp_get_ticks_u64(6);   // time in units of 1/16MHz.
#endif


        add_tt(time,NRF_TIMER4->CC[1]);
#endif // 0

    }


}

void led_flash_seq_stop(void)
{
    NRF_TIMER_Type * const timer = NRF_TIMER4;

    nrf_ppi_channel_disable(NRF_PPI_CHANNEL7);

    nrfx_pwm_uninit(&m_pwm0);
    timer->TASKS_STOP = 1;
    timer->TASKS_SHUTDOWN = 1;   // required -- errata anomaly 78
    NVIC_DisableIRQ(TIMER4_IRQn);
}

static uint32_t led_flash_seq_setup(void)
{
    /*
     * This demo plays back two concatenated sequences:
     * - Sequence 0: Light intensity is increased in 25 steps during one second.
     * - Sequence 1: LED blinks twice (100 ms off, 100 ms on), then stays off
     *   for 200 ms.
     * The same output is generated on all 4 channels (LED 1 - LED 4).
     * The playback is repeated in a loop.
     */

    enum { // [local constants]
        TOP        = 10000,
        STEP_COUNT = 25
    };

    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            BSP_LED_0 | NRF_DRV_PWM_PIN_INVERTED, // channel 0
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 1
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 2
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_500kHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = TOP,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL));

    // This array cannot be allocated on stack (hence "static") and it must
    // be in RAM.
    static nrf_pwm_values_common_t seq0_values[2*STEP_COUNT];
    uint16_t value = 0;
    uint16_t step  = TOP / STEP_COUNT;
    uint8_t  i;
    for (i = 0; i < STEP_COUNT; ++i)
    {
        value         += step;
        seq0_values[i] = value;
    }
    for (; i < 2*STEP_COUNT; ++i)
    {
        value         -= step;
        seq0_values[i] = value;
    }

    nrf_pwm_sequence_t const seq0 =
    {
        .values.p_common = seq0_values,
        .length          = NRF_PWM_VALUES_LENGTH(seq0_values),
        .repeats         = 1,
        .end_delay       = 50
    };

    // This array cannot be allocated on stack (hence "static") and it must
    // be in RAM (hence no "const", though its content is not changed).
    static nrf_pwm_values_common_t /*const*/ seq1_values[] =
    {
             0,
        0x8000,
             0,
             0
    };
    nrf_pwm_sequence_t const seq1 =
    {
        .values.p_common = seq1_values,
        .length          = NRF_PWM_VALUES_LENGTH(seq1_values),
        .repeats         = 1, /* ignored */
        .end_delay       = 0  /* ignored */
    };

    return nrf_drv_pwm_complex_playback(&m_pwm0, &seq0, &seq1, 1,
                                       NRF_DRV_PWM_FLAG_START_VIA_TASK);

}

void led_flash_seq_init(void)
{
    // No action -- all is done within "start" function (below)
}


void led_flash_seq_start(void)
{
    NRF_LOG_INFO("Starting LED flash sequence");
    uint32_t task = led_flash_seq_setup();

    NRF_TIMER_Type * const timer = NRF_TIMER4;

    NVIC_DisableIRQ(TIMER4_IRQn);
    timer->INTENCLR = 0xFFFFFFFFU;

    timer->TASKS_STOP  = 1;
    timer->TASKS_CLEAR = 1;
    timer->PRESCALER   = 9;
    timer->BITMODE     = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
    timer->MODE        = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;;
    timer->CC[0]       = 31250*10;   // 10 seconds
    timer->CC[1]       = 31250*5;
    timer->CC[2]       = 0xFFFFFFFF;
    timer->CC[3]       = 0xFFFFFFFF;
    if (timer == NRF_TIMER3 || timer == NRF_TIMER4)
    {
        // TIMERS 0,1, and 2 only have 4 compare registers
        timer->CC[4]   = 0xFFFFFFFF;
        timer->CC[5]   = 0xFFFFFFFF;
    }
    timer->SHORTS      = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    timer->TASKS_START = 1;

    // Set up PPI channel 7 to be the "start LED sequence" trigger -- triggered from TIMER4 CC0.
    nrf_ppi_channel_endpoint_setup(
        NRF_PPI_CHANNEL7, 
        (uint32_t) nrf_timer_event_address_get(timer, NRF_TIMER_EVENT_COMPARE1),
        task);

    timer->INTENSET = 0x00030000U;
    NVIC_EnableIRQ(TIMER4_IRQn);
}

static void led_identification_handler(nrf_drv_pwm_evt_type_t event_type)
{
    if (event_type == NRF_DRV_PWM_EVT_FINISHED || event_type == NRF_DRV_PWM_EVT_STOPPED)
    {
        nrf_drv_pwm_uninit(&m_pwm0);
        (void) led_flash_seq_setup();
        borrow_led = false;
    }
}

// "Borrow" the LED flash sequence to indicate an identification sequence.
void led_identification_seq_init(void)
{
    // Stop the other LED sequence being triggered by TIMER4.
    borrow_led = true;
    nrf_ppi_channel_disable(NRF_PPI_CHANNEL7);

    nrf_drv_pwm_uninit(&m_pwm0);

    // Set up the sequence
    NRF_LOG_INFO("Starting LED identification flash sequence");

    enum { // [local constants]
        TOP        = 10000,
        STEP_COUNT = 25
    };

    nrf_drv_pwm_config_t const config1 =
    {
        .output_pins =
        {
            BSP_LED_0 | NRF_DRV_PWM_PIN_INVERTED, // channel 0
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 1
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 2
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_500kHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = TOP,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config1, &led_identification_handler));

    // This array cannot be allocated on stack (hence "static") and it must
    // be in RAM.
    static nrf_pwm_values_common_t seq0_values[2*STEP_COUNT];
    uint8_t  i;
    for (i = 0; i < 2*STEP_COUNT; ++i)
    {
        // Alternating values
        seq0_values[i] = (i&1)?0x33:0x00;
    }


    nrf_pwm_sequence_t const seq0 =
    {
        .values.p_common = seq0_values,
        .length          = NRF_PWM_VALUES_LENGTH(seq0_values),
        .repeats         = 1,
        .end_delay       = 50
    };

    uint32_t task = nrf_drv_pwm_simple_playback(&m_pwm0, &seq0, 1,
                                       NRF_DRV_PWM_FLAG_STOP);

    (void) task;


}


// Recalculate the value that will align the LED flash with the time sync.
void realign_led_flash(void)
{
    uint64_t time = ts_timestamp_get_ticks_u64(6);   // time in units of 1/16MHz.
    uint64_t  modulo = time % 16000000*10;  // Where in the LED flash sequence we should be

    (void) modulo;
}
