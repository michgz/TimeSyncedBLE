/**
 * Setup of the sensor (accelerometer) used by the application
 */

#include "HwSensor.h"
#include "boards.h"
#include "TimedCircBuffer.h"
#include "HwTimers.h"
#include "app_error.h"

#include <string.h>
#include <stdbool.h>

#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpiote.h"
#include "nrf_atomic.h"
#include "nrf_drv_timer.h"
#include "nrf_timer.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* Common addresses definition for accelerometer sensor. */
#define LIS2DH_ADDR          (0x30U >> 1)

#define LIS2DH_CTRL_REG1    0x20U
#define LIS2DH_CTRL_REG3    0x22U
#define LIS2DH_CTRL_REG4    0x23U
#define LIS2DH_OUT_X_L      0x28U   // 0x28-0x2D are output registers
#define LIS2DH_WHO_AM_I     0x0FU

/* Mode for LIS2DH. */
#define NORMAL_MODE 0x97U

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;
static volatile bool m_xfer_error = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;

/**
 * @brief Function for setting active mode on MMA7660 accelerometer.
 */
void LIS2DH_set_mode(void)
{
    ret_code_t err_code;

    /* Writing to LIS2DH_CTRL_REG1 "0" set accelerometer sensor in NORMAL mode. */
    uint8_t reg[2] = {LIS2DH_CTRL_REG1, NORMAL_MODE};
    err_code = nrf_drv_twi_tx(&m_twi, LIS2DH_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    /* Writing to next byte. */
    m_xfer_done = false;
    // :
}


nrf_atomic_u32_t  count_of_Accelerometer_reads;

static volatile uint8_t d_buf [100];
static unsigned int d_ptr = 0;
static void add_d(uint8_t x)
{
    if (d_ptr >= 100) {d_ptr = 0;}
    
    {d_buf[d_ptr ++] = x;}
}

/**
 * @brief Function for handling data from accelerometer sensor.
 *
 * @param[in] temp          Value
 */
__STATIC_INLINE void data_handler(uint8_t temp)
{
    //NRF_LOG_INFO("Who am I: %d.", temp);
    add_d (temp);
}

static volatile uint8_t m_samples[6];

static volatile  int32_t sums[3];
static volatile uint32_t number;


#define XX_N    (256)
static volatile uint32_t   xx[XX_N];
static volatile unsigned int xx_ptr = 0U;

// Change an unsigned 12-bit number to signed 32-bit
static inline int32_t SIGNED_12_TO_32(uint16_t x)
{
    if ((x & 0x8000U) == 0U)
    {
        // Positive
        return (int32_t )((uint32_t)x >> 4);
    }
    else
    {
        // Negative
        return (int32_t )(0xFFFFF800UL | ((uint32_t)x >> 4));
    }
}

const static inline bool use_debug_buffer(void)
{
    return false;
}

__STATIC_INLINE nrfx_err_t twim_xfer_2(NRF_TWIM_Type * p_twim)
{
    nrfx_err_t err_code = NRFX_SUCCESS;

    /* Block TWI interrupts to ensure that function is not interrupted by TWI interrupt. */
    nrf_twim_int_disable(p_twim, NRF_TWIM_ALL_INTS_MASK);

    nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_STOPPED);
    nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_ERROR);

    nrf_twim_shorts_set(p_twim, NRF_TWIM_SHORT_LASTTX_STARTRX_MASK |
                                NRF_TWIM_SHORT_LASTRX_STOP_MASK);
    nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_RESUME);

    /*Now restore interrupts  */
    nrf_twim_int_enable(p_twim,NRF_TWIM_INT_STOPPED_MASK | NRF_TWIM_INT_ERROR_MASK);

#if NRFX_CHECK(NRFX_TWIM_NRF52_ANOMALY_109_WORKAROUND_ENABLED)
    twim_list_enable_handle(p_twim, 0);
    p_twim->FREQUENCY = 0;
    nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_TXSTARTED);
    nrf_twim_int_enable(p_twim, NRF_TWIM_INT_TXSTARTED_MASK);
#endif

    return err_code;
}


static void trigger_reading(void)
{
    /* Trigger a reading that has been established with nrf_drv_twi_xfer()
     * and flag NRFX_TWIM_FLAG_HOLD_XFER.  */

    m_xfer_done = false;

    NRF_TWIM0->TASKS_STARTTX = 1;
    (void)NRF_TWIM0->TASKS_STARTTX;
}


static bool tick_without_interrupts = false;

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{

add_d(0xE0 + p_event->type);

    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            tick_without_interrupts = false;
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample);
            }
            else if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TXRX)
            {
                nrf_atomic_u32_add(&count_of_Accelerometer_reads, 1U);

                int32_t x = SIGNED_12_TO_32( * ((uint16_t *) &m_samples[0]) );
                int32_t y = SIGNED_12_TO_32( * ((uint16_t *) &m_samples[2]) );
                int32_t z = SIGNED_12_TO_32( * ((uint16_t *) &m_samples[4]) );

                sums[0] += x;
                sums[1] += y;
                sums[2] += z;
                number ++;
                
                if (use_debug_buffer())
                {
                    // Only do this if needed for debugging.
                    if (xx_ptr >= (XX_N - 4))
                        xx_ptr = 0U;

                    xx[xx_ptr ++] = (uint32_t)sums[0];
                    xx[xx_ptr ++] = (uint32_t)sums[1];
                    xx[xx_ptr ++] = (uint32_t)sums[2];
                    xx[xx_ptr ++] = (uint32_t)number;
                }

                /* Re-establish the action ready for the next trigger */
                twim_xfer_2(NRF_TWIM0);

            }
            m_xfer_done = true;
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            m_xfer_error = true;
            m_xfer_done = true;
            //APP_ERROR_CHECK(NRF_ERROR_DRV_TWI_ERR_ANACK);
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            m_xfer_error = true;
            m_xfer_done = true;
            //APP_ERROR_CHECK(NRF_ERROR_DRV_TWI_ERR_DNACK);
            break;
        default:
            break;
    }
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lis2dh_config = {
       .scl                = TWI_SCL_PIN,
       .sda                = TWI_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lis2dh_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**
 * @brief Function for reading data from temperature sensor.
 */
static void read_sensor_data(uint8_t addr)
{
    m_xfer_done = false;

    /* Read from register at address "addr". */
    uint8_t reg[1] = {addr | 0x80U};
    ret_code_t err_code = nrf_drv_twi_tx(&m_twi, LIS2DH_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    m_xfer_done = false;

    /* Read 1 byte from the specified address. */
    err_code = nrf_drv_twi_rx(&m_twi, LIS2DH_ADDR, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code);
}

static void write_reg(uint8_t addr, uint8_t data)
{
    ret_code_t err_code;

    m_xfer_done = false;

    /* Writing to LIS2DH_CTRL_REG1 "0" set accelerometer sensor in NORMAL mode. */
    uint8_t reg[2] = {addr | 0x00, data};
    err_code = nrf_drv_twi_tx(&m_twi, LIS2DH_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false) ;
}

static uint8_t read_reg(uint8_t addr)
{
    read_sensor_data(addr);
    do
    {
        __WFE();
    }while (m_xfer_done == false);

    return m_sample;
}

static uint8_t reg[1] = {0 | 0x80U};

static uint8_t read_regs(uint8_t addr)
{
    ret_code_t err_code;
    m_xfer_done = false;

    /* Read from register. */
    reg[0] = addr | 0x80U;

    nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TXRX(LIS2DH_ADDR,
                                                  reg,
                                                  sizeof(reg),
                                                  (uint8_t*)&m_samples,
                                                  sizeof(m_samples));

    uint32_t flags = NRF_DRV_TWI_FLAG_TX_NO_STOP | NRFX_TWIM_FLAG_HOLD_XFER;

    err_code = nrf_drv_twi_xfer(&m_twi, &xfer, flags);
    APP_ERROR_CHECK(err_code);
}

#define PIN_IN      INT_ACCEL_1

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    trigger_reading();

    //nrf_gpio_pin_toggle(LED_1);
}

/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    //nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    //err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
    //APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_NOPULL;

    err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IN, true);
}



void sensor_init(void)
{

    NRF_LOG_INFO("\r\nTWI sensor example started.");
    NRF_LOG_FLUSH();
    twi_init();

    nrf_delay_ms(35);
    (void)read_reg(LIS2DH_WHO_AM_I);
    NRF_LOG_FLUSH();

    nrf_delay_ms(5);
    write_reg(LIS2DH_CTRL_REG1, 0x97 );  // Data Rate
    nrf_delay_ms(50);
    NRF_LOG_FLUSH();

    nrf_delay_ms(5);
    write_reg(LIS2DH_CTRL_REG4, 0x88 ); // High Resolution
    nrf_delay_ms(50);
    NRF_LOG_FLUSH();

    nrf_delay_ms(5);
    write_reg(LIS2DH_CTRL_REG3, 0x10 ); // Interrupt on new data available
    nrf_delay_ms(50);
    NRF_LOG_FLUSH();

    nrf_delay_ms(5);
    (void)read_reg(LIS2DH_CTRL_REG1);
    nrf_delay_ms(5);
    (void)read_reg(LIS2DH_CTRL_REG4);
    nrf_delay_ms(5);
    (void)read_reg(LIS2DH_CTRL_REG3);
    nrf_delay_ms(15);

    read_regs(LIS2DH_OUT_X_L);
}


void sensor_off(void)
{

hw_timers_stop();

nrfx_gpiote_in_event_disable(PIN_IN);

/*
    nrf_delay_ms(50);
    write_reg(LIS2DH_CTRL_REG1, 0x00 );  // OFf
    nrf_delay_ms(50);
    NRF_LOG_FLUSH();
*/

//    nrf_drv_twi_disable(&m_twi);
    nrf_delay_ms(5);


    nrf_gpio_pin_set(SEN_ENABLE); // accelerometer power off




}







/* This function is never called. It doesn't matter.   */
void timer_led_init(void)
{
    TimedCircBuffer_Init();
}


/**
 * @brief Handler for timer events.
 */
void timer_led_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    static uint32_t i;
    uint32_t led_to_invert = 0;//((i++) % LEDS_NUMBER);

    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            {
            /* Record the current level of the edge interrupt. Should normally be low. */
            uint32_t x = (uint32_t) nrfx_gpiote_in_is_set(PIN_IN);

            int32_t sums_rec [3];
            uint32_t  num_rec;

            sums_rec[0] = sums[0];
            sums_rec[1] = sums[1];
            sums_rec[2] = sums[2];
            num_rec = number;

            number = 0;
            sums[0] = sums[1] = sums[2] = 0L;

            // Now do the division.
            XYZ_T pt;
            if (num_rec > 32)
                num_rec = 32;

            memset((void *)&pt, 0, sizeof(XYZ_T));

            if (num_rec != 0)
            {
                pt.xyz[0] = (int16_t) (sums_rec[0] / ((int32_t) num_rec));
                pt.xyz[1] = (int16_t) (sums_rec[1] / ((int32_t) num_rec));
                pt.xyz[2] = (int16_t) (sums_rec[2] / ((int32_t) num_rec));
            }
            pt.n = (uint16_t) num_rec;

            TimedCircBuffer_Add(&pt);

            // Get and clear the number of reads since the last timer event
            uint32_t count = nrf_atomic_u32_fetch_store(&count_of_Accelerometer_reads, 0U);


            /* If the level is high for two consecutive ticks with no interrupts between then something
             * has halted. Trigger the mechanism.   */
            if (x) {
                if (tick_without_interrupts) {trigger_reading();}
                else
                {
                    tick_without_interrupts = true; 
                    //bsp_board_led_invert(led_to_invert);
                }
            }
            else
            {
                tick_without_interrupts = false;
                //bsp_board_led_invert(led_to_invert);}
            }
            }
            break;

        default:
            //Do nothing.
            break;
    }
}
