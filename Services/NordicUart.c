/**
 * A definition of the Nordic UART service
 */

#include "NordicUart.h"
#include "ble.h"
#include "sdk_common.h"
#include "app_error.h"

#ifdef CENTRAL
#include "ble_nus_c.h"
#endif
#if defined PERIPHERAL || defined CENTRAL
#include "ble_nus.h"
#endif

#include "TimedCircBuffer.h"
#include "app_fifo.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include <string.h>
#include <stdlib.h>


// Unused  ////////////
#if 0
static ble_hrs_t m_hrs;                                             /**< Heart Rate Service instance. */
static ble_rscs_t m_rscs;                                           /**< Running Speed and Cadence Service instance. */
static ble_hrs_c_t m_hrs_c;                                         /**< Heart Rate Service client instance. */
static ble_rscs_c_t m_rscs_c;                                       /**< Running Speed and Cadence Service client instance. */

static uint16_t m_conn_handle_hrs_c  = BLE_CONN_HANDLE_INVALID;     /**< Connection handle for the HRS central application */
static uint16_t m_conn_handle_rscs_c = BLE_CONN_HANDLE_INVALID;     /**< Connection handle for the RSC central application */
#endif
/////////////////////



BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */

static void nus_data_handler(ble_nus_evt_t * p_evt);
#ifdef CENTRAL

#define ECHOBACK_BLE_UART_DATA  1                                       /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */

BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
static void nus_error_handler(uint32_t nrf_error);
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt);
#endif


static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len);


//#ifdef PERIPHERAL

static app_fifo_t tx_fifo;
static app_fifo_t rx_fifo;
static uint8_t tx_fifo_buf[256];
static uint8_t rx_fifo_buf[256];


static bool  is_output_started = false;

static void start_output(void)
{
    if (! is_output_started)
    {
        is_output_started = true;
    }
}

static void stop_output(void)
{
    is_output_started = false;
}

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */

#define MAX_SEND_BYTES    (23)

static bool output_send_a_pkt(void)
{
    if (! is_output_started)
    {
        return false;
    }

    ret_code_t err_code;
    uint32_t length = MAX_SEND_BYTES;
    uint8_t data_array [MAX_SEND_BYTES];
    err_code = app_fifo_read(&tx_fifo, data_array, &length);
    if (err_code == NRF_ERROR_NOT_FOUND || !length)   // ERROR_NOT_FOUND means no data in the fifo. Not a problem, just return.
    {
        return false;
    }
    else
    {
        APP_ERROR_CHECK(err_code);
    }

    do
    {
        err_code = ble_nus_data_send(&m_nus, data_array, (uint16_t *)&length, m_conn_handle);
        if ((err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
    } while (err_code == NRF_ERROR_RESOURCES);

    if (err_code != NRF_SUCCESS)
    {
        return false;
    }

    // Let the timed circular buffer add more data if need be
    TimedCircBuffer_FifoFill(&tx_fifo);

    return (err_code == NRF_SUCCESS);
}





// Populate the data that we're going to send. For now just use random numbers.
static void populate_output(void)
{
#ifdef PERIPHERAL
    int i;

    srand( *((unsigned int *)0x10000080 ) );   // FICR "Encryption Root". It's an unchanging but somewhat random number.

    for(i = 0; i < 256; i ++)
    {
        APP_ERROR_CHECK(app_fifo_put(&tx_fifo, (uint8_t) rand()));
    }
#endif

#ifdef CENTRAL
#if 0   //  -- need access to the leaf list structure
    memset((uint8_t *) output_buff, 0U, 256);
    memcpy((uint8_t *) output_buff, &leaf_list[0], sizeof(LEAF_T) * leaf_list_ptr);
#endif
#endif

}

//#endif




void nus_init(void)
{
    ret_code_t         err_code;
    ble_nus_init_t     nus_init;


    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);


    APP_ERROR_CHECK(app_fifo_init(&tx_fifo, tx_fifo_buf, sizeof(tx_fifo_buf)));
    APP_ERROR_CHECK(app_fifo_init(&rx_fifo, rx_fifo_buf, sizeof(rx_fifo_buf)));
}



#ifdef CENTRAL
/**@brief Function for initializing the Nordic UART Service (NUS) client. */
void nus_c_init(nrf_ble_gq_t * gatt_queue)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler   = ble_nus_c_evt_handler;
    init.error_handler = nus_error_handler;
    init.p_gatt_queue  = gatt_queue;

    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
}
#endif

#if defined PERIPHERAL || defined CENTRAL

void nus_queue_tx_data(uint8_t const * p_data, unsigned int n_data)
{
    if (n_data == 0 || n_data > 250)
        return;

    ret_code_t err_code = app_fifo_write(&tx_fifo, p_data, &n_data);
    APP_ERROR_CHECK(err_code);

    start_output();

    if ( m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        if (! output_send_a_pkt())
        {
            stop_output();
        }
    }
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Processing");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        // Handle the input. Fill it into an app_fifo and process out of that.
        uint32_t length = p_evt->params.rx_data.length;
        app_fifo_write(&rx_fifo, p_evt->params.rx_data.p_data, &length);

        // Now process the fifo if there is enough data there.
        length = 0;
        app_fifo_read(&rx_fifo, NULL, &length);

        if ( length >= 8 )
        {
            uint32_t op [2];

            length = sizeof(op);
            app_fifo_read(&rx_fifo, (uint8_t *) &op, &length);

            (void) TimedCircBuffer_RxOperation(op[0], op[1]);
        }
    }
    else if (p_evt->type == BLE_NUS_EVT_TX_RDY)
    {
        if (m_conn_handle == BLE_CONN_HANDLE_INVALID || ! output_send_a_pkt())
        {
            stop_output();
        }
    }
    else if (p_evt->type == BLE_NUS_EVT_COMM_STARTED)
    {
        m_conn_handle = p_evt->conn_handle;
        if (! output_send_a_pkt())
        {
            stop_output();
        }

    }
    else if (p_evt->type == BLE_NUS_EVT_COMM_STOPPED)
    {
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
        stop_output();
    }

}
/**@snippet [Handling the data received over BLE] */

#endif









#if 0
#ifdef CENTRAL

/**@brief Heart Rate Collector initialization.
 */
static void hrs_c_init(void)
{
    ret_code_t       err_code;
    ble_hrs_c_init_t hrs_c_init_obj;

    hrs_c_init_obj.evt_handler   = hrs_c_evt_handler;
    hrs_c_init_obj.error_handler = service_error_handler;
    hrs_c_init_obj.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_hrs_c_init(&m_hrs_c, &hrs_c_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief RSC collector initialization.
 */
static void rscs_c_init(void)
{
    ret_code_t        err_code;
    ble_rscs_c_init_t rscs_c_init_obj;

    rscs_c_init_obj.evt_handler   = rscs_c_evt_handler;
    rscs_c_init_obj.error_handler = service_error_handler;
    rscs_c_init_obj.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_rscs_c_init(&m_rscs_c, &rscs_c_init_obj);
    APP_ERROR_CHECK(err_code);
}

#endif
#endif


#ifdef CENTRAL
/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;

    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete.");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Nordic UART Service.");
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
        //    scan_start();       <-----  Need to handle this somewhere else.
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */


#if 0
/**@brief Handles events coming from the Heart Rate central module.
 */
static void hrs_c_evt_handler(ble_hrs_c_t * p_hrs_c, ble_hrs_c_evt_t * p_hrs_c_evt)
{
    switch (p_hrs_c_evt->evt_type)
    {
        case BLE_HRS_C_EVT_DISCOVERY_COMPLETE:
        {
            if (m_conn_handle_hrs_c == BLE_CONN_HANDLE_INVALID)
            {
                ret_code_t err_code;

                m_conn_handle_hrs_c = p_hrs_c_evt->conn_handle;
                NRF_LOG_INFO("HRS discovered on conn_handle 0x%x", m_conn_handle_hrs_c);

                filter_settings_change();

                err_code = ble_hrs_c_handles_assign(p_hrs_c,
                                                    m_conn_handle_hrs_c,
                                                    &p_hrs_c_evt->params.peer_db);
                APP_ERROR_CHECK(err_code);
                // Initiate bonding.
                err_code = pm_conn_secure(m_conn_handle_hrs_c, false);
                if (err_code != NRF_ERROR_BUSY)
                {
                    APP_ERROR_CHECK(err_code);
                }

                // Heart rate service discovered. Enable notification of Heart Rate Measurement.
                err_code = ble_hrs_c_hrm_notif_enable(p_hrs_c);
                APP_ERROR_CHECK(err_code);
            }
        } break; // BLE_HRS_C_EVT_DISCOVERY_COMPLETE

        case BLE_HRS_C_EVT_HRM_NOTIFICATION:
        {
            ret_code_t err_code;

            NRF_LOG_INFO("Heart Rate = %d", p_hrs_c_evt->params.hrm.hr_value);

            err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, p_hrs_c_evt->params.hrm.hr_value);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != NRF_ERROR_RESOURCES) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                )
            {
                APP_ERROR_HANDLER(err_code);
            }
        } break; // BLE_HRS_C_EVT_HRM_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Handles events coming from  Running Speed and Cadence central module.
 */
static void rscs_c_evt_handler(ble_rscs_c_t * p_rscs_c, ble_rscs_c_evt_t * p_rscs_c_evt)
{
    switch (p_rscs_c_evt->evt_type)
    {
        case BLE_RSCS_C_EVT_DISCOVERY_COMPLETE:
        {
            if (m_conn_handle_rscs_c == BLE_CONN_HANDLE_INVALID)
            {
                ret_code_t err_code;

                m_conn_handle_rscs_c = p_rscs_c_evt->conn_handle;
                NRF_LOG_INFO("Running Speed and Cadence service discovered on conn_handle 0x%x",
                             m_conn_handle_rscs_c);

                filter_settings_change();

                err_code = ble_rscs_c_handles_assign(p_rscs_c,
                                                    m_conn_handle_rscs_c,
                                                    &p_rscs_c_evt->params.rscs_db);
                APP_ERROR_CHECK(err_code);

                // Initiate bonding.
                err_code = pm_conn_secure(m_conn_handle_rscs_c, false);
                if (err_code != NRF_ERROR_BUSY)
                {
                    APP_ERROR_CHECK(err_code);
                }

                // Running Speed Cadence Service discovered. Enable notifications.
                err_code = ble_rscs_c_rsc_notif_enable(p_rscs_c);
                APP_ERROR_CHECK(err_code);
            }
        } break; // BLE_RSCS_C_EVT_DISCOVERY_COMPLETE:

        case BLE_RSCS_C_EVT_RSC_NOTIFICATION:
        {
            ret_code_t      err_code;
            ble_rscs_meas_t rscs_measurment;

            NRF_LOG_INFO("Speed      = %d", p_rscs_c_evt->params.rsc.inst_speed);

            rscs_measurment.is_running                  = p_rscs_c_evt->params.rsc.is_running;
            rscs_measurment.is_inst_stride_len_present  = p_rscs_c_evt->params.rsc.is_inst_stride_len_present;
            rscs_measurment.is_total_distance_present   = p_rscs_c_evt->params.rsc.is_total_distance_present;

            rscs_measurment.inst_stride_length = p_rscs_c_evt->params.rsc.inst_stride_length;
            rscs_measurment.inst_cadence       = p_rscs_c_evt->params.rsc.inst_cadence;
            rscs_measurment.inst_speed         = p_rscs_c_evt->params.rsc.inst_speed;
            rscs_measurment.total_distance     = p_rscs_c_evt->params.rsc.total_distance;

            err_code = ble_rscs_measurement_send(&m_rscs, &rscs_measurment);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != NRF_ERROR_RESOURCES) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                )
            {
                APP_ERROR_HANDLER(err_code);
            }
        } break; // BLE_RSCS_C_EVT_RSC_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}
#endif

void nus_c_db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    //ble_hrs_on_db_disc_evt(&m_hrs_c, p_evt);
    //ble_rscs_on_db_disc_evt(&m_rscs_c, p_evt);

    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}


/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    NRF_LOG_DEBUG("Receiving data.");
    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);

    for (uint32_t i = 0; i < data_len; i++)
    {
        do
        {
            (void)(p_data[i]);  // TODO: do something with this
        } while (ret_val == NRF_ERROR_BUSY);
    }
    if (ECHOBACK_BLE_UART_DATA)
    {
        // Send data back to the peripheral.
        do
        {
            ret_val = ble_nus_c_string_send(&m_ble_nus_c, p_data, data_len);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
}


/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nus_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

#endif


