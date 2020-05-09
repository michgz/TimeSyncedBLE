/**
 */
/** @file
 *
 * @defgroup main.c
 * @{
 * @}
 * @ingroup  
 * @brief    
 *
 * This file contains the source code for a sample application.
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "boards.h"
#include "nordic_common.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#ifdef FREERTOS
    #include "nrf_sdh_freertos.h"
#endif
#include "bsp.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_conn_state.h"
//#include "nrf_fstorage.h"
//#include "fds.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_timer.h"
#include "nrf_ble_scan.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpiote.h"

#include "time_sync.h"
#include "nrf_ppi.h"
#include "ServiceConfig.h"
#include "ServiceDebug.h"
#include "TimedCircBuffer.h"
#include "ScanList.h"
#include "BroadcastAdvertising.h"
#include "Config.h"
#include "amt.h"

#include "HwTimers.h"
#include "HwSensor.h"

// Relay application related LEDs
#define PERIPHERAL_ADVERTISING_LED      -1
#define PERIPHERAL_CONNECTED_LED        -1
#define CENTRAL_SCANNING_LED            -1
#define CENTRAL_CONNECTED_LED           -1

// AMT related LEDs
#define READY_LED                       -1
#define PROGRESS_LED                    -1
#define DONE_LED                        -1


#define CENTRAL_DEVICE_NAME             "nRF Relay\0"                               /**< Name of device used for advertising. */
#define PERIPHERAL_DEVICE_NAME          "nRF_Node"
#define AMTS_SERVICE_UUID_TYPE          BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(500, UNIT_1_25_MS)             /**< The advertising interval (in units of 0.625 ms). This value corresponds to 187.5 ms. */

#define APP_ADV_DURATION_FIRST          400                                         /**< The advertising duration (180 seconds) in units of 10 milliseconds of the first advertising period. */
#define APP_ADV_DURATION                800                                         /**< The advertising duration (180 seconds) in units of 10 milliseconds of all subsequent periods. */

#define APP_BLE_CONN_CFG_TAG            1                                           /**< Tag that identifies the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SEC_PARAM_BOND                  0                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size in octets. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size in octets. */

/**@brief   Priority of the application BLE event handler.
 * @note    You shouldn't need to modify this value.
 */
#define APP_BLE_OBSERVER_PRIO           3

#define DB_DISCOVERY_INSTANCE_CNT       1                                           /**< Number of DB Discovery instances. */

NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                                    /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                                           /**< Scanning Module instance. */
NRF_BLE_QWRS_DEF(m_qwr,                                                             /**< Context for the Queued Write module.*/
               NRF_SDH_BLE_TOTAL_LINK_COUNT);    
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_discovery,                                          /**< Database discovery module instances. */
                                DB_DISCOVERY_INSTANCE_CNT);

//BLE_DBG_DEF(m_dbg);

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current peripheral-role connection. */

/**@brief   Definition of the application timers used.
 */
APP_TIMER_DEF(m_sync_timer);
APP_TIMER_DEF(m_sleep_timer);
APP_TIMER_DEF(m_scan_timer);
APP_TIMER_DEF(notif_timeout);
APP_TIMER_DEF(m_reading_timer);
APP_TIMER_DEF(led_flash_timer);

#define SLEEP_TIMEOUT    0            /**< Timeout for peripheral to wait before sleeping -- currently 0 to indicate no timeout. */

#define SCAN_TIMEOUT     APP_TIMER_TICKS(120000)                 /**< Time between doing scans  */

static void notif_timeout_handler(void * p_context);
static void scan_timer_timeout_handler(void * p_context);
static void do_time_sync(void);

//NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
//               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
//               NRF_BLE_GQ_QUEUE_SIZE);
//static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
//                        ^----------------   never used ???

/**@brief AMT Service UUID. */
static ble_uuid_t const m_scan_uuid =
{
    .uuid = AMT_SERVICE_UUID,
    .type = AMTS_SERVICE_UUID_TYPE
};

/**@brief UUIDs to advertise. */
static ble_uuid_t m_adv_uuids[] =
{
    {AMT_SERVICE_UUID,        AMTS_SERVICE_UUID_TYPE}
};


/**@brief Names that the central application scans for, and that are advertised by the peripherals.
 *  If these are set to empty strings, the UUIDs defined below are used.
 */
static char const m_target_periph_name[] = "";


static ble_gap_scan_params_t m_scan_param =                 /**< Scan parameters requested for scanning and connection. */
{
    .active        = 0x01,
    .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
    .window        = NRF_BLE_SCAN_SCAN_WINDOW,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .timeout       = NRF_BLE_SCAN_SCAN_DURATION,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
    .extended      = true,
};

static ble_gap_scan_params_t m_scan_param_peripheral =                 /**< Scan parameters requested for scanning only. */
{
    .active        = 0x00,
    .extended      = 0x01,
    .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
    .window        = NRF_BLE_SCAN_SCAN_WINDOW,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .timeout       = NRF_BLE_SCAN_SCAN_DURATION,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
    .extended      = true,
};

static void scan_start (void);

#ifdef PERIPHERAL
//    static bool isPeripheral(void) {return false;}
//    static bool isCentral(void) {return true;}

#define DEVICE_ID_BYTE_0_ADDR   0x10000060
#define DEVICE_ID_BYTE_0     (*((uint8_t const * const)DEVICE_ID_BYTE_0_ADDR))

static const bool isCentral(void) {return (DEVICE_ID_BYTE_0 == 0x07);}
static const bool isPeripheral(void) {return !isCentral();}
#endif
#ifdef CENTRAL
    static bool isPeripheral(void) {return false;}
    static bool isCentral(void) {return true;}
#endif


static void amts_evt_handler(nrf_ble_amts_evt_t);
static nrf_ble_amts_t    m_amts;
NRF_SDH_BLE_OBSERVER(m_amts_ble_obs, BLE_AMTS_BLE_OBSERVER_PRIO, nrf_ble_amts_on_ble_evt, &m_amts);

static void amtc_evt_handler(nrf_ble_amtc_t *, nrf_ble_amtc_evt_t *);
static nrf_ble_amtc_t     m_amtc;
NRF_SDH_BLE_OBSERVER(m_amtc_ble_obs, BLE_AMTC_BLE_OBSERVER_PRIO, nrf_ble_amtc_on_ble_evt, &m_amtc);

static ret_code_t BroadcastAdvertising_SetUp(ble_advertising_t * const p_advertising, bool isBroadcast);

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static bool isAwake = false;

static void do_sleep(void)
{
    if (isAwake)
    {
        isAwake = false;
        nrf_ppi_channel_disable(NRF_PPI_CHANNEL7);
        sensor_off();
    }
}

static void do_wake(void)
{
    isAwake = true;
    nrf_ppi_channel_enable(NRF_PPI_CHANNEL7);
}

bool IsAwake(void) {return isAwake;}

static void on_sleep_timer_timeout(void * p_context)
{
    (void) p_context;

    ret_code_t err_code;

    // The timer has timed out. Central should not make any changes, peripheral can sleep
    // at this point (continuing advertising).

    if (isPeripheral())
    {
        do_sleep();
    }
}

static void flash_led(void)
{
    ret_code_t err_code;

    bsp_board_led_on(BSP_BOARD_LED_0);

    err_code = app_timer_start(led_flash_timer, APP_TIMER_TICKS(200), (void *) 0);
    APP_ERROR_CHECK(err_code);
}

static void led_flash_timeout_handler(void * p_context)
{
    bsp_board_led_off(BSP_BOARD_LED_0);
}

static void reading_timeout_handler(void * p_context);
static void BroadcastAdvertising_SetData(uint32_t);

static bool m_bb = false;

// Supported by Central only. Trigger a reading in all leaf devices.
void doTrigger(void)
{
    if (!isCentral())
    {
        APP_ERROR_CHECK(NRF_ERROR_FORBIDDEN);
    }

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)  // Only do broadcast if we're connected as peripheral -- otherwise are busy advertising
    {
        ret_code_t err_code;

        //(void) sd_ble_gap_adv_stop(m_advertising.adv_handle);

        if (isUseSyncTimer())
        {
            BroadcastAdvertising_SetData(  (uint32_t) (ts_timestamp_get_ticks_u64(6) / 32000ULL)  );
        }

        err_code = BroadcastAdvertising_SetUp(&m_advertising, true);
        APP_ERROR_CHECK(err_code);

        // Now do a simplified version of ble_advertising_Start() with a non-default version of properties.type
        m_advertising.adv_mode_current = BLE_ADV_MODE_FAST;
        if (true)
            m_advertising.adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
        else
            m_advertising.adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
        err_code = sd_ble_gap_adv_set_configure(&m_advertising.adv_handle, m_advertising.p_adv_data, &m_advertising.adv_params);
        APP_ERROR_CHECK(err_code);
        err_code = sd_ble_gap_adv_start(m_advertising.adv_handle, m_advertising.conn_cfg_tag);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sleep_timer, APP_TIMER_MODE_SINGLE_SHOT, on_sleep_timer_timeout);
    APP_ERROR_CHECK(err_code);

    do_wake();

    if (SLEEP_TIMEOUT != 0)
    {
        err_code = app_timer_start(m_sleep_timer, SLEEP_TIMEOUT, (void *) 0);
        APP_ERROR_CHECK(err_code); 
    }

    if (isCentral())
    {
        err_code = app_timer_create(&m_scan_timer, APP_TIMER_MODE_REPEATED, scan_timer_timeout_handler);
        APP_ERROR_CHECK(err_code);

        err_code = app_timer_start(m_scan_timer, SCAN_TIMEOUT, (void *) 0);
        APP_ERROR_CHECK(err_code);

        err_code = app_timer_create(&notif_timeout, APP_TIMER_MODE_REPEATED, notif_timeout_handler);
        APP_ERROR_CHECK(err_code);

        err_code = app_timer_create(&m_reading_timer, APP_TIMER_MODE_SINGLE_SHOT, reading_timeout_handler);
        APP_ERROR_CHECK(err_code);
    }

    err_code = app_timer_create(&led_flash_timer, APP_TIMER_MODE_SINGLE_SHOT, led_flash_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(const bool central)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                                central ? (const uint8_t *)CENTRAL_DEVICE_NAME : (const uint8_t *)PERIPHERAL_DEVICE_NAME,
                                                central ? strlen(CENTRAL_DEVICE_NAME) : strlen(PERIPHERAL_DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

#if 1 // def PERIPHERAL
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
#else
    gap_conn_params.min_conn_interval = m_scan.conn_params.min_conn_interval;
    gap_conn_params.max_conn_interval = m_scan.conn_params.max_conn_interval;
    gap_conn_params.slave_latency     = m_scan.conn_params.slave_latency;
    gap_conn_params.conn_sup_timeout  = m_scan.conn_params.conn_sup_timeout;
#endif

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


config_service_t cfgs;

/**@brief Function for handling events from the Config module.
 */
uint16_t cfgs_handler(ble_evt_t const * p_evt)
{
    return service_cfg_on_ble_evt(&cfgs, p_evt);
}



/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr[0], &qwr_init);
    APP_ERROR_CHECK(err_code);

    //err_code = ble_debug_service_init(&m_dbg);
    //APP_ERROR_CHECK(err_code);

    // Initialise AMTS service
    // A Central device needs a big TX FIFO in order to relay large data sets.
    // The Peripheral does not, as it just uses data directly from its own
    // buffer spaces.
    nrf_ble_amts_init(&m_amts, amts_evt_handler, (isCentral() ? 16384 : 1024), 256);

    if (isCentral())
    {
        // Initialise the ATMC service.
        nrf_ble_amtc_init_t amtc_init;

        memset(&amtc_init, 0, sizeof(amtc_init));

        amtc_init.evt_handler  = amtc_evt_handler;
        amtc_init.p_gatt_queue = &m_ble_gatt_queue;

        err_code = nrf_ble_amtc_init(&m_amtc, &amtc_init);
        APP_ERROR_CHECK(err_code);
    }

    // Initialise the Config service
    service_config_init_t cfgs_init;
    cfgs_init.evt_handler = NULL;
    cfgs_init.error_handler = NULL;
    cfgs_init.p_cfgs_ctx = NULL;
    (void) service_config_init(&cfgs_init, &cfgs);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_CONN_HANDLE_INVALID; // Start upon connection.
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


#ifdef PERIPHERAL
/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
//    err_code = bsp_btn_ble_sleep_mode_prepare();
//    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}
#endif


/**@brief Function for handling the Heart Rate Service Client
 *        Running Speed and Cadence Service Client.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void check_manu_data(char const * x, int len)
{
    // Check if manufacturer data meets the expected format.
    if (len >= 6)
    {
        ble_advdata_manuf_data_t const * manu = (ble_advdata_manuf_data_t const *) x;
        if (manu->company_identifier == MY_MANUFACTURER_ID)
        {
            if (isPeripheral())
            {
                // Pretend like we've received a lock instruction INSTRUCTION_CODE__LOCK
                const uint32_t INSTRUCTION_CODE__LOCK = 0x0A;
                (void) TimedCircBuffer_RxOperation_NoResponse(INSTRUCTION_CODE__LOCK, *((uint32_t const *) &x[2]));
            }
        }
    }

}

void trigger(uint32_t size)
{
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {

        if (isUploadTriggerSize())
        {
            uint32_t root_size = (uint32_t) sqrtf((float) size);

            uint32_t data [3] = {0x80000022, root_size, (uint32_t) (ts_timestamp_get_ticks_u64(6) / 32000ULL)  };

            amts_queue_tx_data((uint8_t const *) &data, 3*sizeof(uint32_t));
        }
        else if (!isLedOnBroadcast() && LeafListCount() > 0)
        {
            doTrigger();
        }

    }

} 


/** Returns true if triggered.   */
static bool trigger_from_scan_timer_timeout(void)
{
    if (isLedOnBroadcast() && m_conn_handle != BLE_CONN_HANDLE_INVALID && LeafListCount() > 0)
    {
        bool debugCase = false;

        // Discard a few random numbers in case there are some stuck values at the beginning
        (void)rand();
        (void)rand();
        (void)rand();

        int r = rand();

        if ((r&7) <= 5)
        {
            // This is a debug condition. In 87.5% percent of scans, doesn't connect just uploads scan results
            debugCase = true;
        }

        if (!debugCase)
        {
            // Need to have advertising available to us.
            if (m_advertising.adv_mode_current != BLE_ADV_MODE_IDLE)
            {
                return false;
            }

            doTrigger();
            return true;
        }
        else
        {
            if (debugCase || isUploadScanResults())
            {
                ScanListClearReading();
                StartSending(&ScanList_FifoFill);
            }

            return false;
        }

    }
    return false;
}

/** Returns true if scan started.   */
static bool scan_from_scan_timer_timeout(void)
{
    if (isCentral() && m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // If not connected, time syncs are aligned with advertising timeouts. When
        // connected there is not that route, so we must do here.
        do_time_sync();
    }
    else
    {
        scan_start();
    }
    return true;
}

static bool doingDownload = false;
static unsigned int downloadPtr = 0;

#define READING_TIMEOUT_FIRST        APP_TIMER_TICKS(5000)   // Need a long initial timeout to let the readings complete
#define READING_TIMEOUT              APP_TIMER_TICKS(200)

static bool do_a_connection(void)
{
    // Connect to next unit

    if (ConnectLeaf(downloadPtr, &m_scan))
    {
        downloadPtr ++;
        return true;
    }
    else
    {
        // The readout is finished.
        doingDownload = false;
        return false;
    }
}

static bool doNextConnection(void)
{
    if (doingDownload)
    {
        app_timer_start(m_reading_timer, READING_TIMEOUT, (void *) 0);
        return true;
    }
    return false;
}

static bool doConnectionAndDownload(void)
{
    // If any found, then connect to them
    if (LeafListCount() > 0)
    {
        doingDownload = true;
        downloadPtr = 0;
        app_timer_start(m_reading_timer, READING_TIMEOUT_FIRST, (void *) 0);
        return true;
    }
    return false;
}

static void reading_timeout_handler(void * p_context)
{
    do_a_connection();
}

/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
        } break;

        case NRF_BLE_SCAN_EVT_CONNECTED:
        {
            ble_gap_evt_connected_t const * p_connected =
                             p_scan_evt->params.connected.p_connected;
            // Scan is automatically stopped by the connection.
            NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                     p_connected->peer_addr.addr[0],
                     p_connected->peer_addr.addr[1],
                     p_connected->peer_addr.addr[2],
                     p_connected->peer_addr.addr[3],
                     p_connected->peer_addr.addr[4],
                     p_connected->peer_addr.addr[5]
                     );
        } break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
            NRF_LOG_INFO("Scan timed out.");
            //scan_start();

            // Scan completed. Now start beaconing.
#if 0
            upload_leaf_list();
#endif


            // (void)doConnectionAndDownload();


        } break;

        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
        {
            on_scan_list_scan_evt(p_scan_evt);

            // Initiate connection - don't need to do this, since using "connect_if_match".
            //err_code = sd_ble_gap_connect(&p_adv->peer_addr,
            //                              p_scan_param,
            //                              &m_scan.conn_params,
            //                              APP_BLE_CONN_CFG_TAG);
            //APP_ERROR_CHECK(err_code);
        } break;


        case NRF_BLE_SCAN_EVT_NOT_FOUND:
        {
            // This is called if a device is seen without filters enabled, i.e. the peripheral case.
            ble_gap_evt_adv_report_t const * adv = p_scan_evt->params.p_not_found;


            //if (adv->peer_addr.addr[0] == 0x8b)  // temporary -- look for a known address
            {

            // Only recognise non-connectable, non-scannable and non-directed advertisements.
            if (!adv->type.connectable && !!adv->type.scannable && !adv->type.directed)
            {
                uint16_t offs = 0;
                uint16_t len = ble_advdata_search(adv->data.p_data, adv->data.len, &offs, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA);
                if (len > 0)
                {
                    check_manu_data(&adv->data.p_data[offs], len);
                }
            }
            }

        } break;

        default:
            break;
    }
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    if (isPeripheral())
    {
        ret_code_t          err_code;
        nrf_ble_scan_init_t init_scan;

        memset(&init_scan, 0, sizeof(init_scan));

        init_scan.connect_if_match = false;
        init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;
        init_scan.p_scan_param = &m_scan_param_peripheral;

        err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
        APP_ERROR_CHECK(err_code);
    }
    else if (isCentral())
    {
        ret_code_t          err_code;
        nrf_ble_scan_init_t init_scan;

        memset(&init_scan, 0, sizeof(init_scan));

        init_scan.connect_if_match = false;
        init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;
        init_scan.p_scan_param = &m_scan_param;

        err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
        APP_ERROR_CHECK(err_code);

        //if (strlen(m_target_periph_name) != 0)
        //{
        //    err_code = nrf_ble_scan_filter_set(&m_scan, 
        //                                       SCAN_NAME_FILTER, 
        //                                       m_target_periph_name);
        //    APP_ERROR_CHECK(err_code);
        //}

        err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_scan_uuid);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t err_code;

    ClearLeafList();

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);
}

static unsigned int debug_pkt_count = 0;

static bool DebugPacketFillFifo(app_fifo_t * const p_fifo)
{
    if (debug_pkt_count >= (10000/4))
    {
        return false;
    }

    uint32_t len;
    app_fifo_write(p_fifo, NULL, &len);
    while(len >= 4)
    {
        uint32_t len_2;

        len_2 = 4;
        app_fifo_write(p_fifo, (const uint8_t *)&debug_pkt_count, &len_2);
        len -= 4;
        debug_pkt_count ++;
        if (debug_pkt_count >= (10000/4))
        {
            break;
        }
    }
    return true;

}

static bool debug_packet_from_scan_timer_timeout(void)
{
    debug_pkt_count = 0;
    StartSending(&DebugPacketFillFifo);
    return true;
}

/**@brief Handler for timeout of the regular scan timer. */
static void scan_timer_timeout_handler(void * p_context)
{
    if (isCentral())
    {
        if (isDebugPacket() && debug_packet_from_scan_timer_timeout())
        {
        }
        else if (trigger_from_scan_timer_timeout())
        {
            // Has triggered a reading. It will proceed from here.
            // Flash the LED if configured to.
            if (isLedOnBroadcast())
            {
                flash_led();
            }
        }
        else if (scan_from_scan_timer_timeout())
        {
            // Has started a scan. It will proceed automatically from here, no action needed.
        }
    }
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_db_discovery_t const * p_db = (ble_db_discovery_t *)p_evt->params.p_db_instance;

//    nus_c_db_disc_handler(p_evt);

    if (p_evt->evt_type == BLE_DB_DISCOVERY_AVAILABLE) {
        NRF_LOG_INFO("DB Discovery instance %p available on conn handle: %d",
                     p_db,
                     p_evt->conn_handle);
        NRF_LOG_INFO("Found %d services on conn_handle: %d",
                     p_db->srv_count,
                     p_evt->conn_handle);
    }

    nrf_ble_amtc_on_db_disc_evt(&m_amtc, p_evt);
}


/**@brief Function for initializing the advertising and the scanning.
 */
static void adv_scan_start(void)
{
    ret_code_t err_code;

    //check if there are no flash operations in progress
    //if (!nrf_fstorage_is_busy(NULL))
    {
        // Start scanning for broadcasts.

        if (isPeripheral())   // << for testing only, don't want central scanning
        {
            scan_start();
        }

        // Turn on the LED to signal scanning.
        if (CENTRAL_SCANNING_LED != -1)
            bsp_board_led_on(CENTRAL_SCANNING_LED);

        if (!isCentral() || m_conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            // Start advertising.
            err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);
        }
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            adv_scan_start();
            break;

        default:
            break;
    }
}

static void on_sync_timer_timeout(void * p_context)
{
    (void) p_context;

    ret_code_t err_code;

    if (isCentral())
    {
        err_code = ts_tx_stop();
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("Stopping sync beacon\r\n");
    }

    if (isCentral() && m_conn_handle != BLE_CONN_HANDLE_INVALID && m_advertising.adv_mode_current == BLE_ADV_MODE_IDLE)
    {
        scan_start();
    }
    else
    {
        // New fast advertising timeout in units of 10ms. I think it's okay to just
        // write in a new value like this.
        m_advertising.adv_modes_config.ble_adv_fast_timeout = APP_ADV_DURATION;

        // Restart advertising and scanning
        adv_scan_start();
    }
}

static void do_time_sync(void)
{
    ret_code_t err_code;

    if (isCentral() && isUseSyncTimer())
    {
        nrf_ble_scan_stop();

        err_code = ts_tx_start(200);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("Starting sync beacon transmission!\r\n");

        err_code = app_timer_start(m_sync_timer, 5000, (void *)0 );
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // The peripheral doesn't need to send a sync here. Just call the callback immediately
        // to re-start advertising
        on_sync_timer_timeout((void *)0 );
    }
}


/**@brief Function for handling advertising events.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
        {
            // Advertising has timed out. Stop scanning as well and give a time sync. After that, can start
            // advertising and scanning again.

            do_time_sync();

        } break;

        default:
            break;
    }
}

static void on_adv_evt_broadcast(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_IDLE:
            // Timed out?
    ///        err_code = BroadcastAdvertising_SetUp(&m_advertising, false);
     //       APP_ERROR_CHECK(err_code);

            // Start scanning for peripherals and initiate connection to devices which
            // advertise Heart Rate or Running speed and cadence UUIDs.
       //  scan_start();


            // We've finished broadcasting. Prepare to do readings.
            (void)doConnectionAndDownload();


            break;

        default:
            break;
    }
}


static uint8_t BroadcastData[4] = {0xDE, 0xAD, 0xBE, 0xEF};

static void BroadcastAdvertising_SetData(uint32_t x)
{
    memcpy(BroadcastData, (uint8_t *) &x, 4);
}

static ret_code_t BroadcastAdvertising_SetUp(ble_advertising_t * const p_advertising, bool isBroadcast)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    if (!isBroadcast)
    {
        init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
        init.advdata.include_appearance = false;
        init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

        init.advdata.uuids_more_available.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
        init.advdata.uuids_more_available.p_uuids  = m_adv_uuids;

        init.config.ble_adv_fast_enabled  = true;
        init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
        init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

        init.evt_handler = on_adv_evt;
    }
    else
    {
        // Initialise a broadcast advertising type

        init.advdata.name_type          = BLE_ADVDATA_NO_NAME;
        init.advdata.include_appearance = false;
        init.advdata.flags              = 0; //  non-discoverable?? Somehow this makes it invisible to the BlueNRG, but not to the peripheral device.

        ble_advdata_manuf_data_t manu_data;
        manu_data.company_identifier = MY_MANUFACTURER_ID;
        manu_data.data.size = 4;
        manu_data.data.p_data = (uint8_t *) BroadcastData;
        init.advdata.p_manuf_specific_data = &manu_data;

        init.config.ble_adv_fast_enabled  = true;
        init.config.ble_adv_fast_interval = MSEC_TO_UNITS(150, UNIT_1_25_MS);
        init.config.ble_adv_fast_timeout  = APP_ADV_DURATION_FIRST;

        init.evt_handler = on_adv_evt_broadcast;
    }

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

    return err_code;
}

/**@brief Function for assigning new connection handle to available instance of QWR module.
 *
 * @param[in] conn_handle New connection handle.
 */
static void multi_qwr_conn_handle_assign(uint16_t conn_handle)
{
    for (uint32_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
    {
        if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            ret_code_t err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], conn_handle);
            APP_ERROR_CHECK(err_code);
            break;
        }
    }
}


#ifdef CENTRAL
/**
 * @brief Function for handling shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);
#endif


/**@brief Function for checking whether a bluetooth stack event is an advertising timeout.
 *
 * @param[in] p_ble_evt Bluetooth stack event.
 */
static bool ble_evt_is_advertising_timeout(ble_evt_t const * p_ble_evt)
{
    return (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_SET_TERMINATED);
}

static ble_gap_addr_t m_curr_periph_addr;
static uint16_t m_curr_periph_conn_handle;


/**@brief   Function for handling BLE events from peripheral and central applications.
 * @details Updates the status LEDs used to report the activity of the applications.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t const * p_ble_evt)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            if (isPeripheral())
            {
                NRF_LOG_INFO("Peripheral connected");
                if (PERIPHERAL_ADVERTISING_LED != -1)
                    bsp_board_led_off(PERIPHERAL_ADVERTISING_LED);
                if (PERIPHERAL_CONNECTED_LED != -1)
                    bsp_board_led_on(PERIPHERAL_CONNECTED_LED);
            }
            else if (isCentral())
            {
                NRF_LOG_INFO("Central connected");

                // We must be in the central role in order to do discovery
                if (p_ble_evt->evt.gap_evt.params.connected.role == BLE_GAP_ROLE_CENTRAL)
                {
                    m_curr_periph_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                    memcpy(&m_curr_periph_addr, &p_ble_evt->evt.gap_evt.params.connected.peer_addr, sizeof(ble_gap_addr_t));

                    err_code = ble_db_discovery_start(&m_db_discovery[0], p_ble_evt->evt.gap_evt.conn_handle);
                    APP_ERROR_CHECK(err_code);
                }
            }

            if (p_ble_evt->evt.gap_evt.params.connected.role == BLE_GAP_ROLE_PERIPH)
            {
                m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                // On connection, the advertising will always be stopped.
                m_advertising.adv_mode_current = BLE_ADV_MODE_IDLE; 
            }

            // Assign connection handle to the QWR module.
            multi_qwr_conn_handle_assign(p_ble_evt->evt.gap_evt.conn_handle);


            if (0 && isCentral())
            {
                // Update LEDs status, and check whether to look for more peripherals to connect to.
                if (CENTRAL_CONNECTED_LED != -1)
                    bsp_board_led_on(CENTRAL_CONNECTED_LED);
                if (ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT)
                {
                    // Have filled the quota of central connections. Don't scan
                    if (CENTRAL_SCANNING_LED != -1)
                        bsp_board_led_off(CENTRAL_SCANNING_LED);
                }
                else
                {
                    // Resume scanning.
                    if (CENTRAL_SCANNING_LED != -1)
                        bsp_board_led_on(CENTRAL_SCANNING_LED);
                    scan_start();
                }
            }

            if (isPeripheral())
            {
                // Any connection wakes the device up. Restart the timer.

                do_wake();
                APP_ERROR_CHECK(app_timer_stop(m_sleep_timer));
            }

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            if (p_ble_evt->evt.gap_evt.conn_handle == m_conn_handle)
            {
                m_conn_handle = BLE_CONN_HANDLE_INVALID;

                if (m_advertising.adv_mode_current == BLE_ADV_MODE_IDLE)
                {
                    err_code = BroadcastAdvertising_SetUp(&m_advertising, false);
                    APP_ERROR_CHECK(err_code);

                    // Now do a simplified version of ble_advertising_Start()
                    m_advertising.adv_mode_current = BLE_ADV_MODE_FAST;
                    m_advertising.adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
                    err_code = sd_ble_gap_adv_set_configure(&m_advertising.adv_handle, m_advertising.p_adv_data, &m_advertising.adv_params);
                    APP_ERROR_CHECK(err_code);
                    err_code = sd_ble_gap_adv_start(m_advertising.adv_handle, m_advertising.conn_cfg_tag);
                    APP_ERROR_CHECK(err_code);
                }

            }

            if (isPeripheral())
            {
                NRF_LOG_INFO("Peripheral disconnected. conn_handle: 0x%x, reason: 0x%x",
                             p_gap_evt->conn_handle,
                             p_gap_evt->params.disconnected.reason);

                if (PERIPHERAL_CONNECTED_LED != -1)
                    bsp_board_led_off(PERIPHERAL_CONNECTED_LED);

                if (SLEEP_TIMEOUT != 0)
                {
                    err_code = app_timer_start(m_sleep_timer, SLEEP_TIMEOUT, (void *) 0);
                    APP_ERROR_CHECK(err_code);
                }
            }
            else if (isCentral())
            {
                // Start scanning.
                //scan_start();

                if (p_ble_evt->evt.gap_evt.conn_handle == m_curr_periph_conn_handle)
                {
                    m_curr_periph_conn_handle = BLE_CONN_HANDLE_INVALID;
                    memset(&m_curr_periph_addr, 0, sizeof(ble_gap_addr_t));
                }

                if (ble_conn_state_central_conn_count() == 0)
                {
                    if (CENTRAL_CONNECTED_LED != -1)
                        bsp_board_led_off(CENTRAL_CONNECTED_LED);
                }
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (isCentral())
            {
                // No timeout for scanning is specified, so only connection attempts can timeout.
                if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
                {
                    NRF_LOG_INFO("Connection Request timed out.");
                }
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            if (isCentral())
            {
                // Accept parameters requested by peer.
                err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                            &p_gap_evt->params.conn_param_update_request.conn_params);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            //if (isPeripheral())
            {
                // No system attributes have been stored.
                err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gap_evt.conn_handle, NULL, 0, 0);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief AMT server event handler. */
static void amts_evt_handler(nrf_ble_amts_evt_t evt)
{
    ret_code_t err_code;

    switch (evt.evt_type)
    {
        case NRF_BLE_AMTS_EVT_NOTIF_ENABLED:
        {
            NRF_LOG_INFO("Notifications enabled.");

//            nrf_ble_amts_notif_spam(&m_amts);

            if (READY_LED != -1)
                bsp_board_led_on(READY_LED);
#if 0
            m_notif_enabled = true;

/*
            if (m_board_role != BOARD_TESTER)
            {
                return;
            }
*/

            if (m_test_params.conn_interval != CONN_INTERVAL_DEFAULT)
            {
                NRF_LOG_DEBUG("Updating connection parameters..");
                m_conn_param.min_conn_interval = m_test_params.conn_interval;
                m_conn_param.max_conn_interval = m_test_params.conn_interval;
                err_code = sd_ble_gap_conn_param_update(m_conn_handle, &m_conn_param);

                if (err_code != NRF_SUCCESS)
                {
                    NRF_LOG_ERROR("sd_ble_gap_conn_param_update() failed: 0x%x.", err_code);
                }
            }
            else
            {
                m_conn_interval_configured = true;
            }
#endif // 0
        } break;

        case NRF_BLE_AMTS_EVT_NOTIF_DISABLED:
        {
            NRF_LOG_INFO("Notifications disabled.");
            if (READY_LED != -1)
                bsp_board_led_off(READY_LED);
        } break;

        case NRF_BLE_AMTS_EVT_TRANSFER_1KB:
        {
            NRF_LOG_INFO("Sent %u KBytes", (evt.bytes_transfered_cnt / 1024));
            if (PROGRESS_LED != -1)
                bsp_board_led_invert(PROGRESS_LED);
        } break;

        case NRF_BLE_AMTS_EVT_TRANSFER_FINISHED:
        {
#if 0
            counter_stop();
#endif // 0
            if (PROGRESS_LED != -1)
                bsp_board_led_off(PROGRESS_LED);
            if (DONE_LED != -1)
                bsp_board_led_on(DONE_LED);

#if 0
            uint32_t time_ms      = counter_get();
            uint32_t bit_count    = (evt.bytes_transfered_cnt * 8);
            float throughput_kbps = ((bit_count / (time_ms / 1000.f)) / 1000.f);

            NRF_LOG_INFO("Done.");
            NRF_LOG_INFO("=============================");
            NRF_LOG_INFO("Time: %u.%.2u seconds elapsed.", (time_ms / 1000), (time_ms % 1000));
            NRF_LOG_INFO("Throughput: " NRF_LOG_FLOAT_MARKER " Kbps.",
                         NRF_LOG_FLOAT(throughput_kbps));
            NRF_LOG_INFO("=============================");
            NRF_LOG_INFO("Sent %u bytes of ATT payload.", evt.bytes_transfered_cnt);
            NRF_LOG_INFO("Retrieving amount of bytes received from peer...");

            err_code = nrf_ble_amtc_rcb_read(&m_amtc);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("nrf_ble_amtc_rcb_read() failed: 0x%x.", err_code);
                test_terminate();
            }
#endif // 0
        } break;
    }
}


static uint32_t time_start_of_tfer;   // from app_timer_cnt_get(), so units of 1/32786 sec

static int notif_time = 0;  // Counts down. 1/4 seconds


static void notif_timeout_handler(void * p_context)
{
    if (notif_time > 0)
    {
        notif_time --;

        if (notif_time == 0)
        {
            uint32_t time_taken = app_timer_cnt_diff_compute(app_timer_cnt_get(), time_start_of_tfer);
            app_timer_stop(notif_timeout);

            nrf_ble_amtc_t * p_amt_c = (nrf_ble_amtc_t *) p_context;

            if (!p_amt_c)
            {
                APP_ERROR_CHECK(NRF_ERROR_NULL);
            }

            // Timed out. In the summary case, upload the summary now.
            if(isCentral() && isRelaySummary())
            {
                uint32_t summ [6] = {0x8000000F, 0, 0, time_taken, p_amt_c->bytes_rcvd_cnt, amts_get_rejected_byte_count()};

                if (p_amt_c->conn_handle != BLE_CONN_HANDLE_INVALID)
                {
                    // Get the ble_addr_t to summ[1:2]

                    if (p_amt_c->conn_handle == m_curr_periph_conn_handle)
                    {
                        memcpy((void *) &summ[1], &m_curr_periph_addr, sizeof(ble_gap_addr_t));
                    }

                }
                amts_queue_tx_data((uint8_t *) summ, sizeof(summ));
            }

            ret_code_t err_code = sd_ble_gap_disconnect(p_amt_c->conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);

            (void) doNextConnection();
        }
    }
}

/**@brief AMT Client event handler.  */
static void amtc_evt_handler(nrf_ble_amtc_t * p_amt_c, nrf_ble_amtc_evt_t * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_type)
    {
        case NRF_BLE_AMT_C_EVT_DISCOVERY_COMPLETE:
        {
            NRF_LOG_INFO("AMT service discovered at peer.");

            err_code = nrf_ble_amtc_handles_assign(p_amt_c,
                                                   p_evt->conn_handle,
                                                   &p_evt->params.peer_db);
            APP_ERROR_CHECK(err_code);

            // Enable notifications.
            err_code = nrf_ble_amtc_notif_enable(p_amt_c);
            APP_ERROR_CHECK(err_code);
        } break;

        case NRF_BLE_AMT_C_EVT_NOTIFICATIONS_SET:
        {
            notif_time = 4;

            amts_clear_rejected_byte_count();

            err_code = app_timer_start(notif_timeout, APP_TIMER_TICKS(250), (void *) p_amt_c);
            APP_ERROR_CHECK(err_code);

            time_start_of_tfer = app_timer_cnt_get();

            uint32_t op[2] = {0x000000e,0x000000e};

            err_code = nrf_ble_amtc_write(p_amt_c, (uint8_t const *) op, sizeof(op));
            APP_ERROR_CHECK(err_code);
        } break;

        case NRF_BLE_AMT_C_EVT_NOTIFICATION:
        {

            // Restart the timer
            if (notif_time < 2)
                notif_time = 2;

            // Relay on
            amts_queue_tx_data(p_evt->params.hvx.data, p_evt->params.hvx.notif_len);

#if 0
            static uint32_t bytes_cnt  = 0;
            static uint32_t kbytes_cnt = 0;

            if (p_evt->params.hvx.bytes_sent == 0)
            {
                bytes_cnt  = 0;
                kbytes_cnt = 0;
            }

            bytes_cnt += p_evt->params.hvx.notif_len;

            if (bytes_cnt > 1024)
            {
                if (PROGRESS_LED != -1)
                    bsp_board_led_invert(PROGRESS_LED);

                bytes_cnt -= 1024;
                kbytes_cnt++;

                NRF_LOG_INFO("Received %u kbytes", kbytes_cnt);

                nrf_ble_amts_rbc_set(&m_amts, p_evt->params.hvx.bytes_rcvd);
            }

            if (p_evt->params.hvx.bytes_rcvd >= AMT_BYTE_TRANSFER_CNT)
            {
                if (PROGRESS_LED != -1)
                    bsp_board_led_off(PROGRESS_LED);

                bytes_cnt  = 0;
                kbytes_cnt = 0;

                NRF_LOG_INFO("Transfer complete, received %u bytes of ATT payload.",
                             p_evt->params.hvx.bytes_rcvd);

                nrf_ble_amts_rbc_set(&m_amts, p_evt->params.hvx.bytes_rcvd);
            }
#endif
        } break;

        case NRF_BLE_AMT_C_EVT_RBC_READ_RSP:
        {
            NRF_LOG_INFO("Peer received %u bytes of ATT payload.", (p_evt->params.rcv_bytes_cnt));
#if 0
            test_terminate();
#endif
        } break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    // Dispatch to the handler.
    on_ble_evt(p_ble_evt);

    // TODO: make this an observer
    cfgs_handler(p_ble_evt);
}

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Increase the attribute table size.
    ble_cfg_t cfg;
    cfg.gatts_cfg.attr_tab_size.attr_tab_size = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT + 1000;
    err_code = sd_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, &cfg, ram_start);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for initializing the Peer Manager.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    if (isCentral())
    {
        //NRF_LOG_INFO("Erase bonds!");

        //err_code = pm_peers_delete();
        //APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        //m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        //NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);

    nrf_ble_amts_on_gatt_evt(&m_amts, p_evt);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    //if (isPeripheral())
    {
        err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
        APP_ERROR_CHECK(err_code);
    }
    if (isCentral())
    {
        //err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
        //APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        default:
            break;
    }
}

/**@brief Function for initializing the advertising functionality.
 */
static void advertising_init(void)
{
    // Both peripheral and central start off with normal (non-broadcast) advertising.
    APP_ERROR_CHECK(BroadcastAdvertising_SetUp(&m_advertising, false));
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_NONE   /*| BSP_INIT_LEDS*/ /*| BSP_INIT_BUTTONS*/, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = false;
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);

    ble_uuid_t uuid;
    uuid.type = AMTS_SERVICE_UUID_TYPE;
    uuid.uuid = AMT_SERVICE_UUID;

    err_code = ble_db_discovery_evt_register(&uuid);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Set up the time sync functionality (both transmit & receive).
 */
static void sync_timer_init(void)
{
    uint32_t       err_code;
    uint8_t        rf_address[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x19};
    ts_params_t    ts_params;
    
    // Debug pin: 
    // nRF52-DK (PCA10040) Toggle P0.24 from sync timer to allow pin measurement
    // nRF52840-DK (PCA10056) Toggle P1.14 from sync timer to allow pin measurement
#if defined(BOARD_PCA10040)
    nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(0, 24), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
    nrf_gpiote_task_enable(3);
#elif defined(BOARD_PCA10056)
    nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(1, 14), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
    nrf_gpiote_task_enable(3);
#elif defined(BOARD_CUSTOM)
    // Use the HW_REV1 pin as debug pin. There's a nice test pad there.
    nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(0, HWREV_PIN_1), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
    nrf_gpiote_task_enable(3);
#else
#warning Debug pin not set
#endif
    
    nrf_ppi_channel_endpoint_setup(
        NRF_PPI_CHANNEL0, 
        (uint32_t) nrf_timer_event_address_get(NRF_TIMER3, NRF_TIMER_EVENT_COMPARE4),
        nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
    nrf_ppi_channel_enable(NRF_PPI_CHANNEL0);
    
    ts_params.high_freq_timer[0] = NRF_TIMER3;
    ts_params.high_freq_timer[1] = NRF_TIMER2;
    ts_params.rtc             = NRF_RTC1;
    ts_params.egu             = NRF_EGU3;
    ts_params.egu_irq_type    = SWI3_EGU3_IRQn;
    ts_params.ppi_chg         = 0;
    ts_params.ppi_chns[0]     = 1;
    ts_params.ppi_chns[1]     = 2;
    ts_params.ppi_chns[2]     = 3;
    ts_params.ppi_chns[3]     = 4;
    ts_params.rf_chn          = 125; /* For testing purposes */
    memcpy(ts_params.rf_addr, rf_address, sizeof(rf_address));
    
    err_code = ts_init(&ts_params);
    APP_ERROR_CHECK(err_code);
    
    err_code = ts_enable();
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_INFO("Started listening for beacons.\r\n");
    NRF_LOG_INFO("Call ts_tx_start() to start sending sync beacons\r\n");
}


/**@brief Main task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
static void main_task_function (void * pvParameter)
{
    ret_code_t err_code = NRF_SUCCESS;

    log_init();

    nrf_drv_clock_lfclk_request(NULL);

    config_init(&cfgs);

    if (!isLedOnBroadcast())
    {
        led_flash_seq_init();
    }

    // Initialise the circular buffer with a fixed number of points. In the
    // case of a central device, it does not need to store any points, and
    // we wish to save memory to be used for relay buffering.
    TimedCircBuffer_Init(isCentral() ? 0 : 2500);

    nrf_gpio_cfg_output(SEN_ENABLE);
    nrf_gpio_pin_set(SEN_ENABLE); // accelerometer power off
    nrf_delay_ms(450);   // give time to reset

    if (!isTestDevice())
    {
        if (isPeripheral() || !isTestDevice())
        {
            nrf_gpio_pin_clear(SEN_ENABLE); // accelerometer power on

            // Now do the self-test
            nrf_delay_us(250);

            sensor_init();

            gpio_init();

          //  read_regs(LIS2DH_OUT_X_L);
        }
    }

    //Configure all leds on board.
    bsp_board_init(BSP_INIT_LEDS);

    hw_timers_init();

    bool erase_bonds;

    // Initialize.
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init(isCentral());
    gatt_init();
    conn_params_init();
    if (isCentral())
    {
        db_discovery_init();
      // peer_manager_init();
    }

    services_init();
    advertising_init();
    scan_init();


    if (isUseSyncTimer())
    {
        sync_timer_init();
    }

    err_code = app_timer_create(&m_sync_timer, APP_TIMER_MODE_SINGLE_SHOT, on_sync_timer_timeout);
    APP_ERROR_CHECK(err_code);

    // Start execution.
    if (erase_bonds == true)
    {
        // Scanning and advertising is done upon PM_EVT_PEERS_DELETE_SUCCEEDED event.
        delete_bonds();
    }
    else
    {
        adv_scan_start();
    }

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/** @
 * ==================  START OF FREERTOS STUFF ============================================
 */

extern void InitializeUserMemorySections(void);  // Machine code routine to initialise memory. 

TaskHandle_t  main_task_handle;   /**< Reference to main FreeRTOS task. */

int main(void)
{
    ret_code_t err_code;

    /* Set up the SDK's memory sections   */
    InitializeUserMemorySections();

    /* Run unit tests before starting any hardware stuff.  */
    TimedCircBuffer_UnitTests();

    /* Initialize clock driver for better time accuracy in FREERTOS */
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    /* Configure LED-pins as outputs */
    bsp_board_init(BSP_INIT_LEDS);

    /* At the moment is working fine with only 1 task. Add more here if needed...  */

    /* Activate deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

#ifdef FREERTOS

    // Create a FreeRTOS task for the BLE stack.
    // The task will run advertising_start() before entering its loop.
    nrf_sdh_freertos_init(main_task_function, 0);

    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

#else

    /* Not using FreeRTOS. Only 1 task is possible, and
     * it should be called directly here.  */
    main_task_function(0);

#endif

    while (true)
    {
        /* FreeRTOS should not be here... FreeRTOS goes back to the start of stack
         * in vTaskStartScheduler function. */
    }

}


/** @
 * ==================  END OF FREERTOS STUFF ==============================================
 */


/**
 * @}
 */
