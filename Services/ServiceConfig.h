/**
 * A BLE GATT service that allows configuring the unit
 */

#ifndef __H_SERVICE_CONFIG
#define __H_SERVICE_CONFIG

#include <stdbool.h>
#include <stdint.h>

#include "ble.h"
#include "ble_srv_common.h"
#include "app_error.h"

#ifdef __cplusplus
extern "C" {
#endif


/**@brief Config Service event types. */
typedef enum
{
    BLE_QWRS_CHECK_RCVD_DATA,                        /* On this event, the application shall only verify if the data are correct.*/
    BLE_QWRS_NEW_DATA_RCVD,                          /* On this event, the application can act upon the received data*/
} config_service_evt_type;


/**@brief Config Service event. */
typedef struct
{
    config_service_evt_type evt_type;                        //!< Type of event.
    //uint16_t              rcv_length;
    //uint8_t               rcvd_data[NRF_BLE_QWRS_MAX_RCV_SIZE];
} config_service_evt_t;


// Forward declaration of the config_service_t type.
struct config_service_t;

/**@brief Config Service event handler type. returns a BLE_GATT_STATUS_CODES */
typedef uint16_t (*config_service_evt_handler_t) (config_service_evt_t    * p_evt);


typedef struct
{
    config_service_evt_handler_t   evt_handler;       //!< Event handler to be called for handling events in the Queued Write Example  Service.
    ble_srv_error_handler_t      error_handler;     //!< Function to be called in case of an error.
    struct config_service_t              * p_cfgs_ctx;         //!< pointer to the initialized service context
} service_config_init_t;




typedef struct config_service_t
{
    uint8_t                    uuid_type;               //!< UUID type.
    uint16_t                   service_handle;          //!< Handle of config Service (as provided by the BLE stack).
    uint16_t                   conn_handle;             //!< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).
    config_service_evt_handler_t evt_handler;             //!< Event handler to be called for handling events in the config Service.
    ble_srv_error_handler_t    error_handler;           //!< Function to be called in case of an error.
    ble_gatts_char_handles_t   value_1_handles;    //!< Handles related to the configuration value characteristic.
    ble_gatts_char_handles_t   value_2_handles;    //!< Handles related to the configuration value characteristic.
    ble_gatts_char_handles_t   value_3_handles;    //!< Handles related to the configuration value characteristic.

    // Now the current values of the config items.
    uint8_t     value_1;
    uint32_t    value_2;
    uint8_t     value_3;

} config_service_t;

#define CONFIG_3_SIMPLE_TRIGGER_MASK 0x02
#define CONFIG_3_UPLOAD_LEAF_LIST_MASK 0x08

extern uint16_t service_cfg_on_ble_evt(config_service_t *p_cfgs, ble_evt_t const * p_evt);

extern ret_code_t service_config_init(service_config_init_t * p_cfgs_init, config_service_t * p_ctx);

// Set up the storage of the config. May be called before the SoftDevice is started.
extern void config_init(config_service_t *);

#ifdef __cplusplus
}
#endif

#endif /* __H_SERVICE_CONFIG */
