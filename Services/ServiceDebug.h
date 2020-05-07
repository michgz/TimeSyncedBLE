/**
 * A BLE GATT service that returns debug information from the unit
 */

#ifndef __H_SERVICE_DEBUG
#define __H_SERVICE_DEBUG

#include <stdbool.h>
#include <stdint.h>

#include "ble.h"
#include "../Services/uuids.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a debug service instance.
 *
 * @param     _name            Name of the instance.
 * @hideinitializer
 */
#define BLE_DBG_DEF(_name)                                        \
    static ble_dbg_t _name;                                       \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,                           \
                         BLE_DBG_BLE_OBSERVER_PRIO,               \
                         ble_dbg_on_ble_evt,                      \
                         &_name)

#define BLE_DBG_BLE_OBSERVER_PRIO  BLE_NUS_BLE_OBSERVER_PRIO


typedef struct ble_dbg_s
{
    uint8_t                         uuid_type;          /**< UUID type for Nordic UART Service Base UUID. */
    uint16_t                        service_handle;     /**< Handle of Nordic UART Service (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        tx_handles;         /**< Handles related to the TX characteristic (as provided by the SoftDevice). */
} ble_dbg_t;

extern uint32_t ble_debug_service_init(ble_dbg_t *);

/**@brief   Function for handling the Service's BLE events.
 *
 * @details The Nordic UART Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the Service event handler of the
 * application if necessary.
 *
 * @param[in] p_ble_evt     Event received from the SoftDevice.
 * @param[in] p_context     Service structure.
 */
void ble_dbg_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

#ifdef __cplusplus
}
#endif

#endif /* __H_SERVICE_DEBUG */
