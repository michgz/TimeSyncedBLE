/**
 * A definition of the Nordic UART service
 */

#ifndef __H_SERVICE_NORDIC_UART
#define __H_SERVICE_NORDIC_UART

#include <stdbool.h>
#include <stdint.h>

#include "ble.h"


#ifdef CENTRAL
#include "ble_nus_c.h"
#endif
#include "nrf_ble_gq.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#if defined PERIPHERAL || defined CENTRAL
#include "ble_nus.h"
#endif
#include "ble_db_discovery.h"

extern void nus_init(void);
extern void nus_c_init(nrf_ble_gq_t * gatt_queue);
extern void nus_queue_tx_data(uint8_t const * p_data, unsigned int n_data);

extern void nus_c_db_disc_handler(ble_db_discovery_evt_t * p_evt);

#endif /* __H_SERVICE_NORDIC_UART */
