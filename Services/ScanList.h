/**
 * Maintains the result of the scan
 */

#ifndef __H_SCAN_LIST
#define __H_SCAN_LIST

#include <stdbool.h>
#include <stdint.h>

#include "ble.h"
#include "nrf_ble_scan.h"
#include "app_fifo.h"

extern void on_scan_list_scan_evt(scan_evt_t const * p_scan_evt);

extern int LeafListCount(void);

extern void ClearLeafList (void);

extern bool ConnectLeaf(int index, nrf_ble_scan_t const * p_scan);

extern void ScanListClearReading(void);

extern bool ScanList_FifoFill(app_fifo_t * const);


#endif /* __H_SCAN_LIST */
