/**
 * Maintains the result of the scan
 */

#include "../Services/ScanList.h"
#include <string.h>
#include "app_error.h"


// ================  LEAF LIST STRUCTURE ==========================================
#define LEAF_LIST_N    (5)
typedef struct LEAF_tag
{
    ble_gap_addr_t          gap_addr;
    int8_t                  rssi;
    ble_gap_scan_params_t   scan_params;

} LEAF_T;

static LEAF_T  leaf_list[LEAF_LIST_N];
static unsigned int  leaf_list_ptr = 0U;

static void add_leaf_list(ble_gap_addr_t * gap_addr, ble_gap_scan_params_t const * scan_params, int8_t rssi)
{
    // Check if already in the list
    int i;
    for (i = 0; i < leaf_list_ptr; i ++)
    {
        if (0 == memcmp(&leaf_list[i], gap_addr, sizeof(ble_gap_addr_t)))
        {
            // Match! Can exit, since nothing more to do.
            return;
        }
    }

    // Not found. Add it.
    if (leaf_list_ptr < LEAF_LIST_N)
    {
        memcpy(&leaf_list[leaf_list_ptr].gap_addr, gap_addr, sizeof(ble_gap_addr_t));
        memcpy(&leaf_list[leaf_list_ptr].scan_params, scan_params, sizeof(ble_gap_scan_params_t));
        leaf_list[leaf_list_ptr].rssi = rssi;
        leaf_list_ptr ++;
    }
}
// ======================================================================================

int LeafListCount(void) {return leaf_list_ptr;}


void on_scan_list_scan_evt(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
        {
            ble_gap_evt_adv_report_t const * p_adv = 
                           p_scan_evt->params.filter_match.p_adv_report;
            ble_gap_scan_params_t    const * p_scan_param = 
                           p_scan_evt->p_scan_params;

            add_leaf_list((ble_gap_addr_t *) &p_adv->peer_addr, p_scan_param, p_adv->rssi);

        } break;
    }


}

void ClearLeafList (void)
{
    leaf_list_ptr = 0U;
}

bool ConnectLeaf(int index, nrf_ble_scan_t const * p_scan)
{
    // Connect to next unit
    if ((leaf_list_ptr > 0) && (index < leaf_list_ptr))
    {
        ret_code_t err_code;

        err_code = sd_ble_gap_connect(&leaf_list[index].gap_addr, &leaf_list[index].scan_params, &p_scan->conn_params, p_scan->conn_cfg_tag);
        APP_ERROR_CHECK(err_code);

        return true;
    }
    return false;
}

static int read_ptr = 0;

void ScanListClearReading(void)
{
    read_ptr = 0;
}

bool ScanList_FifoFill(app_fifo_t * const p_fifo)
{
    if (!p_fifo)
    {
        APP_ERROR_CHECK(NRF_ERROR_NULL);
    }

    if (read_ptr > leaf_list_ptr)
    {
        return false;  // Finished
    }

    uint32_t available;
    app_fifo_write(p_fifo, NULL, &available);

    while(available >= 8)
    {
        uint32_t len;
        if (read_ptr == 0)
        {
            // First one
            uint32_t read_out [2] = {0x1A, leaf_list_ptr};
            len = 8;
            app_fifo_write(p_fifo, (uint8_t * const)read_out, &len);
            available -= len;
            read_ptr ++;
        }
        else
        {
            if (read_ptr > leaf_list_ptr)
            {
                return true;  // Finished
            }

            uint8_t tmp [8];
            memset(tmp, 0, 8);
            memcpy(&tmp[0], (uint8_t * const)&leaf_list[read_ptr-1].gap_addr.addr, BLE_GAP_ADDR_LEN);
            tmp[7] = (uint8_t) leaf_list[read_ptr-1].rssi;

            len = 8;
            app_fifo_write(p_fifo, (uint8_t * const)&leaf_list[read_ptr-1].gap_addr, &len);
            available -= len;

            read_ptr ++;
        }
    }
    return true;
}
