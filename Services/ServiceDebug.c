/**
 * A BLE GATT service that returns debug information from the unit
 */

#include "ServiceDebug.h"
#include "ble.h"
#include "sdk_common.h"
#include "app_error.h"
#include "time_sync.h"
#include "../Services/uuids.h"

#include <string.h>

// Use the same base UUID as for the NUS service.
#define NUS_BASE_UUID                 SYNC_APP_UUID_BASE


static uint32_t the_value [4];

uint32_t ble_debug_service_init(ble_dbg_t * p_dbg)
{
    uint32_t            err_code;
    ble_uuid128_t       nus_base_uuid = {NUS_BASE_UUID};
    ble_uuid_t          ble_uuid;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_value;
    ble_uuid_t          char_uuid;
    ble_gatts_attr_md_t attr_md;


    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&nus_base_uuid, &p_dbg->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_dbg->uuid_type;
    ble_uuid.uuid = BLE_UUID_DEBUG_SERVICE;

    // Add service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_dbg->service_handle);

    VERIFY_SUCCESS(err_code);

    // Add debug value characteristic
    memset(&char_md, 0, sizeof(ble_gatts_char_md_t));

    char_md.char_props.read = 1;

    memset(&attr_value, 0, sizeof(ble_gatts_attr_t));

    char_uuid.type = p_dbg->uuid_type;
    char_uuid.uuid = BLE_UUID_DEBUG_WORD_CHARACTERISTIC;

    attr_value.p_uuid = &char_uuid;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

    attr_md.vlen = 0;
    attr_md.vloc = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth = 1;
    attr_md.wr_auth = 0;

    attr_value.p_attr_md = &attr_md;
    attr_value.init_len = sizeof(the_value);
    attr_value.init_offs = 0;
    attr_value.max_len = sizeof(the_value);

    memset(the_value, 0, sizeof(the_value));

    attr_value.p_value = (uint8_t *)the_value;

    err_code = sd_ble_gatts_characteristic_add(p_dbg->service_handle, &char_md, &attr_value, &p_dbg->tx_handles);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

void ts_timestamps_get_all(uint32_t x[4], uint8_t ppi_chn);


void ble_dbg_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_dbg_t * p_dbg = (ble_dbg_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            {
            ble_gatts_evt_rw_authorize_request_t const *  p_auth = &p_ble_evt->evt.gatts_evt.params.authorize_request;
            if (p_auth->request.read.handle == p_dbg->tx_handles.value_handle)
            {
                ble_gatts_rw_authorize_reply_params_t   rw_auth;

                memset(&rw_auth, 0, sizeof(ble_gatts_rw_authorize_reply_params_t));

                rw_auth.type = p_auth->type;
                if (p_auth->type == BLE_GATTS_AUTHORIZE_TYPE_READ)
                {
                    rw_auth.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;
                    rw_auth.params.read.update = 1;
                    rw_auth.params.read.offset = 0;
                    rw_auth.params.read.len = sizeof(the_value);

                    ts_timestamps_get_all(the_value, 3);  // Note: PPI channel 4 doesn't work!! 1,2,3 are probably all okay.

                    rw_auth.params.read.p_data = (uint8_t *)the_value;
                }
                else
                {
                    rw_auth.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_WRITE_NOT_PERMITTED;
                }
                APP_ERROR_CHECK(sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle, &rw_auth));
            }
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}
