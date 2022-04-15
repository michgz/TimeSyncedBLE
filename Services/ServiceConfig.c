/**
 * A BLE GATT service that allows configuring the unit
 */

#include "ServiceConfig.h"
#include "ble.h"
#include "sdk_common.h"
#include "app_error.h"
#include "nrf_ble_qwr.h"
#include "nrf_sdh_ble.h"
#include "nrf_nvic.h"
#include "../Services/uuids.h"

#include "fds.h"

#include <string.h>
#include <stdbool.h>

// Use the same base UUID as for the NUS service.
#define NUS_BASE_UUID                  SYNC_APP_UUID_BASE


NRF_BLE_QWR_DEF(m_qwr_cfg);                                                        /**< Context for the Queued Write module.*/

typedef unsigned char  config_value_t;

static bool is_connected = false;

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static bool reset_on_disconnect = false;

#define FILE_ID   0x8888
#define REC_KEY_1   0x9999
#define REC_KEY_2   0x999A

uint16_t service_cfg_on_ble_evt(config_service_t *p_cfgs,
                                 ble_evt_t const * p_ble_evt)
{
    if (p_cfgs->evt_handler != NULL)
    {
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            {
                ret_code_t err_code;
                ble_gatts_evt_write_t const * p_write = &p_ble_evt->evt.gatts_evt.params.write;

                if (p_write->handle == p_cfgs->value_1_handles.value_handle && p_write->len >= CONFIG_1_LEN && p_write->op != BLE_GATTS_OP_INVALID)
                {
                    static uint32_t new_value = 0U; // Static because fds_...() functions are queued
                    ((uint32_t *)&new_value)[0] = p_write->data[0];

                    if ((uint8_t)new_value != p_cfgs->value_1)
                    {
                        p_cfgs->value_1 = (uint8_t)new_value;

                        fds_record_desc_t desc = {0};
                        fds_find_token_t  tok = {0};

                        fds_record_t new_rec;

                        new_rec.file_id = FILE_ID;
                        new_rec.key = REC_KEY_1;
                        new_rec.data.length_words = 1;
                        new_rec.data.p_data = &new_value;

                        err_code = fds_record_find_by_key(REC_KEY_1, &desc, &tok);
                        if (err_code == FDS_ERR_NOT_FOUND)
                        {
                            // Does not exist -- write new
                            err_code = fds_record_write(&desc, &new_rec);
                        }
                        else if (err_code == NRF_SUCCESS)
                        {
                            // Does exist -- update
                            err_code = fds_record_update(&desc, &new_rec);
                        }
                        else
                        {
                            // Error
                        }
                        APP_ERROR_CHECK(err_code);
                    }
                }
                else if (p_write->handle == p_cfgs->value_2_handles.value_handle && p_write->len >= CONFIG_2_LEN && p_write->op != BLE_GATTS_OP_INVALID)
                {
                    static uint32_t new_value = 0U;   // Static because fds_...() functions are queued.
                    ((uint32_t *)&new_value)[0] = p_write->data[0];

                    if (new_value != p_cfgs->value_2)
                    {
                        p_cfgs->value_2 = new_value;

                        fds_record_desc_t desc = {0};
                        fds_find_token_t  tok = {0};

                        fds_record_t new_rec;

                        new_rec.file_id = FILE_ID;
                        new_rec.key = REC_KEY_2;
                        new_rec.data.length_words = 1;
                        new_rec.data.p_data = &new_value;

                        err_code = fds_record_find_by_key(REC_KEY_2, &desc, &tok);
                        if (err_code == FDS_ERR_NOT_FOUND)
                        {
                            // Does not exist -- write new
                            err_code = fds_record_write(&desc, &new_rec);
                        }
                        else if (err_code == NRF_SUCCESS)
                        {
                            // Does exist -- update
                            err_code = fds_record_update(&desc, &new_rec);
                        }
                        else
                        {
                            // Error
                        }
                        APP_ERROR_CHECK(err_code);
                    }
                }

            }
            break;
        case BLE_GAP_EVT_CONNECTED:
            // We'll reset on the first disconnection after a non-volatile value being changed.
            
            is_connected = true;
            
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            // We'll reset on the first disconnection after a non-volatile value being changed.
            
            is_connected = false;
            
            if (reset_on_disconnect)
            {
                reset_on_disconnect = false;

                (void) sd_nvic_SystemReset();
                while(true);
            }
            break;

        default:
            ;
    }

    return BLE_GATT_STATUS_SUCCESS;
}

volatile static uint8_t ready = 0;

void fds_handler(fds_evt_t const * p_evt)
{
    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            reset_on_disconnect = false;
            ready = 1;
            break;
        case FDS_EVT_WRITE:
        case FDS_EVT_UPDATE:
            {
                if (p_evt->result == NRF_SUCCESS)
                {
                    // A write or update has completed. Now reset to run with the new value.

                    if (is_connected)
                    {
                        reset_on_disconnect = true;
                    }
                    else
                    {
                        (void) sd_nvic_SystemReset();
                        while(true);
                    }

                }
            }
            break;
        default:
            ;
    }
}

// Initialise the FDS-located config. Does not need SD running
void config_init(config_service_t * p_ctx)
{
    ret_code_t         err_code;

    is_connected = false;

    if (!p_ctx)
    {
        APP_ERROR_CHECK(NRF_ERROR_NULL);
    }

    // Initialise FDS
    err_code = fds_register(fds_handler);
    APP_ERROR_CHECK(err_code);

    ready = 0;

    err_code = fds_init();
    APP_ERROR_CHECK(err_code);

    uint32_t new_value_1 = 0;
    uint32_t new_value_2 = 0;

    while(!ready) ;

    // Read value_1
    fds_record_desc_t desc = {0};
    fds_find_token_t  tok = {0};
    err_code = fds_record_find_by_key(REC_KEY_1, &desc, &tok);

    if (err_code == NRF_SUCCESS)
    {
        fds_flash_record_t rec;

        err_code = fds_record_open(&desc, &rec);
        APP_ERROR_CHECK(err_code);

        if (rec.p_header)
        {
            if (rec.p_header->length_words >= 1)
            {
                if (rec.p_data)
                {
                    new_value_1 = ((uint32_t const *) rec.p_data)[0];
                }
            }
        }

        err_code = fds_record_close(&desc);
        APP_ERROR_CHECK(err_code);
    }
    else if (err_code == FDS_ERR_NOT_FOUND)
    {
        new_value_1 = CONFIG_1_DEFAULT;
    }
    else
    {
        APP_ERROR_CHECK(err_code);
    }


    // Now read value_2
    memset((void *) &desc, 0, sizeof(desc));
    memset((void *) &tok, 0, sizeof(tok));
    err_code = fds_record_find_by_key(REC_KEY_2, &desc, &tok);

    if (err_code == NRF_SUCCESS)
    {
        fds_flash_record_t rec;

        err_code = fds_record_open(&desc, &rec);
        APP_ERROR_CHECK(err_code);

        if (rec.p_header)
        {
            if (rec.p_header->length_words >= 1)
            {
                if (rec.p_data)
                {
                    new_value_2 = ((uint32_t const *) rec.p_data)[0];
                }
            }
        }

        err_code = fds_record_close(&desc);
        APP_ERROR_CHECK(err_code);
    }
    else if (err_code == FDS_ERR_NOT_FOUND)
    {
        new_value_2 = CONFIG_2_DEFAULT;
    }
    else
    {
        APP_ERROR_CHECK(err_code);
    }


    p_ctx->value_1 = (uint8_t)new_value_1;
    p_ctx->value_2 = new_value_2;
    p_ctx->value_3 = CONFIG_3_DEFAULT;

}


ret_code_t service_config_init(service_config_init_t * p_cfgs_init, config_service_t * p_ctx)
{
    ret_code_t         err_code;

    if (!p_ctx)
    {
        APP_ERROR_CHECK(NRF_ERROR_NULL);
    }

    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr_cfg, &qwr_init);
    APP_ERROR_CHECK(err_code);



    ble_uuid_t    ble_uuid;

    // Initialize service structure.
    p_ctx->evt_handler   = p_cfgs_init->evt_handler;
    p_ctx->error_handler = p_cfgs_init->error_handler;
    p_ctx->conn_handle   = BLE_CONN_HANDLE_INVALID;

    // Add service.
    ble_uuid128_t base_uuid = {SYNC_APP_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_ctx->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_ctx->uuid_type;
    ble_uuid.uuid = BLE_UUID_CONFIG_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_ctx->service_handle);
    VERIFY_SUCCESS(err_code);

    //Add characteristic value_1
    ble_add_char_params_t add_char_params;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid             = BLE_UUID_CONFIG_CHARACTERISTIC_1;
    add_char_params.uuid_type        = p_ctx->uuid_type;
    add_char_params.max_len          = sizeof(config_value_t);
    add_char_params.init_len         = sizeof(config_value_t);
    add_char_params.char_props.write = true;
    add_char_params.char_props.read  = true;
    add_char_params.read_access      = SEC_OPEN;    
    add_char_params.write_access     = SEC_OPEN;
    add_char_params.p_init_value     = (void *) &p_ctx->value_1;

    err_code = characteristic_add(p_ctx->service_handle,
                              &add_char_params,
                              &p_ctx->value_1_handles);
    VERIFY_SUCCESS(err_code);

    //Add characteristic value_2
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid             = BLE_UUID_CONFIG_CHARACTERISTIC_2;
    add_char_params.uuid_type        = p_ctx->uuid_type;
    add_char_params.max_len          = sizeof(p_ctx->value_2);
    add_char_params.init_len         = sizeof(p_ctx->value_2);
    add_char_params.char_props.write = true;
    add_char_params.char_props.read  = true;
    add_char_params.read_access      = SEC_OPEN;    
    add_char_params.write_access     = SEC_OPEN;
    add_char_params.p_init_value     = (void *) &p_ctx->value_2;

    err_code = characteristic_add(p_ctx->service_handle,
                              &add_char_params,
                              &p_ctx->value_2_handles);
    VERIFY_SUCCESS(err_code);

    //Add characteristic value_3 (the mode parameter)
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid             = BLE_UUID_CONFIG_CHARACTERISTIC_3;
    add_char_params.uuid_type        = p_ctx->uuid_type;
    add_char_params.max_len          = sizeof(p_ctx->value_3);
    add_char_params.init_len         = sizeof(p_ctx->value_3);
    add_char_params.char_props.write = true;
    add_char_params.char_props.read  = true;
    add_char_params.read_access      = SEC_OPEN;    
    add_char_params.write_access     = SEC_OPEN;
    add_char_params.p_init_value     = (void *) &p_ctx->value_3;

    err_code = characteristic_add(p_ctx->service_handle,
                              &add_char_params,
                              &p_ctx->value_3_handles);
    VERIFY_SUCCESS(err_code);

    if (p_cfgs_init->p_cfgs_ctx != NULL)
    {
        //err_code = nrf_ble_qwr_attr_register(p_cfgs_init->p_cfgs_ctx,
        //                                     p_ctx->value_handles.value_handle);
        //VERIFY_SUCCESS(err_code);
    }
    return err_code;

}
