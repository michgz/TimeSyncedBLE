/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**@cond To Make Doxygen skip documentation generation for this file.
 * @{
 */

#include "amt.h"
#include "nrf_error.h"
#include "sdk_common.h"
#include "app_error.h"
#include "app_fifo.h"
#include "ble_err.h"
#include "ble_srv_common.h"

#include "TimedCircBuffer.h"

#define NRF_LOG_MODULE_NAME AMTS
#define NRF_LOG_LEVEL 4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define OPCODE_LENGTH 1 /**< Length of opcode inside a notification. */
#define HANDLE_LENGTH 2 /**< Length of handle inside a notification. */


static void char_notification_send(nrf_ble_amts_t * p_ctx);
static void init_random(void);


static app_fifo_t tx_fifo;
static app_fifo_t rx_fifo;
//static uint8_t tx_fifo_buf[16384];
//static uint8_t rx_fifo_buf[256];

// Both sizes must be powers of 2.
static void fifos_init(uint32_t tx_fifo_size, uint32_t rx_fifo_size)
{
    ret_code_t err_code;

    uint8_t * fifo_buf;

    fifo_buf = malloc(tx_fifo_size);
    if (!fifo_buf)
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }
    err_code = app_fifo_init(&tx_fifo, fifo_buf, tx_fifo_size);
    APP_ERROR_CHECK(err_code);

    fifo_buf = malloc(rx_fifo_size);
    if (!fifo_buf)
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }
    err_code = app_fifo_init(&rx_fifo, fifo_buf, rx_fifo_size);
    APP_ERROR_CHECK(err_code);
}

static bool haveCurrentCmd = false;

static nrf_ble_amts_t * static_ctx = NULL;

static unsigned int rejectedBytes = 0;

unsigned int amts_get_rejected_byte_count(void)
{
    unsigned int x = rejectedBytes;
    rejectedBytes = 0;
    return x;
}

void amts_clear_rejected_byte_count(void)
{
    rejectedBytes = 0;
}

static FILL_FN currentCmdFn;

void amts_queue_tx_data(uint8_t const * p_data, unsigned int n_data)
{
    if (static_ctx)
    {
        if (!static_ctx->busy)
        {
            uint32_t n_data_cpy = n_data;
            if (NRF_SUCCESS == app_fifo_write(&tx_fifo, p_data, &n_data_cpy))
            {
                if (n_data_cpy < n_data)
                {
                    rejectedBytes += (n_data - n_data_cpy);
                }
                currentCmdFn = NULL; // illegal function pointer. Will crash if called!
                haveCurrentCmd = true;
                nrf_ble_amts_notif_spam(static_ctx);
            }
            else
            {
                rejectedBytes += n_data;
            }
        }
        else
        {
            uint32_t n_data_cpy = n_data;
            if (NRF_SUCCESS == app_fifo_write(&tx_fifo, p_data, &n_data_cpy))
            {
                if (n_data_cpy < n_data)
                {
                    rejectedBytes += (n_data - n_data_cpy);
                }
            }
            else
            {
                rejectedBytes += n_data;
            }
        }
    }

}

#define BLE_AMTS_MAX_RX_CHAR_LEN       16           /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_AMTS_MAX_TX_CHAR_LEN       -            /**< Maximum length of the TX Characteristic (in bytes). Unused */


/**@brief Function for handling the Connect event.
 *
 * @param     p_ctx       Pointer to the AMTS structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_connect(nrf_ble_amts_t * p_ctx, ble_evt_t const * p_ble_evt)
{
    // We must be in the central role
    if (p_ble_evt->evt.gap_evt.params.connected.role == BLE_GAP_ROLE_PERIPH)
    {
        p_ctx->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    }
}


/**@brief Function for handling the Disconnect event.
 *
 * @param     p_ctx         Pointer to the AMTS structure.
 * @param[in] p_ble_evt     Event received from the BLE stack.
 */
static void on_disconnect(nrf_ble_amts_t * p_ctx, ble_evt_t const * p_ble_evt)
{
    if (p_ctx->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
    {
        p_ctx->conn_handle = BLE_CONN_HANDLE_INVALID;
    }
}




/**@brief Function for handling the TX_COMPLETE event.
 *
 * @param   p_ctx   Pointer to the AMTS structure.
 */
static void on_tx_complete(nrf_ble_amts_t * p_ctx)
{
    if (p_ctx->busy)
    {
        // Get as many bytes as possible
        if (!!currentCmdFn)
        {
            (void) currentCmdFn(&tx_fifo);
        }

        p_ctx->busy = false;
        char_notification_send(p_ctx);
    }
}


/**@brief Function for handling the Write event.
 *
 * @param     p_ctx       Pointer to the AMTS structure.
 * @param[in] p_ble_evt   Event received from the BLE stack.
 */
static void on_write(nrf_ble_amts_t * p_ctx, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_ctx->amts_char_handles.cccd_handle) && (p_evt_write->len == 2))
    {
        // CCCD written, call the application event handler.
        nrf_ble_amts_evt_t evt;

        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            evt.evt_type = NRF_BLE_AMTS_EVT_NOTIF_ENABLED;
            p_ctx->notification_enabled = true;
        }
        else
        {
            evt.evt_type = NRF_BLE_AMTS_EVT_NOTIF_DISABLED;
            p_ctx->notification_enabled = false;
        }

        p_ctx->evt_handler(evt);
    }



    else if ((p_evt_write->handle == p_ctx->rx_handles.value_handle) &&
             (p_ctx->data_handler != NULL))
    {
        ret_code_t                    err_code;
        nrf_ble_amts_evt_t                 evt;
      //  ble_nus_client_context_t    * p_client;

      /*  err_code = blcm_link_ctx_get(p_nus->p_link_ctx_storage,
                                     p_ble_evt->evt.gatts_evt.conn_handle,
                                     (void *) &p_client);*/
       /* if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                          p_ble_evt->evt.gatts_evt.conn_handle);
        }*/

        memset(&evt, 0, sizeof(nrf_ble_amts_evt_t));
   //     evt.p_nus       = p_nus;
   //     evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
   //     evt.p_link_ctx  = p_client;

        evt.evt_type                  = BLE_ATMS_EVT_RX_DATA;
        evt.params.rx_data.p_data = p_evt_write->data;
        evt.params.rx_data.length = p_evt_write->len;

        p_ctx->data_handler(&evt);
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }

}


void nrf_ble_amts_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    nrf_ble_amts_t * p_ctx = (nrf_ble_amts_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            init_random();
            on_connect(p_ctx, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ctx, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_ctx, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            on_tx_complete(p_ctx);
            break;

        default:
            break;
    }
}

void StartSending(FILL_FN fn, uint32_t ctx)
{
    if (!static_ctx->busy)
    {
        currentCmdFn = fn;
        haveCurrentCmd = true;

        static_ctx->context = ctx;

         // Get as many bytes as possible
        (void) currentCmdFn(&tx_fifo);

        char_notification_send(static_ctx);
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
static void amts_data_handler(nrf_ble_amts_evt_t * p_evt)
{
    if (p_evt->evt_type == BLE_ATMS_EVT_RX_DATA)
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

            if (op[0] == 0x0E)
            {
                // A "start sending" message. Needs to be handled differently
                TimedCircBuffer_RxOperation_NoResponse(op[0], op[1]);
                StartSending(&TimedCircBuffer_FifoFill, 0);
            }
            else {(void) TimedCircBuffer_RxOperation(op[0], op[1]);}
        }
    }

}
/**@snippet [Handling the data received over BLE] */






void nrf_ble_amts_init(nrf_ble_amts_t * p_ctx, amts_evt_handler_t evt_handler, uint32_t tx_fifo_size, uint32_t rx_fifo_size)
{
    ret_code_t    err_code;
    uint16_t      service_handle;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t base_uuid = {SERVICE_UUID_BASE};

    fifos_init(tx_fifo_size, rx_fifo_size);

    static_ctx = p_ctx;

    p_ctx->context = 0UL;

    err_code = sd_ble_uuid_vs_add(&base_uuid, &(p_ctx->uuid_type));
    APP_ERROR_CHECK(err_code);

    ble_uuid.type = p_ctx->uuid_type;
    ble_uuid.uuid = AMT_SERVICE_UUID;

    // Add service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &service_handle);
    APP_ERROR_CHECK(err_code);

    // Add AMTS characteristic.
    ble_add_char_params_t amt_params;
    memset(&amt_params, 0, sizeof(amt_params));

    amt_params.uuid              = AMTS_CHAR_UUID;
    amt_params.uuid_type         = p_ctx->uuid_type;
    amt_params.max_len           = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
    amt_params.char_props.notify = 1;
    amt_params.cccd_write_access = SEC_OPEN;
    amt_params.is_var_len        = 1;

    err_code = characteristic_add(service_handle, &amt_params, &(p_ctx->amts_char_handles));
    APP_ERROR_CHECK(err_code);

    // Add AMT Received Bytes Count characteristic.
    ble_add_char_params_t amt_rbc_params;
    memset(&amt_rbc_params, 0, sizeof(amt_rbc_params));

    amt_rbc_params.uuid            = AMT_RCV_BYTES_CNT_CHAR_UUID;
    amt_rbc_params.uuid_type       = p_ctx->uuid_type;
    amt_rbc_params.max_len         = AMT_RCV_BYTES_CNT_MAX_LEN;
    amt_rbc_params.char_props.read = 1;
    amt_rbc_params.read_access     = SEC_OPEN;

    err_code = characteristic_add(service_handle, &amt_rbc_params, &(p_ctx->amt_rbc_char_handles));
    APP_ERROR_CHECK(err_code);

    p_ctx->evt_handler = evt_handler;

    ble_add_char_params_t add_char_params;

    // Add the RX Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = BLE_UUID_NUS_RX_CHARACTERISTIC;  // Use the standard UUID value with a vendor-specific UUID type
    add_char_params.uuid_type                = p_ctx->uuid_type;
    add_char_params.max_len                  = BLE_AMTS_MAX_RX_CHAR_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.write         = 1;
    add_char_params.char_props.write_wo_resp = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;

    err_code = characteristic_add(service_handle, &add_char_params, &p_ctx->rx_handles);
    if (err_code != NRF_SUCCESS)
    {
        return ;//err_code;
    }

    p_ctx->data_handler = amts_data_handler;

    return ;//NRF_SUCCESS;
}


void nrf_ble_amts_notif_spam(nrf_ble_amts_t * p_ctx)
{
    p_ctx->kbytes_sent = 0;
    p_ctx->bytes_sent  = 0;
    char_notification_send(p_ctx);
}


void nrf_ble_amts_on_gatt_evt(nrf_ble_amts_t * p_ctx, nrf_ble_gatt_evt_t const * p_gatt_evt)
{
    if (p_ctx->conn_handle == p_gatt_evt->conn_handle && p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        p_ctx->max_payload_len =
            p_gatt_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    }
}

static void init_random(void)
{
    // Seed the random number generator
    srand(0);
}

static void fill_randomly(char * buf, const int len)
{
    int i;
    for (i = 0; i < len; i ++)
    {
        buf[i] = (char)rand();
    }
}

static void finished(nrf_ble_amts_t * p_ctx)
{
    nrf_ble_amts_evt_t evt;

    haveCurrentCmd = false;
    currentCmdFn = NULL;

    evt.bytes_transfered_cnt = p_ctx->bytes_sent;
    evt.params.transfer_finished.context = p_ctx->context;
    evt.evt_type             = NRF_BLE_AMTS_EVT_TRANSFER_FINISHED;

    p_ctx->context = 0UL;

    p_ctx->evt_handler(evt);

    p_ctx->busy        = false;
    p_ctx->bytes_sent  = 0;
    p_ctx->kbytes_sent = 0;
}



static volatile uint32_t used_1 = 0;
static volatile uint32_t used_2 = 0;
static volatile uint32_t used_3 = 0;



static void char_notification_send(nrf_ble_amts_t * p_ctx)
{
    static uint8_t     data[256];
    static unsigned int waiting_len = 0;
    uint16_t           payload_len = p_ctx->max_payload_len;
    nrf_ble_amts_evt_t evt;

    if (p_ctx->conn_handle == BLE_CONN_HANDLE_INVALID
        || !p_ctx->notification_enabled)
    {
        waiting_len = 0;
        finished(p_ctx);
        return;
    }

    ble_gatts_hvx_params_t const hvx_param =
    {
        .type   = BLE_GATT_HVX_NOTIFICATION,
        .handle = p_ctx->amts_char_handles.value_handle,
        .p_data = data,
        .p_len  = &payload_len,
    };

    uint32_t err_code = NRF_SUCCESS;
    while (err_code == NRF_SUCCESS)
    {
        uint32_t available;

        if (waiting_len > 0)
        {
            available = waiting_len;
            payload_len = available;
        }
        else
        {
            app_fifo_read(&tx_fifo, NULL, &available);

            if (available == 0)
            {
                // We've run out of data. Get some more.
                if (!!currentCmdFn)
                {
                    (void) currentCmdFn(&tx_fifo);
                    app_fifo_read(&tx_fifo, NULL, &available);
                }
                if (available == 0)
                {
                    // Have now genuinely run out of data. Finish the transmission
                    finished(p_ctx);
                    return;
                }
            }

            if (payload_len > available)
            {
                payload_len = available;
            }

            available = payload_len;
            app_fifo_read(&tx_fifo, data, &available);

            used_1 += available;

        }

        err_code = sd_ble_gatts_hvx(p_ctx->conn_handle, &hvx_param);

        if (err_code == NRF_ERROR_RESOURCES)
        {
            // Wait for BLE_GATTS_EVT_HVN_TX_COMPLETE.
            p_ctx->busy = true;
            waiting_len = available;
            break;
        }
        else if (err_code != NRF_SUCCESS)
        {
            used_3 += 1;
            NRF_LOG_ERROR("sd_ble_gatts_hvx() failed: 0x%x", err_code);
        }
        else
        {
            used_2 += payload_len;
        }

        waiting_len = 0;

        p_ctx->bytes_sent += payload_len;

        if (p_ctx->kbytes_sent != (p_ctx->bytes_sent / 1024))
        {
            p_ctx->kbytes_sent = (p_ctx->bytes_sent / 1024);

            evt.evt_type             = NRF_BLE_AMTS_EVT_TRANSFER_1KB;
            evt.bytes_transfered_cnt = p_ctx->bytes_sent;
            p_ctx->evt_handler(evt);
        }
    }
}


void nrf_ble_amts_rbc_set(nrf_ble_amts_t * p_ctx, uint32_t byte_cnt)
{
    uint8_t  data[AMT_RCV_BYTES_CNT_MAX_LEN];
    uint16_t len;

    ble_gatts_value_t value_param;

    memset(&value_param, 0x00, sizeof(value_param));

    len                 = (uint16_t)uint32_encode(byte_cnt, data);
    value_param.len     = len;
    value_param.p_value = data;

    ret_code_t err_code = sd_ble_gatts_value_set(p_ctx->conn_handle,
                                                 p_ctx->amt_rbc_char_handles.value_handle,
                                                 &value_param);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("sd_ble_gatts_value_set() failed: 0x%x", err_code);
    }
}

/** @}
 *  @endcond
 */
