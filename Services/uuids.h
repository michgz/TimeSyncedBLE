#ifndef UUIDS_H__
#define UUIDS_H__

#define  SYNC_APP_UUID_BASE   {0x5c, 0xcc, 0x51, 0x68, 0x2c, 0x5d, 0x6b, 0xb6, 0xa5, 0x4a, 0x40, 0x47, 0x0e, 0x95, 0x17, 0x21}


#define AMT_SERVICE_UUID                    0x1523
#define AMTS_CHAR_UUID                      0x1524
#define AMT_RCV_BYTES_CNT_CHAR_UUID         0x1525   // unused -- should remove


// Debug service characteristics
#define BLE_UUID_DEBUG_SERVICE     0x2000
#define BLE_UUID_DEBUG_WORD_CHARACTERISTIC 0x2001

// Config service characteristics
#define BLE_UUID_CONFIG_SERVICE         0x000B
#define BLE_UUID_CONFIG_CHARACTERISTIC_1    0x000C
#define BLE_UUID_CONFIG_CHARACTERISTIC_2    0x000D
#define BLE_UUID_CONFIG_CHARACTERISTIC_3    0x000E

// AMTS characteristics
#define BLE_UUID_NUS_TX_CHARACTERISTIC 0x0003               /**< The UUID of the TX Characteristic. */
#define BLE_UUID_NUS_RX_CHARACTERISTIC 0x0002               /**< The UUID of the RX Characteristic. */

// Public IDs
#define MY_MANUFACTURER_ID       0xCAFE    // Must register with BT-SIG to get a proper ID

#endif // UUIDS_H__
