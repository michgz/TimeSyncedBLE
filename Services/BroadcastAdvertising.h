/**
 * Handles the central unit switching from normal advertising to broadcast
 */

#ifndef __H_BROADCAST_ADVERTISING
#define __H_BROADCAST_ADVERTISING

#include <stdbool.h>
#include <stdint.h>

#include "nrfx.h"
#include "ble_advertising.h"

enum BroadcastTypes
{
    BroadcastType_Invalid = 0x00,
    BroadcastType_RequestLock = 0x01,
};

typedef struct __attribute(( packed )) BroadcastData_tag
{
    uint8_t data [4];
    uint8_t type;
    uint8_t unused_padding;
    
} BroadcastData_T;

#endif /* __H_BROADCAST_ADVERTISING */
