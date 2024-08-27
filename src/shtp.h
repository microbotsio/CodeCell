/*
 * Copyright 2015-18 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Hillcrest Sensor Hub Transport Protocol (SHTP) API
 */

#ifndef SHTP_H
#define SHTP_H

#include <stdint.h>
#include <stdbool.h>

#include "sh2_hal.h"

// Advertisement TLV tags
#define TAG_NULL 0
#define TAG_GUID 1
#define TAG_MAX_CARGO_PLUS_HEADER_WRITE 2
#define TAG_MAX_CARGO_PLUS_HEADER_READ 3
#define TAG_MAX_TRANSFER_WRITE 4
#define TAG_MAX_TRANSFER_READ 5
#define TAG_NORMAL_CHANNEL 6
#define TAG_WAKE_CHANNEL 7
#define TAG_APP_NAME 8
#define TAG_CHANNEL_NAME 9
#define TAG_ADV_COUNT 10
#define TAG_APP_SPECIFIC 0x80

typedef enum shtp_Event_e {
    SHTP_TX_DISCARD = 0,
    SHTP_SHORT_FRAGMENT = 1,
    SHTP_TOO_LARGE_PAYLOADS = 2,
    SHTP_BAD_RX_CHAN = 3,
    SHTP_BAD_TX_CHAN = 4,
} shtp_Event_t;

typedef void shtp_Callback_t(void * cookie, uint8_t *payload, uint16_t len, uint32_t timestamp);
typedef void shtp_AdvertCallback_t(void * cookie, uint8_t tag, uint8_t len, uint8_t *value);
typedef void shtp_SendCallback_t(void *cookie);
typedef void shtp_EventCallback_t(void *cookie, shtp_Event_t shtpEvent);

// Takes HAL pointer, returns shtp ID for use in future calls.
// HAL will be opened by this call.
void * shtp_open(sh2_Hal_t *pHal);

// Releases resources associated with this SHTP instance.
// HAL will not be closed.
void shtp_close(void *pShtp);

// Provide the point of the callback function for reporting SHTP asynchronous events
void shtp_setEventCallback(void *pInstance,
                           shtp_EventCallback_t * eventCallback, 
                           void *eventCookie);

// Register a listener for an SHTP channel
int shtp_listenChan(void *pShtp,
                    uint16_t guid, const char * chan,
                    shtp_Callback_t *callback, void * cookie);

// Register a listener for SHTP advertisements 
int shtp_listenAdvert(void *pShtp,
                      uint16_t guid,
                      shtp_AdvertCallback_t *advertCallback, void * cookie);

// Look up the channel number for a particular app, channel.
uint8_t shtp_chanNo(void *pShtp,
                    const char * appName, const char * chanName);

// Send an SHTP payload on a particular channel
int shtp_send(void *pShtp,
              uint8_t channel, const uint8_t *payload, uint16_t len);

// Check for received data and process it.
void shtp_service(void *pShtp);

// #ifdef SHTP_H
#endif
