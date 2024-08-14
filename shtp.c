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

#include "shtp.h"
#include "sh2_err.h"
#include "sh2_util.h"

#include <string.h>

// ------------------------------------------------------------------------
// Private types

#define SH2_MAX_APPS (5)
#define SHTP_APP_NAME_LEN (32)
#define SH2_MAX_CHANS (8)
#define SHTP_CHAN_NAME_LEN (32)

// Defined Globally Unique Identifiers
#define GUID_SHTP (0)

// Command Channel commands and parameters
#define SHTP_CHAN_COMMAND (0)
#define CMD_ADVERTISE (0)
#define CMD_ADVERTISE_SHTP (0)
#define CMD_ADVERTISE_ALL (1)
#define RESP_ADVERTISE (0)

#define SHTP_HDR_LEN (4)

#define TAG_SHTP_VERSION 0x80

typedef struct shtp_App_s {
    uint32_t guid;
    char appName[SHTP_APP_NAME_LEN];
} shtp_App_t;

typedef struct shtp_AppListener_s {
    uint16_t guid;
    shtp_AdvertCallback_t *callback;
    void *cookie;
} shtp_AppListener_t;

typedef struct shtp_ChanListener_s {
    uint16_t guid;
    char chanName[SHTP_CHAN_NAME_LEN];
    shtp_Callback_t *callback;
    void *cookie;
} shtp_ChanListener_t;

typedef struct shtp_Channel_s {
    uint8_t nextOutSeq;
    uint8_t nextInSeq;
    uint32_t guid;  // app id
    char chanName[SHTP_CHAN_NAME_LEN];
    bool wake;
    shtp_Callback_t *callback;
    void *cookie;
} shtp_Channel_t;

typedef enum {
    ADVERT_NEEDED,
    ADVERT_REQUESTED,
    ADVERT_IDLE,
} advert_phase_t;

// Per-instance data for SHTP
typedef struct shtp_s {
    // Associated SHTP HAL
    // If 0, this indicates the SHTP instance is available for new opens
    sh2_Hal_t *pHal;

    // Asynchronous Event callback and it's cookie
    shtp_EventCallback_t *eventCallback;
    void * eventCookie;

    // Data from adverts
    char shtpVersion[8];
    uint16_t outMaxPayload;
    uint16_t outMaxTransfer;

    // Transmit support
    uint8_t outTransfer[SH2_HAL_MAX_TRANSFER_OUT];

    // Receive support
    uint16_t inMaxTransfer;
    uint16_t inRemaining;
    uint8_t  inChan;
    uint8_t  inPayload[SH2_HAL_MAX_PAYLOAD_IN];
    uint16_t inCursor;
    uint32_t inTimestamp;
    uint8_t inTransfer[SH2_HAL_MAX_TRANSFER_IN];
    
    // What stage of advertisement processing are we in.
    advert_phase_t advertPhase;
    
    // Applications
    shtp_App_t app[SH2_MAX_APPS];
    uint8_t    nextApp;
    
    // Advert registrations
    uint8_t nextAppListener;
    shtp_AppListener_t appListener[SH2_MAX_APPS];

    // SHTP Channels
    shtp_Channel_t      chan[SH2_MAX_CHANS];

    // Channel listeners
    shtp_ChanListener_t chanListener[SH2_MAX_CHANS];
    uint8_t             nextChanListener;
    
    // Stats
    uint32_t txDiscards;
    uint32_t shortFragments;
    uint32_t tooLargePayloads;
    uint32_t badRxChan;
    uint32_t badTxChan;

} shtp_t;


// ------------------------------------------------------------------------
// Private data

// Advertisement request
static const uint8_t advertise[] = {
    CMD_ADVERTISE,
    CMD_ADVERTISE_ALL
};

#define MAX_INSTANCES (1)
static shtp_t instances[MAX_INSTANCES];

static bool shtp_initialized = false;

// ------------------------------------------------------------------------
// Private functions

static void shtp_init(void)
{
    // clear instance memory.
    // In particular, this clears the pHal pointers which are used
    // to determine if an instance is open and in-use.
    memset(instances, 0, sizeof(instances));

    // Set the initialized flag so this doesn't happen again.
    shtp_initialized = true;
}

static shtp_t *getInstance(void)
{
    for (int n = 0; n < MAX_INSTANCES; n++) {
        if (instances[n].pHal == 0) {
            // This instance is free
            return &instances[n];
        }
    }

    // Can't give an instance, none are free
    return 0;
}

// Register a listener for an app (advertisement listener)
static void addAdvertListener(shtp_t *pShtp, uint16_t guid,
                              shtp_AdvertCallback_t *callback, void * cookie)
{
    shtp_AppListener_t *pAppListener = 0;

    // Bail out if no space for more apps
    if (pShtp->nextAppListener >= SH2_MAX_APPS) return;

    // Register this app
    pAppListener = &pShtp->appListener[pShtp->nextAppListener];
    pShtp->nextAppListener++;
    pAppListener->guid = guid;
    pAppListener->callback = callback;
    pAppListener->cookie = cookie;
}

// Try to match registered listeners with their channels.
// This is performed every time the underlying Channel, App, Listener data structures are updated.
// As a result, channel number to callback association is fast when receiving packets
static void updateCallbacks(shtp_t *pShtp)
{
    // Figure out which callback is associated with each channel.
    //   Channel -> (GUID, Chan name).
    //   GUID -> App name.
    //   (App name, Chan name) -> Callback

    uint32_t guid;
    const char * chanName = 0;
    
    for (int chanNo = 0; chanNo < SH2_MAX_CHANS; chanNo++) {
        // Reset callback for this channel until we find the right one.
        pShtp->chan[chanNo].callback = 0;
            
        if (pShtp->chan[chanNo].guid == 0xFFFFFFFF) {
            // This channel entry not used.
            continue;
        }

        // Get GUID and Channel Name for this channel
        guid = pShtp->chan[chanNo].guid;
        chanName = pShtp->chan[chanNo].chanName;

        // Look for a listener registered with this guid, channel name
        for (int listenerNo = 0; listenerNo < SH2_MAX_CHANS; listenerNo++)
        {
            if ((pShtp->chanListener[listenerNo].callback != 0) &&
                (pShtp->chanListener[listenerNo].guid == guid) &&
                (strcmp(chanName, pShtp->chanListener[listenerNo].chanName) == 0))
            {
            
                // This listener is the one for this channel
                pShtp->chan[chanNo].callback = pShtp->chanListener[listenerNo].callback;
                pShtp->chan[chanNo].cookie = pShtp->chanListener[listenerNo].cookie;
                break;
            }
        }
    }
}

// Register a new channel listener
static int addChanListener(shtp_t *pShtp,
                           uint16_t guid, const char * chanName,
                           shtp_Callback_t *callback, void *cookie)
{
    shtp_ChanListener_t *pListener = 0;

    // Bail out if there are too many listeners registered
    if (pShtp->nextChanListener >= SH2_MAX_CHANS) return SH2_ERR;

    // Register channel listener
    pListener = &pShtp->chanListener[pShtp->nextChanListener];
    pShtp->nextChanListener++;
    pListener->guid = guid;
    strcpy(pListener->chanName, chanName);
    pListener->callback = callback;
    pListener->cookie = cookie;

    // re-evaluate channel callbacks
    updateCallbacks(pShtp);

    return SH2_OK;
}

static inline uint16_t min_u16(uint16_t a, uint16_t b)
{
    if (a < b) {
        return a;
    }
    else {
        return b;
    }
}

// Send a cargo as a sequence of transports
static int txProcess(shtp_t *pShtp, uint8_t chan, const uint8_t* pData, uint32_t len)
{
    int status = SH2_OK;
    
    bool continuation = false;
    uint16_t cursor = 0;
    uint16_t remaining;
    uint16_t transferLen;  // length of transfer, minus the header
    uint16_t lenField;

    cursor = 0;
    remaining = len;
    while (remaining > 0) {
        // How much data (not header) can we send in next transfer
        transferLen = min_u16(remaining, pShtp->outMaxTransfer-SHTP_HDR_LEN);
        
        // Length field will be transferLen + SHTP_HDR_LEN
        lenField = transferLen + SHTP_HDR_LEN;

        // Put the header in the out buffer
        pShtp->outTransfer[0] = lenField & 0xFF;
        pShtp->outTransfer[1] = (lenField >> 8) & 0xFF;
        if (continuation) {
            pShtp->outTransfer[1] |= 0x80;
        }
        pShtp->outTransfer[2] = chan;
        pShtp->outTransfer[3] = pShtp->chan[chan].nextOutSeq++;

        // Stage one tranfer in the out buffer
        memcpy(pShtp->outTransfer+SHTP_HDR_LEN, pData+cursor, transferLen);
        remaining -= transferLen;
        cursor += transferLen;

        // Transmit (try repeatedly while HAL write returns 0)
        status = pShtp->pHal->write(pShtp->pHal, pShtp->outTransfer, lenField);
        while (status == 0)
        {
            shtp_service(pShtp);
            status = pShtp->pHal->write(pShtp->pHal, pShtp->outTransfer, lenField);
        }
        
        if (status < 0)
        {
            // Error, throw away this cargo
            pShtp->txDiscards++;
            return status;
        }

        // For the rest of this transmission, packets are continuations.
        continuation = true;
    }

    return SH2_OK;
}

// Callback for SHTP app-specific advertisement tags
static void shtpAdvertHdlr(void *cookie, uint8_t tag, uint8_t len, uint8_t *val)
{
    shtp_t *pShtp = (shtp_t *)cookie;

    switch (tag) {
        case TAG_SHTP_VERSION:
            if (strlen((const char *)val) < sizeof(pShtp->shtpVersion)) {
                strcpy(pShtp->shtpVersion, (const char *)val);
            }
            break;
        default:
            break;
    }
}

// Add one to the set of known Apps
static void addApp(shtp_t *pShtp, uint32_t guid)
{
    shtp_App_t *pApp = 0;

    // Bail out if this GUID is already registered
    for (int n = 0; n < pShtp->nextApp; n++) {
        if (pShtp->app[n].guid == guid) return;
    }

    // Bail out if no space for more apps
    if (pShtp->nextApp >= SH2_MAX_APPS) return;

    // Register this app
    pApp = &pShtp->app[pShtp->nextApp];
    pShtp->nextApp++;
    pApp->guid = guid;
    strcpy(pApp->appName, "");

    // Re-evaluate channel callbacks
    updateCallbacks(pShtp);
}

static void setAppName(shtp_t *pShtp, uint32_t guid, const char * appName)
{
    shtp_App_t *pApp = 0;
    
    // Find the app entry with this GUID
    for (unsigned n = 0; n < pShtp->nextApp; n++) {
        if (pShtp->app[n].guid == guid) {
            pApp = &pShtp->app[n];
            strcpy(pApp->appName, appName);
            return;
        }
    }
}

// Add one to the set of known channels
static void addChannel(shtp_t *pShtp, uint8_t chanNo, uint32_t guid, const char * chanName, bool wake)
{
    if (chanNo >= SH2_MAX_CHANS) return;

    shtp_Channel_t * pChan = &pShtp->chan[chanNo];

    // Store channel definition
    pChan->guid = guid;
    strcpy(pChan->chanName, chanName);
    pChan->wake = wake;

    // Init channel-associated data
    pChan->nextOutSeq = 0;
    pChan->nextInSeq = 0;
    pChan->callback = 0;
    pChan->cookie = 0;

    // Re-evaluate channel callbacks
    updateCallbacks(pShtp);
}

static void callAdvertHandler(shtp_t *pShtp, uint32_t guid,
                              uint8_t tag, uint8_t len, uint8_t *val)
{
    // Find listener for this app
    for (int n = 0; n < SH2_MAX_APPS; n++)
    {
        if (pShtp->appListener[n].guid == guid) {
            // Found matching App entry
            if (pShtp->appListener[n].callback != 0) {
                pShtp->appListener[n].callback(pShtp->appListener[n].cookie, tag, len, val);
                return;
            }
        }
    }
}

static void processAdvertisement(shtp_t *pShtp, uint8_t *payload, uint16_t payloadLen)
{
    uint16_t x;
    uint8_t tag;
    uint8_t len;
    uint8_t *val;
    uint16_t cursor = 1;
    uint32_t guid = 0;
    char appName[SHTP_APP_NAME_LEN];
    char chanName[SHTP_CHAN_NAME_LEN];
    uint8_t chanNo = 0;
    bool wake = false;

    strcpy(appName, "");
    strcpy(chanName, "");

    pShtp->advertPhase = ADVERT_IDLE;
        
    while (cursor < payloadLen) {
        tag = payload[cursor++];
        len = payload[cursor++];
        val = payload+cursor;
        cursor += len;

        // Process tag
        switch (tag) {
            case TAG_NULL:
                // Reserved value, not a valid tag.
                break;
            case TAG_GUID:
                // A new GUID is being established so terminate advertisement process with earlier app, if any.
                callAdvertHandler(pShtp, guid, TAG_NULL, 0, 0);
                
                guid = readu32(val);
                addApp(pShtp, guid);
            
                strcpy(appName, "");
                strcpy(chanName, "");
                break;
            case TAG_MAX_CARGO_PLUS_HEADER_WRITE:
                x = readu16(val) - SHTP_HDR_LEN;
            
                if (x < SH2_HAL_MAX_PAYLOAD_OUT) {
                    pShtp->outMaxPayload = x;
                }
                break;
            case TAG_MAX_CARGO_PLUS_HEADER_READ:
                x = readu16(val) - SHTP_HDR_LEN;
                // No need to store this!
                break;
            case TAG_MAX_TRANSFER_WRITE:
                x = readu16(val) - SHTP_HDR_LEN;
                if (x < SH2_HAL_MAX_TRANSFER_OUT) {
                    pShtp->outMaxTransfer = x;
                } else {
                    pShtp->outMaxTransfer = SH2_HAL_MAX_TRANSFER_OUT;
                }
                break;
            case TAG_MAX_TRANSFER_READ:
                x = readu16(val) - SHTP_HDR_LEN;
                if (x < SH2_HAL_MAX_TRANSFER_IN) {
                    pShtp->inMaxTransfer = x;
                }
                break;
            case TAG_NORMAL_CHANNEL:
                chanNo = readu8(val);
                wake = false;
                break;
            case TAG_WAKE_CHANNEL:
                chanNo = readu8(val);
                wake = true;
                break;
            case TAG_APP_NAME:
                strcpy(appName, (const char *)val);
                setAppName(pShtp, guid, appName);
            
                break;
            case TAG_CHANNEL_NAME:
                strcpy(chanName, (const char *)val);
                addChannel(pShtp, chanNo, guid, (const char *)val, wake);

                // Store channel metadata
                if (chanNo < SH2_MAX_CHANS) {
                    pShtp->chan[chanNo].guid = guid;
                    strcpy(pShtp->chan[chanNo].chanName, chanName);
                    pShtp->chan[chanNo].wake = wake;
                }
                break;
            case TAG_ADV_COUNT:
                // Not yet supported.
                break;
            default:
                // Nothing special needs to be done with this tag.
                break;
        }
        
        // Deliver a TLV entry to the app's handler
        callAdvertHandler(pShtp, guid, tag, len, val);
    }

    // terminate advertisement process with last app
    callAdvertHandler(pShtp, guid, TAG_NULL, 0, 0);
}

// Callback for SHTP command channel
static void shtpCmdListener(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    shtp_t *pShtp = (shtp_t *)cookie;
    
    if ((payload == 0) || (len == 0)) return;
    
    uint8_t response = payload[0];

    switch (response) {
        case RESP_ADVERTISE:
            processAdvertisement(pShtp, payload, len);
            break;
        default:
            // unknown response
            break;
    }
}

static void rxAssemble(shtp_t *pShtp, uint8_t *in, uint16_t len, uint32_t t_us)
{
    uint16_t payloadLen;
    bool continuation;
    uint8_t chan = 0;
    uint8_t seq = 0;

    // discard invalid short fragments
    if (len < SHTP_HDR_LEN) {
        pShtp->shortFragments++;
        return;
    }
    
    // Interpret header fields
    payloadLen = (in[0] + (in[1] << 8)) & (~0x8000);
    continuation = ((in[1] & 0x80) != 0);
    chan = in[2];
    seq = in[3];
    
    if (payloadLen < SHTP_HDR_LEN) {
      pShtp->shortFragments++;

      if (pShtp->eventCallback) {
          pShtp->eventCallback(pShtp->eventCookie, SHTP_SHORT_FRAGMENT);
      }
      return;
    }
        
    if ((chan >= SH2_MAX_CHANS) ||
        (chan >= pShtp->nextChanListener)) {
        // Invalid channel id.
        pShtp->badRxChan++;

        if (pShtp->eventCallback) {
            pShtp->eventCallback(pShtp->eventCookie, SHTP_BAD_RX_CHAN);
        }
        return;
    }

    // Discard earlier assembly in progress if the received data doesn't match it.
    if (pShtp->inRemaining) {
        // Check this against previously received data.
        if (!continuation ||
            (chan != pShtp->inChan) ||
            (seq != pShtp->chan[chan].nextInSeq)) {
            // This fragment doesn't fit with previous one, discard earlier data
            pShtp->inRemaining = 0;
        }
    }

    if (pShtp->inRemaining == 0) {
        if (payloadLen > sizeof(pShtp->inPayload)) {
            // Error: This payload won't fit! Discard it.
            pShtp->tooLargePayloads++;
            
            if (pShtp->eventCallback) {
                pShtp->eventCallback(pShtp->eventCookie, SHTP_TOO_LARGE_PAYLOADS);
            }
            return;
        }

        // This represents a new payload

        // Store timestamp
        pShtp->inTimestamp = t_us;

        // Start a new assembly.
        pShtp->inCursor = 0;
        pShtp->inChan = chan;
    }

    // Append the new fragment to the payload under construction.
    if (len > payloadLen) {
        // Only use the valid portion of the transfer
        len = payloadLen;
    }
    memcpy(pShtp->inPayload + pShtp->inCursor, in+SHTP_HDR_LEN, len-SHTP_HDR_LEN);
    pShtp->inCursor += len-SHTP_HDR_LEN;
    pShtp->inRemaining = payloadLen - len;

    // If whole payload received, deliver it to channel listener.
    if (pShtp->inRemaining == 0) {

        // Call callback if there is one.
        if (pShtp->chan[chan].callback != 0) {
            pShtp->chan[chan].callback(pShtp->chan[chan].cookie,
                                       pShtp->inPayload, pShtp->inCursor,
                                       pShtp->inTimestamp);
        }
    }

    // Remember next sequence number we expect for this channel.
    pShtp->chan[chan].nextInSeq = seq + 1;
}

// ------------------------------------------------------------------------
// Public functions

// Takes HAL pointer, returns shtp ID for use in future calls.
// HAL will be opened by this call.
void *shtp_open(sh2_Hal_t *pHal)
{
    if (!shtp_initialized) {
        // Perform one-time module initialization
        shtp_init();
    }
    
    // Validate params
    if (pHal == 0) {
        // Error
        return 0;
    }

    // Find an available instance for this open
    shtp_t *pShtp = getInstance();
    if (pShtp == 0) {
        // No instances available, return error
        return 0;
    }

    // Clear the SHTP instance as a shortcut to initializing all fields
    memset(pShtp, 0, sizeof(shtp_t));
    
    // Store reference to the HAL
    pShtp->pHal = pHal;

    // Clear the asynchronous event callback point
    pShtp->eventCallback = 0;
    pShtp->eventCookie = 0;

    // Initialize state vars (be prepared for adverts)
    pShtp->outMaxPayload = SH2_HAL_MAX_PAYLOAD_OUT;
    pShtp->outMaxTransfer = SH2_HAL_MAX_TRANSFER_OUT;

    // Establish SHTP App and command channel a priori
    addApp(pShtp, GUID_SHTP);
    addChannel(pShtp, 0, GUID_SHTP, "command", false);
    
    // Register SHTP advert listener and command channel listener
    shtp_listenAdvert(pShtp, GUID_SHTP, shtpAdvertHdlr, pShtp);
    shtp_listenChan(pShtp, GUID_SHTP, "command", shtpCmdListener, pShtp);

    // When we open the HAL, it resets the device and adverts are sent automatically.
    // So we go to ADVERT_REQUESTED state.  They are on the way.
    pShtp->advertPhase = ADVERT_REQUESTED;

    // Open HAL
    pHal->open(pHal);

    return pShtp;
}

// Releases resources associated with this SHTP instance.
// HAL will not be closed.
void shtp_close(void *pInstance)
{
    shtp_t *pShtp = (shtp_t *)pInstance;

    pShtp->pHal->close(pShtp->pHal);
    
    // Clear pShtp
    // (Resetting pShtp->pHal to 0, returns this instance to the free pool)
    memset(pShtp, 0, sizeof(shtp_t));
}

// Register the pointer of the callback function for reporting asynchronous events
void shtp_setEventCallback(void *pInstance, 
                           shtp_EventCallback_t * eventCallback, 
                           void *eventCookie) {
    shtp_t *pShtp = (shtp_t *)pInstance;

    pShtp->eventCallback = eventCallback;
    pShtp->eventCookie = eventCookie;
}

// Register a listener for an SHTP channel
int shtp_listenChan(void *pInstance,
                    uint16_t guid, const char * chan,
                    shtp_Callback_t *callback, void * cookie)
{
    shtp_t *pShtp = (shtp_t *)pInstance;
    
    // Balk if channel name isn't valid
    if ((chan == 0) || (strlen(chan) == 0)) return SH2_ERR_BAD_PARAM;

    return addChanListener(pShtp, guid, chan, callback, cookie);
}

// Register a listener for SHTP advertisements 
int shtp_listenAdvert(void *pInstance,
                      uint16_t guid,
                      shtp_AdvertCallback_t *advertCallback, void * cookie)
{
    shtp_t *pShtp = (shtp_t *)pInstance;
    
    // Register the advert listener
    addAdvertListener(pShtp, guid, advertCallback, cookie);

    // Arrange for a new set of advertisements, for this listener
    if (pShtp->advertPhase == ADVERT_IDLE) {
        pShtp->advertPhase = ADVERT_NEEDED;
    }

    return SH2_OK;
}

// Look up the channel number for a particular app, channel.
uint8_t shtp_chanNo(void *pInstance,
                    const char * appName, const char * chanName)
{
    shtp_t *pShtp = (shtp_t *)pInstance;
    
    int chan = 0;
    uint32_t guid = 0xFFFFFFFF;

    // Determine GUID for this appname
    for (int n = 0; n < SH2_MAX_APPS; n++) {
        if (strcmp(pShtp->app[n].appName, appName) == 0) {
            guid = pShtp->app[n].guid;
            break;
        }
    }
    if (guid == 0xFFFFFFFF) return -1;

    for (chan = 0; chan < SH2_MAX_CHANS; chan++) {
        if ((strcmp(pShtp->chan[chan].chanName, chanName) == 0) &&
            pShtp->chan[chan].guid == guid) {
            // Found match
            return chan;
        }
    }

    // Not found
    return 0xFF;
}

// Send an SHTP payload on a particular channel
int shtp_send(void *pInstance,
              uint8_t channel, const uint8_t *payload, uint16_t len)
{
    shtp_t *pShtp = (shtp_t *)pInstance;
    int ret = SH2_OK;
    
    if (len > pShtp->outMaxPayload) {
        return SH2_ERR_BAD_PARAM;
    }
    if (channel >= SH2_MAX_CHANS) {
        pShtp->badTxChan++;
        return SH2_ERR_BAD_PARAM;
    }
    
    ret = txProcess(pShtp, channel, payload, len);

    return ret;
}

// Check for received data and process it.
void shtp_service(void *pInstance)
{
    shtp_t *pShtp = (shtp_t *)pInstance;
    uint32_t t_us = 0;

    if (pShtp->advertPhase == ADVERT_NEEDED) {
        pShtp->advertPhase = ADVERT_REQUESTED;  // do this before send, to avoid recursion.
        int status = shtp_send(pShtp, SHTP_CHAN_COMMAND, advertise, sizeof(advertise));
        if (status != SH2_OK) {
            // Oops, advert request faiLED.  Go back to needing one.
            pShtp->advertPhase = ADVERT_NEEDED;
        }
    }

    int len = pShtp->pHal->read(pShtp->pHal, pShtp->inTransfer, sizeof(pShtp->inTransfer), &t_us);
    if (len) {
        rxAssemble(pShtp, pShtp->inTransfer, len, t_us);
    }
}
