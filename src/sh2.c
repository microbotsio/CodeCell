/*
 * Copyright 2015-2018 Hillcrest Laboratories, Inc.
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

/**
 * @file sh2.c
 * @author David Wheeler
 * @date 22 Sept 2015
 * @brief API Definition for Hillcrest SH-2 Sensor Hub.
 *
 * The sh2 API provides functions for opening a session with
 * the sensor hub and performing all supported operations with it.
 * This includes enabling sensors and reading events as well as
 * other housekeeping functions.
 *
 */

#include "sh2.h"
#include "sh2_err.h"
#include "shtp.h"
#include "sh2_util.h"

#include <string.h>

// ------------------------------------------------------------------------
// Private type definitions

#define GUID_EXECUTABLE (1)
#define GUID_SENSORHUB (2)

// executable/device channel responses
#define EXECUTABLE_DEVICE_CMD_RESET (1)
#define EXECUTABLE_DEVICE_CMD_ON    (2)
#define EXECUTABLE_DEVICE_CMD_SLEEP (3)

// executable/device channel responses
#define EXECUTABLE_DEVICE_RESP_RESET_COMPLETE (1)

// Tags for sensorhub app advertisements.
#define TAG_SH2_VERSION (0x80)
#define TAG_SH2_REPORT_LENGTHS (0x81)

// Max length of sensorhub version string.
#define MAX_VER_LEN (16)

// Max number of report ids supported
#define SH2_MAX_REPORT_IDS (64)

#if defined(_MSC_VER)
#define PACKED_STRUCT struct
#pragma pack(push, 1)
#elif defined(__GNUC__)
#define PACKED_STRUCT struct __attribute__((packed))
#else 
#define PACKED_STRUCT __packed struct
#endif

#define ADVERT_TIMEOUT_US (200000)

// Command and Subcommand values
#define SH2_CMD_ERRORS                 1
#define SH2_CMD_COUNTS                 2
#define     SH2_COUNTS_GET_COUNTS          0
#define     SH2_COUNTS_CLEAR_COUNTS        1
#define SH2_CMD_TARE                   3
#define     SH2_TARE_TARE_NOW              0
#define     SH2_TARE_PERSIST_TARE          1
#define     SH2_TARE_SET_REORIENTATION     2
#define SH2_CMD_INITIALIZE             4
#define     SH2_INIT_SYSTEM                1
#define     SH2_INIT_UNSOLICITED           0x80
// #define SH2_CMD_FRS                    5 /* Depreciated */
#define SH2_CMD_DCD                    6
#define SH2_CMD_ME_CAL                 7
#define SH2_CMD_DCD_SAVE               9
#define SH2_CMD_GET_OSC_TYPE           0x0A
#define SH2_CMD_CLEAR_DCD_AND_RESET    0x0B
#define SH2_CMD_CAL                    0x0C
#define     SH2_CAL_START                   0
#define     SH2_CAL_FINISH                  1
#define SH2_CMD_BOOTLOADER             0x0D     /* SH-2 Reference Manual 6.4.12 */
#define     SH2_BL_MODE_REQ                 0
#define     SH2_BL_STATUS_REQ               1
#define SH2_CMD_INTERACTIVE_ZRO        0x0E     /* SH-2 Reference Manual 6.4.13 */

// SENSORHUB_COMMAND_REQ
#define SENSORHUB_COMMAND_REQ        (0xF2)
#define COMMAND_PARAMS (9)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t seq;
    uint8_t command;
    uint8_t p[COMMAND_PARAMS];
} CommandReq_t;

// SENSORHUB_COMMAND_RESP
#define SENSORHUB_COMMAND_RESP       (0xF1)
#define RESPONSE_VALUES (11)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t seq;
    uint8_t command;
    uint8_t commandSeq;
    uint8_t respSeq;
    uint8_t r[RESPONSE_VALUES];
} CommandResp_t;

// SENSORHUB_PROD_ID_REQ
#define SENSORHUB_PROD_ID_REQ        (0xF9)
typedef PACKED_STRUCT {
    uint8_t reportId;  
    uint8_t reserved;
} ProdIdReq_t;

// SENSORHUB_PROD_ID_RESP
#define SENSORHUB_PROD_ID_RESP       (0xF8)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t resetCause;
    uint8_t swVerMajor;
    uint8_t swVerMinor;
    uint32_t swPartNumber;
    uint32_t swBuildNumber;
    uint16_t swVerPatch;
    uint8_t reserved0;
    uint8_t reserved1;
} ProdIdResp_t;

// Report definitions
// Bit fields for Feature Report flags
#define FEAT_CHANGE_SENSITIVITY_RELATIVE (1)
#define FEAT_CHANGE_SENSITIVITY_ABSOLUTE (0)
#define FEAT_CHANGE_SENSITIVITY_ENABLED  (2)
#define FEAT_CHANGE_SENSITIVITY_DISABLED (0)
#define FEAT_WAKE_ENABLED                (4)
#define FEAT_WAKE_DISABLED               (0)
#define FEAT_ALWAYS_ON_ENABLED           (8)
#define FEAT_ALWAYS_ON_DISABLED          (0)

// GET_FEATURE_REQ
#define SENSORHUB_GET_FEATURE_REQ    (0xFE)
typedef PACKED_STRUCT{
    uint8_t reportId;
    uint8_t featureReportId;
} GetFeatureReq_t;

// SENSORHUB_GET_FEATURE_RESP
#define SENSORHUB_GET_FEATURE_RESP   (0xFC)
typedef PACKED_STRUCT{
    uint8_t reportId;
    uint8_t featureReportId;      // sensor id
    uint8_t flags;                // FEAT_... values
    uint16_t changeSensitivity;
    uint32_t reportInterval_uS;
    uint32_t batchInterval_uS;
    uint32_t sensorSpecific;
} GetFeatureResp_t;


typedef struct sh2_s sh2_t;

typedef int (sh2_OpStart_t)(sh2_t *pSh2);
typedef void (sh2_OpRx_t)(sh2_t *pSh2, const uint8_t *payload, uint16_t len);

typedef struct sh2_Op_s {
    uint32_t timeout_us;
    sh2_OpStart_t *start;
    sh2_OpRx_t *rx;
} sh2_Op_t;

// Parameters and state information for the operation in progress
typedef union {
    struct {
        CommandReq_t req;
    } sendCmd;
    struct {
        sh2_ProductIds_t *pProdIds;
        uint8_t nextEntry;
        uint8_t expectedEntries;
    } getProdIds;
    struct {
        sh2_SensorConfig_t *pConfig;
        sh2_SensorId_t sensorId;
    } getSensorConfig;
    struct {
        const sh2_SensorConfig_t *pConfig;
        sh2_SensorId_t sensorId;
    } setSensorConfig;
    struct {
        uint16_t frsType;
        uint32_t *pData;
        uint16_t *pWords;
        uint16_t nextOffset;
    } getFrs;
    struct {
        uint16_t frsType;
        uint32_t *pData;
        uint16_t words;
        uint16_t offset;
    } setFrs;
    struct {
        uint8_t severity;
        sh2_ErrorRecord_t *pErrors;
        uint16_t *pNumErrors;
        uint16_t errsRead;
    } getErrors;
    struct {
        sh2_SensorId_t sensorId;
        sh2_Counts_t *pCounts;
    } getCounts;
    struct {
        uint8_t sensors;
    } calConfig;
    struct {
        uint8_t *pSensors;
    } getCalConfig;
    struct {
        sh2_SensorId_t sensorId;
    } forceFlush;
    struct {
        sh2_OscType_t *pOscType;
    } getOscType;
    struct {
        uint32_t interval_us;
    } startCal;
    struct {
        sh2_CalStatus_t status;
    } finishCal;
} sh2_OpData_t;

// Max length of an FRS record, words.
#define MAX_FRS_WORDS (72)

struct sh2_s {
    // Pointer to the SHTP HAL
    sh2_Hal_t *pHal;

    // associated SHTP instance
    void *pShtp;
    
    volatile bool resetComplete;
    bool advertDone;
    uint8_t executableChan;
    uint8_t controlChan;
    char version[MAX_VER_LEN+1];

    // Report lengths
    struct {
        uint8_t id;
        uint8_t len;
    } report[SH2_MAX_REPORT_IDS];

    // Multi-step operation support
    const sh2_Op_t *pOp;
    int opStatus;
    sh2_OpData_t opData;
    uint8_t lastCmdId;
    uint8_t cmdSeq;
    uint8_t nextCmdSeq;
    
    // Event callback and it's cookie
    sh2_EventCallback_t *eventCallback;
    void * eventCookie;

    // Sensor callback and it's cookie
    sh2_SensorCallback_t *sensorCallback;
    void * sensorCookie;

    // Storage space for reading sensor metadata
    uint32_t frsData[MAX_FRS_WORDS];
    uint16_t frsDataLen;

    // Stats
    uint32_t execBadPayload;
    uint32_t emptyPayloads;
    uint32_t unknownReportIds;

};

#define SENSORHUB_BASE_TIMESTAMP_REF (0xFB)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint32_t timebase;
} BaseTimestampRef_t;

#define SENSORHUB_TIMESTAMP_REBASE   (0xFA)
typedef PACKED_STRUCT {
    uint8_t reportId;
    int32_t timebase;
} TimestampRebase_t;

// SENSORHUB_FORCE_SENSOR_FLUSH
#define SENSORHUB_FORCE_SENSOR_FLUSH (0xF0)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t sensorId;
} ForceFlushReq_t;

// SENSORHUB_FLUSH_COMPLETED    
#define SENSORHUB_FLUSH_COMPLETED    (0xEF)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t sensorId;
} ForceFlushResp_t;

// ------------------------------------------------------------------------
// Private data

// SH2 state
sh2_t _sh2;

// SH2 Async Event Message
static sh2_AsyncEvent_t sh2AsyncEvent;

// ------------------------------------------------------------------------
// Private functions

// SH-2 transaction phases
static int opStart(sh2_t *pSh2, const sh2_Op_t *pOp)
{
    // return error if another operation already in progress
    if (pSh2->pOp) return SH2_ERR_OP_IN_PROGRESS;

    // Establish this operation as the new operation in progress
    pSh2->pOp = pOp;
    pSh2->opStatus = SH2_OK;
    int rc = pOp->start(pSh2);  // Call start method
    if (rc != SH2_OK) {
        // Unregister this operation
        pSh2->opStatus = rc;
        pSh2->pOp = 0;
    }

    return rc;
}

static void opRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{ 
    if ((pSh2->pOp != 0) &&                      // An operation is in progress
        (pSh2->pOp->rx != 0)) {                  // and it has an rx method
        pSh2->pOp->rx(pSh2, payload, len);  // Call receive method
    }
}

static void sensorhubAdvertHdlr(void *cookie, uint8_t tag, uint8_t len, uint8_t *value)
{
    sh2_t *pSh2 = (sh2_t *)cookie;
    
    switch (tag) {
        case TAG_SH2_VERSION:
            strcpy(pSh2->version, (const char *)value);
            break;

        case TAG_SH2_REPORT_LENGTHS:
        {
            uint8_t reports = len/2;
            if (reports > SH2_MAX_REPORT_IDS) {
                // Hub gave us more report lengths than we can store!
                reports = SH2_MAX_REPORT_IDS;
            }
        
            for (int n = 0; n < reports; n++) {
                pSh2->report[n].id = value[n*2];
                pSh2->report[n].len = value[n*2 + 1];
            }
            break;
        }
    
        case 0:
        {
            // 0 tag indicates end of advertisements for this app
            // At this time, the SHTP layer can give us our channel numbers
            pSh2->executableChan = shtp_chanNo(pSh2->pShtp, "executable", "device");
            pSh2->controlChan = shtp_chanNo(pSh2->pShtp, "sensorhub", "control");

            pSh2->advertDone = true;
            break;
        }
        
        default:
            break;
    }
}

static void sensorhubControlHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_t *pSh2 = (sh2_t *)cookie;

    uint16_t cursor = 0;
    uint32_t count = 0;
    CommandResp_t * pResp = 0;
    
    if (len == 0) {
        pSh2->emptyPayloads++;
        return;
    }

    while (cursor < len) {
        // Get next report id
        count++;
        uint8_t reportId = payload[cursor];

        // Determine report length
        uint8_t reportLen = 0;
        for (int n = 0; n < SH2_MAX_REPORT_IDS; n++) {
            if (pSh2->report[n].id == reportId) {
                reportLen = pSh2->report[n].len;
                break;
            }
        }
        if (reportLen == 0) {
            // An unrecognized report id
            pSh2->unknownReportIds++;
            return;
        }
        else {
            // Check for unsolicited initialize response
            if (reportId == SENSORHUB_COMMAND_RESP) {
                pResp = (CommandResp_t *)(payload+cursor);
                if ((pResp->command == (SH2_CMD_INITIALIZE | SH2_INIT_UNSOLICITED)) &&
                    (pResp->r[1] == SH2_INIT_SYSTEM)) {
                    // This is an unsolicited INIT message.
                    // Is it time to call reset callback?
                }

            } // Check for Get Feature Response
            else if (reportId == SENSORHUB_GET_FEATURE_RESP) {
                if (pSh2->eventCallback) {
                    GetFeatureResp_t * pGetFeatureResp;
                    pGetFeatureResp = (GetFeatureResp_t *)(payload + cursor);

                    sh2AsyncEvent.eventId = SH2_GET_FEATURE_RESP;
                    sh2AsyncEvent.sh2SensorConfigResp.sensorId = pGetFeatureResp->featureReportId;
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.changeSensitivityEnabled = ((pGetFeatureResp->flags & FEAT_CHANGE_SENSITIVITY_ENABLED) != 0);
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.changeSensitivityRelative = ((pGetFeatureResp->flags & FEAT_CHANGE_SENSITIVITY_RELATIVE) != 0);
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.wakeupEnabled = ((pGetFeatureResp->flags & FEAT_WAKE_ENABLED) != 0);
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.alwaysOnEnabled = ((pGetFeatureResp->flags & FEAT_ALWAYS_ON_ENABLED) != 0);
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.changeSensitivity = pGetFeatureResp->changeSensitivity;
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.reportInterval_us = pGetFeatureResp->reportInterval_uS;
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.batchInterval_us = pGetFeatureResp->batchInterval_uS;
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.sensorSpecific = pGetFeatureResp->sensorSpecific;

                    pSh2->eventCallback(pSh2->eventCookie, &sh2AsyncEvent);
                }
            }

            // Hand off to operation in progress, if any
            opRx(pSh2, payload+cursor, reportLen);
            cursor += reportLen;
        }
    }
}

static int opCompleted(sh2_t *pSh2, int status)
{
    // Record status
    pSh2->opStatus = status;

    // Signal that op is done.
    pSh2->pOp = 0;

    return SH2_OK;
}

static int opProcess(sh2_t *pSh2, const sh2_Op_t *pOp)
{
    int status = SH2_OK;
    uint32_t start_us = 0;

    start_us = pSh2->pHal->getTimeUs(pSh2->pHal);
    
    status = opStart(&_sh2, pOp);
    if (status != SH2_OK) {
        return status;
    }

    uint32_t now_us = start_us;
    
    // While op not complete and not timed out.
    while ((pSh2->pOp != 0) &&
           ((pOp->timeout_us == 0) ||
            ((now_us-start_us) < pOp->timeout_us))) {
        // Service SHTP to poll the device.
        shtp_service(pSh2->pShtp);

        // Update the time
        now_us = pSh2->pHal->getTimeUs(pSh2->pHal);
    }

    if (pSh2->pOp != 0) {
        // Operation has timed out.  Clean up.
        pSh2->pOp = 0;
        pSh2->opStatus = SH2_ERR_TIMEOUT;
    }

    return pSh2->opStatus;
}

static uint8_t getReportLen(sh2_t *pSh2, uint8_t reportId)
{
    for (int n = 0; n < SH2_MAX_REPORT_IDS; n++) {
        if (pSh2->report[n].id == reportId) {
            return pSh2->report[n].len;
        }
    }

    return 0;
}

// Produce 64-bit microsecond timestamp for a sensor event
static uint64_t touSTimestamp(uint32_t hostInt, int32_t referenceDelta, uint16_t delay)
{
    static uint32_t lastHostInt = 0;
    static uint32_t rollovers = 0;
    uint64_t timestamp;

    // Count times hostInt timestamps rolLED over to produce upper bits
    if (hostInt < lastHostInt) {
        rollovers++;
    }
    lastHostInt = hostInt;
    
    timestamp = ((uint64_t)rollovers << 32);
    timestamp += hostInt + (referenceDelta + delay) * 100;

    return timestamp;
}

static void sensorhubInputHdlr(sh2_t *pSh2, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_SensorEvent_t event;
    uint16_t cursor = 0;

    uint32_t referenceDelta;

    referenceDelta = 0;

    while (cursor < len) {
        // Get next report id
        uint8_t reportId = payload[cursor];

        // Determine report length
        uint8_t reportLen = getReportLen(pSh2, reportId);
        if (reportLen == 0) {
            // An unrecognized report id
            pSh2->unknownReportIds++;
            return;
        }
        else {
            if (reportId == SENSORHUB_BASE_TIMESTAMP_REF) {
                const BaseTimestampRef_t *rpt = (const BaseTimestampRef_t *)(payload+cursor);
                
                // store base timestamp reference
                referenceDelta = -rpt->timebase;
            }
            else if (reportId == SENSORHUB_TIMESTAMP_REBASE) {
                const TimestampRebase_t *rpt = (const TimestampRebase_t *)(payload+cursor);

                referenceDelta += rpt->timebase;
            }
            else if (reportId == SENSORHUB_FLUSH_COMPLETED) {
                // Route this as if it arrived on command channel.
                opRx(pSh2, payload+cursor, reportLen);
            }
            else {
                uint8_t *pReport = payload+cursor;
                uint16_t delay = ((pReport[2] & 0xFC) << 6) + pReport[3];
                event.timestamp_uS = touSTimestamp(timestamp, referenceDelta, delay);
                event.reportId = reportId;
                memcpy(event.report, pReport, reportLen);
                event.len = reportLen;
                if (pSh2->sensorCallback != 0) {
                    pSh2->sensorCallback(pSh2->sensorCookie, &event);
                }
            }
            cursor += reportLen;
        }
    }
}

static void sensorhubInputNormalHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_t *pSh2 = (sh2_t *)cookie;

    sensorhubInputHdlr(pSh2, payload, len, timestamp);
}

static void sensorhubInputWakeHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_t *pSh2 = (sh2_t *)cookie;
    
    sensorhubInputHdlr(pSh2, payload, len, timestamp);
}

static void sensorhubInputGyroRvHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_t *pSh2 = (sh2_t *)cookie;
    sh2_SensorEvent_t event;
    uint16_t cursor = 0;

    uint8_t reportId = SH2_GYRO_INTEGRATED_RV;
    uint8_t reportLen = getReportLen(pSh2, reportId);

    while (cursor < len) {
        event.timestamp_uS = timestamp;
        event.reportId = reportId;
        memcpy(event.report, payload+cursor, reportLen);
        event.len = reportLen;

        if (pSh2->sensorCallback != 0) {
            pSh2->sensorCallback(pSh2->sensorCookie, &event);
        }

        cursor += reportLen;
    }
}

static void executableAdvertHdlr(void *cookie, uint8_t tag, uint8_t len, uint8_t *value)
{
    // Ignore.  No known TLV tags for this app.
}

static void executabLEDeviceHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_t *pSh2 = (sh2_t *)cookie;

    // Discard if length is bad
    if (len != 1) {
        pSh2->execBadPayload++;
        return;
    }
    
    switch (payload[0]) {
        case EXECUTABLE_DEVICE_RESP_RESET_COMPLETE:
            // reset process is now done.
            pSh2->resetComplete = true;
            
            // Notify client that reset is complete.
            sh2AsyncEvent.eventId = SH2_RESET;
            if (pSh2->eventCallback) {
                pSh2->eventCallback(pSh2->eventCookie, &sh2AsyncEvent);
            }
            break;
        default:
            pSh2->execBadPayload++;
            break;
    }
}

static int sendExecutable(sh2_t *pSh2, uint8_t cmd)
{
    return shtp_send(pSh2->pShtp, pSh2->executableChan, &cmd, 1);
}

static int sendCtrl(sh2_t *pSh2, const uint8_t *data, uint16_t len)
{
    return shtp_send(pSh2->pShtp, pSh2->controlChan, data, len);
}

static int16_t toQ14(double x)
{
    int16_t retval = (int16_t)(x * (1<<14));
    
    return retval;
}

// ------------------------------------------------------------------------
// Get Product ID support

// Get Product ID Op handler
static int getProdIdStart(sh2_t *pSh2)
{
    int rc = SH2_OK;
    ProdIdReq_t req;
    
    pSh2->opData.getProdIds.nextEntry = 0;
    pSh2->opData.getProdIds.expectedEntries = 4;  // Most products supply 4 product ids.
                                                // When the first arrives, we'll know if
                                                // we need to adjust this.
    
    // Set up request to issue
    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_PROD_ID_REQ;
    rc = sendCtrl(pSh2, (uint8_t *)&req, sizeof(req));

    return rc;
}

static void getProdIdRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    ProdIdResp_t *resp = (ProdIdResp_t *)payload;
    
    // skip this if it isn't the product id response.
    if (resp->reportId != SENSORHUB_PROD_ID_RESP) return;

    // Store this product id, if we can
    sh2_ProductIds_t *pProdIds = pSh2->opData.getProdIds.pProdIds;
    
    if (pProdIds) {
        // Store the product id response
        if (pSh2->opData.getProdIds.nextEntry < pSh2->opData.getProdIds.expectedEntries) {
            sh2_ProductId_t *pProdId = &pProdIds->entry[pSh2->opData.getProdIds.nextEntry];
            
            pProdId->resetCause = resp->resetCause;
            pProdId->swVersionMajor = resp->swVerMajor;
            pProdId->swVersionMinor = resp->swVerMinor;
            pProdId->swPartNumber = resp->swPartNumber;
            pProdId->swBuildNumber = resp->swBuildNumber;
            pProdId->swVersionPatch = resp->swVerPatch;
            pProdId->reserved0 = resp->reserved0;
            pProdId->reserved1 = resp->reserved1;

            if (pProdId->swPartNumber == 10004095) {
                // FSP200 has 5 product id entries
                pSh2->opData.getProdIds.expectedEntries = 5;
            }


            pSh2->opData.getProdIds.nextEntry++;
        }
    }

    // Complete this operation if there is no storage for more product ids
    if ((pSh2->opData.getProdIds.pProdIds == 0) ||
        (pSh2->opData.getProdIds.nextEntry >= pSh2->opData.getProdIds.expectedEntries)) {
        
        pSh2->opData.getProdIds.pProdIds->numEntries = pSh2->opData.getProdIds.nextEntry;
        opCompleted(pSh2, SH2_OK);
    }

    return;
}

const sh2_Op_t getProdIdOp = {
    .start = getProdIdStart,
    .rx = getProdIdRx,
};

// ------------------------------------------------------------------------
// Set Sensor Config

static int getSensorConfigStart(sh2_t *pSh2)
{
    int rc = SH2_OK;
    GetFeatureReq_t req;
    
    // set up request to issue
    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_GET_FEATURE_REQ;
    req.featureReportId = pSh2->opData.getSensorConfig.sensorId;
    rc = sendCtrl(pSh2, (uint8_t *)&req, sizeof(req));

    return rc;
}

static void getSensorConfigRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    GetFeatureResp_t *resp = (GetFeatureResp_t *)payload;
    sh2_SensorConfig_t *pConfig;
    
    // skip this if it isn't the response we're waiting for.
    if (resp->reportId != SENSORHUB_GET_FEATURE_RESP) return;
    if (resp->featureReportId != pSh2->opData.getSensorConfig.sensorId) return;

    // Copy out data
    pConfig = pSh2->opData.getSensorConfig.pConfig;
    
    pConfig->changeSensitivityEnabled = ((resp->flags & FEAT_CHANGE_SENSITIVITY_ENABLED) != 0);
    pConfig->changeSensitivityRelative = ((resp->flags & FEAT_CHANGE_SENSITIVITY_RELATIVE) != 0);
    pConfig->wakeupEnabled = ((resp->flags & FEAT_WAKE_ENABLED) != 0);
    pConfig->alwaysOnEnabled = ((resp->flags & FEAT_ALWAYS_ON_ENABLED) != 0);
    pConfig->changeSensitivity = resp->changeSensitivity;
    pConfig->reportInterval_us = resp->reportInterval_uS;
    pConfig->batchInterval_us = resp->batchInterval_uS;
    pConfig->sensorSpecific = resp->sensorSpecific;

    // Complete this operation
    opCompleted(pSh2, SH2_OK);

    return;
}

const sh2_Op_t getSensorConfigOp = {
    .start = getSensorConfigStart,
    .rx = getSensorConfigRx,
};

// ------------------------------------------------------------------------
// Set Sensor Config

// SENSORHUB_SET_FEATURE_CMD
#define SENSORHUB_SET_FEATURE_CMD    (0xFD)
typedef PACKED_STRUCT {
    uint8_t reportId;             // 0xFD
    uint8_t featureReportId;      // sensor id
    uint8_t flags;                // FEAT_... values
    uint16_t changeSensitivity;
    uint32_t reportInterval_uS;
    uint32_t batchInterval_uS;
    uint32_t sensorSpecific;
} SetFeatureReport_t;

static int setSensorConfigStart(sh2_t *pSh2)
{
    SetFeatureReport_t req;
    uint8_t flags = 0;
    int rc;
    sh2_SensorConfig_t *pConfig = pSh2->opData.getSensorConfig.pConfig;
    
    if (pConfig->changeSensitivityEnabled)  flags |= FEAT_CHANGE_SENSITIVITY_ENABLED;
    if (pConfig->changeSensitivityRelative) flags |= FEAT_CHANGE_SENSITIVITY_RELATIVE;
    if (pConfig->wakeupEnabled)             flags |= FEAT_WAKE_ENABLED;
    if (pConfig->alwaysOnEnabled)           flags |= FEAT_ALWAYS_ON_ENABLED;

    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_SET_FEATURE_CMD;
    req.featureReportId = pSh2->opData.setSensorConfig.sensorId;
    req.flags = flags;
    req.changeSensitivity = pConfig->changeSensitivity;
    req.reportInterval_uS = pConfig->reportInterval_us;
    req.batchInterval_uS = pConfig->batchInterval_us;
    req.sensorSpecific = pConfig->sensorSpecific;

    rc = sendCtrl(pSh2, (uint8_t *)&req, sizeof(req));
    opCompleted(pSh2, rc);

    return rc;
}

const sh2_Op_t setSensorConfigOp = {
    .start = setSensorConfigStart,
};

// ------------------------------------------------------------------------
// Get FRS.

// SENSORHUB_FRS_WRITE_REQ
#define SENSORHUB_FRS_WRITE_REQ      (0xF7)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t reserved;
    uint16_t length;
    uint16_t frsType;
} FrsWriteReq_t;

// SENSORHUB_FRS_WRITE_DATA_REQ
#define SENSORHUB_FRS_WRITE_DATA_REQ (0xF6)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t reserved;
    uint16_t offset;
    uint32_t data0;
    uint32_t data1;
} FrsWriteDataReq_t;

// FRS write status values
#define FRS_WRITE_STATUS_RECEIVED (0)
#define FRS_WRITE_STATUS_UNRECOGNIZED_FRS_TYPE (1)
#define FRS_WRITE_STATUS_BUSY (2)
#define FRS_WRITE_STATUS_WRITE_COMPLETED (3)
#define FRS_WRITE_STATUS_READY (4)
#define FRS_WRITE_STATUS_FAILED (5)
#define FRS_WRITE_STATUS_NOT_READY (6) // data received when not in write mode
#define FRS_WRITE_STATUS_INVALID_LENGTH (7)
#define FRS_WRITE_STATUS_RECORD_VALID (8)
#define FRS_WRITE_STATUS_INVALID_RECORD (9)
#define FRS_WRITE_STATUS_DEVICE_ERROR (10)
#define FRS_WRITE_STATUS_READ_ONLY (11)

// SENSORHUB_FRS_WRITE_RESP
#define SENSORHUB_FRS_WRITE_RESP     (0xF5)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t status;
    uint16_t wordOffset;
} FrsWriteResp_t;

// RESP_FRS_READ_REQ
#define SENSORHUB_FRS_READ_REQ       (0xF4)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t reserved;
    uint16_t readOffset;
    uint16_t frsType;
    uint16_t blockSize;
} FrsReadReq_t;

// Get Datalen portion of len_status field
#define FRS_READ_DATALEN(x) ((x >> 4) & 0x0F)

// Get status portion of len_status field
#define FRS_READ_STATUS(x) ((x) & 0x0F)

// Status values
#define FRS_READ_STATUS_NO_ERROR                        0
#define FRS_READ_STATUS_UNRECOGNIZED_FRS_TYPE           1
#define FRS_READ_STATUS_BUSY                            2
#define FRS_READ_STATUS_READ_RECORD_COMPLETED           3
#define FRS_READ_STATUS_OFFSET_OUT_OF_RANGE             4
#define FRS_READ_STATUS_RECORD_EMPTY                    5
#define FRS_READ_STATUS_READ_BLOCK_COMPLETED            6
#define FRS_READ_STATUS_READ_BLOCK_AND_RECORD_COMPLETED 7
#define FRS_READ_STATUS_DEVICE_ERROR                    8

// SENSORHUB_FRS_READ_RESP
#define SENSORHUB_FRS_READ_RESP      (0xF3)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t len_status;  // See FRS_READ... macros above
    uint16_t wordOffset;
    uint32_t data0;
    uint32_t data1;
    uint16_t frsType;
    uint8_t reserved0;
    uint8_t reserved1;
} FrsReadResp_t;

static int getFrsStart(sh2_t *pSh2)
{
    int rc = SH2_OK;
    FrsReadReq_t req;

    pSh2->opData.getFrs.nextOffset = 0;
    
    // set up request to issue
    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_FRS_READ_REQ;
    req.reserved = 0;
    req.readOffset = 0; // read from start
    req.frsType = pSh2->opData.getFrs.frsType;
    req.blockSize = 0;  // read all avail data

    rc = sendCtrl(pSh2, (uint8_t *)&req, sizeof(req));

    return rc;
}

static void getFrsRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    FrsReadResp_t *resp = (FrsReadResp_t *)payload;
    uint8_t status;

    // skip this if it isn't the response we're looking for
    if (resp->reportId != SENSORHUB_FRS_READ_RESP) return;

    // Check for errors: Unrecognized FRS type, Busy, Out of range, Device error
    status = FRS_READ_STATUS(resp->len_status);
    if ((status == FRS_READ_STATUS_UNRECOGNIZED_FRS_TYPE) ||
        (status == FRS_READ_STATUS_BUSY) ||
        (status == FRS_READ_STATUS_OFFSET_OUT_OF_RANGE) ||
        (status == FRS_READ_STATUS_DEVICE_ERROR)
        ) {
        // Operation faiLED
        opCompleted(pSh2, SH2_ERR_HUB);
        return;
    }

    if (status == FRS_READ_STATUS_RECORD_EMPTY) {
        // Empty record, return zero length.
        *(pSh2->opData.getFrs.pWords) = 0;
        opCompleted(pSh2, SH2_OK);
    }

    // Store the contents from this response
    uint16_t offset = resp->wordOffset;

    // check for missed offsets, resulting in error.
    if (offset != pSh2->opData.getFrs.nextOffset) {
        // Some data was dropped.
        *(pSh2->opData.getFrs.pWords) = 0;
        opCompleted(pSh2, SH2_ERR_IO);
    }
    
    // store first word, if we have room
    if ((*(pSh2->opData.getFrs.pWords) == 0) ||
        (offset <= *(pSh2->opData.getFrs.pWords))) {
        pSh2->opData.getFrs.pData[offset] = resp->data0;
        pSh2->opData.getFrs.nextOffset = offset+1;
    }

    // store second word if there is one and we have room
    if ((FRS_READ_DATALEN(resp->len_status) == 2)  &&
        ((*(pSh2->opData.getFrs.pWords) == 0) ||
         (offset <= *(pSh2->opData.getFrs.pWords)))) {
        pSh2->opData.getFrs.pData[offset+1] = resp->data1;
        pSh2->opData.getFrs.nextOffset = offset+2;
    }

    // If read is done, complete the operation
    if ((status == FRS_READ_STATUS_READ_RECORD_COMPLETED) ||
        (status == FRS_READ_STATUS_READ_BLOCK_COMPLETED) ||
        (status == FRS_READ_STATUS_READ_BLOCK_AND_RECORD_COMPLETED)) {
        *(pSh2->opData.getFrs.pWords) = pSh2->opData.getFrs.nextOffset;

        opCompleted(pSh2, SH2_OK);
    }

    return;
}

const sh2_Op_t getFrsOp = {
    .start = getFrsStart,
    .rx = getFrsRx,
};

// ------------------------------------------------------------------------
// Support for sh2_getMetadata

const static struct {
    sh2_SensorId_t sensorId;
    uint16_t recordId;
} sensorToRecordMap[] = {
    { SH2_RAW_ACCELEROMETER,            FRS_ID_META_RAW_ACCELEROMETER },
    { SH2_ACCELEROMETER,                FRS_ID_META_ACCELEROMETER },
    { SH2_LINEAR_ACCELERATION,          FRS_ID_META_LINEAR_ACCELERATION },
    { SH2_GRAVITY,                      FRS_ID_META_GRAVITY },
    { SH2_RAW_GYROSCOPE,                FRS_ID_META_RAW_GYROSCOPE },
    { SH2_GYROSCOPE_CALIBRATED,         FRS_ID_META_GYROSCOPE_CALIBRATED },
    { SH2_GYROSCOPE_UNCALIBRATED,       FRS_ID_META_GYROSCOPE_UNCALIBRATED },
    { SH2_RAW_MAGNETOMETER,             FRS_ID_META_RAW_MAGNETOMETER },
    { SH2_MAGNETIC_FIELD_CALIBRATED,    FRS_ID_META_MAGNETIC_FIELD_CALIBRATED },
    { SH2_MAGNETIC_FIELD_UNCALIBRATED,  FRS_ID_META_MAGNETIC_FIELD_UNCALIBRATED },
    { SH2_ROTATION_VECTOR,              FRS_ID_META_ROTATION_VECTOR },
    { SH2_GAME_ROTATION_VECTOR,         FRS_ID_META_GAME_ROTATION_VECTOR },
    { SH2_GEOMAGNETIC_ROTATION_VECTOR,  FRS_ID_META_GEOMAGNETIC_ROTATION_VECTOR },
    { SH2_PRESSURE,                     FRS_ID_META_PRESSURE },
    { SH2_AMBIENT_LIGHT,                FRS_ID_META_AMBIENT_LIGHT },
    { SH2_HUMIDITY,                     FRS_ID_META_HUMIDITY },
    { SH2_PROXIMITY,                    FRS_ID_META_PROXIMITY },
    { SH2_TEMPERATURE,                  FRS_ID_META_TEMPERATURE },
    { SH2_TAP_DETECTOR,                 FRS_ID_META_TAP_DETECTOR },
    { SH2_STEP_DETECTOR,                FRS_ID_META_STEP_DETECTOR },
    { SH2_STEP_COUNTER,                 FRS_ID_META_STEP_COUNTER },
    { SH2_SIGNIFICANT_MOTION,           FRS_ID_META_SIGNIFICANT_MOTION },
    { SH2_STABILITY_CLASSIFIER,         FRS_ID_META_STABILITY_CLASSIFIER },
    { SH2_SHAKE_DETECTOR,               FRS_ID_META_SHAKE_DETECTOR },
    { SH2_FLIP_DETECTOR,                FRS_ID_META_FLIP_DETECTOR },
    { SH2_PICKUP_DETECTOR,              FRS_ID_META_PICKUP_DETECTOR },
    { SH2_STABILITY_DETECTOR,           FRS_ID_META_STABILITY_DETECTOR },
    { SH2_PERSONAL_ACTIVITY_CLASSIFIER, FRS_ID_META_PERSONAL_ACTIVITY_CLASSIFIER },
    { SH2_SLEEP_DETECTOR,               FRS_ID_META_SLEEP_DETECTOR },
    { SH2_TILT_DETECTOR,                FRS_ID_META_TILT_DETECTOR },
    { SH2_POCKET_DETECTOR,              FRS_ID_META_POCKET_DETECTOR },
    { SH2_CIRCLE_DETECTOR,              FRS_ID_META_CIRCLE_DETECTOR },
};

static void stuffMetadata(sh2_SensorMetadata_t *pData, uint32_t *frsData)
{
    // Populate the sensorMetadata structure with results
    pData->meVersion        = (frsData[0] >> 0) & 0xFF;
    pData->mhVersion        = (frsData[0] >> 8) & 0xFF;
    pData->shVersion        = (frsData[0] >> 16) & 0xFF;
    pData->range            = frsData[1];
    pData->resolution       = frsData[2];
    pData->power_mA         = (frsData[3] >> 0) & 0xFFFF;    // 16.10 format
    pData->revision         = (frsData[3] >> 16) & 0xFFFF;
    pData->minPeriod_uS     = frsData[4];
    pData->maxPeriod_uS     = 0;  // ...unless reading format 4 metadata below
    pData->fifoMax          = (frsData[5] >> 0) & 0xFFFF;
    pData->fifoReserved     = (frsData[5] >> 16) & 0xFFFF;
    pData->batchBufferBytes = (frsData[6] >> 0) & 0xFFFF;;
    pData->vendorIdLen      = (frsData[6] >> 16) & 0xFFFF;

    // Init fields that may not be present, depending on metadata revision
    pData->qPoint1           = 0;
    pData->qPoint2           = 0;
    pData->qPoint3           = 0;
    pData->sensorSpecificLen = 0;
    strcpy(pData->vendorId, ""); // init with empty string in case vendorIdLen == 0

    int vendorIdOffset = 8;
    // Get revision-specific fields
    if (pData->revision == 0) {
        // No fixed fields, vendor id copied after if-else block
    }
    else if (pData->revision == 1) {
        pData->qPoint1        = (frsData[7] >> 0) & 0xFFFF;
        pData->qPoint2        = (frsData[7] >> 16) & 0xFFFF;
    }
    else if (pData->revision == 2) {
        pData->qPoint1        = (frsData[7] >> 0) & 0xFFFF;
        pData->qPoint2        = (frsData[7] >> 16) & 0xFFFF;
        pData->sensorSpecificLen = (frsData[8] >> 0) & 0xFFFF;
        memcpy(pData->sensorSpecific, (uint8_t *)&frsData[9], pData->sensorSpecificLen);
        vendorIdOffset = 9 + ((pData->sensorSpecificLen+3)/4); // 9 + one word for every 4 bytes of SS data
    }
    else if (pData->revision == 3) {
        pData->qPoint1        = (frsData[7] >> 0) & 0xFFFF;
        pData->qPoint2        = (frsData[7] >> 16) & 0xFFFF;
        pData->sensorSpecificLen = (frsData[8] >> 0) & 0xFFFF;
        pData->qPoint3        = (frsData[8] >> 16) & 0xFFFF;
        memcpy(pData->sensorSpecific, (uint8_t *)&frsData[9], pData->sensorSpecificLen);
        vendorIdOffset = 9 + ((pData->sensorSpecificLen+3)/4); // 9 + one word for every 4 bytes of SS data
    }
    else if (pData->revision == 4) {
        pData->qPoint1        = (frsData[7] >> 0) & 0xFFFF;
        pData->qPoint2        = (frsData[7] >> 16) & 0xFFFF;
        pData->sensorSpecificLen = (frsData[8] >> 0) & 0xFFFF;
        pData->qPoint3        = (frsData[8] >> 16) & 0xFFFF;
        pData->maxPeriod_uS   = frsData[9];
        memcpy(pData->sensorSpecific, (uint8_t *)&frsData[10], pData->sensorSpecificLen);
        vendorIdOffset = 10 + ((pData->sensorSpecificLen+3)/4); // 9 + one word for every 4 bytes of SS data
    }
    else {
        // Unrecognized revision!
    }

    // Copy vendor id
    memcpy(pData->vendorId, (uint8_t *)&frsData[vendorIdOffset],
           pData->vendorIdLen);
}

// ------------------------------------------------------------------------
// Set FRS.

static int setFrsStart(sh2_t *pSh2)
{
    int rc = SH2_OK;
    FrsWriteReq_t req;

    pSh2->opData.setFrs.offset = 0;
    
    // set up request to issue
    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_FRS_WRITE_REQ;
    req.reserved = 0;
    req.length = pSh2->opData.setFrs.words;
    req.frsType = pSh2->opData.getFrs.frsType;

    rc = sendCtrl(pSh2, (uint8_t *)&req, sizeof(req));

    return rc;
}

static void setFrsRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    FrsWriteResp_t *resp = (FrsWriteResp_t *)payload;
    FrsWriteDataReq_t req;
    uint8_t status;
    bool sendMoreData = false;
    bool completed = false;
    int rc = SH2_OK;

    // skip this if it isn't the response we're looking for.
    if (resp->reportId != SENSORHUB_FRS_WRITE_RESP) return;

    // Check for errors: Unrecognized FRS type, Busy, Out of range, Device error
    status = resp->status;
    switch(status) {
        case FRS_WRITE_STATUS_RECEIVED:
        case FRS_WRITE_STATUS_READY:
            sendMoreData = true;
            break;
        case FRS_WRITE_STATUS_UNRECOGNIZED_FRS_TYPE:
        case FRS_WRITE_STATUS_BUSY:
        case FRS_WRITE_STATUS_FAILED:
        case FRS_WRITE_STATUS_NOT_READY:
        case FRS_WRITE_STATUS_INVALID_LENGTH:
        case FRS_WRITE_STATUS_INVALID_RECORD:
        case FRS_WRITE_STATUS_DEVICE_ERROR:
        case FRS_WRITE_STATUS_READ_ONLY:
            rc = SH2_ERR_HUB;
            completed = true;
            break;
        case FRS_WRITE_STATUS_WRITE_COMPLETED:
            // Successful completion
            rc = SH2_OK;
            completed = true;
            break;
        case FRS_WRITE_STATUS_RECORD_VALID:
            // That's nice, keep waiting
            break;
    }

    // if we should send more data, do it.
    if (sendMoreData &&
        (pSh2->opData.setFrs.offset < pSh2->opData.setFrs.words)) {
        uint16_t offset = pSh2->opData.setFrs.offset;
        
        memset(&req, 0, sizeof(req));
        req.reportId = SENSORHUB_FRS_WRITE_DATA_REQ;
        req.reserved = 0;
        req.offset = offset;
        req.data0 = pSh2->opData.setFrs.pData[offset++];
        if (offset < pSh2->opData.setFrs.words) {
            req.data1 = pSh2->opData.setFrs.pData[offset++];
        } else {
            req.data1 = 0;
        }
        pSh2->opData.setFrs.offset = offset;
        
        rc = sendCtrl(pSh2, (uint8_t *)&req, sizeof(req));
    }

    // if the operation is done or has to be aborted, complete it
    if (completed) {
        opCompleted(pSh2, rc);
    }

    return;
}

const sh2_Op_t setFrsOp = {
    .start = setFrsStart,
    .rx = setFrsRx,
};

// ------------------------------------------------------------------------
// Support for sending commands

static int sendCmd(sh2_t *pSh2, uint8_t cmd, uint8_t p[COMMAND_PARAMS])
{
    int rc = SH2_OK;
    CommandReq_t req;

    // Clear request structure
    memset(&req, 0, sizeof(req));
    
    // Create a command sequence number for this command
    pSh2->lastCmdId = cmd;
    pSh2->cmdSeq = pSh2->nextCmdSeq++;
    
    // set up request to issue
    req.reportId = SENSORHUB_COMMAND_REQ;
    req.seq = pSh2->cmdSeq;
    req.command = cmd;
    for (int n = 0; n < COMMAND_PARAMS; n++) {
        req.p[n] = p[n];
    }
    
    rc = sendCtrl(pSh2, (uint8_t *)&req, sizeof(req));
    
    return rc;
}

// Send a command with 0 parameters
static int sendCmd0(sh2_t *pSh2, uint8_t cmd)
{
    uint8_t p[COMMAND_PARAMS];

    memset(p, 0, COMMAND_PARAMS);

    return sendCmd(pSh2, cmd, p);
}

// Send a command with 1 parameter
static int sendCmd1(sh2_t *pSh2, uint8_t cmd, uint8_t p0)
{
    uint8_t p[COMMAND_PARAMS];

    memset(p, 0, COMMAND_PARAMS);

    p[0] = p0;
    return sendCmd(pSh2, cmd, p);
}

// Send a command with 2 parameters
static int sendCmd2(sh2_t *pSh2, uint8_t cmd, uint8_t p0, uint8_t p1)
{
    uint8_t p[COMMAND_PARAMS];

    memset(p, 0, COMMAND_PARAMS);
    
    p[0] = p0;
    p[1] = p1;
    return sendCmd(pSh2, cmd, p);
}

static bool wrongResponse(sh2_t *pSh2, CommandResp_t *resp)
{
    if (resp->reportId != SENSORHUB_COMMAND_RESP) return true;
    if (resp->command != pSh2->lastCmdId) return true;
    if (resp->commandSeq != pSh2->cmdSeq) return true;

    return false;
}

// ------------------------------------------------------------------------
// Get Errors

static int getErrorsStart(sh2_t *pSh2)
{
    // Initialize state
    pSh2->opData.getErrors.errsRead = 0;

    return sendCmd1(pSh2, SH2_CMD_ERRORS, pSh2->opData.getErrors.severity);
}

static void getErrorsRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;
    
    // skip this if it isn't the right response
    if (wrongResponse(pSh2, resp)) return;

    if (resp->r[2] == 255) {
        // No error to report, operation is complete
        *(pSh2->opData.getErrors.pNumErrors) = pSh2->opData.getErrors.errsRead;
        opCompleted(pSh2, SH2_OK);
    } else {
        // Copy data for invoker.
        unsigned int index = pSh2->opData.getErrors.errsRead;
        if (index < *(pSh2->opData.getErrors.pNumErrors)) {
            // We have room for this one.
            pSh2->opData.getErrors.pErrors[index].severity = resp->r[0];
            pSh2->opData.getErrors.pErrors[index].sequence = resp->r[1];
            pSh2->opData.getErrors.pErrors[index].source = resp->r[2];
            pSh2->opData.getErrors.pErrors[index].error = resp->r[3];
            pSh2->opData.getErrors.pErrors[index].module = resp->r[4];
            pSh2->opData.getErrors.pErrors[index].code = resp->r[5];

            pSh2->opData.getErrors.errsRead++;
        }
    }

    return;
}

const sh2_Op_t getErrorsOp = {
    .start = getErrorsStart,
    .rx = getErrorsRx,
};

// ------------------------------------------------------------------------
// Get Counts

static int getCountsStart(sh2_t *pSh2)
{
    return sendCmd2(pSh2, SH2_CMD_COUNTS, SH2_COUNTS_GET_COUNTS, pSh2->opData.getCounts.sensorId);
}

static void getCountsRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;

    if (wrongResponse(pSh2, resp)) return;
    
    // Store results
    if (resp->respSeq == 0) {
        pSh2->opData.getCounts.pCounts->offered = readu32(&resp->r[3]);
        pSh2->opData.getCounts.pCounts->accepted = readu32(&resp->r[7]);
    }
    else {
        pSh2->opData.getCounts.pCounts->on = readu32(&resp->r[3]);
        pSh2->opData.getCounts.pCounts->attempted = readu32(&resp->r[7]);
    }
    
    // Complete this operation if we've received last response
    if (resp->respSeq == 1) {
        opCompleted(pSh2, SH2_OK);
    }

    return;
}

const sh2_Op_t getCountsOp = {
    .start = getCountsStart,
    .rx = getCountsRx,
};

// ------------------------------------------------------------------------
// Generic Send Command

static int sendCmdStart(sh2_t *pSh2)
{
    int status = sendCmd(pSh2, pSh2->opData.sendCmd.req.command,
                     pSh2->opData.sendCmd.req.p);

    opCompleted(pSh2, status);

    return status;
}

const sh2_Op_t sendCmdOp = {
    .start = sendCmdStart,
};

// ------------------------------------------------------------------------
// Reinit

static int reinitStart(sh2_t *pSh2)
{
    int status = sendCmd1(pSh2, SH2_CMD_INITIALIZE, SH2_INIT_SYSTEM);

    return status;
}

static void reinitRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;

    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse(pSh2, resp)) return;

    // Get return status
    int status = SH2_OK;
    if (resp->r[0] != 0) {
        status = SH2_ERR_HUB;
    }

    opCompleted(pSh2, status);
}

const sh2_Op_t reinitOp = {
    .start = reinitStart,
    .rx = reinitRx,
};

// ------------------------------------------------------------------------
// Save DCD Now

static int saveDcdNowStart(sh2_t *pSh2)
{
    int status = sendCmd0(pSh2, SH2_CMD_DCD);

    return status;
}

static void saveDcdNowRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;

    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse(pSh2, resp)) return;

    // Get return status
    int status = SH2_OK;
    if (resp->r[0] != 0) {
        status = SH2_ERR_HUB;
    }

    opCompleted(pSh2, status);
}

const sh2_Op_t saveDcdNowOp = {
    .start = saveDcdNowStart,
    .rx = saveDcdNowRx,
};

// ------------------------------------------------------------------------
// Get Osc Type

static int getOscTypeStart(sh2_t *pSh2)
{
    return sendCmd0(pSh2, SH2_CMD_GET_OSC_TYPE);
}

static void getOscTypeRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;
    sh2_OscType_t *pOscType;
    
    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse(pSh2, resp)) return;

    // Read out data
    pOscType = pSh2->opData.getOscType.pOscType;
    *pOscType = (sh2_OscType_t)resp->r[0];

    // Complete this operation
    opCompleted(pSh2, SH2_OK);
}

const sh2_Op_t getOscTypeOp = {
    .start = getOscTypeStart,
    .rx = getOscTypeRx,
};

// ------------------------------------------------------------------------
// Set Cal Config

static int setCalConfigStart(sh2_t *pSh2)
{
    uint8_t p[COMMAND_PARAMS];

    // Clear p.  (Importantly, set subcommand in p[3] to 0, CONFIGURE)
    memset(p, 0, COMMAND_PARAMS);
    
    // Which cal processes to enable/disable
    p[0] = (pSh2->opData.calConfig.sensors & SH2_CAL_ACCEL) ? 1 : 0; // accel cal
    p[1] = (pSh2->opData.calConfig.sensors & SH2_CAL_GYRO)  ? 1 : 0; // gyro cal
    p[2] = (pSh2->opData.calConfig.sensors & SH2_CAL_MAG)   ? 1 : 0; // mag cal
    p[4] = (pSh2->opData.calConfig.sensors & SH2_CAL_PLANAR) ? 1 : 0; // planar cal
    
    return sendCmd(pSh2, SH2_CMD_ME_CAL, p);
}

static void setCalConfigRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;
    
    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse(pSh2, resp)) return;

    // Read out data
    int status = SH2_OK;
    if (resp->r[0] != 0) {
        status = SH2_ERR_HUB;
    }

    // Complete this operation
    opCompleted(pSh2, status);
}

const sh2_Op_t setCalConfigOp = {
    .start = setCalConfigStart,
    .rx = setCalConfigRx,
};

// ------------------------------------------------------------------------
// Get Cal Config

static int getCalConfigStart(sh2_t *pSh2)
{
    uint8_t p[COMMAND_PARAMS];

    // Clear p.  (Importantly, set subcommand in p[3] to 0, CONFIGURE)
    memset(p, 0, COMMAND_PARAMS);
    
    // Subcommand: Get ME Calibration
    p[3] = 0x01;
    
    return sendCmd(pSh2, SH2_CMD_ME_CAL, p);
}

static void getCalConfigRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;
    
    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse(pSh2, resp)) return;

    // Read out data
    int status = SH2_OK;
    if (resp->r[0] != 0) {
        status = SH2_ERR_HUB;
    }
    else {
        // Unload results into pSensors
        uint8_t sensors = 0;
        if (resp->r[1]) sensors |= SH2_CAL_ACCEL;
        if (resp->r[2]) sensors |= SH2_CAL_GYRO;
        if (resp->r[3]) sensors |= SH2_CAL_MAG;
        if (resp->r[4]) sensors |= SH2_CAL_PLANAR;
        *(pSh2->opData.getCalConfig.pSensors) = sensors;
    }
    
    // Complete this operation
    opCompleted(pSh2, status);
}


const sh2_Op_t getCalConfigOp = {
    .start = getCalConfigStart,
    .rx = getCalConfigRx,
};

// ------------------------------------------------------------------------
// Force Flush

static int forceFlushStart(sh2_t *pSh2)
{
    ForceFlushReq_t req;

    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_FORCE_SENSOR_FLUSH;
    req.sensorId = pSh2->opData.forceFlush.sensorId;
    
    return sendCtrl(pSh2, (uint8_t *)&req, sizeof(req));
}

static void forceFlushRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    ForceFlushResp_t *resp = (ForceFlushResp_t *)payload;
    
    // Ignore message if it doesn't pertain to this operation
    if (resp->reportId != SENSORHUB_FLUSH_COMPLETED) return;
    if (resp->sensorId != pSh2->opData.forceFlush.sensorId) return;

    // Complete this operation
    opCompleted(pSh2, SH2_OK);
}

const sh2_Op_t forceFlushOp = {
    .start = forceFlushStart,
    .rx = forceFlushRx,
};

// ------------------------------------------------------------------------
// Start Cal

static int startCalStart(sh2_t *pSh2)
{
    uint8_t p[COMMAND_PARAMS];

    // Clear p.  (Importantly, set subcommand in p[3] to 0, CONFIGURE)
    memset(p, 0, COMMAND_PARAMS);
    
    // Subcommand: Get ME Calibration
    p[0] = SH2_CAL_START;
    p[1] = pSh2->opData.startCal.interval_us & 0xFF;          // LSB
    p[2] = (pSh2->opData.startCal.interval_us >> 8) & 0xFF;
    p[3] = (pSh2->opData.startCal.interval_us >> 16) & 0xFF;
    p[4] = (pSh2->opData.startCal.interval_us >> 24) & 0xFF;  // MSB
    
    return sendCmd(pSh2, SH2_CMD_CAL, p);
}

static void startCalRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;
    
    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse(pSh2, resp)) return;

    // Complete this operation
    opCompleted(pSh2, SH2_OK);
}

const sh2_Op_t startCalOp = {
    .start = startCalStart,
    .rx = startCalRx,
};

// ------------------------------------------------------------------------
// Start Cal

static int finishCalStart(sh2_t *pSh2)
{
    return sendCmd1(pSh2, SH2_CMD_CAL, SH2_CAL_FINISH);
}

static void finishCalRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;
    
    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse(pSh2, resp)) return;

    pSh2->opData.finishCal.status = (sh2_CalStatus_t)resp->r[1];

    // Complete this operation
    if (pSh2->opData.finishCal.status == SH2_CAL_SUCCESS) {
        opCompleted(pSh2, SH2_OK);
    }
    else {
        opCompleted(pSh2, SH2_ERR_HUB);
    }
}

const sh2_Op_t finishCalOp = {
    .start = finishCalStart,
    .rx = finishCalRx,
};


// ------------------------------------------------------------------------
// SHTP Event Callback

static void shtpEventCallback(void *cookie, shtp_Event_t shtpEvent) {
    sh2_t *pSh2 = &_sh2;

    sh2AsyncEvent.eventId = SH2_SHTP_EVENT;
    sh2AsyncEvent.shtpEvent = shtpEvent;
    if (pSh2->eventCallback) {
        pSh2->eventCallback(pSh2->eventCookie, &sh2AsyncEvent);
    }
}

// ------------------------------------------------------------------------
// Public functions

/**
 * @brief Open a session with a sensor hub.
 *
 * This function should be calLED before others in this API.
 * An instance of an SH2 HAL should be passed in.
 * This call will result in the open() function of the HAL being calLED.
 *
 * As part of the initialization process, a callback function is registered that will
 * be invoked when the device generates certain events.  (See sh2_AsyncEventId)
 *
 * @param pHal Pointer to an SH2 HAL instance, provided by the target system.
 * @param  eventCallback Will be calLED when events, such as reset complete, occur.
 * @param  eventCookie Will be passed to eventCallback.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_open(sh2_Hal_t *pHal,
             sh2_EventCallback_t *eventCallback, void *eventCookie)
{
    sh2_t *pSh2 = &_sh2;
    
    // Validate parameters
    if (pHal == 0) return SH2_ERR_BAD_PARAM;

    // Clear everything in sh2 structure.
    memset(&_sh2, 0, sizeof(_sh2));
        
    pSh2->resetComplete = false;  // will go true after reset response from SH.
    pSh2->controlChan = 0xFF;  // An invalid value since we don't know yet.
    
    // Store reference to HAL for future use.
    pSh2->pHal = pHal;
    pSh2->eventCallback = eventCallback;
    pSh2->eventCookie = eventCookie;
    pSh2->sensorCallback = 0;
    pSh2->sensorCookie = 0;

    // Open SHTP layer
    pSh2->pShtp = shtp_open(pSh2->pHal);
    if (pSh2->pShtp == 0) {
        // Error opening SHTP
        return SH2_ERR;
    }

    // Register SHTP event callback
    shtp_setEventCallback(pSh2->pShtp, shtpEventCallback, &_sh2);

    // Register with SHTP
    // Register SH2 handlers
    shtp_listenAdvert(pSh2->pShtp, GUID_SENSORHUB, sensorhubAdvertHdlr, &_sh2);
    shtp_listenChan(pSh2->pShtp, GUID_SENSORHUB, "control", sensorhubControlHdlr, &_sh2);
    shtp_listenChan(pSh2->pShtp, GUID_SENSORHUB, "inputNormal", sensorhubInputNormalHdlr, &_sh2);
    shtp_listenChan(pSh2->pShtp, GUID_SENSORHUB, "inputWake", sensorhubInputWakeHdlr, &_sh2);
    shtp_listenChan(pSh2->pShtp, GUID_SENSORHUB, "inputGyroRv", sensorhubInputGyroRvHdlr, &_sh2);

    // Register EXECUTABLE handlers
    shtp_listenAdvert(pSh2->pShtp, GUID_EXECUTABLE, executableAdvertHdlr, &_sh2);
    shtp_listenChan(pSh2->pShtp, GUID_EXECUTABLE, "device", executabLEDeviceHdlr, &_sh2);

    // Wait for reset notifications to arrive.
    // The client can't talk to the sensor hub until that happens.
    uint32_t start_us = pSh2->pHal->getTimeUs(pSh2->pHal);
    uint32_t now_us = start_us;
    while (((now_us - start_us) < ADVERT_TIMEOUT_US) &&
           (!pSh2->resetComplete))
    {
        shtp_service(pSh2->pShtp);
        now_us = pSh2->pHal->getTimeUs(pSh2->pHal);
    }
    
    // No errors.
    return SH2_OK;
}

/**
 * @brief Close a session with a sensor hub.
 *
 * This should be calLED at the end of a sensor hub session.  
 * The underlying SHTP and HAL instances will be closed.
 */
void sh2_close(void)
{
    sh2_t *pSh2 = &_sh2;
    
    shtp_close(pSh2->pShtp);

    // Clear everything in sh2 structure.
    memset(pSh2, 0, sizeof(sh2_t));
}

/**
 * @brief Service the SH2 device, reading any data that is available and dispatching callbacks.
 *
 * This function should be calLED periodically by the host system to service an open sensor hub.
 */
void sh2_service(void)
{
    sh2_t *pSh2 = &_sh2;
    
    shtp_service(pSh2->pShtp);
}

/**
 * @brief Register a function to receive sensor events.
 *
 * @param  callback A function that will be calLED each time a sensor event is received.
 * @param  cookie  A value that will be passed to the sensor callback function.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setSensorCallback(sh2_SensorCallback_t *callback, void *cookie)
{
    sh2_t *pSh2 = &_sh2;
    
    pSh2->sensorCallback = callback;
    pSh2->sensorCookie = cookie;

    return SH2_OK;
}

/**
 * @brief Reset the sensor hub device by sending RESET (1) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devReset(void)
{
    sh2_t *pSh2 = &_sh2;

    return sendExecutable(pSh2, EXECUTABLE_DEVICE_CMD_RESET);
}

/**
 * @brief Turn sensor hub on by sending RESET (1) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devOn(void)
{
    sh2_t *pSh2 = &_sh2;

    return sendExecutable(pSh2, EXECUTABLE_DEVICE_CMD_ON);
}

/**
 * @brief Put sensor hub in sleep state by sending SLEEP (2) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devSleep(void)
{
    sh2_t *pSh2 = &_sh2;

    return sendExecutable(pSh2, EXECUTABLE_DEVICE_CMD_SLEEP);
}

/**
 * @brief Get Product ID information from Sensorhub.
 *
 * @param  prodIds Pointer to structure that will receive results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getProdIds(sh2_ProductIds_t *prodIds)
{
    sh2_t *pSh2 = &_sh2;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    pSh2->opData.getProdIds.pProdIds = prodIds;

    return opProcess(pSh2, &getProdIdOp);
}

/**
 * @brief Get sensor configuration.
 *
 * @param  sensorId Which sensor to query.
 * @param  config SensorConfig structure to store results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getSensorConfig(sh2_SensorId_t sensorId, sh2_SensorConfig_t *pConfig)
{
    sh2_t *pSh2 = &_sh2;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    // Set up operation
    pSh2->opData.getSensorConfig.sensorId = sensorId;
    pSh2->opData.getSensorConfig.pConfig = pConfig;

    return opProcess(pSh2, &getSensorConfigOp);
}

/**
 * @brief Set sensor configuration. (e.g enable a sensor at a particular rate.)
 *
 * @param  sensorId Which sensor to configure.
 * @param  pConfig Pointer to structure holding sensor configuration.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setSensorConfig(sh2_SensorId_t sensorId, const sh2_SensorConfig_t *pConfig)
{
    sh2_t *pSh2 = &_sh2;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    // Set up operation
    pSh2->opData.setSensorConfig.sensorId = sensorId;
    pSh2->opData.setSensorConfig.pConfig = pConfig;

    return opProcess(pSh2, &setSensorConfigOp);
}

/**
 * @brief Get metadata related to a sensor.
 *
 * @param  sensorId Which sensor to query.
 * @param  pData Pointer to structure to receive the results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getMetadata(sh2_SensorId_t sensorId, sh2_SensorMetadata_t *pData)
{
    sh2_t *pSh2 = &_sh2;
    
    // pData must be non-null
    if (pData == 0) return SH2_ERR_BAD_PARAM;
  
    // Convert sensorId to metadata recordId
    int i;
    for (i = 0; i < ARRAY_LEN(sensorToRecordMap); i++) {
        if (sensorToRecordMap[i].sensorId == sensorId) {
            break;
        }
    }
    if (i >= ARRAY_LEN(sensorToRecordMap)) {
        // no match was found
        return SH2_ERR_BAD_PARAM;
    }
    uint16_t recordId = sensorToRecordMap[i].recordId;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    // Set up an FRS read operation
    pSh2->opData.getFrs.frsType = recordId;
    pSh2->opData.getFrs.pData = pSh2->frsData;
    pSh2->frsDataLen = ARRAY_LEN(pSh2->frsData);
    pSh2->opData.getFrs.pWords = &pSh2->frsDataLen;

    // Read an FRS record
    int status = opProcess(pSh2, &getFrsOp);
    
    // Copy the results into pData
    if (status == SH2_OK) {
        stuffMetadata(pData, pSh2->frsData);
    }

    return status;
}

/**
 * @brief Get an FRS record.
 *
 * @param  recordId Which FRS Record to retrieve.
 * @param  pData pointer to buffer to receive the results
 * @param[in] words Size of pData buffer, in 32-bit words.
 * @param[out] words Number of 32-bit words retrieved.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getFrs(uint16_t recordId, uint32_t *pData, uint16_t *words)
{
    sh2_t *pSh2 = &_sh2;
    
    if ((pData == 0) || (words == 0)) {
        return SH2_ERR_BAD_PARAM;
    }
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    // Store params for this op
    pSh2->opData.getFrs.frsType = recordId;
    pSh2->opData.getFrs.pData = pData;
    pSh2->opData.getFrs.pWords = words;

    return opProcess(pSh2, &getFrsOp);
}

/**
 * @brief Set an FRS record
 *
 * @param  recordId Which FRS Record to set.
 * @param  pData pointer to buffer containing the new data.
 * @param  words number of 32-bit words to write.  (0 to delete record.)
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setFrs(uint16_t recordId, uint32_t *pData, uint16_t words)
{
    sh2_t *pSh2 = &_sh2;
    
    if ((pData == 0) && (words != 0)) {
        return SH2_ERR_BAD_PARAM;
    }
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    pSh2->opData.setFrs.frsType = recordId;
    pSh2->opData.setFrs.pData = pData;
    pSh2->opData.setFrs.words = words;

    return opProcess(pSh2, &setFrsOp);
}

/**
 * @brief Get error counts.
 *
 * @param  severity Only errors of this severity or greater are returned.
 * @param  pErrors Buffer to receive error codes.
 * @param  numErrors size of pErrors array
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getErrors(uint8_t severity, sh2_ErrorRecord_t *pErrors, uint16_t *numErrors)
{
    sh2_t *pSh2 = &_sh2;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    pSh2->opData.getErrors.severity = severity;
    pSh2->opData.getErrors.pErrors = pErrors;
    pSh2->opData.getErrors.pNumErrors = numErrors;
    
    return opProcess(pSh2, &getErrorsOp);
}

/**
 * @brief Read counters related to a sensor.
 *
 * @param  sensorId Which sensor to operate on.
 * @param  pCounts Pointer to Counts structure that will receive data.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getCounts(sh2_SensorId_t sensorId, sh2_Counts_t *pCounts)
{
    sh2_t *pSh2 = &_sh2;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    pSh2->opData.getCounts.sensorId = sensorId;
    pSh2->opData.getCounts.pCounts = pCounts;
    
    return opProcess(pSh2, &getCountsOp);
}

/**
 * @brief Clear counters related to a sensor.
 *
 * @param  sensorId which sensor to operate on.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_clearCounts(sh2_SensorId_t sensorId)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    
    pSh2->opData.sendCmd.req.command = SH2_CMD_COUNTS;
    pSh2->opData.sendCmd.req.p[0] = SH2_COUNTS_CLEAR_COUNTS;
    pSh2->opData.sendCmd.req.p[1] = sensorId;

    return opProcess(pSh2, &sendCmdOp);
}

/**
 * @brief Perform a tare operation on one or more axes.
 *
 * @param  axes Bit mask specifying which axes should be tared.
 * @param  basis Which rotation vector to use as the basis for Tare adjustment.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setTareNow(uint8_t axes,    // SH2_TARE_X | SH2_TARE_Y | SH2_TARE_Z
                   sh2_TareBasis_t basis)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    
    pSh2->opData.sendCmd.req.command = SH2_CMD_TARE;
    pSh2->opData.sendCmd.req.p[0] = SH2_TARE_TARE_NOW;
    pSh2->opData.sendCmd.req.p[1] = axes;
    pSh2->opData.sendCmd.req.p[2] = basis;

    return opProcess(pSh2, &sendCmdOp);
}

/**
 * @brief Clears the previously applied tare operation.
 *
 * @return SH2_OK \n");
 */
int sh2_clearTare(void)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    
    pSh2->opData.sendCmd.req.command = SH2_CMD_TARE;
    pSh2->opData.sendCmd.req.p[0] = SH2_TARE_SET_REORIENTATION;

    return opProcess(pSh2, &sendCmdOp);
}

/**
 * @brief Persist the results of last tare operation to flash.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_persistTare(void)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    
    pSh2->opData.sendCmd.req.command = SH2_CMD_TARE;
    pSh2->opData.sendCmd.req.p[0] = SH2_TARE_PERSIST_TARE;

    return opProcess(pSh2, &sendCmdOp);
}

/**
 * @brief Set the current run-time sensor reorientation. (Set to zero to clear tare.)
 *
 * @param  orientation Quaternion rotation vector to apply as new tare.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setReorientation(sh2_Quaternion_t *orientation)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    
    pSh2->opData.sendCmd.req.command = SH2_CMD_TARE;
    pSh2->opData.sendCmd.req.p[0] = SH2_TARE_SET_REORIENTATION;

    // save me a lot of typing and you a lot of reading
    uint8_t *p = pSh2->opData.sendCmd.req.p;

    // Put new orientation in command parameters
    writeu16(&p[1], toQ14(orientation->x));
    writeu16(&p[3], toQ14(orientation->y));
    writeu16(&p[5], toQ14(orientation->z));
    writeu16(&p[7], toQ14(orientation->w));

    return opProcess(pSh2, &sendCmdOp);
}

/**
 * @brief Command the sensorhub to reset.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_reinitialize(void)
{
    sh2_t *pSh2 = &_sh2;

    return opProcess(pSh2, &reinitOp);
}

/**
 * @brief Save Dynamic Calibration Data to flash.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_saveDcdNow(void)
{
    sh2_t *pSh2 = &_sh2;

    return opProcess(pSh2, &saveDcdNowOp);
}

/**
 * @brief Get Oscillator type.
 *
 * @param  pOscType pointer to data structure to receive results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getOscType(sh2_OscType_t *pOscType)
{
    sh2_t *pSh2 = &_sh2;

    pSh2->opData.getOscType.pOscType = pOscType;

    return opProcess(pSh2, &getOscTypeOp);
}

/**
 * @brief Enable/Disable dynamic calibration for certain sensors
 *
 * @param  sensors Bit mask to configure which sensors are affected.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setCalConfig(uint8_t sensors)
{
    sh2_t *pSh2 = &_sh2;

    pSh2->opData.calConfig.sensors = sensors;

    return opProcess(pSh2, &setCalConfigOp);
}

/**
 * @brief Get dynamic calibration configuration settings.
 *
 * @param  pSensors pointer to Bit mask, set on return.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getCalConfig(uint8_t *pSensors)
{
    sh2_t *pSh2 = &_sh2;

    pSh2->opData.getCalConfig.pSensors = pSensors;

    return opProcess(pSh2, &getCalConfigOp);
}

/**
 * @brief Configure automatic saving of dynamic calibration data.
 *
 * @param  enabLED Enable or Disable DCD auto-save.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setDcdAutoSave(bool enabLED)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    pSh2->opData.sendCmd.req.command = SH2_CMD_DCD_SAVE;
    pSh2->opData.sendCmd.req.p[0] = enabLED ? 0 : 1;

    return opProcess(pSh2, &sendCmdOp);
}

/**
 * @brief Immediately issue all buffered sensor reports from a given sensor.
 *
 * @param  sensorId Which sensor reports to flush.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_flush(sh2_SensorId_t sensorId)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    pSh2->opData.forceFlush.sensorId = sensorId;

    return opProcess(pSh2, &forceFlushOp);
}

/**
 * @brief Command clear DCD in RAM, then reset sensor hub.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_clearDcdAndReset(void)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    pSh2->opData.sendCmd.req.command = SH2_CMD_CLEAR_DCD_AND_RESET;

    return opProcess(pSh2, &sendCmdOp);
}

/**
 * @brief Start simple self-calibration procedure.
 *
 * @parameter interval_us sensor report interval, uS.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_startCal(uint32_t interval_us)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    pSh2->opData.startCal.interval_us = interval_us;

    return opProcess(pSh2, &startCalOp);
}

/**
 * @brief Finish simple self-calibration procedure.
 *
 * @parameter status contains calibration status code on return.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_finishCal(sh2_CalStatus_t *status)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    return opProcess(pSh2, &finishCalOp);
}

/**
 * @brief send Interactive ZRO Request.
 *
 * @parameter intent Inform the sensor hub what sort of motion should be in progress.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setIZro(sh2_IZroMotionIntent_t intent)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));

    // set up opData for iZRO request
    pSh2->opData.sendCmd.req.command = SH2_CMD_INTERACTIVE_ZRO;
    pSh2->opData.sendCmd.req.p[0] = intent;

    // Send command
    return opProcess(pSh2, &sendCmdOp);
}
