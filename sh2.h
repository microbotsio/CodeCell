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
 * @file sh2.h
 * @author David Wheeler
 * @date 22 Sept 2015
 * @brief API Definition for Hillcrest SH-2 Sensor Hub.
 *
 * The sh2 API provides an interface to the Hillcrest Labs sensor hub devices.
 */

#ifndef SH2_H
#define SH2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "sh2_hal.h"

/***************************************************************************************
 * Public type definitions
 ***************************************************************************************/

/**
 * @brief Sensor Event
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define SH2_MAX_SENSOR_EVENT_LEN (16)
typedef struct sh2_SensorEvent {
    uint64_t timestamp_uS;
    uint8_t len;
    uint8_t reportId;
    uint8_t report[SH2_MAX_SENSOR_EVENT_LEN];
} sh2_SensorEvent_t;

typedef void (sh2_SensorCallback_t)(void * cookie, sh2_SensorEvent_t *pEvent);

/**
 * @brief Product Id value
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_ProductId_s {
    uint8_t resetCause;
    uint8_t swVersionMajor;
    uint8_t swVersionMinor;
    uint32_t swPartNumber;
    uint32_t swBuildNumber;
    uint16_t swVersionPatch;
    uint8_t reserved0;
    uint8_t reserved1;
} sh2_ProductId_t;

#define SH2_MAX_PROD_ID_ENTRIES (5)
typedef struct sh2_ProductIds_s {
    sh2_ProductId_t entry[SH2_MAX_PROD_ID_ENTRIES];
    uint8_t numEntries;
} sh2_ProductIds_t;

/**
 * @brief List of sensor types supported by the hub
 *
 * See the SH-2 Reference Manual for more information on each type.
 */
enum sh2_SensorId_e {
    SH2_RAW_ACCELEROMETER = 0x14,
    SH2_ACCELEROMETER = 0x01,
    SH2_LINEAR_ACCELERATION = 0x04,
    SH2_GRAVITY = 0x06,
    SH2_RAW_GYROSCOPE = 0x15,
    SH2_GYROSCOPE_CALIBRATED = 0x02,
    SH2_GYROSCOPE_UNCALIBRATED = 0x07,
    SH2_RAW_MAGNETOMETER = 0x16,
    SH2_MAGNETIC_FIELD_CALIBRATED = 0x03,
    SH2_MAGNETIC_FIELD_UNCALIBRATED = 0x0f,
    SH2_ROTATION_VECTOR = 0x05,
    SH2_GAME_ROTATION_VECTOR = 0x08,
    SH2_GEOMAGNETIC_ROTATION_VECTOR = 0x09,
    SH2_PRESSURE = 0x0a,
    SH2_AMBIENT_LIGHT = 0x0b,
    SH2_HUMIDITY = 0x0c,
    SH2_PROXIMITY = 0x0d,
    SH2_TEMPERATURE = 0x0e,
    SH2_RESERVED = 0x17,
    SH2_TAP_DETECTOR = 0x10,
    SH2_STEP_DETECTOR = 0x18,
    SH2_STEP_COUNTER = 0x11,
    SH2_SIGNIFICANT_MOTION = 0x12,
    SH2_STABILITY_CLASSIFIER = 0x13,
    SH2_SHAKE_DETECTOR = 0x19,
    SH2_FLIP_DETECTOR = 0x1a,
    SH2_PICKUP_DETECTOR = 0x1b,
    SH2_STABILITY_DETECTOR = 0x1c,
    SH2_PERSONAL_ACTIVITY_CLASSIFIER = 0x1e,
    SH2_SLEEP_DETECTOR = 0x1f,
    SH2_TILT_DETECTOR = 0x20,
    SH2_POCKET_DETECTOR = 0x21,
    SH2_CIRCLE_DETECTOR = 0x22,
    SH2_HEART_RATE_MONITOR = 0x23,
    SH2_ARVR_STABILIZED_RV = 0x28,
    SH2_ARVR_STABILIZED_GRV = 0x29,
    SH2_GYRO_INTEGRATED_RV = 0x2A,
    SH2_IZRO_MOTION_REQUEST = 0x2B,

    // UPDATE to reflect greatest sensor id
    SH2_MAX_SENSOR_ID = 0x2B,
};
typedef uint8_t sh2_SensorId_t;

/**
 * @brief Sensor Configuration settings
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_SensorConfig {
    /* Change sensitivity enabLED */
    bool changeSensitivityEnabled;  /**< @brief Enable reports on change */

    /* Change sensitivity - true if relative; false if absolute */
    bool changeSensitivityRelative;  /**< @brief Change reports relative (vs absolute) */

    /* Wake-up enabLED */
    bool wakeupEnabled;  /**< @brief Wake host on event */

    /* Always on enabLED */
    bool alwaysOnEnabled;  /**< @brief Sensor remains on in sleep state */
    /* 16-bit signed fixed point integer representing the value a
     * sensor output must exceed in order to trigger another input
     * report. A setting of 0 causes all reports to be sent.
     */
    uint16_t changeSensitivity;  /**< @brief Report-on-change threshold */

    /* Interval in microseconds between asynchronous input reports. */
    uint32_t reportInterval_us;  /**< @brief [uS] Report interval */

    /* Reserved field, not used. */
    uint32_t batchInterval_us;  /**< @brief [uS] Batch interval */

    /* Meaning is sensor specific */
    uint32_t sensorSpecific;  /**< @brief See SH-2 Reference Manual for details. */
} sh2_SensorConfig_t;

/**
 * @brief Sensor Metadata Record
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_SensorMetadata {
    uint8_t meVersion;   /**< @brief Motion Engine Version */
    uint8_t mhVersion;  /**< @brief Motion Hub Version */
    uint8_t shVersion;  /**< @brief SensorHub Version */
    uint32_t range;  /**< @brief Same units as sensor reports */
    uint32_t resolution;  /**< @brief Same units as sensor reports */
    uint16_t revision;  /**< @brief Metadata record format revision */
    uint16_t power_mA;    /**< @brief [mA] Fixed point 16Q10 format */
    uint32_t minPeriod_uS;  /**< @brief [uS] */
    uint32_t maxPeriod_uS;  /**< @brief [uS] */
    uint32_t fifoReserved;  /**< @brief (Unused) */
    uint32_t fifoMax;  /**< @brief (Unused) */
    uint32_t batchBufferBytes;  /**< @brief (Unused) */
    uint16_t qPoint1;     /**< @brief q point for sensor values */
    uint16_t qPoint2;     /**< @brief q point for accuracy or bias fields */
    uint16_t qPoint3;     /**< @brief q point for sensor data change sensitivity */
    uint32_t vendorIdLen; /**< @brief [bytes] */
    char vendorId[48];  /**< @brief Vendor name and part number */
    uint32_t sensorSpecificLen;  /**< @brief [bytes] */
    uint8_t sensorSpecific[48];  /**< @brief See SH-2 Reference Manual */
} sh2_SensorMetadata_t;

/**
 * @brief SensorHub Error Record
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_ErrorRecord {
    uint8_t severity;   /**< @brief Error severity, 0: most severe. */
    uint8_t sequence;   /**< @brief Sequence number (by severity) */
    uint8_t source;     /**< @brief 1-MotionEngine, 2-MotionHub, 3-SensorHub, 4-Chip  */
    uint8_t error;      /**< @brief See SH-2 Reference Manual */
    uint8_t module;     /**< @brief See SH-2 Reference Manual */
    uint8_t code;       /**< @brief See SH-2 Reference Manual */
} sh2_ErrorRecord_t;

/**
 * @brief SensorHub Counter Record
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Counts {
    uint32_t offered;   /**< @brief [events] */
    uint32_t accepted;  /**< @brief [events] */
    uint32_t on;        /**< @brief [events] */
    uint32_t attempted; /**< @brief [events] */
} sh2_Counts_t;

/**
 * @brief Values for specifying tare basis
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef enum sh2_TareBasis {
    SH2_TARE_BASIS_ROTATION_VECTOR = 0,             /**< @brief Use Rotation Vector */
    SH2_TARE_BASIS_GAMING_ROTATION_VECTOR = 1,      /**< @brief Use Game Rotation Vector */
    SH2_TARE_BASIS_GEOMAGNETIC_ROTATION_VECTOR = 2, /**< @brief Use Geomagnetic R.V. */
} sh2_TareBasis_t;

/**
 * @brief Bit Fields for specifying tare axes.
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef enum sh2_TareAxis {
    SH2_TARE_X = 1,  /**< @brief sh2_tareNow() axes bit field */
    SH2_TARE_Y = 2,  /**< @brief sh2_tareNow() axes bit field */
    SH2_TARE_Z = 4,  /**< @brief sh2_tareNow() axes bit field */
} sh2_TareAxis_t;

/**
 * @brief Quaternion (double precision floating point representation.)
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Quaternion {
    double x;
    double y;
    double z;
    double w;
} sh2_Quaternion_t;

/**
 * @brief Oscillator type: Internal or External
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef enum {
    SH2_OSC_INTERNAL    = 0,
    SH2_OSC_EXT_CRYSTAL = 1,
    SH2_OSC_EXT_CLOCK   = 2,
} sh2_OscType_t;

/**
 * @brief Calibration result
 *
 * See the SH-2 Reference Manual, Finish Calibration Response.
 */
typedef enum {
    SH2_CAL_SUCCESS = 0,
    SH2_CAL_NO_ZRO,
    SH2_CAL_NO_STATIONARY_DETECTION,
    SH2_CAL_ROTATION_OUTSIDE_SPEC,
    SH2_CAL_ZRO_OUTSIDE_SPEC,
    SH2_CAL_ZGO_OUTSIDE_SPEC,
    SH2_CAL_GYRO_GAIN_OUTSIDE_SPEC,
    SH2_CAL_GYRO_PERIOD_OUTSIDE_SPEC,
    SH2_CAL_GYRO_DROPS_OUTSIDE_SPEC,
} sh2_CalStatus_t;

// FRS Record Ids
#define STATIC_CALIBRATION_AGM                   (0x7979)
#define NOMINAL_CALIBRATION                      (0x4D4D)
#define STATIC_CALIBRATION_SRA                   (0x8A8A)
#define NOMINAL_CALIBRATION_SRA                  (0x4E4E)
#define DYNAMIC_CALIBRATION                      (0x1F1F)
#define ME_POWER_MGMT                            (0xD3E2)
#define SYSTEM_ORIENTATION                       (0x2D3E)
#define ACCEL_ORIENTATION                        (0x2D41)
#define SCREEN_ACCEL_ORIENTATION                 (0x2D43)
#define GYROSCOPE_ORIENTATION                    (0x2D46)
#define MAGNETOMETER_ORIENTATION                 (0x2D4C)
#define ARVR_STABILIZATION_RV                    (0x3E2D)
#define ARVR_STABILIZATION_GRV                   (0x3E2E)
#define TAP_DETECT_CONFIG                        (0xC269)
#define SIG_MOTION_DETECT_CONFIG                 (0xC274)
#define SHAKE_DETECT_CONFIG                      (0x7D7D)
#define MAX_FUSION_PERIOD                        (0xD7D7)
#define SERIAL_NUMBER                            (0x4B4B)
#define ES_PRESSURE_CAL                          (0x39AF)
#define ES_TEMPERATURE_CAL                       (0x4D20)
#define ES_HUMIDITY_CAL                          (0x1AC9)
#define ES_AMBIENT_LIGHT_CAL                     (0x39B1)
#define ES_PROXIMITY_CAL                         (0x4DA2)
#define ALS_CAL                                  (0xD401)
#define PROXIMITY_SENSOR_CAL                     (0xD402)
#define PICKUP_DETECTOR_CONFIG                   (0x1B2A)
#define FLIP_DETECTOR_CONFIG                     (0xFC94)
#define STABILITY_DETECTOR_CONFIG                (0xED85)
#define ACTIVITY_TRACKER_CONFIG                  (0xED88)
#define SLEEP_DETECTOR_CONFIG                    (0xED87)
#define TILT_DETECTOR_CONFIG                     (0xED89)
#define POCKET_DETECTOR_CONFIG                   (0xEF27)
#define CIRCLE_DETECTOR_CONFIG                   (0xEE51)
#define USER_RECORD                              (0x74B4)
#define ME_TIME_SOURCE_SELECT                    (0xD403)
#define UART_FORMAT                              (0xA1A1)
#define GYRO_INTEGRATED_RV_CONFIG                (0xA1A2)
#define FRS_ID_META_RAW_ACCELEROMETER            (0xE301)
#define FRS_ID_META_ACCELEROMETER                (0xE302)
#define FRS_ID_META_LINEAR_ACCELERATION          (0xE303)
#define FRS_ID_META_GRAVITY                      (0xE304)
#define FRS_ID_META_RAW_GYROSCOPE                (0xE305)
#define FRS_ID_META_GYROSCOPE_CALIBRATED         (0xE306)
#define FRS_ID_META_GYROSCOPE_UNCALIBRATED       (0xE307)
#define FRS_ID_META_RAW_MAGNETOMETER             (0xE308)
#define FRS_ID_META_MAGNETIC_FIELD_CALIBRATED    (0xE309)
#define FRS_ID_META_MAGNETIC_FIELD_UNCALIBRATED  (0xE30A)
#define FRS_ID_META_ROTATION_VECTOR              (0xE30B)
#define FRS_ID_META_GAME_ROTATION_VECTOR         (0xE30C)
#define FRS_ID_META_GEOMAGNETIC_ROTATION_VECTOR  (0xE30D)
#define FRS_ID_META_PRESSURE                     (0xE30E)
#define FRS_ID_META_AMBIENT_LIGHT                (0xE30F)
#define FRS_ID_META_HUMIDITY                     (0xE310)
#define FRS_ID_META_PROXIMITY                    (0xE311)
#define FRS_ID_META_TEMPERATURE                  (0xE312)
#define FRS_ID_META_TAP_DETECTOR                 (0xE313)
#define FRS_ID_META_STEP_DETECTOR                (0xE314)
#define FRS_ID_META_STEP_COUNTER                 (0xE315)
#define FRS_ID_META_SIGNIFICANT_MOTION           (0xE316)
#define FRS_ID_META_STABILITY_CLASSIFIER         (0xE317)
#define FRS_ID_META_SHAKE_DETECTOR               (0xE318)
#define FRS_ID_META_FLIP_DETECTOR                (0xE319)
#define FRS_ID_META_PICKUP_DETECTOR              (0xE31A)
#define FRS_ID_META_STABILITY_DETECTOR           (0xE31B)
#define FRS_ID_META_PERSONAL_ACTIVITY_CLASSIFIER (0xE31C)
#define FRS_ID_META_SLEEP_DETECTOR               (0xE31D)
#define FRS_ID_META_TILT_DETECTOR                (0xE31E)
#define FRS_ID_META_POCKET_DETECTOR              (0xE31F)
#define FRS_ID_META_CIRCLE_DETECTOR              (0xE320)
#define FRS_ID_META_HEART_RATE_MONITOR           (0xE321)
#define FRS_ID_META_ARVR_STABILIZED_RV           (0xE322)
#define FRS_ID_META_ARVR_STABILIZED_GRV          (0xE323)
#define FRS_ID_META_GYRO_INTEGRATED_RV           (0xE324)

/**
 * @brief Interactive ZRO Motion Intent
 *
 * See the SH-2 Reference Manual, 6.4.13
 */
typedef enum {
    SH2_IZRO_MI_UNKNOWN = 0,
    SH2_IZRO_MI_STATIONARY_NO_VIBRATION,
    SH2_IZRO_MI_STATIONARY_WITH_VIBRATION,
    SH2_IZRO_MI_IN_MOTION,
} sh2_IZroMotionIntent_t;

/**
 * @brief Interactive ZRO Motion Intent
 *
 * See the SH-2 Reference Manual, 6.4.13
 */
typedef enum {
    SH2_IZRO_MR_NO_REQUEST = 0,
    SH2_IZRO_MR_STAY_STATIONARY,
    SH2_IZRO_MR_STATIONARY_NON_URGENT,
    SH2_IZRO_MR_STATIONARY_URGENT,
} sh2_IZroMotionRequest_t;


/**
* @brief Asynchronous Event
*
* Represents reset events and other non-sensor events received from SH-2 sensor hub.
*/

enum sh2_AsyncEventId_e {
    SH2_RESET,
    SH2_SHTP_EVENT,
    SH2_GET_FEATURE_RESP,
};
typedef enum sh2_AsyncEventId_e sh2_AsyncEventId_t;

enum sh2_ShtpEvent_e {
    SH2_SHTP_TX_DISCARD = 0,
    SH2_SHTP_SHORT_FRAGMENT = 1,
    SH2_SHTP_TOO_LARGE_PAYLOADS = 2,
    SH2_SHTP_BAD_RX_CHAN = 3,
    SH2_SHTP_BAD_TX_CHAN = 4,
};
typedef uint8_t sh2_ShtpEvent_t;

typedef struct sh2_SensorConfigResp_e {
    sh2_SensorId_t sensorId;
    sh2_SensorConfig_t sensorConfig;
} sh2_SensorConfigResp_t;

typedef struct sh2_AsyncEvent {
    uint32_t eventId;
    union {
        sh2_ShtpEvent_t shtpEvent;
        sh2_SensorConfigResp_t sh2SensorConfigResp;
    };
} sh2_AsyncEvent_t;

typedef void (sh2_EventCallback_t)(void * cookie, sh2_AsyncEvent_t *pEvent);


/***************************************************************************************
 * Public API
 **************************************************************************************/

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
             sh2_EventCallback_t *eventCallback, void *eventCookie);

/**
 * @brief Close a session with a sensor hub.
 *
 * This should be calLED at the end of a sensor hub session.  
 * The underlying SHTP and HAL instances will be closed.
 *
 */
void sh2_close(void);

/**
 * @brief Service the SH2 device, reading any data that is available and dispatching callbacks.
 *
 * This function should be calLED periodically by the host system to service an open sensor hub.
 *
 */
void sh2_service(void);

/**
 * @brief Register a function to receive sensor events.
 *
 * @param  callback A function that will be calLED each time a sensor event is received.
 * @param  cookie  A value that will be passed to the sensor callback function.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setSensorCallback(sh2_SensorCallback_t *callback, void *cookie);

/**
 * @brief Reset the sensor hub device by sending RESET (1) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devReset(void);

/**
 * @brief Turn sensor hub on by sending ON (2) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devOn(void);

/**
 * @brief Put sensor hub in sleep state by sending SLEEP (3) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devSleep(void);

/**
 * @brief Get Product ID information from Sensorhub.
 *
 * @param  prodIds Pointer to structure that will receive results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getProdIds(sh2_ProductIds_t *prodIds);

/**
 * @brief Get sensor configuration.
 *
 * @param  sensorId Which sensor to query.
 * @param  config SensorConfig structure to store results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getSensorConfig(sh2_SensorId_t sensorId, sh2_SensorConfig_t *config);

/**
 * @brief Set sensor configuration. (e.g enable a sensor at a particular rate.)
 *
 * @param  sensorId Which sensor to configure.
 * @param  pConfig Pointer to structure holding sensor configuration.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setSensorConfig(sh2_SensorId_t sensorId, const sh2_SensorConfig_t *pConfig);

/**
 * @brief Get metadata related to a sensor.
 *
 * @param  sensorId Which sensor to query.
 * @param  pData Pointer to structure to receive the results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getMetadata(sh2_SensorId_t sensorId, sh2_SensorMetadata_t *pData);

/**
 * @brief Get an FRS record.
 *
 * @param  recordId Which FRS Record to retrieve.
 * @param  pData pointer to buffer to receive the results
 * @param[in] words Size of pData buffer, in 32-bit words.
 * @param[out] words Number of 32-bit words retrieved.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getFrs(uint16_t recordId, uint32_t *pData, uint16_t *words);

/**
 * @brief Set an FRS record
 *
 * @param  recordId Which FRS Record to set.
 * @param  pData pointer to buffer containing the new data.
 * @param  words number of 32-bit words to write.  (0 to delete record.)
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setFrs(uint16_t recordId, uint32_t *pData, uint16_t words);

/**
 * @brief Get error counts.
 *
 * @param  severity Only errors of this severity or greater are returned.
 * @param  pErrors Buffer to receive error codes.
 * @param  numErrors size of pErrors array
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getErrors(uint8_t severity, sh2_ErrorRecord_t *pErrors, uint16_t *numErrors);

/**
 * @brief Read counters related to a sensor.
 *
 * @param  sensorId Which sensor to operate on.
 * @param  pCounts Pointer to Counts structure that will receive data.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getCounts(sh2_SensorId_t sensorId, sh2_Counts_t *pCounts);

/**
 * @brief Clear counters related to a sensor.
 *
 * @param  sensorId which sensor to operate on.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_clearCounts(sh2_SensorId_t sensorId);

/**
 * @brief Perform a tare operation on one or more axes.
 *
 * @param  axes Bit mask specifying which axes should be tared.
 * @param  basis Which rotation vector to use as the basis for Tare adjustment.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setTareNow(uint8_t axes,    // SH2_TARE_X | SH2_TARE_Y | SH2_TARE_Z
                   sh2_TareBasis_t basis);

/**
 * @brief Clears the previously applied tare operation.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_clearTare(void);

/**
 * @brief Persist the results of last tare operation to flash.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_persistTare(void);

/**
 * @brief Set the current run-time sensor reorientation. (Set to zero to clear tare.)
 *
 * @param  orientation Quaternion rotation vector to apply as new tare.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setReorientation(sh2_Quaternion_t *orientation);

/**
 * @brief Command the sensorhub to reset.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_reinitialize(void);

/**
 * @brief Save Dynamic Calibration Data to flash.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_saveDcdNow(void);

/**
 * @brief Get Oscillator type.
 *
 * @param  pOscType pointer to data structure to receive results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getOscType(sh2_OscType_t *pOscType);

// Flags for sensors field of sh_calConfig
#define SH2_CAL_ACCEL (0x01)
#define SH2_CAL_GYRO  (0x02)
#define SH2_CAL_MAG   (0x04)
#define SH2_CAL_PLANAR (0x08)

/**
 * @brief Enable/Disable dynamic calibration for certain sensors
 *
 * @param  sensors Bit mask to configure which sensors are affected.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setCalConfig(uint8_t sensors);

/**
 * @brief Get dynamic calibration configuration settings.
 *
 * @param  pSensors pointer to Bit mask, set on return.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getCalConfig(uint8_t *pSensors);

/**
 * @brief Configure automatic saving of dynamic calibration data.
 *
 * @param  enabLED Enable or Disable DCD auto-save.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setDcdAutoSave(bool enabLED);

/**
 * @brief Immediately issue all buffered sensor reports from a given sensor.
 *
 * @param  sensorId Which sensor reports to flush.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_flush(sh2_SensorId_t sensorId);

/**
 * @brief Command clear DCD in RAM, then reset sensor hub.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_clearDcdAndReset(void);

/**
 * @brief Start simple self-calibration procedure.
 *
 * @parameter interval_us sensor report interval, uS.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_startCal(uint32_t interval_us);

/**
 * @brief Finish simple self-calibration procedure.
 *
 * @parameter status contains calibration status code on return.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_finishCal(sh2_CalStatus_t *status);

/**
 * @brief send Interactive ZRO Request.
 *
 * @parameter intent Inform the sensor hub what sort of motion should be in progress.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setIZro(sh2_IZroMotionIntent_t intent);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
