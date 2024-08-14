/*
 * Copyright 2015-16 Hillcrest Laboratories, Inc.
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
 * @file sh2_SensorValue.h 
 * @author David Wheeler
 * @date 10 Nov 2015
 * @brief Support for converting sensor events (messages) into natural data structures.
 *
 */

#ifndef SH2_SENSORVALUE_H
#define SH2_SENSORVALUE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "sh2.h"

/* Note on quaternion naming conventions:
 * Quaternions are values with four real components that are usually
 * interpreted as coefficients in the complex quantity, Q.
 *
 * As in, Q = W + Xi + Yj + Zk
 *
 * Where i, j and k represent the three imaginary dimensions.
 *
 * So W represents the Real components and X, Y and Z the Imaginary ones.
 *
 * In the Hillcrest datasheets and in this code, however, the four components
 * are named real, i, j and k, to make it explicit which is which.  If you 
 * need to translate these names into the "wxyz" or "xyzw" convention, then, the
 * appropriate mapping is this:
 *     w = real
 *     x = i
 *     y = j
 *     z = k
 */
	
/**
 * @brief Raw Accelerometer
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_RawAccelerometer {
    /* Units are ADC counts */
    int16_t x;  /**< @brief [ADC counts] */
    int16_t y;  /**< @brief [ADC counts] */
    int16_t z;  /**< @brief [ADC counts] */

    /* Microseconds */
    uint32_t timestamp;  /**< @brief [uS] */
} sh2_RawAccelerometer_t;

/**
 * @brief Accelerometer
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Accelerometer {
    float x;
    float y;
    float z;
} sh2_Accelerometer_t;

/**
 * @brief Raw gyroscope
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_RawGyroscope {
    /* Units are ADC counts */
    int16_t x;  /**< @brief [ADC Counts] */
    int16_t y;  /**< @brief [ADC Counts] */
    int16_t z;  /**< @brief [ADC Counts] */
    int16_t temperature;  /**< @brief [ADC Counts] */

    /* Microseconds */
    uint32_t timestamp;  /**< @brief [uS] */
} sh2_RawGyroscope_t;

/**
 * @brief Gyroscope
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Gyroscope {
    /* Units are rad/s */
    float x;
    float y;
    float z;
} sh2_Gyroscope_t;

/**
 * @brief Uncalibrated gyroscope
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_GyroscopeUncalibrated {
    /* Units are rad/s */
    float x;  /**< @brief [rad/s] */
    float y;  /**< @brief [rad/s] */
    float z;  /**< @brief [rad/s] */
    float biasX;  /**< @brief [rad/s] */
    float biasY;  /**< @brief [rad/s] */
    float biasZ;  /**< @brief [rad/s] */
} sh2_GyroscopeUncalibrated_t;

/**
 * @brief Raw Magnetometer
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_RawMagnetometer {
    /* Units are ADC counts */
    int16_t x;  /**< @brief [ADC Counts] */
    int16_t y;  /**< @brief [ADC Counts] */
    int16_t z;  /**< @brief [ADC Counts] */

    /* Microseconds */
    uint32_t timestamp;  /**< @brief [uS] */
} sh2_RawMagnetometer_t;

/**
 * @brief Magnetic field
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_MagneticField {
    /* Units are uTesla */
    float x;  /**< @brief [uTesla] */
    float y;  /**< @brief [uTesla] */
    float z;  /**< @brief [uTesla] */
} sh2_MagneticField_t;

/**
 * @brief Uncalibrated magnetic field
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_MagneticFieldUncalibrated {
    /* Units are uTesla */
    float x;  /**< @brief [uTesla] */
    float y;  /**< @brief [uTesla] */
    float z;  /**< @brief [uTesla] */
    float biasX;  /**< @brief [uTesla] */
    float biasY;  /**< @brief [uTesla] */
    float biasZ;  /**< @brief [uTesla] */
} sh2_MagneticFieldUncalibrated_t;

/**
 * @brief Rotation Vector with Accuracy
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_RotationVectorWAcc {
    float i;  /**< @brief Quaternion component i */
    float j;  /**< @brief Quaternion component j */
    float k;  /**< @brief Quaternion component k */
    float real;  /**< @brief Quaternion component, real */
    float accuracy;  /**< @brief Accuracy estimate [radians] */
} sh2_RotationVectorWAcc_t;

/**
 * @brief Rotation Vector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_RotationVector {
    float i;  /**< @brief Quaternion component i */
    float j;  /**< @brief Quaternion component j */
    float k;  /**< @brief Quaternion component k */
    float real;  /**< @brief Quaternion component real */
} sh2_RotationVector_t;

/**
 * @brief Atmospheric Pressure
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Pressure {
    float value;  /**< @brief Atmospheric Pressure.  [hectopascals] */
} sh2_Pressure_t;

/**
 * @brief Ambient Light
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_AmbientLight {
    float value;  /**< @brief Ambient Light.  [lux] */
} sh2_AmbientLight_t;

/**
 * @brief Humidity
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Humidity {
    float value;  /**< @brief Relative Humidity.  [percent] */
} sh2_Humidity_t;

/**
 * @brief Proximity
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Proximity {
    float value;  /**< @brief Proximity.  [cm] */
} sh2_Proximity_t;

/**
 * @brief Temperature
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Temperature {
    float value;  /**< @brief Temperature.  [C] */
} sh2_Temperature_t;

/**
 * @brief Reserved
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Reserved {
    float tbd;  /**< @brief Reserved */
} sh2_Reserved_t;

/**
 * @brief TapDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define TAPDET_X      (1)
#define TAPDET_X_POS  (2)
#define TAPDET_Y      (4)
#define TAPDET_Y_POS  (8)
#define TAPDET_Z      (16)
#define TAPDET_Z_POS  (32)
#define TAPDET_DOUBLE (64)
typedef struct sh2_TapDetector {
    uint8_t flags;  /**< @brief TapDetector.  */
} sh2_TapDetector_t;

/**
 * @brief StepDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_StepDetector {
    uint32_t latency;  /**< @brief Step detect latency [uS].  */
} sh2_StepDetector_t;

/**
 * @brief StepCounter
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_StepCounter {
    uint32_t latency;  /**< @brief Step counter latency [uS].  */
    uint16_t steps;    /**< @brief Steps counted. */
} sh2_StepCounter_t;

/**
 * @brief SigMotion
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_SigMotion {
    uint16_t motion;
} sh2_SigMotion_t;

/**
 * @brief StabilityClassifier
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define STABILITY_CLASSIFIER_UNKNOWN (0)
#define STABILITY_CLASSIFIER_ON_TABLE (1)
#define STABILITY_CLASSIFIER_STATIONARY (2)
#define STABILITY_CLASSIFIER_STABLE (3)
#define STABILITY_CLASSIFIER_MOTION (4)
#define STABILITY_CLASSIFIER_RESERVED (5)
typedef struct sh2_StabilityClassifier {
    uint8_t classification;
} sh2_StabilityClassifier_t;

/**
 * @brief ShakeDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define SHAKE_X (1)
#define SHAKE_Y (2)
#define SHAKE_Z (4)
typedef struct sh2_ShakeDetector {
    uint16_t shake;
} sh2_ShakeDetector_t;

/**
 * @brief flipDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_FlipDetector {
    uint16_t flip;
} sh2_FlipDetector_t;

/**
 * @brief pickupDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define PICKUP_LEVEL_TO_NOT_LEVEL (1)
#define PICKUP_STOP_WITHIN_REGION (2)
typedef struct sh2_PickupDetector {
    uint16_t pickup;   /**< flag field with bits defined above. */
} sh2_PickupDetector_t;

/**
 * @brief stabilityDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define STABILITY_ENTERED (1)
#define STABILITY_EXITED  (2)
typedef struct sh2_StabilityDetector {
    uint16_t stability;  /**< flag field with bits defined above. */
} sh2_StabilityDetector_t;

/**
 * @brief Personal Activity Classifier
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define PAC_UNKNOWN (0)
#define PAC_IN_VEHICLE (1)
#define PAC_ON_BICYCLE (2)
#define PAC_ON_FOOT (3)
#define PAC_STILL (4)
#define PAC_TILTING (5)
#define PAC_WALKING (6)
#define PAC_RUNNING (7)
typedef struct sh2_PersonalActivityClassifier {
    uint8_t page;
    bool lastPage;
    uint8_t mostLikelyState;
    uint8_t confidence[10];
} sh2_PersonalActivityClassifier_t;

/**
 * @brief sleepDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_SleepDetector {
    uint8_t sleepState;
} sh2_SleepDetector_t;

/**
 * @brief tiltDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_TiltDetector {
    uint16_t tilt;
} sh2_TiltDetector_t;

/**
 * @brief pocketDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_PocketDetector {
    uint16_t pocket;
} sh2_PocketDetector_t;

/**
 * @brief circLEDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_CircLEDetector {
    uint16_t circle;
} sh2_CircLEDetector_t;

/**
 * @brief heartRateMonitor
 *
 * See SH-2 Reference Manual for details.
 */
typedef struct sh2_HeartRateMonitor {
    uint16_t heartRate; /**< heart rate in beats per minute. */
} sh2_HeartRateMonitor_t;

/**
 * @brief Gyro Integrated Rotation Vector
 *
 * See SH-2 Reference Manual for details.
 */
typedef struct sh2_GyroIntegratedRV {
    float i;        /**< @brief Quaternion component i */
    float j;        /**< @brief Quaternion component j */
    float k;        /**< @brief Quaternion component k */
    float real;     /**< @brief Quaternion component real */
    float angVelX;  /**< @brief Angular velocity about x [rad/s] */
    float angVelY;  /**< @brief Angular velocity about y [rad/s] */
    float angVelZ;  /**< @brief Angular velocity about z [rad/s] */
} sh2_GyroIntegratedRV_t;

typedef struct sh2_IZroRequest {
    sh2_IZroMotionIntent_t intent;
    sh2_IZroMotionRequest_t request;
} sh2_IZroRequest_t;

typedef struct sh2_SensorValue {
    
    /** Which sensor produced this event. */
    uint8_t sensorId;

    /** @brief 8-bit unsigned integer used to track reports.
     *
     * The sequence number increments once for each report sent.  Gaps
     * in the sequence numbers indicate missing or dropped reports.
     */
    uint8_t sequence;

    /* Status of a sensor
     *   0 - Unreliable
     *   1 - Accuracy low
     *   2 - Accuracy medium
     *   3 - Accuracy high
     */
    uint8_t status; /**< @brief bits 7-5: reserved, 4-2: exponent delay, 1-0: Accuracy */

    uint64_t timestamp;  /**< [uS] */

    uint32_t delay; /**< @brief [uS] value is delay * 2^exponent (see status) */

    /** @brief Sensor Data
     *
     * Use the structure based on the value of the sensor
     * field.
     */
    union {
        sh2_RawAccelerometer_t rawAccelerometer;
        sh2_Accelerometer_t accelerometer; 
        sh2_Accelerometer_t linearAcceleration; 
        sh2_Accelerometer_t gravity; 
        sh2_RawGyroscope_t rawGyroscope; 
        sh2_Gyroscope_t gyroscope; 
        sh2_GyroscopeUncalibrated_t gyroscopeUncal; 
        sh2_RawMagnetometer_t rawMagnetometer; 
        sh2_MagneticField_t magneticField; 
        sh2_MagneticFieldUncalibrated_t magneticFieldUncal; 
        sh2_RotationVectorWAcc_t rotationVector; 
        sh2_RotationVector_t gameRotationVector; 
        sh2_RotationVectorWAcc_t geoMagRotationVector;
        sh2_Pressure_t pressure;
        sh2_AmbientLight_t ambientLight;
        sh2_Humidity_t humidity;
        sh2_Proximity_t proximity;
        sh2_Temperature_t temperature;
        sh2_Reserved_t reserved;
        sh2_TapDetector_t tapDetector;
        sh2_StepDetector_t stepDetector;
        sh2_StepCounter_t stepCounter;
        sh2_SigMotion_t sigMotion;
        sh2_StabilityClassifier_t stabilityClassifier;
        sh2_ShakeDetector_t shakeDetector;
        sh2_FlipDetector_t flipDetector;
        sh2_PickupDetector_t pickupDetector;
        sh2_StabilityDetector_t stabilityDetector;
        sh2_PersonalActivityClassifier_t personalActivityClassifier;
        sh2_SleepDetector_t sleepDetector;
        sh2_TiltDetector_t tiltDetector;
        sh2_PocketDetector_t pocketDetector;
        sh2_CircLEDetector_t circLEDetector;
        sh2_HeartRateMonitor_t heartRateMonitor;
        sh2_RotationVectorWAcc_t arvrStabilizedRV;
        sh2_RotationVector_t arvrStabilizedGRV;
        sh2_GyroIntegratedRV_t gyroIntegratedRV;
        sh2_IZroRequest_t izroRequest;
    } un;
} sh2_SensorValue_t;

int sh2_decodeSensorEvent(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);


#ifdef __cplusplus
} // extern "C"
#endif

#endif
