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

/*
 * BNO08x Sensor Event decoding
 */

#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_util.h"

#define SCALE_Q(n) (1.0f / (1 << n))

const float scaleRadToDeg = 180.0 / 3.14159265358;

// ------------------------------------------------------------------------
// Forward declarations

static int decodeRawAccelerometer(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeAccelerometer(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeLinearAcceleration(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeGravity(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeRawGyroscope(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeGyroscopeCalibrated(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeGyroscopeUncal(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeRawMagnetometer(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeMagneticFieldCalibrated(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeMagneticFieldUncal(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeRotationVector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeGameRotationVector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeGeomagneticRotationVector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodePressure(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeAmbientLight(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeHumidity(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeProximity(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeTemperature(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeReserved(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeTapDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeStepDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeStepCounter(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeSignificantMotion(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeStabilityClassifier(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeShakeDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeFlipDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodePickupDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeStabilityDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodePersonalActivityClassifier(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeSleepDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeTiltDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodePocketDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeCircLEDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeHeartRateMonitor(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeArvrStabilizedRV(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeArvrStabilizedGRV(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeGyroIntegratedRV(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeIZroRequest(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);

// ------------------------------------------------------------------------
// Public API

int sh2_decodeSensorEvent(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    // Fill out fields of *value based on *event, converting data from message representation
    // to natural representation.

    int rc = SH2_OK;

    value->sensorId = event->reportId;
    value->timestamp = event->timestamp_uS;

    if (value->sensorId != SH2_GYRO_INTEGRATED_RV) {
        value->sequence = event->report[1];
        value->status = event->report[2] & 0x03;
    }
    else {
        value->sequence = 0;
        value->status = 0;
    }

    // extract delay field (100uS units)
    
    
    switch (value->sensorId) {
        case SH2_RAW_ACCELEROMETER:
            rc = decodeRawAccelerometer(value, event);
            break;
        case SH2_ACCELEROMETER:
            rc = decodeAccelerometer(value, event);
            break;
        case SH2_LINEAR_ACCELERATION:
            rc = decodeLinearAcceleration(value, event);
            break;
        case SH2_GRAVITY:
            rc = decodeGravity(value, event);
            break;
        case SH2_RAW_GYROSCOPE:
            rc = decodeRawGyroscope(value, event);
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            rc = decodeGyroscopeCalibrated(value, event);
            break;
        case SH2_GYROSCOPE_UNCALIBRATED:
            rc = decodeGyroscopeUncal(value, event);
            break;
        case SH2_RAW_MAGNETOMETER:
            rc = decodeRawMagnetometer(value, event);
            break;
        case SH2_MAGNETIC_FIELD_CALIBRATED:
            rc = decodeMagneticFieldCalibrated(value, event);
            break;
        case SH2_MAGNETIC_FIELD_UNCALIBRATED:
            rc = decodeMagneticFieldUncal(value, event);
            break;
        case SH2_ROTATION_VECTOR:
            rc = decodeRotationVector(value, event);
            break;
        case SH2_GAME_ROTATION_VECTOR:
            rc = decodeGameRotationVector(value, event);
            break;
        case SH2_GEOMAGNETIC_ROTATION_VECTOR:
            rc = decodeGeomagneticRotationVector(value, event);
            break;
        case SH2_PRESSURE:
            rc = decodePressure(value, event);
            break;
        case SH2_AMBIENT_LIGHT:
            rc = decodeAmbientLight(value, event);
            break;
        case SH2_HUMIDITY:
            rc = decodeHumidity(value, event);
            break;
        case SH2_PROXIMITY:
            rc = decodeProximity(value, event);
            break;
        case SH2_TEMPERATURE:
            rc = decodeTemperature(value, event);
            break;
        case SH2_RESERVED:
            rc = decodeReserved(value, event);
            break;
        case SH2_TAP_DETECTOR:
            rc = decodeTapDetector(value, event);
            break;
        case SH2_STEP_DETECTOR:
            rc = decodeStepDetector(value, event);
            break;
        case SH2_STEP_COUNTER:
            rc = decodeStepCounter(value, event);
            break;
        case SH2_SIGNIFICANT_MOTION:
            rc = decodeSignificantMotion(value, event);
            break;
        case SH2_STABILITY_CLASSIFIER:
            rc = decodeStabilityClassifier(value, event);
            break;
        case SH2_SHAKE_DETECTOR:
            rc = decodeShakeDetector(value, event);
            break;
        case SH2_FLIP_DETECTOR:
            rc = decodeFlipDetector(value, event);
            break;
        case SH2_PICKUP_DETECTOR:
            rc = decodePickupDetector(value, event);
            break;
        case SH2_STABILITY_DETECTOR:
            rc = decodeStabilityDetector(value, event);
            break;
        case SH2_PERSONAL_ACTIVITY_CLASSIFIER:
            rc = decodePersonalActivityClassifier(value, event);
            break;
        case SH2_SLEEP_DETECTOR:
            rc = decodeSleepDetector(value, event);
            break;
        case SH2_TILT_DETECTOR:
            rc = decodeTiltDetector(value, event);
            break;
        case SH2_POCKET_DETECTOR:
            rc = decodePocketDetector(value, event);
            break;
        case SH2_CIRCLE_DETECTOR:
            rc = decodeCircLEDetector(value, event);
            break;
        case SH2_HEART_RATE_MONITOR:
            rc = decodeHeartRateMonitor(value, event);
            break;
        case SH2_ARVR_STABILIZED_RV:
            rc = decodeArvrStabilizedRV(value, event);
            break;
        case SH2_ARVR_STABILIZED_GRV:
            rc = decodeArvrStabilizedGRV(value, event);
            break;
        case SH2_GYRO_INTEGRATED_RV:
            rc = decodeGyroIntegratedRV(value, event);
            break;
        case SH2_IZRO_MOTION_REQUEST:
            rc = decodeIZroRequest(value, event);
            break;
        default:
            // Unknown report id
            rc = SH2_ERR;
            break;
    }

    return rc;
}

// ------------------------------------------------------------------------
// Private utility functions

static int decodeRawAccelerometer(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.rawAccelerometer.x = read16(&event->report[4]);
    value->un.rawAccelerometer.y = read16(&event->report[6]);
    value->un.rawAccelerometer.z = read16(&event->report[8]);
    value->un.rawAccelerometer.timestamp = read32(&event->report[12]);

    return SH2_OK;
}

static int decodeAccelerometer(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.accelerometer.x = read16(&event->report[4]) * SCALE_Q(8);
    value->un.accelerometer.y = read16(&event->report[6]) * SCALE_Q(8);
    value->un.accelerometer.z = read16(&event->report[8]) * SCALE_Q(8);

    return SH2_OK;
}

static int decodeLinearAcceleration(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.linearAcceleration.x = read16(&event->report[4]) * SCALE_Q(8);
    value->un.linearAcceleration.y = read16(&event->report[6]) * SCALE_Q(8);
    value->un.linearAcceleration.z = read16(&event->report[8]) * SCALE_Q(8);

    return SH2_OK;
}

static int decodeGravity(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.gravity.x = read16(&event->report[4]) * SCALE_Q(8);
    value->un.gravity.y = read16(&event->report[6]) * SCALE_Q(8);
    value->un.gravity.z = read16(&event->report[8]) * SCALE_Q(8);

    return SH2_OK;
}

static int decodeRawGyroscope(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.rawGyroscope.x = read16(&event->report[4]);
    value->un.rawGyroscope.y = read16(&event->report[6]);
    value->un.rawGyroscope.z = read16(&event->report[8]);
    value->un.rawGyroscope.temperature = read16(&event->report[10]);
    value->un.rawGyroscope.timestamp = read32(&event->report[12]);

    return SH2_OK;
}

static int decodeGyroscopeCalibrated(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.gyroscope.x = read16(&event->report[4]) * SCALE_Q(9);
    value->un.gyroscope.y = read16(&event->report[6]) * SCALE_Q(9);
    value->un.gyroscope.z = read16(&event->report[8]) * SCALE_Q(9);

    return SH2_OK;
}

static int decodeGyroscopeUncal(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.gyroscopeUncal.x = read16(&event->report[4]) * SCALE_Q(9);
    value->un.gyroscopeUncal.y = read16(&event->report[6]) * SCALE_Q(9);
    value->un.gyroscopeUncal.z = read16(&event->report[8]) * SCALE_Q(9);

    value->un.gyroscopeUncal.biasX = read16(&event->report[10]) * SCALE_Q(9);
    value->un.gyroscopeUncal.biasY = read16(&event->report[12]) * SCALE_Q(9);
    value->un.gyroscopeUncal.biasZ = read16(&event->report[14]) * SCALE_Q(9);

    return SH2_OK;
}

static int decodeRawMagnetometer(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.rawMagnetometer.x = read16(&event->report[4]);
    value->un.rawMagnetometer.y = read16(&event->report[6]);
    value->un.rawMagnetometer.z = read16(&event->report[8]);
    value->un.rawMagnetometer.timestamp = read32(&event->report[12]);

    return SH2_OK;
}

static int decodeMagneticFieldCalibrated(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.magneticField.x = read16(&event->report[4]) * SCALE_Q(4);
    value->un.magneticField.y = read16(&event->report[6]) * SCALE_Q(4);
    value->un.magneticField.z = read16(&event->report[8]) * SCALE_Q(4);

    return SH2_OK;
}

static int decodeMagneticFieldUncal(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.magneticFieldUncal.x = read16(&event->report[4]) * SCALE_Q(4);
    value->un.magneticFieldUncal.y = read16(&event->report[6]) * SCALE_Q(4);
    value->un.magneticFieldUncal.z = read16(&event->report[8]) * SCALE_Q(4);

    value->un.magneticFieldUncal.biasX = read16(&event->report[10]) * SCALE_Q(4);
    value->un.magneticFieldUncal.biasY = read16(&event->report[12]) * SCALE_Q(4);
    value->un.magneticFieldUncal.biasZ = read16(&event->report[14]) * SCALE_Q(4);

    return SH2_OK;
}

static int decodeRotationVector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.rotationVector.i = read16(&event->report[4]) * SCALE_Q(14);
    value->un.rotationVector.j = read16(&event->report[6]) * SCALE_Q(14);
    value->un.rotationVector.k = read16(&event->report[8]) * SCALE_Q(14);
    value->un.rotationVector.real = read16(&event->report[10]) * SCALE_Q(14);
    value->un.rotationVector.accuracy = read16(&event->report[12]) * SCALE_Q(12);

    return SH2_OK;
}

static int decodeGameRotationVector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.gameRotationVector.i = read16(&event->report[4]) * SCALE_Q(14);
    value->un.gameRotationVector.j = read16(&event->report[6]) * SCALE_Q(14);
    value->un.gameRotationVector.k = read16(&event->report[8]) * SCALE_Q(14);
    value->un.gameRotationVector.real = read16(&event->report[10]) * SCALE_Q(14);

    return SH2_OK;
}

static int decodeGeomagneticRotationVector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.geoMagRotationVector.i = read16(&event->report[4]) * SCALE_Q(14);
    value->un.geoMagRotationVector.j = read16(&event->report[6]) * SCALE_Q(14);
    value->un.geoMagRotationVector.k = read16(&event->report[8]) * SCALE_Q(14);
    value->un.geoMagRotationVector.real = read16(&event->report[10]) * SCALE_Q(14);
    value->un.geoMagRotationVector.accuracy = read16(&event->report[12]) * SCALE_Q(12);

    return SH2_OK;
}

static int decodePressure(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.pressure.value = read32(&event->report[4]) * SCALE_Q(20);

    return SH2_OK;
}

static int decodeAmbientLight(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.ambientLight.value = read32(&event->report[4]) * SCALE_Q(8);

    return SH2_OK;
}

static int decodeHumidity(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.humidity.value = read16(&event->report[4]) * SCALE_Q(8);

    return SH2_OK;
}

static int decodeProximity(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.proximity.value = read16(&event->report[4]) * SCALE_Q(4);

    return SH2_OK;
}

static int decodeTemperature(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.temperature.value = read16(&event->report[4]) * SCALE_Q(7);

    return SH2_OK;
}

static int decodeReserved(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.reserved.tbd = read16(&event->report[4]) * SCALE_Q(7);

    return SH2_OK;
}

static int decodeTapDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.tapDetector.flags = event->report[4];

    return SH2_OK;
}

static int decodeStepDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.stepDetector.latency = readu32(&event->report[4]);

    return SH2_OK;
}

static int decodeStepCounter(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.stepCounter.latency = readu32(&event->report[4]);
    value->un.stepCounter.steps = readu32(&event->report[8]);

    return SH2_OK;
}

static int decodeSignificantMotion(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.sigMotion.motion = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodeStabilityClassifier(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.stabilityClassifier.classification = event->report[4];

    return SH2_OK;
}

static int decodeShakeDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.shakeDetector.shake = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodeFlipDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.flipDetector.flip = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodePickupDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.pickupDetector.pickup = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodeStabilityDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.stabilityDetector.stability = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodePersonalActivityClassifier(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.personalActivityClassifier.page = event->report[4] & 0x7F;
    value->un.personalActivityClassifier.lastPage = ((event->report[4] & 0x80) != 0);
    value->un.personalActivityClassifier.mostLikelyState = event->report[5];
    for (int n = 0; n < 10; n++) {
        value->un.personalActivityClassifier.confidence[n] = event->report[6+n];
    }
    
    return SH2_OK;
}

static int decodeSleepDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.sleepDetector.sleepState = event->report[4];

    return SH2_OK;
}

static int decodeTiltDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.tiltDetector.tilt = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodePocketDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.pocketDetector.pocket = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodeCircLEDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.circLEDetector.circle = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodeHeartRateMonitor(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.heartRateMonitor.heartRate = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodeArvrStabilizedRV(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.arvrStabilizedRV.i = read16(&event->report[4]) * SCALE_Q(14);
    value->un.arvrStabilizedRV.j = read16(&event->report[6]) * SCALE_Q(14);
    value->un.arvrStabilizedRV.k = read16(&event->report[8]) * SCALE_Q(14);
    value->un.arvrStabilizedRV.real = read16(&event->report[10]) * SCALE_Q(14);
    value->un.arvrStabilizedRV.accuracy = read16(&event->report[12]) * SCALE_Q(12);

    return SH2_OK;
}

static int decodeArvrStabilizedGRV(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.arvrStabilizedGRV.i = read16(&event->report[4]) * SCALE_Q(14);
    value->un.arvrStabilizedGRV.j = read16(&event->report[6]) * SCALE_Q(14);
    value->un.arvrStabilizedGRV.k = read16(&event->report[8]) * SCALE_Q(14);
    value->un.arvrStabilizedGRV.real = read16(&event->report[10]) * SCALE_Q(14);

    return SH2_OK;
}

static int decodeGyroIntegratedRV(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.gyroIntegratedRV.i = read16(&event->report[0]) * SCALE_Q(14);
    value->un.gyroIntegratedRV.j = read16(&event->report[2]) * SCALE_Q(14);
    value->un.gyroIntegratedRV.k = read16(&event->report[4]) * SCALE_Q(14);
    value->un.gyroIntegratedRV.real = read16(&event->report[6]) * SCALE_Q(14);
    value->un.gyroIntegratedRV.angVelX = read16(&event->report[8]) * SCALE_Q(10);
    value->un.gyroIntegratedRV.angVelY = read16(&event->report[10]) * SCALE_Q(10);
    value->un.gyroIntegratedRV.angVelZ = read16(&event->report[12]) * SCALE_Q(10);

    return SH2_OK;
}

static int decodeIZroRequest(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.izroRequest.intent = (sh2_IZroMotionIntent_t)event->report[4];
    value->un.izroRequest.request = (sh2_IZroMotionRequest_t)event->report[5];

    return SH2_OK;
}
