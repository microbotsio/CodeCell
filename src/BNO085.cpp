#include "BNO085.h"

static TwoWire *_i2cPort = NULL;  //The generic connection to user's chosen I2C hardware
static uint8_t _address = 0x4A;   //Keeps track of I2C address. setI2CAddress changes this.

static sh2_SensorValue_t *_sensor_value = NULL;
static bool _reset_occurred = false;

static int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static void i2chal_close(sh2_Hal_t *self);
static int i2chal_open(sh2_Hal_t *self);

static uint32_t hal_getTimeUs(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent);

size_t _maxBufferSize = 32;
size_t maxBufferSize();


bool BNO085::begin(TwoWire &wirePort) {
  _address = 0x4A;
  _i2cPort = &wirePort;

  _i2cPort->beginTransmission((uint8_t)_address);

  _HAL.open = i2chal_open;
  _HAL.close = i2chal_close;
  _HAL.read = i2chal_read;
  _HAL.write = i2chal_write;
  _HAL.getTimeUs = hal_getTimeUs;

  return _init();
}

bool BNO085::_init(int32_t sensor_id) {
  int status;

  // Open SH2 interface (also registers non-sensor event handler.)
  status = sh2_open(&_HAL, hal_callback, NULL);
  if (status != SH2_OK) {
    return false;
  }

  // Check connection partially by getting the product id's
  memset(&prodIds, 0, sizeof(prodIds));
  status = sh2_getProdIds(&prodIds);
  if (status != SH2_OK) {
    return false;
  }

  // Register sensor listener
  sh2_setSensorCallback(sensorHandler, NULL);

  return true;
}

bool BNO085::getSensorEvent() {
  _sensor_value = &sensorValue;

  _i2cPort->beginTransmission((uint8_t)_address);

  _sensor_value->timestamp = 0;

  sh2_service();

  if (_sensor_value->timestamp == 0 && _sensor_value->sensorId != SH2_GYRO_INTEGRATED_RV) {
    // no new events
    return false;
  }

  return true;
}

bool BNO085::enableReport(sh2_SensorId_t sensorId, uint32_t interval_us,
                          uint32_t sensorSpecific) {
  static sh2_SensorConfig_t config;

  // These sensor options are disabLED or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = sensorSpecific;

  config.reportInterval_us = interval_us;

  int status = sh2_setSensorConfig(sensorId, &config);

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

static int i2chal_open(sh2_Hal_t *self) {

  uint8_t softreset_pkt[] = { 5, 0, 1, 0, 1 };
  bool success = false;
  for (uint8_t attempts = 0; attempts < 5; attempts++) {
    if (I2CWrite(_address, softreset_pkt, 5)) {
      success = true;
      break;
    }
    delay(30);
  }
  if (!success)
    return -1;
  delay(300);
  return 0;
}

static void i2chal_close(sh2_Hal_t *self) {
}

static int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
  uint8_t header[4];
  if (!I2CRead(_address, header, 4)) {
    return 0;
  }

  uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
  // Unset the "continue" bit
  packet_size &= ~0x8000;


  size_t i2c_buffer_max = maxBufferSize();

  if (packet_size > len) {
    return 0;
  }
  uint16_t cargo_remaining = packet_size;
  uint8_t i2c_buffer[i2c_buffer_max];
  uint16_t read_size;
  uint16_t cargo_read_amount = 0;
  bool first_read = true;

  while (cargo_remaining > 0) {
    if (first_read) {
      read_size = min(i2c_buffer_max, (size_t)cargo_remaining);
    } else {
      read_size = min(i2c_buffer_max, (size_t)cargo_remaining + 4);
    }

    if (!I2CRead(_address, i2c_buffer, read_size)) {
      return 0;
    }

    if (first_read) {
      // The first time we're saving the "original" header, so include it in the
      // cargo count
      cargo_read_amount = read_size;
      memcpy(pBuffer, i2c_buffer, cargo_read_amount);
      first_read = false;
    } else {
      // this is not the first read, so copy from 4 bytes after the beginning of
      // the i2c buffer to skip the header included with every new i2c read and
      // don't include the header in the amount of cargo read
      cargo_read_amount = read_size - 4;
      memcpy(pBuffer, i2c_buffer + 4, cargo_read_amount);
    }
    // advance our pointer by the amount of cargo read
    pBuffer += cargo_read_amount;
    // mark the cargo as received
    cargo_remaining -= cargo_read_amount;
  }
  return packet_size;
}

static int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  size_t i2c_buffer_max = maxBufferSize();

  uint16_t write_size = min(i2c_buffer_max, len);

  if (!I2CWrite(_address, pBuffer, write_size)) {
    return 0;
  }

  return write_size;
}


static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
  uint32_t t = millis() * 1000;
  // Serial.print("I2C HAL get time: %d\n", t);
  return t;
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
  // If we see a reset, set a flag so that sensors will be reconfigured.
  if (pEvent->eventId == SH2_RESET) {
    // Serial.println("Reset!");
    _reset_occurred = true;
  }
}

// Handle sensor events.
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
  int rc;

  // Serial.println("Got an event!");

  rc = sh2_decodeSensorEvent(_sensor_value, event);
  if (rc != SH2_OK) {
    //Serial.println("BNO085 - Error decoding sensor event");
    _sensor_value->timestamp = 0;
    return;
  }
}

//Return the sensorID
uint8_t BNO085::getSensorEventID() {
  return _sensor_value->sensorId;
}

//Returns true if I2C device ack's
bool BNO085::isConnected() {
  if (_i2cPort->endTransmission() != 0)
    return (false);  //Sensor did not ACK
  return (true);
}

bool I2CWrite(uint8_t add, uint8_t *buffer, size_t size) {
  Wire.beginTransmission(add);
  // Write the data buffer
  if (Wire.write(buffer, size) != size) {
    // If the number of bytes written is not equal to the length, return false
    Wire.endTransmission();  // Ensure to end transmission even if writing fails
    return false;
  }

  if (Wire.endTransmission() == 0) {
    return true;
  } else {
    return false;
  }
}


bool I2CRead(uint8_t add, uint8_t *buffer, size_t size) {
  size_t pos = 0;
  while (pos < size) {
    size_t read_size;
    if ((size - pos) > maxBufferSize()) {
      read_size = maxBufferSize();
    } else {
      read_size = size - pos;
    }

    size_t recv = Wire.requestFrom(add, read_size);

    if (recv != size) {
      return false;
    }

    for (uint16_t i = 0; i < size; i++) {
      buffer[i] = Wire.read();
    }
    pos += read_size;
  }
  return true;
}

size_t maxBufferSize() {
  return _maxBufferSize;
}


float BNO085::getRot_I() {
  return _sensor_value->un.rotationVector.i;
}

float BNO085::getRot_J() {
  return _sensor_value->un.rotationVector.j;
}

float BNO085::getRot_K() {
  return _sensor_value->un.rotationVector.k;
}

float BNO085::getRot_R() {
  return _sensor_value->un.rotationVector.real;
}

//Return the rotation vector radian accuracy
float BNO085::getRadianAccuracy() {
  return _sensor_value->un.rotationVector.accuracy;
}

//Return the rotation vector sensor event report status accuracy
uint8_t BNO085::getRot_Accuracy() {
  return _sensor_value->status;
}

//Return the game rotation vector quaternion I
float BNO085::getGameI() {
  return _sensor_value->un.gameRotationVector.i;
}

//Return the game rotation vector quaternion J
float BNO085::getGameJ() {
  return _sensor_value->un.gameRotationVector.j;
}

//Return the game rotation vector quaternion K
float BNO085::getGameK() {
  return _sensor_value->un.gameRotationVector.k;
}

//Return the game rotation vector quaternion Real
float BNO085::getGameReal() {
  return _sensor_value->un.gameRotationVector.real;
}

//Return the acceleration component
float BNO085::getAccelX() {
  return _sensor_value->un.accelerometer.x;
}

//Return the acceleration component
float BNO085::getAccelY() {
  return _sensor_value->un.accelerometer.y;
}

//Return the acceleration component
float BNO085::getAccelZ() {
  return _sensor_value->un.accelerometer.z;
}

//Return the acceleration component
uint8_t BNO085::getAccelAccuracy() {
  return _sensor_value->status;
}

float BNO085::getLinAccelX() {
  return _sensor_value->un.linearAcceleration.x;
}

//Return the acceleration component
float BNO085::getLinAccelY() {
  return _sensor_value->un.linearAcceleration.y;
}

//Return the acceleration component
float BNO085::getLinAccelZ() {
  return _sensor_value->un.linearAcceleration.z;
}

//Return the acceleration component
uint8_t BNO085::getLinAccelAccuracy() {
  return _sensor_value->status;
}

//Return the gyro component
float BNO085::getGyroX() {
  return _sensor_value->un.gyroscope.x;
}

//Return the gyro component
float BNO085::getGyroY() {
  return _sensor_value->un.gyroscope.y;
}

//Return the gyro component
float BNO085::getGyroZ() {
  return _sensor_value->un.gyroscope.z;
}

//Return the gyro component
uint8_t BNO085::getGyroAccuracy() {
  return (gyroAccuracy);
}

//Return the gyro component
float BNO085::getUncalibratedGyroX() {
  return _sensor_value->un.gyroscopeUncal.x;
}
//Return the gyro component
float BNO085::getUncalibratedGyroY() {
  return _sensor_value->un.gyroscopeUncal.y;
}
//Return the gyro component
float BNO085::getUncalibratedGyroZ() {
  return _sensor_value->un.gyroscopeUncal.z;
}
//Return the gyro component
float BNO085::getUncalibratedGyroBiasX() {
  return _sensor_value->un.gyroscopeUncal.biasX;
}
//Return the gyro component
float BNO085::getUncalibratedGyroBiasY() {
  return _sensor_value->un.gyroscopeUncal.biasY;
}
//Return the gyro component
float BNO085::getUncalibratedGyroBiasZ() {
  return _sensor_value->un.gyroscopeUncal.biasZ;
}

//Return the gyro component
uint8_t BNO085::getUncalibratedGyroAccuracy() {
  return (UncalibGyroAccuracy);
}

float BNO085::getGravityX() {
  return _sensor_value->un.gravity.x;
}

//Return the gravity component
float BNO085::getGravityY() {
  return _sensor_value->un.gravity.y;
}

//Return the gravity component
float BNO085::getGravityZ() {
  return _sensor_value->un.gravity.z;
}

uint8_t BNO085::getGravityAccuracy() {
  return _sensor_value->status;
}

//Return the magnetometer component
float BNO085::getMagX() {
  return _sensor_value->un.magneticField.x;
}

//Return the magnetometer component
float BNO085::getMagY() {
  return _sensor_value->un.magneticField.y;
}

//Return the magnetometer component
float BNO085::getMagZ() {
  return _sensor_value->un.magneticField.z;
}

//Return the mag component
uint8_t BNO085::getMagAccuracy() {
  return _sensor_value->status;
}

//Return the tap detector
uint8_t BNO085::getTapDetector() {
  uint8_t previousTapDetector = tapDetector;
  tapDetector = 0;  //Reset so user code sees exactly one tap
  return (previousTapDetector);
}

//Return the step count
uint16_t BNO085::getStepCount() {
  return _sensor_value->un.stepCounter.steps;
}

//Return the stability classifier
uint8_t BNO085::getStabilityClassifier() {
  return _sensor_value->un.stabilityClassifier.classification;
}

//Return the activity classifier
uint8_t BNO085::getActivityClassifier() {
  return _sensor_value->un.personalActivityClassifier.mostLikelyState;
}

//Return the activity confindence
uint8_t BNO085::getActivityConfidence(uint8_t activity) {
  return _sensor_value->un.personalActivityClassifier.confidence[activity];
}

//Return the time stamp
uint64_t BNO085::getTimeStamp() {
  return _sensor_value->timestamp;
}

//Return raw mems value for the accel
int16_t BNO085::getRawAccelX() {
  return _sensor_value->un.rawAccelerometer.x;
}
//Return raw mems value for the accel
int16_t BNO085::getRawAccelY() {
  return _sensor_value->un.rawAccelerometer.y;
}
//Return raw mems value for the accel
int16_t BNO085::getRawAccelZ() {
  return _sensor_value->un.rawAccelerometer.z;
}

//Return raw mems value for the gyro
int16_t BNO085::getRawGyroX() {
  return _sensor_value->un.rawGyroscope.x;
}
int16_t BNO085::getRawGyroY() {
  return _sensor_value->un.rawGyroscope.y;
}
int16_t BNO085::getRawGyroZ() {
  return _sensor_value->un.rawGyroscope.z;
}

//Return raw mems value for the mag
int16_t BNO085::getRawMagX() {
  return _sensor_value->un.rawMagnetometer.x;
}
int16_t BNO085::getRawMagY() {
  return _sensor_value->un.rawMagnetometer.y;
}
int16_t BNO085::getRawMagZ() {
  return _sensor_value->un.rawMagnetometer.z;
}

bool BNO085::serviceBus(void) {
  sh2_service();
  return true;
}

//Send command to reset IC
bool BNO085::softReset(void) {
  int status = sh2_devReset();

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

//Set the operating mode to "On"
//(This one is for @jerabaul29)
bool BNO085::modeOn(void) {
  int status = sh2_devOn();

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

//Set the operating mode to "Sleep"
//(This one is for @jerabaul29)
bool BNO085::modeSleep(void) {
  int status = sh2_devSleep();

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

//Get the reason for the last reset
//1 = POR, 2 = Internal reset, 3 = Watchdog, 4 = External reset, 5 = Other
uint8_t BNO085::getResetReason() {
  return prodIds.entry[0].resetCause;
}

//Sends the packet to enable the rotation vector
bool BNO085::enableRotationVector(uint16_t timeBetweenReports) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SH2_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the geomagnetic rotation vector
bool BNO085::enableGeomagneticRotationVector(uint16_t timeBetweenReports) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the ar/vr stabilized rotation vector
bool BNO085::enableARVRStabilizedRotationVector(uint16_t timeBetweenReports) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the rotation vector
bool BNO085::enableGameRotationVector(uint16_t timeBetweenReports) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SH2_GAME_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the ar/vr stabilized rotation vector
bool BNO085::enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
bool BNO085::enableAccelerometer(uint16_t timeBetweenReports) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SH2_ACCELEROMETER, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
bool BNO085::enableLinearAccelerometer(uint16_t timeBetweenReports) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports);
}

//Sends the packet to enable the gravity vector
bool BNO085::enableGravity(uint16_t timeBetweenReports) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SENSOR_REPORTID_GRAVITY, timeBetweenReports);
}

//Sends the packet to enable the gyro
bool BNO085::enableGyro(uint16_t timeBetweenReports) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SENSOR_REPORTID_GYROSCOPE_CALIBRATED, timeBetweenReports);
}

//Sends the packet to enable the uncalibrated gyro
bool BNO085::enableUncalibratedGyro(uint16_t timeBetweenReports) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SENSOR_REPORTID_UNCALIBRATED_GYRO, timeBetweenReports);
}

//Sends the packet to enable the magnetometer
bool BNO085::enableMagnetometer(uint16_t timeBetweenReports) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports);
}

//Sends the packet to enable the tap detector
bool BNO085::enableTapDetector(uint16_t timeBetweenReports) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SENSOR_REPORTID_TAP_DETECTOR, timeBetweenReports);
}

//Sends the packet to enable the step counter
bool BNO085::enableStepCounter(uint16_t timeBetweenReports) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports);
}

//Sends the packet to enable the Stability Classifier
bool BNO085::enableStabilityClassifier(uint16_t timeBetweenReports) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
bool BNO085::enableRawAccelerometer(uint16_t timeBetweenReports) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SENSOR_REPORTID_RAW_ACCELEROMETER, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
bool BNO085::enableRawGyro(uint16_t timeBetweenReports) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SENSOR_REPORTID_RAW_GYROSCOPE, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
bool BNO085::enableRawMagnetometer(uint16_t timeBetweenReports) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SENSOR_REPORTID_RAW_MAGNETOMETER, timeBetweenReports);
}

//Sends the packet to enable the various activity classifiers
bool BNO085::enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable) {
  timeBetweenReports = timeBetweenReports * 1000;  // ms to us
  return enableReport(SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER, timeBetweenReports, activitiesToEnable);
}

// See 2.2 of the Calibration Procedure document 1000-4044
// Set the desired sensors to have active dynamic calibration
bool BNO085::setCalibrationConfig(uint8_t sensors) {
  int status = sh2_setCalConfig(sensors);

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

bool BNO085::tareNow(bool zAxis, sh2_TareBasis_t basis) {
  int status = sh2_setTareNow(zAxis ? TARE_AXIS_Z : TARE_AXIS_ALL, basis);

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

bool BNO085::saveTare() {
  int status = sh2_persistTare();

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

bool BNO085::clearTare() {
  int status = sh2_clearTare();

  if (status != SH2_OK) {
    return false;
  }

  return true;
}


bool BNO085::saveCalibration() {
  int status = sh2_saveDcdNow();
  if (status != SH2_OK) {
    return false;
  }
  return true;
}
