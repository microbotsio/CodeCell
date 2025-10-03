#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#pragma once

#include "Arduino.h"
#include <Wire.h>

//All the ways we can configure or talk to the BNO085, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER SH2_ACCELEROMETER
#define SENSOR_REPORTID_GYROSCOPE_CALIBRATED SH2_GYROSCOPE_CALIBRATED
#define SENSOR_REPORTID_MAGNETIC_FIELD SH2_MAGNETIC_FIELD_CALIBRATED
#define SENSOR_REPORTID_LINEAR_ACCELERATION SH2_LINEAR_ACCELERATION
#define SENSOR_REPORTID_ROTATION_VECTOR SH2_ROTATION_VECTOR
#define SENSOR_REPORTID_GRAVITY SH2_GRAVITY
#define SENSOR_REPORTID_UNCALIBRATED_GYRO SH2_GYROSCOPE_UNCALIBRATED
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR SH2_GYRO_INTEGRATED_RV
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER SH2_STEP_COUNTER
#define SENSOR_REPORTID_STABILITY_CLASSIFIER SH2_STABILITY_CLASSIFIER
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER SH2_PERSONAL_ACTIVITY_CLASSIFIER
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29
#define SENSOR_REPORTID_SHAKE_DETECTOR SH2_SHAKE_DETECTOR
#define SENSOR_REPORTID_STEP_DETECTOR SH2_STEP_DETECTOR
#define SENSOR_REPORTID_STABILITY_DETECTOR SH2_STABILITY_DETECTOR
#define SENSOR_REPORTID_PICKUP_DETECTOR SH2_PICKUP_DETECTOR
#define SENSOR_REPORTID_FLIP_DETECTOR SH2_FLIP_DETECTOR
#define SENSOR_REPORTID_SLEEP_DETECTOR SH2_SLEEP_DETECTOR
#define SENSOR_REPORTID_TILT_DETECTOR SH2_TILT_DETECTOR
#define SENSOR_REPORTID_POCKET_DETECTOR SH2_POCKET_DETECTOR
#define SENSOR_REPORTID_CIRCLE_DETECTOR SH2_CIRCLE_DETECTOR


// Reset complete packet (BNO085 Datasheet p.24 Figure 1-27)
#define EXECUTABLE_RESET_COMPLETE 0x1

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define TARE_AXIS_ALL 0x07
#define TARE_AXIS_Z 0x04

#define TARE_ROTATION_VECTOR 0
#define TARE_GAME_ROTATION_VECTOR 1
#define TARE_GEOMAGNETIC_ROTATION_VECTOR 2
#define TARE_GYRO_INTEGRATED_ROTATION_VECTOR 3
#define TARE_AR_VR_STABILIZED_ROTATION_VECTOR 4
#define TARE_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 5

bool I2CWrite(uint8_t add, uint8_t *buffer, size_t size);
bool I2CRead(uint8_t add, uint8_t *buffer, size_t size);

class BNO085 {
public:
	bool begin(TwoWire &wirePort = Wire);
	bool beginNoReset(TwoWire &wirePort);  // for wake resume: NO reset
	void setNoResetOpen(bool en);
	bool isConnected();
	static bool s_noResetOpen;

	bool enableWakeOnTapDetector(uint32_t interval_us = 0);  // wake sensor

	sh2_ProductIds_t prodIds;  ///< The product IDs returned by the sensor
	sh2_SensorValue_t sensorValue;


	uint8_t getResetReason();  // returns prodIds->resetCause

	bool enableReport(sh2_SensorId_t sensor, uint32_t interval_us = 10000, uint32_t sensorSpecific = 0);

	bool getSensorEvent();
	uint8_t getSensorEventID();

	bool softReset();  //Try to reset the IMU via software
	bool serviceBus(void);
	uint8_t resetReason();  //Query the IMU for the reason it last reset
	bool modeOn();          //Use the executable channel to turn the BNO on
	bool modeSleep();       //Use the executable channel to put the BNO to sleep

	bool enableRotationVector(uint16_t timeBetweenReports = 10);
	bool enableGeomagneticRotationVector(uint16_t timeBetweenReports = 10);
	bool enableGameRotationVector(uint16_t timeBetweenReports = 10);
	bool enableARVRStabilizedRotationVector(uint16_t timeBetweenReports);
	bool enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports);
	bool enableAccelerometer(uint16_t timeBetweenReports = 10);
	bool enableLinearAccelerometer(uint16_t timeBetweenReports = 10);
	bool enableGravity(uint16_t timeBetweenReports = 10);
	bool enableGyro(uint16_t timeBetweenReports = 10);
	bool enableUncalibratedGyro(uint16_t timeBetweenReports = 10);
	bool enableMagnetometer(uint16_t timeBetweenReports = 10);
	bool enableStepCounter(uint16_t timeBetweenReports = 10);
	bool enableStabilityClassifier(uint16_t timeBetweenReports = 10);
	bool enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable);
	bool enableRawAccelerometer(uint16_t timeBetweenReports = 10);
	bool enableRawGyro(uint16_t timeBetweenReports = 10);
	bool enableRawMagnetometer(uint16_t timeBetweenReports = 10);
	bool enableTapDetector(uint16_t timeBetweenReports = 10);
	bool enableShakeDetector(uint16_t timeBetweenReports = 10);
	bool enableStepDetector(uint16_t timeBetweenReports = 10);
	bool enablePickupDetector(uint16_t timeBetweenReports = 10);
	bool enableFlipDetector(uint16_t timeBetweenReports = 10);
	bool enableStabilityDetector(uint16_t timeBetweenReports = 10);
	bool enableSleepDetector(uint16_t timeBetweenReports = 10);
	bool enableTiltDetector(uint16_t timeBetweenReports = 10);
	bool enablePocketDetector(uint16_t timeBetweenReports = 10);
	bool enableCircleDetector(uint16_t timeBetweenReports = 10);


	float getRot_I();
	float getRot_J();
	float getRot_K();
	float getRot_R();
	float getRadianAccuracy();
	uint8_t getRot_Accuracy();

	float getGameI();
	float getGameJ();
	float getGameK();
	float getGameReal();

	float getAccelX();
	float getAccelY();
	float getAccelZ();
	uint8_t getAccelAccuracy();

	float getLinAccelX();
	float getLinAccelY();
	float getLinAccelZ();
	uint8_t getLinAccelAccuracy();

	float getGyroX();
	float getGyroY();
	float getGyroZ();
	uint8_t getGyroAccuracy();

	float getUncalibratedGyroX();
	float getUncalibratedGyroY();
	float getUncalibratedGyroZ();
	float getUncalibratedGyroBiasX();
	float getUncalibratedGyroBiasY();
	float getUncalibratedGyroBiasZ();
	uint8_t getUncalibratedGyroAccuracy();

	float getMagX();
	float getMagY();
	float getMagZ();
	uint8_t getMagAccuracy();

	float getGravityX();
	float getGravityY();
	float getGravityZ();
	uint8_t getGravityAccuracy();

	bool setCalibrationConfig(uint8_t sensors);
	bool saveCalibration();

	bool tareNow(bool zAxis = false, sh2_TareBasis_t basis = SH2_TARE_BASIS_ROTATION_VECTOR);
	bool saveTare();
	bool clearTare();

	uint8_t getTapDetector();
	uint64_t getTimeStamp();
	uint16_t getStepCount();
	uint8_t getStabilityClassifier();
	uint8_t getActivityClassifier();
	uint8_t getActivityConfidence(uint8_t activity);

	int16_t getRawAccelX();
	int16_t getRawAccelY();
	int16_t getRawAccelZ();

	int16_t getRawGyroX();
	int16_t getRawGyroY();
	int16_t getRawGyroZ();

	int16_t getRawMagX();
	int16_t getRawMagY();
	int16_t getRawMagZ();

	float getRoll();
	float getPitch();
	float getYaw();

	//	void sendCommand(uint8_t command);
	//	void sendCalibrateCommand(uint8_t thingToCalibrate);

	//Metadata functions
	// int16_t getQ1(uint16_t recordID);
	// int16_t getQ2(uint16_t recordID);
	// int16_t getQ3(uint16_t recordID);
	// float getResolution(uint16_t recordID);
	// float getRange(uint16_t recordID);
	// uint32_t readFRSword(uint16_t recordID, uint8_t wordNumber);
	// void frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize);
	// bool readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead);

	//Global Variables
	// uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
	// uint8_t shtpData[MAX_PACKET_SIZE];
	// uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
	// uint8_t commandSequenceNumber = 0;				//Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
	// uint32_t metaData[MAX_METADATA_SIZE];			//There is more than 10 words in a metadata record but we'll stop at Q point 3

	//	unsigned long _spiPortSpeed; //Optional user defined port speed
	//	uint8_t _cs;				 //Pins needed for SPI

private:

	Stream *_debugPort;        //The stream to send debug messages to if enabLED. Usually Serial.
	bool _printDebug = false;  //Flag to print debugging variables

	//These are the raw sensor values (without Q applied) pulLED from the user requested Input Report
	uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
	uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
	uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
	uint16_t rawUncalibGyroX, rawUncalibGyroY, rawUncalibGyroZ, rawBiasX, rawBiasY, rawBiasZ, UncalibGyroAccuracy;
	uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
	uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
	uint16_t rawFastGyroX, rawFastGyroY, rawFastGyroZ;
	uint16_t gravityX, gravityY, gravityZ, gravityAccuracy;
	uint8_t tapDetector;
	uint16_t stepCount;
	uint32_t timeStamp;
	uint8_t stabilityClassifier;
	uint8_t activityClassifier;
	uint8_t calibrationStatus;                             //Byte R0 of ME Calibration Response
	uint16_t memsRawAccelX, memsRawAccelY, memsRawAccelZ;  //Raw readings from MEMS sensor
	uint16_t memsRawGyroX, memsRawGyroY, memsRawGyroZ;     //Raw readings from MEMS sensor
	uint16_t memsRawMagX, memsRawMagY, memsRawMagZ;        //Raw readings from MEMS sensor

	//These Q values are defined in the datasheet but can also be obtained by querying the meta data records
	//See the read metadata example for more info
	int16_t rotationVector_Q1 = 14;
	int16_t rotationVectorAccuracy_Q1 = 12;  //Heading accuracy estimate in radians. The Q point is 12.
	int16_t accelerometer_Q1 = 8;
	int16_t linear_accelerometer_Q1 = 8;
	int16_t gyro_Q1 = 9;
	int16_t magnetometer_Q1 = 4;
	int16_t angular_velocity_Q1 = 10;
	int16_t gravity_Q1 = 8;

protected:
	virtual bool _init(int32_t sensor_id = 0);
	sh2_Hal_t _HAL;  ///< The struct representing the SH2 Hardware Abstraction Layer
};