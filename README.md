# CodeCell

<img src="https://microbots.io/cdn/shop/files/penny_600x.png?v=1719926378" alt="CodeCell" width="300" align="right" style="margin-left: 20px;">

CodeCell is a penny-sized board that helps you miniaturize your DIY robots, wearables, and IoT projects with ease! Designed for all makers, it features an Arduino-friendly ESP32-C3 Wi-Fi & BLE module, programmable through a standard USB-C cable ~ which can also charge a LiPo battery that easily plugs into the onboard connector. That’s not all! We've created a CodeCell library to make it easier to interact with the onboard sensors ~ its Light Sensor can detect darkness and measure proximity up to 20 cm! While the optional Motion Sensor features a 9-axis sensor-fusion IMU to measure angular rotations, step counts, personal activity, tapping, gravity, acceleration, rate of rotation, magnetic fields, and more!

## Getting Started with the CodeCell Library

A full guide to get you started with CodeCell can be found [here](https://microbots.io/pages/learn-codecell).

---

### 🧰 Arduino IDE Setup

Make sure the ESP32 boards package is installed  
(*Boards Manager → search for "esp32" by Espressif*).

#### For CodeCell C3 (ESP32-C3)
Board: ESP32C3 Dev Module  
USB CDC On Boot: Enabled        // Required for Serial over USB  
CPU Frequency: 160 MHz  
Flash Size: 4 MB (32 Mb)  
Partition Scheme: Minimal SPIFFS (1.9MB APP with OTA / 190KB SPIFFS)  
Port: Select the COM port for your CodeCell C3

#### For CodeCell C6 (ESP32-C6)
Board: ESP32C6 Dev Module  
USB CDC On Boot: Enabled        // Required for Serial over USB  
CPU Frequency: 160 MHz  
Flash Size: 8 MB (64 Mb)  
Partition Scheme: 8M with SPIFFS (3 MB APP / 1.5 MB SPIFFS)  
Port: Select the COM port for your CodeCell C6

---

### ⚙️ Init()

To initialize the CodeCell, use the `myCodeCell.Init()` function with one or more predefined macros.  
Each macro corresponds to a specific sensing function.

#### Available Macros
LIGHT                          // Light + proximity sensing  
MOTION_ACCELEROMETER           // 3-axis acceleration  
MOTION_GYRO                    // 3-axis angular velocity  
MOTION_MAGNETOMETER            // 3-axis magnetic field  
MOTION_LINEAR_ACC              // Linear acceleration  
MOTION_GRAVITY                 // Gravity vector  
MOTION_ROTATION                // Roll, pitch, yaw (with mag)  
MOTION_ROTATION_NO_MAG         // Roll, pitch, yaw (no mag)  
MOTION_STEP_COUNTER            // Step count  
MOTION_STATE                   // On table / Stationary / Stable / In motion  
MOTION_TAP_DETECTOR            // Tap detection  
MOTION_ACTIVITY                // Activity classification  

#### Example Usage
myCodeCell.Init(LIGHT);                                 // Light only  
myCodeCell.Init(LIGHT + MOTION_ROTATION);               // Light + orientation  
myCodeCell.Init(LIGHT + MOTION_ROTATION + MOTION_STATE);// Light + orientation + state  

💡 Combine multiple macros using the `+` operator to enable multiple sensors.

---

### 🔁 Run()

The `myCodeCell.Run()` function reads sensor data at a defined frequency and manages power and LED status.

#### Example
if (myCodeCell.Run(10)) {
  // Executes code every 10 Hz (every 100 ms)
}

- `myCodeCell.Run(10)` → returns `true` at 10 Hz (every 100 ms)  
- Read all enabled sensors at this rate  
- Also handles the LED power indicator and power status 

**Low battery behavior:**  
Below ~3.3 V → LED blinks red ×10 → enters sleep until USB is reconnected  
On USB → continues running loop(); LED shows charging/charged status  

---

### 🔋 Power & Battery Functions

#### PowerStateRead()
uint8_t state = myCodeCell.PowerStateRead();

Return values:
0 = Running from LiPo Battery  
1 = Running from USB Power  
2 = Power Initializing  
3 = Battery Low  
4 = Battery Fully Charged  
5 = Battery Charging  

#### BatteryVoltageRead()
int mv = myCodeCell.BatteryVoltageRead();   // e.g. 3720 (mV)  
Returns the LiPo voltage in millivolts — great for thresholds or logging.

#### BatteryLevelRead()
uint16_t lvl = myCodeCell.BatteryLevelRead();

Return values:
1–100  = Approximate battery percentage  
101    = Charging (USB connected, charging)  
102    = USB Power (bypassing battery)  

Example:
if (lvl <= 100) { Serial.println("Battery: " + String(lvl) + "%"); }  
else if (lvl == 101) { Serial.println("Charging..."); }  
else if (lvl == 102) { Serial.println("USB Power"); }  

---

### 📈 Reading Sensors

After initialization, you can read the sensors using these functions:

Light_ProximityRead()                                           // Proximity  
Light_WhiteRead()                                               // White light intensity  
Light_AmbientRead()                                             // Ambient light  
Motion_TapRead()                                                // Tap detection (1 = tap, 0 = none)  
Motion_StepCounterRead()                                        // Step count  
Motion_StateRead()                                              // On Table=1, Stationary=2, Stable=3, Motion=4  
Motion_ActivityRead()                                           // Driving=1, Cycling=2, Walking=3/6, Still=4, Tilting=5, Running=7, Climbing=8  
Motion_AccelerometerRead(float &x, float &y, float &z)          // Accelerometer (m/s²)  
Motion_GyroRead(float &x, float &y, float &z)                   // Gyroscope (°/s)  
Motion_MagnetometerRead(float &x, float &y, float &z)           // Magnetometer (µT)  
Motion_GravityRead(float &x, float &y, float &z)                // Gravity vector  
Motion_LinearAccRead(float &x, float &y, float &z)              // Linear acceleration  
Motion_RotationRead(float &roll, float &pitch, float &yaw)      // Rotation (degrees)  
Motion_RotationNoMagRead(float &roll, float &pitch, float &yaw) // Rotation w/o mag  
Motion_RotationVectorRead(float &vec_r, float &vec_i, float &vec_j, float &vec_k) // Quaternion  

#### Example Usage
uint16_t proximity = myCodeCell.Light_ProximityRead();  
myCodeCell.Motion_AccelerometerRead(x, y, z);  
myCodeCell.Motion_RotationRead(roll, pitch, yaw);  

🧠 Tip: Use `myCodeCell.PrintSensors();` to print all enabled sensor readings to the Serial Monitor.

---

### ⚡ GPIO Control
myCodeCell.pinWrite(uint8_t pin, bool value);              // Digital write  
myCodeCell.pinRead(uint8_t pin);                           // Digital read  
myCodeCell.pinPWM(uint8_t pin, uint16_t freq, uint8_t dc); // PWM output (Hz, duty%)  
myCodeCell.pinADC(uint8_t pin);                            // Analog read  

---

### 💡 LED Control
myCodeCell.LED_SetBrightness(0);   // Turn off CodeCell LED  
myCodeCell.LED_SetBrightness(10);  // Full brightness  

---

### 😴 Sleep Functions
myCodeCell.SleepTimer(uint16_t sleep_sec);  
myCodeCell.SleepProximityTrigger(uint16_t threshold);  // C6 only  
myCodeCell.SleepLightTrigger(uint16_t threshold);      // C6 only  
myCodeCell.SleepDarkTrigger(uint16_t threshold);       // C6 only  
myCodeCell.SleepTapTrigger();                          // C6 only  

---

### 📱 MicroLink App Integration
- Download the MicroLink library to connect the CodeCell with the companion smartphone app.  
- Enables wireless sensor streaming, joystick control, and firmware communication.  

---

## Attribution
This 'CodeCell' library contains various features, including device intialization, power managment, light sensning and motion sensing. The VCNL4040 light sensor code does not rely on any external libraries. But some of the BNO085 Motion-sensor functions were adapted from the [SparkFun BNO08x Arduino Library](https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library) and the official library provided by [CEVA](https://github.com/ceva-dsp/sh2/tree/main) for the SH2 sensor hub.

The SparkFun BNO08x library, originally written by Nathan Seidle and adjusted by Pete Lewis at SparkFun Electronics, is released under the MIT License. Significant modifications were made to adapt it and integrate into the 'CodeCell' library.

Additionally, this project incorporates the official [CEVA SH2 sensor hub library files](https://github.com/ceva-dsp/sh2/tree/main), which is licensed under the [Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0).

CEVA’s notice is as follows:

This software is licensed from CEVA, Inc.  
Copyright (c) CEVA, Inc. and its licensors. All rights reserved.  
CEVA and the CEVA logo are trademarks of CEVA, Inc.  
For more information, visit [CEVA's website](https://www.ceva-dsp.com/app/motion-sensing/).

