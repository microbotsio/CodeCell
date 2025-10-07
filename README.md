# CodeCell

<img src="https://microbots.io/cdn/shop/files/penny_600x.png?v=1719926378" alt="CodeCell" width="300" align="right" style="margin-left: 20px;">

## About CodeCell

CodeCell is a family of tiny boards that help you miniaturize your DIY robots, wearables, and IoT projects with ease!  
Designed for makers, each CodeCell features an Arduino-compatible ESP32 microcontroller, onboard sensors, and smart power management - all programmable over a standard USB-C cable. The USB-C port also charges the LiPo battery that connects directly to the onboard connector.

All CodeCell modules are supported by the **CodeCell Arduino Library**, which provides simple, ready-to-use functions for light, proximity, motion, and power management.  
You can even connect your CodeCell to our companion **MicroLink App** to add wireless sliders, buttons, joysticks, real-time sensor data, and a live 20-character display — perfect for debugging or control!

---

### 🔹 Available Versions

| Model | Microcontroller | Features | Typical Use |
|:------|:----------------|:----------|:-------------|
| **CodeCell C3 Light** | ESP32-C3-MINI-1-N4 | 4MB FLash, Light + Proximity Sensor, LiPo Charging via USB-C | Basic sensing and IoT projects |
| **CodeCell C3** | ESP32-C3-MINI-1-N4 | 4MB FLash, Light + Proximity Sensor + 9-Axis IMU Motion Sensor | Robotics & Wearables |
| **CodeCell C6** | ESP32-C6-MINI-1-H8 | 8MB FLash, Wi-Fi 6 + BLE 5 + Zigbee, Light + Proximity + 9-Axis IMU Sensor | Robotics & Wearables with Low-Power Modes |
| **CodeCell C6 Drive** | ESP32-C6-MINI-1-H8 | 8MB FLash, Dual H-Bridge Motor Drivers + Light + Motion Sensors | All-in-One Robotics Controller |

---

### ✳️ Overview

Each version of CodeCell shares the same compact 1.85 cm (C3/C6) or 2.25 cm (C6 Drive) form factor and includes:

- **USB-C Port** – for programming, serial communication, and battery charging.  
- **LiPo Connector** – works with the optional 170 mAh 20C battery (1.25 mm pitch).  
- **BQ24232 Power Management** – enables dynamic power-path control for simultaneous operation and charging.  
- **Programmable GPIO + I2C Pins** – expand your project with additional sensors, displays, or actuators.  
- **CodeCell Library Support** – easy Arduino functions for reading sensors, LEDs, PWM, and sleep triggers.  
- **MicroLink App Integration** – wireless control and real-time feedback from your phone.  

---

### 📦 Included in Every Box
- CodeCell module (C3 Light / C3 / C6 / C6 Drive)  
- Four M1.2 screws  
- Three sets of female headers (optional soldering)  
- 1.25 mm battery cable  
- Optional 170 mAh LiPo battery (add-on)

---

### 🧩 Certifications & Standards
All CodeCell modules are:
- **FCC**, **CE**, **TELEC**, and **Wi-Fi Alliance** certified (via Espressif modules)  
- **RoHS compliant**  
- Built to **IPC-A-600 Class II** standards  

---

## Getting Started with the CodeCell Library

Get started and explore tutorials [here](https://microbots.io/pages/learn-codecell).

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

🧠 Combine multiple macros using the `+` operator to enable multiple sensors.

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

### 💡 Onboard LED Control
myCodeCell.LED_SetBrightness(0);   // Turn off CodeCell LED  
myCodeCell.LED_SetBrightness(10);  // Full brightness  

---

### 💤 Sleep Functions
myCodeCell.SleepTimer(uint16_t sleep_sec);  
myCodeCell.SleepProximityTrigger(uint16_t threshold);  // C6 only  
myCodeCell.SleepLightTrigger(uint16_t threshold);      // C6 only  
myCodeCell.SleepDarkTrigger(uint16_t threshold);       // C6 only  
myCodeCell.SleepTapTrigger();                          // C6 only  

---

### 📱 MicroLink App Integration
- Download the MicroLink library to connect the CodeCell with the companion smartphone app.  
- Enables wireless sensor reading, joystick control, buttons and sliders.  

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

