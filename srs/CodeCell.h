#ifndef CODECELL_H
#define CODECELL_H

#include <Arduino.h>
#include <Wire.h>

#define POWER_BAT_CHRG 0U
#define POWER_USB 1U
#define POWER_INIT 2U
#define POWER_BAT_CHRG_FULL 4U
#define USB_VOLTAGE 4260
#define MIN_BATTERY_VOLTAGE 3350

#define LED_PIN 10U
#define LED_DEFAULT_BRIGHTNESS 5U
#define LED_SLEEP_BRIGHTNESS 1U

#define LED_COLOR_RED 0XFF0000U
#define LED_COLOR_ORANGE 0XC04000U
#define LED_COLOR_YELLOW 0XA06000U
#define LED_COLOR_GREEN 0X00FF00U
#define LED_COLOR_AQUA 0X00A030U
#define LED_COLOR_PINK 0XC00020U
#define LED_COLOR_BLUE 0X0000FFU
#define LED_COLOR_WHITE 0XFFFFFFU

#define LIGHT 0
#define VCNL4040_ADDRESS 0x60
#define VCNL4040_ALS_CONF_REG 0x00
#define VCNL4040_PS_CONF1_2_REG 0x03
#define VCNL4040_PS_CONF3_MS_REG 0x04
#define VCNL4040_PROX_REG 0x08
#define VCNL4040_AMBIENT_REG 0x09
#define VCNL4040_WHITE_REG 0x0A

#define BNO085_ADDRESS 0x4A
#define MOTION_DISABLE 0
#define MOTION_ACCELEROMETER 0b1
#define MOTION_GYRO 0b10
#define MOTION_MAGNETOMETER 0b100
#define MOTION_LINEAR_ACC 0b1000
#define MOTION_GRAVITY 0b10000
#define MOTION_ROTATION 0b100000
#define MOTION_ROTATION_NO_MAG 0b10000000
#define MOTION_STEP_COUNTER 0b100000000
#define MOTION_STATE 0b1000000000
#define MOTION_TAP_DETECTOR 0b10000000000
#define MOTION_ACTIVITY 0b100000000000


class CodeCell {
private:
  bool _LED_Breathing_flag = 0;
  uint16_t _msense = 0U;
  uint16_t _LED_Breathing_counter = 0U;
  uint8_t _bat_flag = 0U;
  uint8_t _charge_state = POWER_INIT;
  uint8_t _i2c_write_array[10] = { 0 };
  uint8_t _i2c_read_array[10] = { 0 };
  uint8_t _i2c_write_size = 0;
  uint16_t xxtemp = 0;
  float xx = 0;
  float yy = 0;
  float zz = 0;

public:
  CodeCell();
  void Init(uint16_t sense_motion);
  void Test();
  void USBSleep(bool cable_polarity);
  void Run();
  void PrintSensors();
  uint16_t BatteryRead();

  void LED_Breathing(uint32_t rgb_color_24bit);
  void LED(uint8_t r, uint8_t g, uint8_t b);

  bool Light_Init();
  void LightReset();
  uint16_t Light_ProximityRead();
  uint16_t Light_WhiteRead();
  uint16_t Light_AmbientRead();

  void Motion_Init();
  void Motion_Read();
  void Motion_AccelerometerRead(float &x, float &y, float &z);
  void Motion_GyroRead(float &x, float &y, float &z);
  void Motion_MagnetometerRead(float &x, float &y, float &z);
  void Motion_GravityRead(float &x, float &y, float &z);
  void Motion_LinearAccRead(float &x, float &y, float &z);
  void Motion_TapRead(uint16_t &x);
  void Motion_StepCounterRead(uint16_t &x);
  void Motion_RotationRead(float &roll, float &pitch, float &yaw);
  void Motion_RotationNoMagRead(float &roll, float &pitch, float &yaw);
  void Motion_StateRead(uint16_t &x);
  void Motion_ActivityRead(uint16_t &x);
};

#endif