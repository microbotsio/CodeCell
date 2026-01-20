#ifndef CODECELL_H
#define CODECELL_H

#include <Arduino.h>
#include <Wire.h>

#if defined(ARDUINO_ESP32C6_DEV)
#include <DriveCell.h>
#define LED_PIN 19U
#define LED_ON_PIN 20U
#define SENS_ON_PIN 18U
#define MOTION_WAKEUP_PIN 7U
#define LIGHT_WAKEUP_PIN 5U
#else
#define LED_PIN 10U
#endif

#define SW_VERSION "2.0.3"
#define MANUFACTURER "MICROBOTS"

#define POWER_BAT_RUN 0U
#define POWER_USB 1U
#define POWER_INIT 2U
#define POWER_BAT_LOW 3U
#define POWER_BAT_FULL 4U
#define POWER_BAT_CHRG 5U
#define USB_VOLTAGE 4180
#define MIN_BATTERY_VOLTAGE 3350

#define LED_DEFAULT_BRIGHTNESS 7U
#define LED_SLEEP_BRIGHTNESS 1U

#define LED_COLOR_RED 0XFF0000U
#define LED_COLOR_ORANGE 0XC04000U
#define LED_COLOR_YELLOW 0XA06000U
#define LED_COLOR_GREEN 0X00FF00U
#define LED_COLOR_AQUA 0X00A030U
#define LED_COLOR_PINK 0XC00020U
#define LED_COLOR_BLUE 0X0000FFU
#define LED_COLOR_WHITE 0XFFFFFFU
#define LED_OFF 0U

#define LIGHT 0b1000000000000
#define VCNL4040_ADDRESS 0x60
#define VCNL4040_ALS_CONF_REG 0x00
#define VCNL4040_ALS_THDH 0x01
#define VCNL4040_ALS_THDL 0x02
#define VCNL4040_PS_CONF1_REG 0x03
#define VCNL4040_PS_CONF3_REG 0x04
#define VCNL4040_PS_THDL 0x06
#define VCNL4040_PS_THDH 0x07
#define VCNL4040_PROX_REG 0x08
#define VCNL4040_AMBIENT_REG 0x09
#define VCNL4040_WHITE_REG 0x0A
#define VCNL4040_INT_FLAG 0x0B


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


#define MOTION_STATE_UNKNOWN 0
#define MOTION_STATE_ONTABLE 1
#define MOTION_STATE_STATIONARY 2
#define MOTION_STATE_STABLE 3
#define MOTION_STATE_MOTION 4
#define MOTION_STATE_STABLE 3


#define PIN_TYPE_OUTPUT 0
#define PIN_TYPE_INPUT 1
#define PIN_TYPE_ADC 2
#define PIN_TYPE_PWM 3
#define PWM_RES 8
#define AVRG_FILTER_SIZE 8U


class CodeCell {
private:
  bool pinCheck(uint8_t pin_num, uint8_t pin_type);
  bool _pinArray[7] = { 0, 0, 0, 0, 0, 0, 0 };
  
  bool _LED_Breathing_flag = 0;
  bool _tap_event = 0;
  bool _wakeup_flag = 0;

  uint8_t _chrg_counter = 0U;
  uint8_t _lowvoltage_counter = 0;
  uint8_t _run_frequency_last = 0;
  uint8_t _power_counter = 250;
  uint8_t _charge_state = POWER_INIT;
  uint8_t _i2c_write_array[10] = { 0 };
  uint8_t _i2c_read_array[10] = { 0 };
  uint8_t _i2c_write_size = 0;
  uint8_t _voltage_index = 0;
  uint8_t _mstate_data = 0;
  uint8_t _activity_data = 0;

  uint32_t _msense = 0U;
  uint16_t _voltage_last = 0xFFU;
  uint16_t _LED_Breathing_counter = 0U;
  uint16_t _voltage_avrg[AVRG_FILTER_SIZE] = { 0 };
  uint16_t _LED_level = LED_DEFAULT_BRIGHTNESS;
  uint16_t _step_data = 0;
  uint16_t _light_data[3] = { 0 };

  uint32_t _charge_color = 0;

  float xx = 0;
  float yy = 0;
  float zz = 0;
  float _motion_data[23] = { 0.0 };

public:
#if defined(ARDUINO_ESP32C6_DEV)
  DriveCell Drive1;
  DriveCell Drive2;
#endif
  CodeCell();
  void Init(uint32_t sense_motion);
  void PrintSensors();
  void Test();
  void USBSleep();
  void SleepTimer(uint16_t sleep_sec);
#if defined(ARDUINO_ESP32C6_DEV)
  void SleepProximityTrigger(uint16_t trigger_threshold);
  void SleepLightTrigger(uint16_t trigger_threshold);
  void SleepDarkTrigger(uint16_t trigger_threshold);
  void SleepTapTrigger();
#endif
  bool WakeUpCheck();
  bool Run(uint8_t run_frequency);
  uint16_t BatteryVoltageRead();
  uint8_t BatteryLevelRead();
  uint8_t PowerStateRead();

  void pinWrite(uint8_t pin_num, bool pin_value);
  bool pinRead(uint8_t pin_num);
  uint16_t pinADC(uint8_t pin_num);
  void pinPWM(uint8_t pin_num, uint16_t pin_freq, uint8_t pin_dutycycle);

  void LED_Breathing(uint32_t rgb_color_24bit);
  void LED(uint8_t r, uint8_t g, uint8_t b);
  void LED_SetBrightness(uint16_t level);

  bool Light_Init();
  void LightReset();
  void Light_Read();
  uint16_t Light_ProximityRead();
  uint16_t Light_WhiteRead();
  uint16_t Light_AmbientRead();

  void Motion_Init(uint32_t sense_motion);
  void Motion_Read();
  void Motion_AccelerometerRead(float &x, float &y, float &z);
  void Motion_GyroRead(float &x, float &y, float &z);
  void Motion_MagnetometerRead(float &x, float &y, float &z);
  void Motion_GravityRead(float &x, float &y, float &z);
  void Motion_LinearAccRead(float &x, float &y, float &z);
  void Motion_RotationRead(float &roll, float &pitch, float &yaw);
  void Motion_RotationVectorRead(float &vec_r, float &vec_i, float &vec_j, float &vec_k);
  void Motion_RotationNoMagRead(float &roll, float &pitch, float &yaw);
  uint16_t Motion_StateRead();
  uint16_t Motion_ActivityRead();
  uint16_t Motion_StepCounterRead();
  bool Motion_TapDetectorRead();
};


#endif


