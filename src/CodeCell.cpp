#include "CodeCell.h"
#include "BNO085.h"

bool serial_flag = 0;
bool cc_timeflag = 1;
esp_sleep_wakeup_cause_t cc_wakeup_reason;
hw_timer_t *cctimer = NULL;

BNO085 Motion;


void IRAM_ATTR cc_Timer() {
  cc_timeflag = 1;
}

CodeCell::CodeCell() {
}

void CodeCell::Init(uint16_t sense_motion) {
  uint8_t light_timer = 0;
  
  delay(1200); /*Uart Setup delay*/
  
  if(Serial){
    serial_flag = 1;
  }

  if (WakeUpCheck()) {
    Serial.println("Waking up..");
    _charge_state = POWER_INIT;
    _chrg_counter = 0;
  } else if (cc_wakeup_reason == ESP_SLEEP_WAKEUP_GPIO) {
    Serial.println("Waking up..");
    _charge_state = POWER_INIT;
    _chrg_counter = 0;
  } else {
    Serial.println(" ");
    Serial.println("Hello World! I am your CodeCell");
  }

  _msense = sense_motion;
  pinMode(0, INPUT); /*Set up Cable indication pin*/
  pinMode(4, INPUT); /*Set up Battery Voltage Read pin*/
  analogSetAttenuation(ADC_11db);
  delay(100);

  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(1, LOW); /*Init Set up to output low*/
  digitalWrite(2, LOW); /*Init Set up to output low*/
  digitalWrite(3, LOW); /*Init Set up to output low*/
  digitalWrite(5, LOW); /*Init Set up to output low*/
  digitalWrite(6, LOW); /*Init Set up to output low*/
  digitalWrite(7, LOW); /*Init Set up to output low*/

  delay(1200); /*Uart Setup delay*/

  Serial.print("Please wait while I setup my sensors.. ");

  while (Light_Init() == 1) {
    delay(1);
    LightReset();
    if (light_timer < 50U) {
      light_timer++;
    } else {
      Serial.println("CodeCell Error - Light Sensor not found");
      esp_restart();
    }
  }

  if (_msense != MOTION_DISABLE) {
    Serial.print("Light Sensing Enabled + ");
    Motion_Init();
  } else {
    Serial.println("Light Sensing Enabled..");
  }
  pinMode(LED_PIN, OUTPUT);   /*Set LED pin as output*/
  digitalWrite(LED_PIN, LOW); /*Init Set up to output low*/
  delay(100);
  rmtInit(LED_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000); /*Configure RMT to run the onboard addressable LED*/

  LED(0, 0, 0);
  LED(0, 0, LED_SLEEP_BRIGHTNESS);
  delay(80);
  LED(LED_SLEEP_BRIGHTNESS, 0, 0);
  delay(80);
  LED(0, LED_SLEEP_BRIGHTNESS, 0);
  delay(80);
  LED(0, 0, 0); /*Clear LED*/

  Serial.print("Printing Sensors Readings.. ");
  delay(80);
  PrintSensors();

  cctimer = timerBegin(1000000);            /*Set timer frequency to 1Mhz*/
  timerAttachInterrupt(cctimer, &cc_Timer); /*Attach interrupt*/
  timerAlarm(cctimer, 100000, true, 0);     /*Set alarm to trigger every 100ms with auto-reload*/


  Serial.println("Now running my default code..");
}

void CodeCell::Test() {
  digitalWrite(1, HIGH); /*Init Set up to output HIGH*/
  digitalWrite(2, HIGH); /*Init Set up to output HIGH*/
  digitalWrite(3, HIGH); /*Init Set up to output HIGH*/
  digitalWrite(5, HIGH); /*Init Set up to output HIGH*/
  digitalWrite(6, HIGH); /*Init Set up to output HIGH*/
  digitalWrite(7, HIGH); /*Init Set up to output HIGH*/
  delay(500);
  digitalWrite(1, LOW); /*Init Set up to output low*/
  digitalWrite(2, LOW); /*Init Set up to output low*/
  digitalWrite(3, LOW); /*Init Set up to output low*/
  digitalWrite(5, LOW); /*Init Set up to output low*/
  digitalWrite(6, LOW); /*Init Set up to output low*/
  digitalWrite(7, LOW); /*Init Set up to output low*/
}


bool CodeCell::WakeUpCheck() {

  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();
  cc_wakeup_reason = wakeup_reason;

  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    return true;
  } else {
    return false;
  }
}

void CodeCell::Sleep(uint16_t sleep_sec) {
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);

  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF3_MS_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x800 & 0xFF);           /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x800 >> 8) & 0xFF);    /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println("CodeCell Error - Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_CONF_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);         /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x01 >> 8) & 0xFF);  /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println("CodeCell Error - Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF1_2_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);           /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x01 >> 8) & 0xFF);    /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println("CodeCell Error - Light Sensor not found");
  }
  _i2c_write_size = 0;

  if (_msense != MOTION_DISABLE) {
    Motion.modeSleep();
  }

  digitalWrite(10, LOW);
  pinMode(10, INPUT);
  Wire.end();  // Stop I2C

  pinMode(8, INPUT);
  pinMode(9, INPUT);
  LED(0, 0, 0);

  esp_sleep_enable_timer_wakeup(sleep_sec * 1000000ULL);
  esp_deep_sleep_start();
}

void CodeCell::USBSleep(bool cable_polarity) {
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);

  if (cable_polarity) {
    Serial.println("Shutting down application.."); /*Shut down but do not go to sleep to allow reprogramming*/
    delay(5000);
    while (digitalRead(0) == 0) {
      delay(100); /*Delay to ensure Serial output is visible*/
    }
    esp_restart();
  } else {
    for (int er = 0; er < 10; er++) {
      LED(255U, 0, 0); /*Set LED to the minimum brightness Red*/
      delay(1000);
      LED(0, 0, 0); /*Set LED to the minimum brightness Red*/
      delay(1000);
    }

    _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF3_MS_REG; /*Address*/
    _i2c_write_array[_i2c_write_size++] = (0x800 & 0xFF);           /*LSB*/
    _i2c_write_array[_i2c_write_size++] = ((0x800 >> 8) & 0xFF);    /*MSB*/
    if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
      Serial.println("CodeCell Error - Light Sensor not found");
    }
    _i2c_write_size = 0;
    _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_CONF_REG; /*Address*/
    _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);         /*LSB*/
    _i2c_write_array[_i2c_write_size++] = ((0x01 >> 8) & 0xFF);  /*MSB*/
    if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
      Serial.println("CodeCell Error - Light Sensor not found");
    }
    _i2c_write_size = 0;
    _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF1_2_REG; /*Address*/
    _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);           /*LSB*/
    _i2c_write_array[_i2c_write_size++] = ((0x01 >> 8) & 0xFF);    /*MSB*/
    if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
      Serial.println("CodeCell Error - Light Sensor not found");
    }
    _i2c_write_size = 0;

    if (_msense != MOTION_DISABLE) {
      Motion.modeSleep();
    }
    delay(1000);

    digitalWrite(10, LOW);
    pinMode(10, INPUT);
    Wire.end();  // Stop I2C

    pinMode(8, INPUT);
    pinMode(9, INPUT);

    esp_deep_sleep_enable_gpio_wakeup(1 << 0, ESP_GPIO_WAKEUP_GPIO_LOW); /*Set Gpio0 as wakeup pin*/
    esp_deep_sleep_start();
    /*Waiting to wake up*/
  }
}

void CodeCell::PrintSensors() {
  Serial.print("Proximity: ");
  Serial.print(Light_ProximityRead());
  Serial.print(" cts, White: ");
  Serial.print(Light_WhiteRead());
  Serial.print(" lx, Ambient: ");
  Serial.print(Light_AmbientRead());
  Serial.print(" lx");

  if (_msense != MOTION_DISABLE) {
    if ((_msense & MOTION_ACCELEROMETER) == MOTION_ACCELEROMETER) {
      Motion_AccelerometerRead(xx, yy, zz);
      Serial.printf(", Acc XYZ: %.2f, %.2f, %.2f m/s^2, ", xx, yy, zz);
    }
    if ((_msense & MOTION_GYRO) == MOTION_GYRO) {
      Motion_GyroRead(xx, yy, zz);
      Serial.printf(", Gyro XYZ: %.2f, %.2f, %.2f rad/sec, ", xx, yy, zz);
    }
    if ((_msense & MOTION_MAGNETOMETER) == MOTION_MAGNETOMETER) {
      Motion_MagnetometerRead(xx, yy, zz);
      Serial.printf(", Magnetometer XYZ: %.2f, %.2f, %.2f uT, ", xx, yy, zz);
    }
    if ((_msense & MOTION_LINEAR_ACC) == MOTION_LINEAR_ACC) {
      Motion_LinearAccRead(xx, yy, zz);
      Serial.printf(", Linear Acc XYZ: %.2f, %.2f, %.2f m/s^2, ", xx, yy, zz);
    }
    if ((_msense & MOTION_GRAVITY) == MOTION_GRAVITY) {
      Motion_GravityRead(xx, yy, zz);
      Serial.printf(", Gravity XYZ: %.2f, %.2f, %.2f m/s^2, ", xx, yy, zz);
    }
    if ((_msense & MOTION_ROTATION) == MOTION_ROTATION) {
      Motion_RotationRead(xx, yy, zz);
      Serial.printf(", Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°, ", xx, yy, zz);
    }
    if ((_msense & MOTION_ROTATION_NO_MAG) == MOTION_ROTATION_NO_MAG) {
      Motion_RotationNoMagRead(xx, yy, zz);
      Serial.printf(", Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°, ", xx, yy, zz);
    }
    if ((_msense & MOTION_STEP_COUNTER) == MOTION_STEP_COUNTER) {
      Motion_StepCounterRead(xxtemp);
      Serial.print(", Steps: ");
      Serial.print(xxtemp);
    }
    if ((_msense & MOTION_TAP_DETECTOR) == MOTION_TAP_DETECTOR) {
      Motion_TapRead(xxtemp);
      Serial.print(", Tap: ");
      Serial.print(xxtemp);
    }
    if ((_msense & MOTION_STATE) == MOTION_STATE) {
      Motion_StateRead(xxtemp);
      Serial.print(", Current State: ");
      switch (xxtemp) {
        case STABILITY_CLASSIFIER_STABLE:
          Serial.print("Stable");
          break;
        case STABILITY_CLASSIFIER_ON_TABLE:
          Serial.print("On Table");
          break;
        case STABILITY_CLASSIFIER_STATIONARY:
          Serial.print("Stationary");
          break;
        case STABILITY_CLASSIFIER_MOTION:
          Serial.print("Motion");
          break;
        default:
          Serial.print("Unkown");
          break;
      }
    }
    if ((_msense & MOTION_ACTIVITY) == MOTION_ACTIVITY) {
      Motion_ActivityRead(xxtemp);
      Serial.print(", Activity: ");
      switch (xxtemp) {
        case 1:
          Serial.print("Driving");
          break;
        case 2:
          Serial.print("Cycling");
          break;
        case 3:
        case 6:
          Serial.print("Walking");
          break;
        case 4:
          Serial.print("Still");
          break;
        case 5:
          Serial.print("Tilting");
          break;
        case 7:
          Serial.print("Running");
          break;
        case 8:
          Serial.print("Climbing Stairs");
          break;
        default:
          Serial.print("Unkown");
          break;
      }
    }
  }
  Serial.println(" ");
}

uint16_t CodeCell::BatteryRead() {
  double voltage = (double)analogRead(4) * 1.448;  //* 5930 / 4095
  uint16_t uvoltage = (uint16_t)voltage;
  return uvoltage;
}

bool CodeCell::Run() {
  bool tt_100ms = 0;
  if (cc_timeflag) {
    tt_100ms = 1;
    cc_timeflag = 0;
    uint16_t battery_voltage = BatteryRead();

    if ((battery_voltage > USB_VOLTAGE) || (digitalRead(0) == 0)) {
      _lowvoltage_counter = 0;
      _chrg_counter = 0;
      if (digitalRead(0) == 0) {
        Serial.print("Battery is now charging.. ");
        LED(0, 0, LED_SLEEP_BRIGHTNESS); /*Set LED to the minimum Blue*/
        USBSleep(1);
        battery_voltage = BatteryRead();
        if (battery_voltage > USB_VOLTAGE) {
          _charge_state = POWER_INIT;
        } else {
          _charge_state = POWER_BAT_CHRG_FULL;
        }
      } else if (_charge_state == POWER_INIT) {
        /*Battery First Time Check*/
        Serial.println("Battery not detected.. Running from USB Power..");
        _charge_state = POWER_USB;
      } else {
        /*USB Cabel connected without battery - Run application*/
        LED_Breathing(LED_COLOR_BLUE);
        if ((!Serial)&&(serial_flag == 1)) {
          Serial.end();         /*Close the current Serial connection*/
          delay(500);           /*Short delay before reinitializing*/
          Serial.begin(115200); /*Reinitialize the Serial connection*/
          Serial.println("Serial connection re-established!");
          delay(100);    /*Short delay before reinitializing*/
          esp_restart(); /*Restart the CodeCell*/
        }
      }
    } else {
      if ((_charge_state == POWER_BAT_CHRG_FULL) || (_charge_state == POWER_INIT)) {
        /*Battery has been charged or USB is disconnected*/
        Serial.println("Running from Battery Power.. ");
        _charge_state = POWER_BAT_CHRG; /*Battery is at an operating voltage level*/
        _chrg_counter = 0;
        _lowvoltage_counter = 0;
      } else {
        if (battery_voltage < MIN_BATTERY_VOLTAGE) {
          /*Voltage is less than 3.3V or higher than 4.3V*/
          if (_lowvoltage_counter < 10) {
            _lowvoltage_counter++;
          } else {
            Serial.println("CodeCell Battery Volotage Error..");
            _chrg_counter = 0;
            USBSleep(0);
          }
        } else {
          _lowvoltage_counter = 0;
          if (_chrg_counter < 10) {
            _chrg_counter++;
          } else {
            _charge_state = POWER_BAT_CHRG; /*Battery is at an operating voltage level*/
            LED_Breathing(LED_COLOR_GREEN);
          }
        }
      }
    }
  } else {
    /*wait*/
  }
  return tt_100ms;
}

void CodeCell::LED(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(LED_PIN, r, g, b); /*RMT ESP32 function for addressable LEDs*/
}

void CodeCell::LED_Breathing(uint32_t rgb_color_24bit) {

  uint32_t r_counter = (rgb_color_24bit >> 16) & 0xFFU;
  uint32_t g_counter = (rgb_color_24bit >> 8) & 0xFFU;
  uint32_t b_counter = rgb_color_24bit & 0xFFU;
  uint16_t getProx = Light_ProximityRead();

  if (getProx > 2000U) {
    getProx = 2000U;
  }

  getProx = (getProx / 100) + 2;
  if (getProx > 10U) {
    getProx = 10U;
  }
  getProx = getProx << 1U;

  if (_LED_Breathing_flag == 1) {
    if (_LED_Breathing_counter <= getProx) {
      _LED_Breathing_counter++;
    } else {
      _LED_Breathing_flag = 0;
    }
  } else {
    if (_LED_Breathing_counter >= 1) {
      _LED_Breathing_counter--;
    } else {
      _LED_Breathing_flag = 1;
    }
  }

  r_counter = (r_counter * _LED_Breathing_counter) / 22U;
  g_counter = (g_counter * _LED_Breathing_counter) / 22U;
  b_counter = (b_counter * _LED_Breathing_counter) / 22U;

  LED(r_counter, g_counter, b_counter); /*Set LED to the default Blue*/
}

uint16_t CodeCell::Light_AmbientRead() {
  uint16_t value = 0;
  Wire.beginTransmission(VCNL4040_ADDRESS);
  Wire.write(VCNL4040_AMBIENT_REG);
  Wire.endTransmission(false); /*Don't release the bus*/
  I2CRead(VCNL4040_ADDRESS, _i2c_read_array, 2);
  Wire.endTransmission();
  value = _i2c_read_array[0];         /*LSB*/
  value |= (_i2c_read_array[1] << 8); /*MSB*/
  return value;
}

uint16_t CodeCell::Light_WhiteRead() {
  uint16_t value = 0;
  Wire.beginTransmission(VCNL4040_ADDRESS);
  Wire.write(VCNL4040_WHITE_REG);
  Wire.endTransmission(false); /*Don't release the bus*/
  I2CRead(VCNL4040_ADDRESS, _i2c_read_array, 2);
  Wire.endTransmission();
  value = _i2c_read_array[0];         /*LSB*/
  value |= (_i2c_read_array[1] << 8); /*MSB*/
  return value;
}

uint16_t CodeCell::Light_ProximityRead() {
  uint16_t value = 0;
  Wire.beginTransmission(VCNL4040_ADDRESS);
  Wire.write(VCNL4040_PROX_REG);
  Wire.endTransmission(false); /*Don't release the bus*/
  I2CRead(VCNL4040_ADDRESS, _i2c_read_array, 2);
  Wire.endTransmission();
  value = _i2c_read_array[0];         /*LSB*/
  value |= (_i2c_read_array[1] << 8); /*MSB*/
  return value;
}

void CodeCell::Motion_AccelerometerRead(float &x, float &y, float &z) {
  Motion_Read();
  if (Motion.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER) {
    x = Motion.getAccelX();
    y = Motion.getAccelY();
    z = Motion.getAccelZ();
  }
}

void CodeCell::Motion_GyroRead(float &x, float &y, float &z) {
  Motion_Read();
  if (Motion.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
    x = Motion.getGyroX();
    y = Motion.getGyroY();
    z = Motion.getGyroZ();
  }
}

void CodeCell::Motion_MagnetometerRead(float &x, float &y, float &z) {
  Motion_Read();
  if (Motion.getSensorEventID() == SENSOR_REPORTID_MAGNETIC_FIELD) {
    x = Motion.getMagX();
    y = Motion.getMagY();
    z = Motion.getMagZ();
  }
}

void CodeCell::Motion_GravityRead(float &x, float &y, float &z) {
  Motion_Read();
  if (Motion.getSensorEventID() == SENSOR_REPORTID_GRAVITY) {
    x = Motion.getGravityX();
    y = Motion.getGravityY();
    z = Motion.getGravityZ();
  }
}

void CodeCell::Motion_LinearAccRead(float &x, float &y, float &z) {
  Motion_Read();
  if (Motion.getSensorEventID() == SENSOR_REPORTID_LINEAR_ACCELERATION) {
    x = Motion.getLinAccelX();
    y = Motion.getLinAccelY();
    z = Motion.getLinAccelZ();
  }
}

void CodeCell::Motion_TapRead(uint16_t &x) {
  x = 0;
  if (Motion.getSensorEvent() == true) {
    if (Motion.getSensorEventID() == SENSOR_REPORTID_TAP_DETECTOR) {
      x = 1;
    }
  }
}

void CodeCell::Motion_StepCounterRead(uint16_t &x) {
  Motion_Read();
  if (Motion.getSensorEventID() == SENSOR_REPORTID_STEP_COUNTER) {
    x = (uint16_t)Motion.getStepCount();
  }
}

void CodeCell::Motion_StateRead(uint16_t &x) {
  Motion_Read();
  if (Motion.getSensorEventID() == SENSOR_REPORTID_STABILITY_CLASSIFIER) {
    x = Motion.getStabilityClassifier();
  }
}

void CodeCell::Motion_ActivityRead(uint16_t &x) {
  //Motion_Read();
  if (Motion.getSensorEvent() == true) {
    if (Motion.getSensorEventID() == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER) {
      x = Motion.getActivityClassifier();
    }
  }
}

void CodeCell::Motion_RotationRead(float &roll, float &pitch, float &yaw) {
  float Rr = 0;
  float Ri = 0;
  float Rj = 0;
  float Rk = 0;
  float sRi = 0;
  float sRj = 0;
  float sRk = 0;
  float sRr = 0;

  Motion_Read();

  if (Motion.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
    Rr = Motion.getRot_R();
    Ri = Motion.getRot_I();
    Rj = Motion.getRot_J();
    Rk = Motion.getRot_K();
    sRi = sq(Ri);
    sRj = sq(Rj);
    sRk = sq(Rk);
    sRr = sq(Rr);

    roll = asin(-2.0 * (Ri * Rk - Rj * Rr) / (sRi + sRj + sRk + sRr)) * RAD_TO_DEG;  /*ROLL*/
    pitch = atan2(2.0 * (Rj * Rk + Ri * Rr), (-sRi - sRj + sRk + sRr)) * RAD_TO_DEG; /*PITCH*/
    yaw = atan2(2.0 * (Ri * Rj + Rk * Rr), (sRi - sRj - sRk + sRr)) * RAD_TO_DEG;    /*YAW*/
  }
}

void CodeCell::Motion_RotationNoMagRead(float &roll, float &pitch, float &yaw) {
  float Rr = 0;
  float Ri = 0;
  float Rj = 0;
  float Rk = 0;
  float sRi = 0;
  float sRj = 0;
  float sRk = 0;
  float sRr = 0;

  Motion_Read();

  if (Motion.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
    Rr = Motion.getGameReal();
    Ri = Motion.getGameI();
    Rj = Motion.getGameJ();
    Rk = Motion.getGameK();
    sRi = sq(Ri);
    sRj = sq(Rj);
    sRk = sq(Rk);
    sRr = sq(Rr);

    roll = asin(-2.0 * (Ri * Rk - Rj * Rr) / (sRi + sRj + sRk + sRr)) * RAD_TO_DEG;  /*ROLL*/
    pitch = atan2(2.0 * (Rj * Rk + Ri * Rr), (-sRi - sRj + sRk + sRr)) * RAD_TO_DEG; /*PITCH*/
    yaw = atan2(2.0 * (Ri * Rj + Rk * Rr), (sRi - sRj - sRk + sRr)) * RAD_TO_DEG;    /*YAW*/
  }
}


void CodeCell::Motion_Read() {
  uint8_t imu_timer = 0U;
  while (Motion.getSensorEvent() == false) {
    delayMicroseconds(1);
    if (imu_timer < 50U) {
      imu_timer++;
    } else {
      Motion.softReset();
      _i2c_write_size = 0;
      LightReset();
      Serial.println("CodeCell Error - Motion Sensor not found");
      esp_restart();
    }
  }
}

bool CodeCell::Light_Init() {
  bool light_error = 0;

  Wire.begin();
  /*Configure - Continuous conversion mode, high dynamic range, integration time of 80 ms*/
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_CONF_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x000 & 0xFF);        /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x000 >> 8) & 0xFF); /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    light_error = 1;
  }
  _i2c_write_size = 0;
  /*Configure - duty cycle to 1/40, 16-bit resolution*/
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF1_2_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x80E & 0xFF);          /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x80E >> 8) & 0xFF);   /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    light_error = 1;
  }
  _i2c_write_size = 0;
  /*Configure - Set LED current to 200 mA, No interrupt settings*/
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF3_MS_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x4710 & 0xFF);          /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x4710 >> 8) & 0xFF);   /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    light_error = 1;
  }
  _i2c_write_size = 0;

  return light_error;
}

void CodeCell::LightReset() {
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_CONF_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);         /*LSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println("CodeCell Error - Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF1_2_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);           /*LSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println("CodeCell Error - Light Sensor not found");
  }
  _i2c_write_size = 0;
}

void CodeCell::Motion_Init() {
  uint8_t imu_timer = 0U;
  Wire.begin();
  while (Motion.begin() == false) {
    delay(1);
    if (imu_timer < 50U) {
      imu_timer++;
    } else {
      Motion.softReset();
      LightReset();
      Serial.println("CodeCell Error - Motion Sensor not found");
      esp_restart();
    }
  }
  if ((_msense & MOTION_ACCELEROMETER) == MOTION_ACCELEROMETER) {
    if (Motion.enableAccelerometer() == true) {
      Serial.println("Accelerometer Motion Sensing Enabled.. ");
    } else {
      Serial.println("Accelerometer Motion Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_GYRO) == MOTION_GYRO) {
    if (Motion.enableGyro() == true) {
      Serial.println("Gyro Motion Sensing Enabled.. ");
    } else {
      Serial.println("Gyro Motion Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_MAGNETOMETER) == MOTION_MAGNETOMETER) {
    if (Motion.enableMagnetometer() == true) {
      Serial.println("Magnetometer Motion Sensing Enabled.. ");
    } else {
      Serial.println("Magnetometer Motion Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_LINEAR_ACC) == MOTION_LINEAR_ACC) {
    if (Motion.enableLinearAccelerometer() == true) {
      Serial.println("Linear Accelerometer Motion Sensing Enabled.. ");
    } else {
      Serial.println("Linear Accelerometer Motion Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_GRAVITY) == MOTION_GRAVITY) {
    if (Motion.enableGravity() == true) {
      Serial.println("Gravity Motion Sensing Enabled.. ");
    } else {
      Serial.println("Gravity Motion Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_ROTATION) == MOTION_ROTATION) {
    if (Motion.enableRotationVector() == true) {
      Serial.println("Rotation Motion Sensing Enabled.. ");
    } else {
      Serial.println("Rotation Motion Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_ROTATION_NO_MAG) == MOTION_ROTATION_NO_MAG) {
    if (Motion.enableGameRotationVector() == true) {
      Serial.println("Rotation Motion (Magnetometer DisabLED) Sensing Enabled.. ");
    } else {
      Serial.println("Rotation Motion (Magnetometer DisabLED) Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_STEP_COUNTER) == MOTION_STEP_COUNTER) {
    if (Motion.enableStepCounter() == true) {
      Serial.println("Step Counter Motion Sensing Enabled.. ");
    } else {
      Serial.println("Step Counter Rotation Motion Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_TAP_DETECTOR) == MOTION_TAP_DETECTOR) {
    if (Motion.enableTapDetector() == true) {
      Serial.println("Tap Detector Motion Sensing Enabled.. ");
    } else {
      Serial.println("Tap Detector Motion Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_STATE) == MOTION_STATE) {
    if (Motion.enableStabilityClassifier() == true) {
      Serial.println("Motion State Sensing Enabled.. ");
    } else {
      Serial.println("Motion State Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_ACTIVITY) == MOTION_ACTIVITY) {
    if (Motion.enableActivityClassifier(1000, 0x1F) == true) {
      Serial.println("Motion Activity Sensing Enabled.. ");
    } else {
      Serial.println("Motion Activity Sensing Failed.. ");
    }
  }
  _i2c_write_size = 0;
}
