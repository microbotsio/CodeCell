#include "CodeCell.h"
#include "BNO085.h"
#include "esp_sleep.h"

bool serial_flag = 0;
bool cc_timeflag = 1;
esp_sleep_wakeup_cause_t cc_wakeup_reason;
hw_timer_t *cctimer = NULL;

BNO085 Motion;

void IRAM_ATTR cc_Timer() {
  cc_timeflag = 1;
}

#if defined(ARDUINO_ESP32C6_DEV)
void IRAM_ATTR LightInterrupt() {
}

void IRAM_ATTR MotionInterrupt() {
}

CodeCell::CodeCell()
  : Drive2(2, 3), Drive1(22, 21) {}
#else
CodeCell::CodeCell() {}
#endif

void CodeCell::Init(uint32_t sense_motion) {
  uint8_t light_timer = 0;

#if defined(ARDUINO_ESP32C6_DEV)
  pinMode(LED_ON_PIN, OUTPUT);    /*Set LED transistor pin as output*/
  digitalWrite(LED_ON_PIN, HIGH); /*Init Set up to output high*/

  pinMode(SENS_ON_PIN, OUTPUT); /*Set LDO enable pin as output*/
  gpio_hold_dis((gpio_num_t)SENS_ON_PIN);
  digitalWrite(SENS_ON_PIN, HIGH); /*Init Set up to output high*/

  pinMode(LIGHT_WAKEUP_PIN, INPUT);   // Assumes active-low button
  pinMode(MOTION_WAKEUP_PIN, INPUT);  // Assumes active-low button
#endif

  delay(1); /*Uart Setup delay*/

  if (Serial) {
    serial_flag = 1;
  }

  if (WakeUpCheck()) {
    Serial.println(">> Waking up..");
    _charge_state = POWER_INIT;
    _chrg_counter = 0;
  } else {
    Serial.println(" ");
    Serial.println(" ");
#if defined(ARDUINO_ESP32C6_DEV)
    Serial.print(">> CodeCell C6 v" SW_VERSION);
#else
    Serial.print(">> CodeCell C3 v" SW_VERSION);
#endif
    Serial.println(" Booting.. ");
  }

  _msense = sense_motion;

  Serial.print(">> Initializing Sensors: ");

  if ((_msense & LIGHT) == LIGHT) {
    while (Light_Init() == 1) {
      delay(10);
      LightReset();
      if (light_timer < 50U) {
        light_timer++;
      } else {
        Serial.print(" Light: Failed | ");
      }
    }
    Serial.print("Light Activated");
  }
  if ((_msense & 0b111111111111) != MOTION_DISABLE) {
    if ((_msense & LIGHT) == LIGHT) {
      Serial.print(" | ");
    }
    Motion_Init(_msense);
  } else {
    //skip
  }
  Serial.println();

  pinMode(LED_PIN, OUTPUT);   /*Set LED pin as output*/
  digitalWrite(LED_PIN, LOW); /*Init Set up to output low*/
  delay(1);
  rmtInit(LED_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000); /*Configure RMT to run the onboard addressable LED*/

  if (!_wakeup_flag) {
    LED(0, 0, 0);
    delay(1);
    LED(0, 0, LED_SLEEP_BRIGHTNESS);
    delay(80);
    LED(LED_SLEEP_BRIGHTNESS, 0, 0);
    delay(80);
    LED(0, LED_SLEEP_BRIGHTNESS, 0);
    delay(80);
    LED(0, 0, 0);
  }

  pinMode(0, INPUT); /*Set up Cable indication pin*/
  pinMode(4, INPUT); /*Set up Battery Voltage Read pin*/
  analogSetAttenuation(ADC_11db);


#if defined(ARDUINO_ESP32C6_DEV)
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  digitalWrite(1, LOW);  /*Init Set up to output low*/
  digitalWrite(2, LOW);  /*Init Set up to output low*/
  digitalWrite(3, LOW);  /*Init Set up to output low*/
  digitalWrite(21, LOW); /*Init Set up to output low*/
  digitalWrite(22, LOW); /*Init Set up to output low*/
  digitalWrite(23, LOW); /*Init Set up to output low*/
#else
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
#endif

  if ((_msense & 0b111111111111) != MOTION_DISABLE) {
    Motion_Read();
    Motion_Read();
  }
  if ((_msense & LIGHT) == LIGHT) {
    Light_Read();
  }

  if (!_wakeup_flag) {
    PrintSensors();
  }
  cctimer = timerBegin(1000000);            /*Set timer frequency to 1Mhz*/
  timerAttachInterrupt(cctimer, &cc_Timer); /*Attach interrupt*/
  timerAlarm(cctimer, 100000, true, 0);     /*Set alarm to trigger every 100ms with auto-reload*/

  double double_battery_voltage = (double)analogRead(4) * 1.448;
#if defined(ARDUINO_ESP32C6_DEV)
  uint16_t battery_voltage = (uint16_t)double_battery_voltage + 1154;
#else
  uint16_t battery_voltage = (uint16_t)double_battery_voltage;
#endif

  if (battery_voltage < USB_VOLTAGE) {
    _charge_color = LED_COLOR_GREEN;
  } else if (battery_voltage < MIN_BATTERY_VOLTAGE) {
    Serial.println(">> Error: Battery Volotage");
    _chrg_counter = 0;
    USBSleep();
  } else {
    _charge_color = LED_COLOR_BLUE;
  }
  for (uint8_t vv = 0; vv < AVRG_FILTER_SIZE; vv++) {
    _voltage_avrg[vv] = battery_voltage;
  }
}

void CodeCell::Test() {
#if defined(ARDUINO_ESP32C6_DEV)
  digitalWrite(1, HIGH);  /*Init Set up to output HIGH*/
  digitalWrite(2, HIGH);  /*Init Set up to output HIGH*/
  digitalWrite(3, HIGH);  /*Init Set up to output HIGH*/
  digitalWrite(21, HIGH); /*Init Set up to output HIGH*/
  digitalWrite(22, HIGH); /*Init Set up to output HIGH*/
  digitalWrite(23, HIGH); /*Init Set up to output HIGH*/
  delay(500);
  digitalWrite(1, LOW);  /*Init Set up to output low*/
  digitalWrite(2, LOW);  /*Init Set up to output low*/
  digitalWrite(3, LOW);  /*Init Set up to output low*/
  digitalWrite(21, LOW); /*Init Set up to output low*/
  digitalWrite(22, LOW); /*Init Set up to output low*/
  digitalWrite(23, LOW); /*Init Set up to output low*/
#else
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
#endif
}

bool CodeCell::WakeUpCheck() {

  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();
  cc_wakeup_reason = wakeup_reason;

  if ((wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) || (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO)) {
    _wakeup_flag = 1;
#if defined(ARDUINO_ESP32C6_DEV)
    pinMode(LED_ON_PIN, OUTPUT);     /*Set LED transistor pin as output*/
    pinMode(SENS_ON_PIN, OUTPUT);    /*Set LDO enable pin as output*/
    digitalWrite(SENS_ON_PIN, HIGH); /*Init Set up to output high*/
    delay(1);
#endif
    return true;
  } else {
    _wakeup_flag = 0;
    return false;
  }
}

void CodeCell::SleepTimer(uint16_t sleep_sec) {
  Serial.println(">> Going To Sleep..");
  Serial.println(" ");
  if (!_wakeup_flag) {
    LED(0, 0, 0);
  }
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);

  _i2c_write_array[_i2c_write_size++] = VCNL4040_INT_FLAG;
  _i2c_write_array[_i2c_write_size++] = 0x00;  // LSB
  _i2c_write_array[_i2c_write_size++] = 0x00;  // MSB
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF3_REG;
  _i2c_write_array[_i2c_write_size++] = 0x00;  // LSB
  _i2c_write_array[_i2c_write_size++] = 0x00;  // MSB
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_CONF_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);         /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x01 >> 8) & 0xFF);  /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF1_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);         /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x01 >> 8) & 0xFF);  /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;

  if ((_msense & 0b111111111111) != MOTION_DISABLE) {
    Motion.modeSleep();
  }

  digitalWrite(10, LOW);
  pinMode(10, INPUT);
  Wire.end();  // Stop I2C

  pinMode(8, INPUT);
  pinMode(9, INPUT);

#if defined(ARDUINO_ESP32C6_DEV)
  delay(10);
  digitalWrite(LED_ON_PIN, LOW);
  digitalWrite(SENS_ON_PIN, LOW);
  delay(10);
#endif

  esp_sleep_enable_timer_wakeup(sleep_sec * 1000000ULL);
  esp_deep_sleep_start();
}


#if defined(ARDUINO_ESP32C6_DEV)
void CodeCell::SleepProximityTrigger(uint16_t trigger_threshold) {
  Serial.println(">> Going To Sleep..");
  Serial.println(" ");
  if (!_wakeup_flag) {
    LED(0, 0, 0);
  }
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);

  //Set up light sensor to wake up by interrupt

  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_THDL;
  _i2c_write_array[_i2c_write_size++] = (0 & 0xFF);        /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0 >> 8) & 0xFF); /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_THDH;
  _i2c_write_array[_i2c_write_size++] = (trigger_threshold & 0xFF);        /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((trigger_threshold >> 8) & 0xFF); /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF1_REG;
  _i2c_write_array[_i2c_write_size++] = 0xCE;  // Duty=1/320, IT=8T, PS_SD=0
  _i2c_write_array[_i2c_write_size++] = 0x01;  // PS_HD=0 (12-bit), PS_INT=01 (“close/high-only”)
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF3_REG;
  _i2c_write_array[_i2c_write_size++] = 0x10;  // SMART_PERS=1, Multi-pulse=1 (lowest on-time)
  _i2c_write_array[_i2c_write_size++] = 0x80;  // White_EN=1 (disable white), INT mode=normal, LED_I=50 mA
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_CONF_REG;
  _i2c_write_array[_i2c_write_size++] = 0x01;  // LSB (SD=1)
  _i2c_write_array[_i2c_write_size++] = 0x00;  // MSB
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;


  Wire.beginTransmission(VCNL4040_ADDRESS);
  Wire.write(VCNL4040_INT_FLAG);
  Wire.endTransmission(false); /*Don't release the bus*/
  I2CRead(VCNL4040_ADDRESS, _i2c_read_array, 2);
  Wire.endTransmission(false);
  uint8_t value = _i2c_read_array[0]; /*LSB*/
  value |= (_i2c_read_array[1] << 8); /*MSB*/

  delay(2);

  if ((_msense & 0b111111111111) != MOTION_DISABLE) {
    Motion.modeSleep();
  }
  delay(2);

  digitalWrite(LED_ON_PIN, LOW);
  digitalWrite(SENS_ON_PIN, LOW);  //Turn off motion sensor

  digitalWrite(10, LOW);
  pinMode(10, INPUT);
  Wire.end();  // Stop I2C

  pinMode(8, INPUT);
  pinMode(9, INPUT);

  pinMode(LIGHT_WAKEUP_PIN, INPUT);
  pinMode(MOTION_WAKEUP_PIN, INPUT);

  esp_deep_sleep_enable_gpio_wakeup(1ULL << LIGHT_WAKEUP_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_deep_sleep_start();
}

void CodeCell::SleepDarkTrigger(uint16_t trigger_threshold) {
  Serial.println(">> Going To Sleep..");
  Serial.println(" ");
  if (!_wakeup_flag) {
    LED(0, 0, 0);
  }
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);

  //Set up light sensor to wake up by interrupt
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_THDL;
  _i2c_write_array[_i2c_write_size++] = (trigger_threshold & 0xFF);        /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((trigger_threshold >> 8) & 0xFF); /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_THDH;
  _i2c_write_array[_i2c_write_size++] = 0xFF; /*LSB*/
  _i2c_write_array[_i2c_write_size++] = 0xFF; /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF1_REG;
  _i2c_write_array[_i2c_write_size++] = 0xCF;  //  PS_SD=1 (shutdown), Duty=1/320, IT=8T (don’t care while SD=1)
  _i2c_write_array[_i2c_write_size++] = 0x00;  // PS_INT disabled, 12-bit mode (don’t care while SD=1)
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF3_REG;
  _i2c_write_array[_i2c_write_size++] = 0x10;  //SMART_PERS=1 (doesn’t matter while PS is off)
  _i2c_write_array[_i2c_write_size++] = 0x80;  //White_EN=1 (disable ALS white channel to shave a bit more)
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_CONF_REG;
  _i2c_write_array[_i2c_write_size++] = 0x02;  /// ALS on, INT enabled, slow sampling
  _i2c_write_array[_i2c_write_size++] = 0x00;
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;


  Wire.beginTransmission(VCNL4040_ADDRESS);
  Wire.write(VCNL4040_INT_FLAG);
  Wire.endTransmission(false); /*Don't release the bus*/
  I2CRead(VCNL4040_ADDRESS, _i2c_read_array, 2);
  Wire.endTransmission(false);
  uint8_t value = _i2c_read_array[0]; /*LSB*/
  value |= (_i2c_read_array[1] << 8); /*MSB*/

  delay(2);
  if ((_msense & 0b111111111111) != MOTION_DISABLE) {
    Motion.modeSleep();
  }
  delay(2);

  digitalWrite(LED_ON_PIN, LOW);
  digitalWrite(SENS_ON_PIN, LOW);  //Turn off motion sensor

  digitalWrite(10, LOW);
  pinMode(10, INPUT);
  Wire.end();  // Stop I2C

  pinMode(8, INPUT);
  pinMode(9, INPUT);

  pinMode(LIGHT_WAKEUP_PIN, INPUT);
  pinMode(MOTION_WAKEUP_PIN, INPUT);

  esp_deep_sleep_enable_gpio_wakeup(1ULL << LIGHT_WAKEUP_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_deep_sleep_start();
}

void CodeCell::SleepLightTrigger(uint16_t trigger_threshold) {
  Serial.println(">> Going To Sleep..");
  Serial.println(" ");
  if (!_wakeup_flag) {
    LED(0, 0, 0);
  }
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);

  //Set up light sensor to wake up by interrupt
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_THDL;
  _i2c_write_array[_i2c_write_size++] = 0;
  _i2c_write_array[_i2c_write_size++] = 0;
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_THDH;
  _i2c_write_array[_i2c_write_size++] = (trigger_threshold & 0xFF);        /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((trigger_threshold >> 8) & 0xFF); /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF1_REG;
  _i2c_write_array[_i2c_write_size++] = 0xCF;  //  PS_SD=1 (shutdown), Duty=1/320, IT=8T (don’t care while SD=1)
  _i2c_write_array[_i2c_write_size++] = 0x00;  // PS_INT disabled, 12-bit mode (don’t care while SD=1)
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF3_REG;
  _i2c_write_array[_i2c_write_size++] = 0x10;  //SMART_PERS=1 (doesn’t matter while PS is off)
  _i2c_write_array[_i2c_write_size++] = 0x80;  //White_EN=1 (disable ALS white channel to shave a bit more)
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_CONF_REG;
  _i2c_write_array[_i2c_write_size++] = 0x02;  /// ALS on, INT enabled, slow sampling
  _i2c_write_array[_i2c_write_size++] = 0x00;
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;


  Wire.beginTransmission(VCNL4040_ADDRESS);
  Wire.write(VCNL4040_INT_FLAG);
  Wire.endTransmission(false); /*Don't release the bus*/
  I2CRead(VCNL4040_ADDRESS, _i2c_read_array, 2);
  Wire.endTransmission(false);
  uint8_t value = _i2c_read_array[0]; /*LSB*/
  value |= (_i2c_read_array[1] << 8); /*MSB*/

  delay(2);
  if ((_msense & 0b111111111111) != MOTION_DISABLE) {
    Motion.modeSleep();
  }
  delay(2);

  digitalWrite(LED_ON_PIN, LOW);
  digitalWrite(SENS_ON_PIN, LOW);  //Turn off motion sensor

  digitalWrite(10, LOW);
  pinMode(10, INPUT);
  Wire.end();  // Stop I2C

  pinMode(8, INPUT);
  pinMode(9, INPUT);

  pinMode(LIGHT_WAKEUP_PIN, INPUT);
  pinMode(MOTION_WAKEUP_PIN, INPUT);

  esp_deep_sleep_enable_gpio_wakeup(1ULL << LIGHT_WAKEUP_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_deep_sleep_start();
}

void CodeCell::SleepTapTrigger() {
  Serial.println(">> Going To Sleep..");
  Serial.println(" ");

  if (!_wakeup_flag) {
    LED(0, 0, 0);  // turn off LEDs
  }

  // Quiet other peripherals (unchanged)
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);

  // Clear VCNL4040 interrupt flags
  _i2c_write_array[_i2c_write_size++] = VCNL4040_INT_FLAG;
  _i2c_write_array[_i2c_write_size++] = 0x00;  // LSB
  _i2c_write_array[_i2c_write_size++] = 0x00;  // MSB
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF3_REG;
  _i2c_write_array[_i2c_write_size++] = 0x00;
  _i2c_write_array[_i2c_write_size++] = 0x00;
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_CONF_REG;
  _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);
  _i2c_write_array[_i2c_write_size++] = ((0x01 >> 8) & 0xFF);
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF1_REG;
  _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);
  _i2c_write_array[_i2c_write_size++] = ((0x01 >> 8) & 0xFF);
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;

  digitalWrite(10, LOW);
  pinMode(10, INPUT);
  digitalWrite(LED_ON_PIN, LOW);

  bool ok = Motion.enableWakeOnTapDetector(10000);  // Enable tap detector as wake sensor
  Motion.modeSleep();                               // Put sensor hub into device sleep

  Wire.end();  // Release I2C bus lines
  pinMode(8, INPUT);
  pinMode(9, INPUT);

  digitalWrite(SENS_ON_PIN, HIGH);  // Hold the IMU’s LDO enable pin high (already done in your code)
  gpio_hold_en((gpio_num_t)SENS_ON_PIN);

  delay(100);

  esp_deep_sleep_enable_gpio_wakeup(1ULL << MOTION_WAKEUP_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_deep_sleep_start();
}

#endif


uint8_t CodeCell::PowerStateRead() {
  return _charge_state;
}

void CodeCell::USBSleep() {
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);

#if defined(ARDUINO_ESP32C6_DEV)
  digitalWrite(LED_ON_PIN, LOW);
  digitalWrite(SENS_ON_PIN, LOW);
#endif
  LED(0, 0, 0); /*Turn off LED*/

  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF3_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x800 & 0xFF);        /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x800 >> 8) & 0xFF); /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_CONF_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);         /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x01 >> 8) & 0xFF);  /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF1_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);         /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x01 >> 8) & 0xFF);  /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;

  if ((_msense & 0b111111111111) != MOTION_DISABLE) {
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

void CodeCell::PrintSensors() {
  Serial.print(">> Reading Sensors: ");
  if ((_msense & LIGHT) == LIGHT) {
    Serial.print("Proximity: ");
    Serial.print(Light_ProximityRead());
    Serial.print(" cts | White: ");
    Serial.print(Light_WhiteRead());
    Serial.print(" lx | Ambient: ");
    Serial.print(Light_AmbientRead());
    Serial.print(" lx |");
  }

  if ((_msense & 0b111111111111) != MOTION_DISABLE) {
    if ((_msense & MOTION_ACCELEROMETER) == MOTION_ACCELEROMETER) {
      Motion_AccelerometerRead(xx, yy, zz);
      Serial.printf(" Acc XYZ: %.2f, %.2f, %.2f m/s^2 |", xx, yy, zz);
    }
    if ((_msense & MOTION_GYRO) == MOTION_GYRO) {
      Motion_GyroRead(xx, yy, zz);
      Serial.printf(" Gyro XYZ: %.2f, %.2f, %.2f rad/sec |", xx, yy, zz);
    }
    if ((_msense & MOTION_MAGNETOMETER) == MOTION_MAGNETOMETER) {
      Motion_MagnetometerRead(xx, yy, zz);
      Serial.printf(" Magnetometer XYZ: %.2f, %.2f, %.2f uT |", xx, yy, zz);
    }
    if ((_msense & MOTION_LINEAR_ACC) == MOTION_LINEAR_ACC) {
      Motion_LinearAccRead(xx, yy, zz);
      Serial.printf(" Linear Acc XYZ: %.2f, %.2f, %.2f m/s^2 |", xx, yy, zz);
    }
    if ((_msense & MOTION_GRAVITY) == MOTION_GRAVITY) {
      Motion_GravityRead(xx, yy, zz);
      Serial.printf(" Gravity XYZ: %.2f, %.2f, %.2f m/s^2 |", xx, yy, zz);
    }
    if ((_msense & MOTION_ROTATION) == MOTION_ROTATION) {
      Motion_RotationRead(xx, yy, zz);
      Serial.printf(" Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f° |", xx, yy, zz);
    }
    if ((_msense & MOTION_ROTATION_NO_MAG) == MOTION_ROTATION_NO_MAG) {
      Motion_RotationNoMagRead(xx, yy, zz);
      Serial.printf(" Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f° |", xx, yy, zz);
    }
    if ((_msense & MOTION_STEP_COUNTER) == MOTION_STEP_COUNTER) {
      Serial.print(" Steps: ");
      Serial.print(Motion_StepCounterRead());
      Serial.print(" |");
    }
    if ((_msense & MOTION_TAP_DETECTOR) == MOTION_TAP_DETECTOR) {
      Serial.print(" Tap Detector: ");
      Serial.print(Motion_TapDetectorRead());
      Serial.print(" |");
    }

    if ((_msense & MOTION_STATE) == MOTION_STATE) {
      Serial.print(" Current State: ");
      switch (Motion_StateRead()) {
        case MOTION_STATE_STABLE:
          Serial.print("Stable");
          break;
        case MOTION_STATE_ONTABLE:
          Serial.print("On Table");
          break;
        case MOTION_STATE_STATIONARY:
          Serial.print("Stationary");
          break;
        case MOTION_STATE_MOTION:
          Serial.print("Motion");
          break;
        default:
          Serial.print("Unkown");
          break;
      }
      Serial.print(" |");
    }
    if ((_msense & MOTION_ACTIVITY) == MOTION_ACTIVITY) {
      Serial.print(" Activity: ");
      switch (Motion_ActivityRead()) {
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
          Serial.print("Stairs");
          break;
        default:
          Serial.print("Reading..");
          break;
      }
      Serial.print(" |");
    }
  }
  Serial.println();
}

uint8_t CodeCell::BatteryLevelRead() {
  uint16_t voltage = 0;
  voltage = ((BatteryVoltageRead() / 8U) - 408U);
  if (_charge_state == POWER_BAT_CHRG) {
    _voltage_last = 101;  //Show Charging Icon in MicroLink App
  } else if (_charge_state == POWER_USB) {
    _voltage_last = 102;  //Show USB Icon in MicroLink App
  } else if (_charge_state == POWER_BAT_FULL) {
    _voltage_last = 100;
  } else if (_voltage_last <= 1) {
    _voltage_last = 1;
  } else if ((voltage <= _voltage_last) && (voltage <= 100)) {
    _voltage_last = voltage;
  } else {
    //Spike Voltage Filter
  }

  voltage = _voltage_last;

  return (uint8_t)_voltage_last;
}

uint16_t CodeCell::BatteryVoltageRead() {
  uint32_t voltage_avrg_total = 0;
  double voltage = (double)analogRead(4) * 1.448;  //* 5930 / 4095

#if defined(ARDUINO_ESP32C6_DEV)
  _voltage_avrg[_voltage_index] = (uint16_t)voltage + 1154;
#else
  _voltage_avrg[_voltage_index] = (uint16_t)voltage;
#endif
  _voltage_index++;
  if (_voltage_index >= AVRG_FILTER_SIZE) {
    _voltage_index = 0;
  }
  for (uint8_t vv = 0; vv < AVRG_FILTER_SIZE; vv++) {
    voltage_avrg_total += _voltage_avrg[vv];
  }
  voltage_avrg_total = voltage_avrg_total / AVRG_FILTER_SIZE;

  return ((uint16_t)voltage_avrg_total);
}

bool CodeCell::Run(uint8_t run_frequency) {
  bool tt_flag = 0;
  if (cc_timeflag) {
    tt_flag = 1;
    cc_timeflag = 0;

    if ((_msense & LIGHT) == LIGHT) {
      Light_Read();
    }
    if ((_msense & 0b111111111111) != MOTION_DISABLE) {
      Motion_Read();
    }

    if (run_frequency > 100) {
      run_frequency = 100;
    } else if (run_frequency < 1) {
      run_frequency = 1;
    } else {
      //skip
    }
    if (run_frequency != _run_frequency_last) {
      Serial.print(">> Sampling Rate: ");
      Serial.print(run_frequency);
      Serial.println("Hz");
      timerStop(cctimer);
      _run_frequency_last = run_frequency;
      timerAlarm(cctimer, ((1000000) / _run_frequency_last), true, 0); /*Set alarm to trigger to new frequency*/
      timerStart(cctimer);
    }

    LED_Breathing(_charge_color);
    _power_counter++;
    if (_power_counter >= (run_frequency / 1)) {
      //Check Power every 1Hz
      _power_counter = 0;

      uint16_t battery_voltage = BatteryVoltageRead();

      if ((battery_voltage > USB_VOLTAGE) || (digitalRead(0) == 0)) {
        _lowvoltage_counter = 0;
        _chrg_counter = 0;
        if (digitalRead(0) == 0) {
          if (_charge_state != POWER_BAT_CHRG) {
            _charge_state = POWER_BAT_CHRG;
            _charge_color = LED_OFF;
            LED(0, 0, LED_SLEEP_BRIGHTNESS); /*Set LED to the minimum Blue*/
            Serial.println(">> Power Status: Battery is charging");
          }
        } else if ((BatteryVoltageRead() > USB_VOLTAGE) && ((_charge_state == POWER_BAT_CHRG) || (_charge_state == POWER_BAT_FULL))) {
          if (_charge_state != POWER_BAT_FULL) {
            _charge_state = POWER_BAT_FULL;
            _charge_color = LED_OFF;
            LED(0, LED_SLEEP_BRIGHTNESS, 0);  //Set LED to the Green to indicate charging is complete
            Serial.println(">> Power Status: Battery Charged ");
          }
        } else if (_charge_state == POWER_INIT) {
          /*Battery First Time Check*/
          Serial.println(">> Power Status: Running from USB Power");
          _charge_state = POWER_USB;
        } else {
          /*USB Cabel connected without battery - Run application*/
          _charge_color = LED_COLOR_BLUE;
          if ((!Serial) && (serial_flag == 1)) {
            Serial.end();         /*Close the current Serial connection*/
            delay(500);           /*Short delay before reinitializing*/
            Serial.begin(115200); /*Reinitialize the Serial connection*/
            Serial.println(">> Serial connection re-established!");
            delay(100);    /*Short delay before reinitializing*/
            esp_restart(); /*Restart the CodeCell*/
          }
        }
      } else {
        if ((_charge_state == POWER_BAT_FULL) || (_charge_state == POWER_INIT)) {
          /*Battery has been charged or USB is disconnected*/
          Serial.println(">> Power Status: Running from Battery Power");
          _charge_state = POWER_BAT_RUN; /*Battery is at an operating voltage level*/
          _chrg_counter = 0;
          _lowvoltage_counter = 0;
          _charge_color = LED_COLOR_GREEN;
        } else {
          if (battery_voltage < MIN_BATTERY_VOLTAGE) {
            /*Voltage is less than 3.3V or higher than 4.3V*/
            _charge_state = POWER_BAT_LOW;
            _charge_color = LED_COLOR_RED;
            if (_lowvoltage_counter < 10) {
              _lowvoltage_counter++;
            } else {
              Serial.println(">> Power Status: Battery Low going to Sleep");
              _chrg_counter = 0;
              USBSleep();
            }
          } else {
            _lowvoltage_counter = 0;
            _charge_color = LED_COLOR_GREEN;
            if (_chrg_counter < 3) {
              _chrg_counter++;
            } else {
              _charge_state = POWER_BAT_RUN; /*Battery is at an operating voltage level*/
            }
          }
        }
      }
    }
  } else {
    /*wait*/
  }
  return tt_flag;
}

void CodeCell::LED(uint8_t r, uint8_t g, uint8_t b) {
#if defined(ARDUINO_ESP32C6_DEV)
  digitalWrite(LED_ON_PIN, HIGH); /*Init Set up to output high*/
#endif
  neopixelWrite(LED_PIN, r, g, b); /*RMT ESP32 function for addressable LEDs*/
}

void CodeCell::LED_SetBrightness(uint16_t level) {
  //Brightness level between 1-10
  if (level == 0) {
    _LED_level = 0;
  } else {
    if (level > 10) {
      level = 10;
    }
    _LED_level = 11 - level;
  }
}

void CodeCell::LED_Breathing(uint32_t rgb_color_24bit) {
  if (_LED_level != 0) {
    if (rgb_color_24bit != LED_OFF) {
      uint32_t r_counter = (rgb_color_24bit >> 16) & 0xFFU;
      uint32_t g_counter = (rgb_color_24bit >> 8) & 0xFFU;
      uint32_t b_counter = rgb_color_24bit & 0xFFU;
      uint16_t getProx = 100;
      if ((_msense & LIGHT) == LIGHT) {
        getProx = Light_ProximityRead();
      }

      if (getProx > 2000U) {
        getProx = 2000U;
      }

      getProx = (getProx / 100) + 2;
      if (getProx > LED_DEFAULT_BRIGHTNESS) {
        getProx = LED_DEFAULT_BRIGHTNESS;
      }
      getProx = getProx << 1U;

      if (_run_frequency_last >= 6) {
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
      } else {
        _LED_Breathing_flag = !_LED_Breathing_flag;
        if (_LED_Breathing_flag == 1) {
          _LED_Breathing_counter = getProx;
        } else {
          _LED_Breathing_counter = 0;
        }
      }

      r_counter = (r_counter * _LED_Breathing_counter) / 22U;
      g_counter = (g_counter * _LED_Breathing_counter) / 22U;
      b_counter = (b_counter * _LED_Breathing_counter) / 22U;

      r_counter = r_counter / _LED_level;
      g_counter = g_counter / _LED_level;
      b_counter = b_counter / _LED_level;

      LED(r_counter, g_counter, b_counter); /*Set LED to the default Blue*/
    }
  } else {
    LED(0, 0, 0); /*Keep LED off*/
  }
}

uint16_t CodeCell::Light_AmbientRead() {
  if (((_msense & LIGHT) == LIGHT) || (_wakeup_flag == 1)) {
    return _light_data[2];
  } else {
    Serial.println(">> Error: Light Sensor not Activated");
    return 0;
  }
}

uint16_t CodeCell::Light_WhiteRead() {
  if (((_msense & LIGHT) == LIGHT) || (_wakeup_flag == 1)) {
    return _light_data[1];
  } else {
    Serial.println(">> Error: Light Sensor not Activated");
    return 0;
  }
}

uint16_t CodeCell::Light_ProximityRead() {
  if (((_msense & LIGHT) == LIGHT) || (_wakeup_flag == 1)) {
    return _light_data[0];
  } else {
    Serial.println(">> Error: Light Sensor not Activated");
    return 0;
  }
}

void CodeCell::Light_Read() {
  uint16_t value = 0;
  Wire.beginTransmission(VCNL4040_ADDRESS);
  Wire.write(VCNL4040_PROX_REG);
  Wire.endTransmission(false); /*Don't release the bus*/
  I2CRead(VCNL4040_ADDRESS, _i2c_read_array, 2);
  Wire.endTransmission(false);
  value = _i2c_read_array[0];         /*LSB*/
  value |= (_i2c_read_array[1] << 8); /*MSB*/
  _light_data[0] = value;

  value = 0;
  Wire.beginTransmission(VCNL4040_ADDRESS);
  Wire.write(VCNL4040_WHITE_REG);
  Wire.endTransmission(false); /*Don't release the bus*/
  I2CRead(VCNL4040_ADDRESS, _i2c_read_array, 2);
  Wire.endTransmission(false);
  value = _i2c_read_array[0];         /*LSB*/
  value |= (_i2c_read_array[1] << 8); /*MSB*/
  _light_data[1] = value;

  value = 0;
  Wire.beginTransmission(VCNL4040_ADDRESS);
  Wire.write(VCNL4040_AMBIENT_REG);
  Wire.endTransmission(false); /*Don't release the bus*/
  I2CRead(VCNL4040_ADDRESS, _i2c_read_array, 2);
  Wire.endTransmission(false);
  value = _i2c_read_array[0];         /*LSB*/
  value |= (_i2c_read_array[1] << 8); /*MSB*/
  _light_data[2] = value;
}

void CodeCell::Motion_RotationVectorRead(float &vec_r, float &vec_i, float &vec_j, float &vec_k) {
  if (((_msense & MOTION_ROTATION) == MOTION_ROTATION) || (_wakeup_flag == 1)) {
    vec_r = _motion_data[0];
    vec_i = _motion_data[1];
    vec_j = _motion_data[2];
    vec_k = _motion_data[3];
  } else {
    Serial.println(">> Error: Motion Rotation Sensor not Activated");
  }
}

void CodeCell::Motion_RotationRead(float &roll, float &pitch, float &yaw) {
  if (((_msense & MOTION_ROTATION) == MOTION_ROTATION) || (_wakeup_flag == 1)) {
    roll = atan2(2.0 * (_motion_data[0] * _motion_data[1] + _motion_data[2] * _motion_data[3]), 1.0 - 2.0 * (_motion_data[1] * _motion_data[1] + _motion_data[2] * _motion_data[2]));
    roll = roll * RAD_TO_DEG;
    pitch = atan2(2.0 * (_motion_data[0] * _motion_data[2] - _motion_data[3] * _motion_data[1]), 1.0 - 2.0 * (_motion_data[2] * _motion_data[2] + _motion_data[1] * _motion_data[1]));
    pitch = pitch * RAD_TO_DEG;
    yaw = atan2(2.0 * (_motion_data[0] * _motion_data[3] + _motion_data[1] * _motion_data[2]), 1.0 - 2.0 * (_motion_data[2] * _motion_data[2] + _motion_data[3] * _motion_data[3]));
    yaw = yaw * RAD_TO_DEG;
  } else {
    Serial.println(">> Error: Motion Rotation Sensor not Activated");
  }
}
void CodeCell::Motion_RotationNoMagRead(float &roll, float &pitch, float &yaw) {
  if (((_msense & MOTION_ROTATION_NO_MAG) == MOTION_ROTATION_NO_MAG) || (_wakeup_flag == 1)) {
    roll = atan2(2.0 * (_motion_data[4] * _motion_data[5] + _motion_data[6] * _motion_data[7]), 1.0 - 2.0 * (_motion_data[5] * _motion_data[5] + _motion_data[6] * _motion_data[6]));
    roll = roll * RAD_TO_DEG;
    pitch = atan2(2.0 * (_motion_data[4] * _motion_data[6] - _motion_data[7] * _motion_data[5]), 1.0 - 2.0 * (_motion_data[6] * _motion_data[6] + _motion_data[5] * _motion_data[5]));
    pitch = pitch * RAD_TO_DEG;
    yaw = atan2(2.0 * (_motion_data[4] * _motion_data[7] + _motion_data[5] * _motion_data[6]), 1.0 - 2.0 * (_motion_data[6] * _motion_data[6] + _motion_data[7] * _motion_data[7]));
    yaw = yaw * RAD_TO_DEG;
  } else {
    Serial.println(">> Error: Motion Rotation (No Magnetometer) Sensor not Activated");
  }
}

void CodeCell::Motion_AccelerometerRead(float &x, float &y, float &z) {
  if (((_msense & MOTION_ACCELEROMETER) == MOTION_ACCELEROMETER) || (_wakeup_flag == 1)) {
    x = _motion_data[8];
    y = _motion_data[9];
    z = _motion_data[10];
  } else {
    Serial.println(">> Error: Motion Accelerometer Sensor not Activated");
  }
}

void CodeCell::Motion_GyroRead(float &x, float &y, float &z) {
  if (((_msense & MOTION_GYRO) == MOTION_GYRO) || (_wakeup_flag == 1)) {
    x = _motion_data[11];
    y = _motion_data[12];
    z = _motion_data[13];
  } else {
    Serial.println(">> Error: Motion Gyroscope Sensor not Activated");
  }
}

void CodeCell::Motion_MagnetometerRead(float &x, float &y, float &z) {
  if (((_msense & MOTION_MAGNETOMETER) == MOTION_MAGNETOMETER) || (_wakeup_flag == 1)) {
    x = _motion_data[14];
    y = _motion_data[15];
    z = _motion_data[16];
  } else {
    Serial.println(">> Error: Motion Magnetometer Sensor not Activated");
  }
}

void CodeCell::Motion_GravityRead(float &x, float &y, float &z) {
  if (((_msense & MOTION_GRAVITY) == MOTION_GRAVITY) || (_wakeup_flag == 1)) {
    x = _motion_data[17];
    y = _motion_data[18];
    z = _motion_data[19];
  } else {
    Serial.println(">> Error: Motion Gravity Sensor not Activated");
  }
}

void CodeCell::Motion_LinearAccRead(float &x, float &y, float &z) {
  if (((_msense & MOTION_LINEAR_ACC) == MOTION_LINEAR_ACC) || (_wakeup_flag == 1)) {
    x = _motion_data[20];
    y = _motion_data[21];
    z = _motion_data[22];
  } else {
    Serial.println(">> Error: Motion Linear Acceleration Sensor not Activated");
  }
}

bool CodeCell::Motion_TapDetectorRead() {
  if (((_msense & MOTION_TAP_DETECTOR) == MOTION_TAP_DETECTOR) || (_wakeup_flag == 1)) {
    return _tap_event;
  } else {
    Serial.println(">> Error: Motion Tap Detector not Activated");
    return false;
  }
}
uint16_t CodeCell::Motion_StepCounterRead() {
  if (((_msense & MOTION_STEP_COUNTER) == MOTION_STEP_COUNTER) || (_wakeup_flag == 1)) {
    return _step_data;
  } else {
    Serial.println(">> Error: Motion Step Sensor not Activated");
    return 0;
  }
}

uint16_t CodeCell::Motion_StateRead() {
  if (((_msense & MOTION_STATE) == MOTION_STATE) || (_wakeup_flag == 1)) {
    return _mstate_data;
  } else {
    Serial.println(">> Error: Motion State Sensor not Activated");
    return 0;
  }
}

uint16_t CodeCell::Motion_ActivityRead() {
  if (((_msense & MOTION_ACTIVITY) == MOTION_ACTIVITY) || (_wakeup_flag == 1)) {
    return _activity_data;
  } else {
    Serial.println(">> Error: Motion Activity Sensor not Activated");
    return 0;
  }
}


void CodeCell::Motion_Read() {
  bool error_flag = 1;
  uint8_t imu_read_timer = 0U;
  uint8_t motion_id = 0xFF;
  _tap_event = false;

  while (Motion.getSensorEvent() == true) {
    motion_id = Motion.getSensorEventID();
    if (motion_id == SENSOR_REPORTID_ROTATION_VECTOR) {
      _motion_data[0] = Motion.getRot_R();
      _motion_data[1] = Motion.getRot_I();
      _motion_data[2] = Motion.getRot_J();
      _motion_data[3] = Motion.getRot_K();
      error_flag = 0;
    }
    if (motion_id == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      _motion_data[4] = Motion.getGameReal();
      _motion_data[5] = Motion.getGameI();
      _motion_data[6] = Motion.getGameJ();
      _motion_data[7] = Motion.getGameK();
      error_flag = 0;
    }
    if (motion_id == SENSOR_REPORTID_ACCELEROMETER) {
      _motion_data[8] = Motion.getAccelX();
      _motion_data[9] = Motion.getAccelY();
      _motion_data[10] = Motion.getAccelZ();
      error_flag = 0;
    }
    if (motion_id == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
      _motion_data[11] = Motion.getGyroX();
      _motion_data[12] = Motion.getGyroY();
      _motion_data[13] = Motion.getGyroZ();
      error_flag = 0;
    }
    if (motion_id == SENSOR_REPORTID_MAGNETIC_FIELD) {
      _motion_data[14] = Motion.getMagX();
      _motion_data[15] = Motion.getMagY();
      _motion_data[16] = Motion.getMagZ();
      error_flag = 0;
    }
    if (motion_id == SENSOR_REPORTID_GRAVITY) {
      _motion_data[17] = Motion.getGravityX();
      _motion_data[18] = Motion.getGravityY();
      _motion_data[19] = Motion.getGravityZ();
      error_flag = 0;
    }
    if (motion_id == SENSOR_REPORTID_LINEAR_ACCELERATION) {
      _motion_data[20] = Motion.getLinAccelX();
      _motion_data[21] = Motion.getLinAccelY();
      _motion_data[22] = Motion.getLinAccelZ();
      error_flag = 0;
    }
    if (motion_id == SENSOR_REPORTID_STEP_COUNTER) {
      _step_data = (uint16_t)Motion.getStepCount();
      error_flag = 0;
    }
    if (motion_id == SENSOR_REPORTID_STABILITY_CLASSIFIER) {
      _mstate_data = Motion.getStabilityClassifier();
      error_flag = 0;
    } else {
      _mstate_data = 0;
      error_flag = 0;
    }
    if (motion_id == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER) {
      _activity_data = Motion.getActivityClassifier();
      error_flag = 0;
    }
    if (motion_id == SENSOR_REPORTID_TAP_DETECTOR) {
      _tap_event = true;
      error_flag = 0;
    } else {
      error_flag = 0;
    }

    if (error_flag) {
      imu_read_timer++;
      if (imu_read_timer > 20U) {
        Serial.println(">> Error: Motion Sensor not found");
        return;
      }
    }
  }
}

bool CodeCell::Light_Init() {
  bool light_error = 0;
  uint8_t error_timer = 0;

  Wire.begin(8, 9, 400000);  // SDA on GPIO8, SCL on GPIO9, 400kHz speed

  Wire.beginTransmission(VCNL4040_ADDRESS);
  if (Wire.endTransmission() != 0) {
    while (error_timer < 100) {
      delay(10);
      Wire.beginTransmission(VCNL4040_ADDRESS);
      if (Wire.endTransmission() != 0) {
        error_timer++;
        if (error_timer >= 99) {
          LED(LED_SLEEP_BRIGHTNESS, 0, 0);
          Serial.println();
          Serial.println(">> Error: Light Sensor not found - Check Hardware");
          while (1) {
            delay(10);  //Wait
          }
        }
      } else {
        error_timer = 200;
      }
    }
  }

  /*Configure - Continuous conversion mode, high dynamic range, integration time of 80 ms*/
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_CONF_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x000 & 0xFF);        /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x000 >> 8) & 0xFF); /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    light_error = 1;
  }
  _i2c_write_size = 0;
  /*Configure - duty cycle to 1/40, 16-bit resolution*/
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF1_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x80E & 0xFF);        /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x80E >> 8) & 0xFF); /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    light_error = 1;
  }
  _i2c_write_size = 0;
  /*Configure - Set LED current to 200 mA, No interrupt settings*/
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF3_REG;  /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x4710 & 0xFF);        /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x4710 >> 8) & 0xFF); /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    light_error = 1;
  }
  _i2c_write_size = 0;
  Wire.endTransmission();


  return light_error;
}

void CodeCell::LightReset() {
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_CONF_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);         /*LSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF1_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);         /*LSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println(">> Error: Light Sensor not found");
  }
  _i2c_write_size = 0;
}

void CodeCell::Motion_Init(uint32_t sense_motion) {
  uint8_t imu_timer = 0U;
  uint8_t error_timer = 0;

  _msense = sense_motion;
  Wire.begin(8, 9, 400000);  // SDA on GPIO8, SCL on GPIO9, 400kHz speed

  if (Motion.begin() == false) {
    Serial.println(">> Error: Motion Sensor not found - Check Hardware");
    while (1) {
      delay(10);  //Wait
    }
  }
  if (Motion.isConnected() == false) {
    Serial.println(">> Error: Motion Sensor not found");
    while (1) {
      delay(10);  //Wait
    }
  }
  if ((_msense & MOTION_ACCELEROMETER) == MOTION_ACCELEROMETER) {
    if (Motion.enableAccelerometer() == true) {
      if (!_wakeup_flag) {
        Serial.print("Accelerometer Activated | ");
      }
    } else {
      Serial.print("Accelerometer: Failed | ");
    }
  }

  if ((_msense & MOTION_GYRO) == MOTION_GYRO) {
    if (Motion.enableGyro() == true) {
      if (!_wakeup_flag) {
        Serial.print("Gyro Activated | ");
      }
    } else {
      Serial.print("Gyro: Failed | ");
    }
  }
  if ((_msense & MOTION_MAGNETOMETER) == MOTION_MAGNETOMETER) {
    if (Motion.enableMagnetometer() == true) {
      if (!_wakeup_flag) {
        Serial.print("Magnetometer Activated | ");
      }
    } else {
      Serial.print("Magnetometer: Failed | ");
    }
  }
  if ((_msense & MOTION_LINEAR_ACC) == MOTION_LINEAR_ACC) {
    if (Motion.enableLinearAccelerometer() == true) {
      if (!_wakeup_flag) {
        Serial.print("Linear Accelerometer Activated | ");
      }
    } else {
      Serial.print("Linear Accelerometer Motion: Failed | ");
    }
  }
  if ((_msense & MOTION_GRAVITY) == MOTION_GRAVITY) {
    if (Motion.enableGravity() == true) {
      if (!_wakeup_flag) {
        Serial.print("Gravity Sensing Activated | ");
      }
    } else {
      Serial.print("Gravity Sensing: Failed | ");
    }
  }
  if ((_msense & MOTION_ROTATION) == MOTION_ROTATION) {
    if (Motion.enableRotationVector() == true) {
      if (!_wakeup_flag) {
        Serial.print("Rotation Sensing Activated | ");
      }
    } else {
      Serial.print("Rotation Sening: Failed | ");
    }
  }
  if ((_msense & MOTION_ROTATION_NO_MAG) == MOTION_ROTATION_NO_MAG) {
    if (Motion.enableGameRotationVector() == true) {
      if (!_wakeup_flag) {
        Serial.print("Compass-Free Rotation Sensing Activated | ");
      }
    } else {
      Serial.print("Compass-Free Rotation Sensing: Failed | ");
    }
  }
  if ((_msense & MOTION_STEP_COUNTER) == MOTION_STEP_COUNTER) {
    if (Motion.enableStepCounter() == true) {
      if (!_wakeup_flag) {
        Serial.print("Step Counter Activated | ");
      }
    } else {
      Serial.print("Step Counter: Failed | ");
    }
  }
  if ((_msense & MOTION_TAP_DETECTOR) == MOTION_TAP_DETECTOR) {
    if (Motion.enableTapDetector(10000) == true) {
      if (!_wakeup_flag) {
        Serial.print("Tap Detector Activated | ");
      }
    } else {
      Serial.print("Tap Detector: Failed | ");
    }
  }
  if ((_msense & MOTION_STATE) == MOTION_STATE) {
    if (Motion.enableStabilityClassifier() == true) {
      if (!_wakeup_flag) {
        Serial.print("Motion State Activated | ");
      }
    } else {
      Serial.print("Motion State: Failed | ");
    }
  }
  if ((_msense & MOTION_ACTIVITY) == MOTION_ACTIVITY) {
    if (Motion.enableActivityClassifier(1000, 0x1F) == true) {
      if (!_wakeup_flag) {
        Serial.print("Motion Activity Activated | ");
      }
    } else {
      Serial.print("Motion Activity: Failed | ");
    }
  }
  _i2c_write_size = 0;
}

bool CodeCell::pinCheck(uint8_t pin_num, uint8_t pin_type) {
  bool free_pin = 0;

#if defined(ARDUINO_ESP32C6_DEV)
  if ((pin_num == 1) && ((pin_type == PIN_TYPE_OUTPUT) || (pin_type == PIN_TYPE_INPUT) || (pin_type == PIN_TYPE_ADC) || (pin_type == PIN_TYPE_PWM))) {
    free_pin = 1;
  } else if ((pin_num == 2) && ((pin_type == PIN_TYPE_OUTPUT) || (pin_type == PIN_TYPE_INPUT) || (pin_type == PIN_TYPE_ADC) || (pin_type == PIN_TYPE_PWM))) {
    free_pin = 1;
  } else if ((pin_num == 3) && ((pin_type == PIN_TYPE_OUTPUT) || (pin_type == PIN_TYPE_INPUT) || (pin_type == PIN_TYPE_ADC) || (pin_type == PIN_TYPE_PWM))) {
    free_pin = 1;
  } else if ((pin_num == 21) && ((pin_type == PIN_TYPE_OUTPUT) || (pin_type == PIN_TYPE_INPUT) || (pin_type == PIN_TYPE_PWM))) {
    free_pin = 1;
  } else if ((pin_num == 22) && ((pin_type == PIN_TYPE_OUTPUT) || (pin_type == PIN_TYPE_INPUT) || (pin_type == PIN_TYPE_PWM))) {
    free_pin = 1;
  } else if ((pin_num == 23) && ((pin_type == PIN_TYPE_OUTPUT) || (pin_type == PIN_TYPE_INPUT) || (pin_type == PIN_TYPE_PWM))) {
    free_pin = 1;
  } else {
    free_pin = 0;
    Serial.println(">> Error: Pin not Supported");
  }
#else
  if ((pin_num == 1) && ((pin_type == PIN_TYPE_OUTPUT) || (pin_type == PIN_TYPE_INPUT) || (pin_type == PIN_TYPE_ADC) || (pin_type == PIN_TYPE_PWM))) {
    free_pin = 1;
  } else if ((pin_num == 2) && ((pin_type == PIN_TYPE_OUTPUT) || (pin_type == PIN_TYPE_INPUT) || (pin_type == PIN_TYPE_ADC) || (pin_type == PIN_TYPE_PWM))) {
    free_pin = 1;
  } else if ((pin_num == 3) && ((pin_type == PIN_TYPE_OUTPUT) || (pin_type == PIN_TYPE_INPUT) || (pin_type == PIN_TYPE_ADC) || (pin_type == PIN_TYPE_PWM))) {
    free_pin = 1;
  } else if ((pin_num == 5) && ((pin_type == PIN_TYPE_OUTPUT) || (pin_type == PIN_TYPE_INPUT) || (pin_type == PIN_TYPE_PWM))) {
    free_pin = 1;
  } else if ((pin_num == 6) && ((pin_type == PIN_TYPE_OUTPUT) || (pin_type == PIN_TYPE_INPUT) || (pin_type == PIN_TYPE_PWM))) {
    free_pin = 1;
  } else if ((pin_num == 7) && ((pin_type == PIN_TYPE_OUTPUT) || (pin_type == PIN_TYPE_INPUT) || (pin_type == PIN_TYPE_PWM))) {
    free_pin = 1;
  } else {
    free_pin = 0;
    Serial.println(">> Error: Pin not Supported");
  }
#endif

  return free_pin;
}

void CodeCell::pinWrite(uint8_t pin_num, bool pin_value) {
  if (pinCheck(pin_num, PIN_TYPE_OUTPUT)) {
    if (_pinArray[pin_num - 1] == 0) {
      _pinArray[pin_num - 1] = 1;
      pinMode(pin_num, OUTPUT);
    }
    digitalWrite(pin_num, pin_value); /*Init Set up to output low*/
  } else {
    //Skip
  }
}

bool CodeCell::pinRead(uint8_t pin_num) {
  if (pinCheck(pin_num, PIN_TYPE_INPUT)) {
    if (_pinArray[pin_num - 1] == 1) {
      _pinArray[pin_num - 1] = 0;
      pinMode(pin_num, INPUT);
    }
    return (bool)digitalRead(pin_num);
  } else {
    return false;  //Skip
  }
}

uint16_t CodeCell::pinADC(uint8_t pin_num) {
  uint16_t ADCValue = 0;
  if (pinCheck(pin_num, PIN_TYPE_ADC)) {
    if (_pinArray[pin_num - 1] == 1) {
      _pinArray[pin_num - 1] = 0;
      pinMode(pin_num, INPUT);
    }
    ADCValue = analogRead(pin_num);
  } else {
    //Skip
  }
  return ADCValue;
}

void CodeCell::pinPWM(uint8_t pin_num, uint16_t pin_freq, uint8_t pin_dutycycle) {
  if (pinCheck(pin_num, PIN_TYPE_PWM)) {
    if (_pinArray[pin_num - 1] == 0) {
      _pinArray[pin_num - 1] = 1;
      pinMode(pin_num, OUTPUT);
      ledcAttach(pin_num, pin_freq, PWM_RES);
    }
    uint8_t dc_val = (pin_dutycycle * 255) / 100U;
    ledcWrite(pin_num, dc_val);
  } else {
    //Skip
  }
}