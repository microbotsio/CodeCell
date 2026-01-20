/*
  Example: CodeCell Zigbee Lamp Control - Tilt-Brightness + Shake Switch
  Boards: CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Endpoint 1 (ZigbeeAnalog): When proximity sensor is pressed, the CodeCell's Pitch angle is mapped to brightness
      Use this value in Home Assistant to set a lamp's brightness.
  - Endpoint 2 (ZigbeeBinary): toggled by a shake gesture, toggles the binary state ON/OFF
      Use this in Home Assistant to turn the lamp on/off.

  Behavior:
  - Press proximity sensor with fingure → proximity detected → "brightness control" is active.
  - Tilt the CodeCell to change Pitch → this updates the brightness value.
  - Remove your fingure → brightness control stops (lamp can stay at last value).
  - Shake the CodeCell → endpoint toggles (can be mapped to toggle lamp automations)

  Required Arduino Tools Settings:
  - Board: ESP32C6 Dev Module
  - Flash Size: 8MB (64Mb)
  - Partition Scheme: Zigbee 8MB with spiffs
  - Zigbee Mode: Zigbee ED (end device)
*/

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include <CodeCell.h>
#include "Zigbee.h"
#include <math.h> 

// Zigbee endpoints
#define ZIGBEE_BRIGHTNESS_ENDPOINT  1   // Numeric brightness value (0–254)
#define ZIGBEE_SHAKE_ENDPOINT       4   // Binary toggle (shake switch)

// Proximity threshold for "brightness control active"
#define PROX_ACTIVE_THRESHOLD       200U   // Adjust to taste

// Pitch → brightness mapping
#define PITCH_MIN_DEG              -45.0f  // Fully down
#define PITCH_MAX_DEG               45.0f  // Fully up

// Shake detection
const float SHAKE_THRESHOLD           = 15.0f;   // Adjust for sensitivity
const unsigned long SHAKE_DEBOUNCE_MS = 800UL;   // Minimum time between shakes

CodeCell myCodeCell;

// Zigbee endpoints
ZigbeeAnalog zbBrightness(ZIGBEE_BRIGHTNESS_ENDPOINT);
ZigbeeBinary zbShake(ZIGBEE_SHAKE_ENDPOINT);

// Motion variables
float Roll  = 0.0f;
float Pitch = 0.0f;
float Yaw   = 0.0f;

float ax = 0.0f, ay = 0.0f, az = 0.0f;

uint8_t lastBrightness           = 0;
unsigned long lastBrightReportMs = 0;
const uint8_t BRIGHT_DELTA_MIN   = 3;      // Minimum change to report
const unsigned long BRIGHT_REPORT_INTERVAL_MS = 300UL;

bool shakeState          = false;      // Toggles ON/OFF on each shake
unsigned long lastShakeMs = 0;

void handleBrightness(uint16_t proximity);
void handleShake();

void setup() {
  Serial.begin(115200);

  // Enable Light + Rotation (pitch) + Accelerometer
  myCodeCell.Init(LIGHT + MOTION_ROTATION + MOTION_ACCELEROMETER);

  myCodeCell.LED(0, 0, 0);//Turn off on-board LED

  // Brightness endpoint (analog-like input) 
  zbBrightness.setManufacturerAndModel("Microbots", "CodeCellC6-PitchBrightness");
  zbBrightness.addAnalogInput();

  // Use Espressif's "percentage" application type so ZHA treats it as a generic level
  zbBrightness.setAnalogInputApplication(ESP_ZB_ZCL_AI_PERCENTAGE_OTHER);
  zbBrightness.setAnalogInputDescription("Brightness level (0–254) from Pitch angle");
  zbBrightness.setAnalogInputResolution(1.0f);
  zbBrightness.setAnalogInputMinMax(0.0f, 254.0f);

  // Shake endpoint 
  zbShake.setManufacturerAndModel("Microbots", "CodeCellC6-ShakeSwitch");
  zbShake.addBinaryInput();
  zbShake.setBinaryInputApplication(BINARY_INPUT_APPLICATION_TYPE_SECURITY_MOTION_DETECTION);
  zbShake.setBinaryInputDescription("Shake to toggle");

  // Register endpoints with Zigbee core 
  Serial.println("Adding Zigbee endpoints (brightness + shake)");
  Zigbee.addEndpoint(&zbBrightness);
  Zigbee.addEndpoint(&zbShake);

  // Start Zigbee
  Serial.println("Starting Zigbee...");
  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start, rebooting...");
    ESP.restart();//If not connected - reset esp32
  }

  Serial.println("Connecting to Zigbee network...");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nZigbee connected!");
}

void loop() {
  if (myCodeCell.Run(10)) { // Run sensors at 10 Hz
    
    uint16_t proximity = myCodeCell.Light_ProximityRead(); // Read proximity first (to know if brightness control is active)

    // Read rotation (roll, pitch, yaw)
    myCodeCell.Motion_RotationRead(Roll, Pitch, Yaw);

    // Handle brightness control using Pitch when proximity is active
    handleBrightness(proximity);

    // Handle shake-based toggle
    handleShake();
  }
}

/**
 * Handle brightness control:
 * - If proximity is above threshold, use Pitch to map to brightness 0–254.
 * - Sends brightness to Zigbee as analog input.
 * - LED shows brightness using green channel.
 */
void handleBrightness(uint16_t proximity) {
  unsigned long now = millis();

  if (proximity < PROX_ACTIVE_THRESHOLD) {
    // No hand close → brightness control not active
    myCodeCell.LED(0, 0, 0);  // LED off
    return;
  }

  // Clamp pitch and map to 0–254
  float pitchClamped = Pitch;
  if (pitchClamped < PITCH_MIN_DEG) {
    pitchClamped = PITCH_MIN_DEG;
  }
  else if (pitchClamped > PITCH_MAX_DEG) {
    pitchClamped = PITCH_MAX_DEG;
  }
  else{
    //skip
  }

  // Normalize Angle
  float normalized = (pitchClamped - PITCH_MIN_DEG) / (PITCH_MAX_DEG - PITCH_MIN_DEG);
  if (normalized < 0.0f) {
    normalized = 0.0f;
  } else if (normalized > 1.0f) {
    normalized = 1.0f;
  }
  else{
    //skip
  }

  uint8_t brightness = (uint8_t)(normalized * 254.0f);

  // Only report if changed enough or enough time passed
  if ( (abs((int32_t)brightness - (int32_t)lastBrightness) >= (int32_t)BRIGHT_DELTA_MIN) ||
       (now - lastBrightReportMs >= BRIGHT_REPORT_INTERVAL_MS) ) {

    lastBrightReportMs = now;
    lastBrightness = brightness;

    zbBrightness.setAnalogInput((float)brightness);
    zbBrightness.reportAnalogInput();

    Serial.print("Pitch: ");
    Serial.print(Pitch);
    Serial.print(" deg -> Brightness: ");
    Serial.println(brightness);
  }

  // Visual feedback: LED green = brightness level
  myCodeCell.LED(0, brightness, 0);
}

/**
 * Handle shake detection:
 * - Read accelerometer.
 * - If total acceleration crosses threshold (and debounce passes),
 *   toggle a Zigbee binary endpoint.
 */
void handleShake() {
  unsigned long now = millis();

  myCodeCell.Motion_AccelerometerRead(ax, ay, az);
  float totalAcceleration = sqrt(ax * ax + ay * ay + az * az);

  if (totalAcceleration > SHAKE_THRESHOLD && (now - lastShakeMs) > SHAKE_DEBOUNCE_MS) {
    lastShakeMs = now;

    // Toggle internal state
    shakeState = !shakeState;

    Serial.print("Shake detected! New switch state: ");
    Serial.println(shakeState ? "ON" : "OFF");

    // Update and report binary input (shake switch) to Zigbee
    zbShake.setBinaryInput(shakeState);
    zbShake.reportBinaryInput();
  }
}
