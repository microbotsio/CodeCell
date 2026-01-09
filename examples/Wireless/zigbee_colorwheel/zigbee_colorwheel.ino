/*
  Example: CodeCell Zigbee Color Light (HA Color Wheel)
  Boards: CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Exposes the CodeCell as a Zigbee color dimmable light
  - Home Assistant color wheel + brightness slider control the onboard RGB LED color 
  - Supports on/off + color + brightness
  
  Required Arduino Tools Settings:
  - Flash Size: 8MB (64Mb)
  - Partition Scheme: Zigbee 8MB with spiffs
  - Zigbee Mode: Zigbee ED (end device)
*/

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include <CodeCell.h>
#include "Zigbee.h"

#define ZIGBEE_COLOR_ENDPOINT 7

CodeCell myCodeCell;
ZigbeeColorDimmableLight zbColorLight(ZIGBEE_COLOR_ENDPOINT);

// RGB callback from Zigbee (XY/RGB mode)
// state  : on/off
// red    : 0–255
// green  : 0–255
// blue   : 0–255
// level  : 0–255 (brightness)
void setCodeCellColor(bool state, uint8_t red, uint8_t green, uint8_t blue, uint8_t level) {
  if (!state || level == 0) {
    // Turn LED off
    myCodeCell.LED(0, 0, 0);
    return;
  }

  // Scale RGB by brightness level (0–255)
  // Use 16-bit math to avoid overflow on multiply
  uint8_t scaledR = (uint16_t(red)   * level) / 255;
  uint8_t scaledG = (uint16_t(green) * level) / 255;
  uint8_t scaledB = (uint16_t(blue)  * level) / 255;

  myCodeCell.LED(scaledR, scaledG, scaledB);
}

void setup() {
  Serial.begin(115200);
  
  myCodeCell.Init(LIGHT); // Init CodeCell
  myCodeCell.LED(0, 0, 0);
  myCodeCell.LED_SetBrightness(0); //Turn off LED

  // Enable RGB/XY and HSV so HA’s color wheel can use either
  uint16_t capabilities =
      ZIGBEE_COLOR_CAPABILITY_X_Y |
      ZIGBEE_COLOR_CAPABILITY_HUE_SATURATION;
  zbColorLight.setLightColorCapabilities(capabilities);

  // Use RGB callback – Zigbee will give us R,G,B + level
  zbColorLight.onLightChangeRgb(setCodeCellColor);

  // Optional: set visible name/model in coordinator
  zbColorLight.setManufacturerAndModel("Microbots", "CodeCellC6-ColorLED");

  // Register endpoint and start Zigbee
  Serial.println("Adding ZigbeeColorDimmableLight endpoint");
  Zigbee.addEndpoint(&zbColorLight);

  Serial.println("Starting Zigbee...");
  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start, rebooting...");
    ESP.restart();
  }

  Serial.println("Connecting to Zigbee network...");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nZigbee connected.");
}

void loop() {
  //wait
}

