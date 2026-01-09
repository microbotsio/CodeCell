/*
  Example: CodeCell Zigbee LightSwitch Demo
  Boards: CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Turns a CodeCell into a simple Zigbee on/off light endpoint
  - The onboard RGB LED behaves as the "light" controlled by your Zigbee hub
  - Compatible with Home Assistant and other standard coordinators

  Required Arduino Tools Settings:
  - Board: ESP32C6 Dev Module
  - Flash Size: 8MB (64Mb)
  - Partition Scheme: Zigbee 8MB with spiffs
  - Zigbee Mode: Zigbee ED (end device)

  Zigbee Behavior:
  - Exposes a standard on/off light cluster on endpoint 10
  - Coordinator toggles the light → CodeCell LED turns ON/OFF accordingly

  Notes:
  - Make sure Zigbee end-device mode is selected; the sketch will error if not
  - LED behavior can be changed to control other peripherals instead
  - Useful for testing Zigbee connectivity, automation triggers, and endpoint behavior

*/


#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include <CodeCell.h>
#include "Zigbee.h"

#define ZIGBEE_LIGHT_ENDPOINT 10

CodeCell myCodeCell;
ZigbeeLight zbLight(ZIGBEE_LIGHT_ENDPOINT);

// Called whenever the Zigbee light state changes
void setCodeCellLED(bool on) {
  if (on) {
    myCodeCell.LED_SetBrightness(10);  //ON
  } else {
    myCodeCell.LED_SetBrightness(0);  // OFF
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Init CodeCell (no sensors needed, just LED)
  myCodeCell.Init(LIGHT);
  myCodeCell.LED_SetBrightness(0);
  myCodeCell.LED(0, 0, 0);

  // Optional: name/model that show up in your Zigbee coordinator
  zbLight.setManufacturerAndModel("Microbots", "CodeCellC6-Light");

  // When the light state changes (from Zigbee), drive the CodeCell LED
  zbLight.onLightChange(setCodeCellLED);

  // Register endpoint and start Zigbee
  Serial.println("Adding ZigbeeLight endpoint");
  Zigbee.addEndpoint(&zbLight);

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
  if (myCodeCell.Run(10)) {  // Run loop at 10 Hz
  }
}
