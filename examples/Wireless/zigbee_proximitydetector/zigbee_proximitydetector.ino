/*
  Example: CodeCell Zigbee Proximity Presence Sensor
  Boards: CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Uses the CodeCell light/proximity sensor as a basic presence detector.
  - Presence is exposed as a Zigbee binary input (ON/OFF-like) to your hub.
  - LED turns RED when presence is detected.

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

#define ZIGBEE_PRESENCE_ENDPOINT 1
#define DISTANCE_RANGE           100U   // Adjust to taste

CodeCell myCodeCell;

// Binary endpoint for presence
ZigbeeBinary zbPresence(ZIGBEE_PRESENCE_ENDPOINT);

bool lastPresence = false;

void setup() {
  Serial.begin(115200);

  myCodeCell.Init(LIGHT); // Init CodeCell with proximity light sensing
  myCodeCell.LED_SetBrightness(10);
  myCodeCell.LED(0, 0, 0);

  // Optional: name/model that show up in the coordinator
  zbPresence.setManufacturerAndModel("Microbots", "CodeCellC6-Proximity");

  // Configure this endpoint as a binary input (presence/motion-like)
  zbPresence.addBinaryInput();

  zbPresence.setBinaryInputApplication(BINARY_INPUT_APPLICATION_TYPE_SECURITY_MOTION_DETECTION);
  zbPresence.setBinaryInputDescription("Proximity Presence");

  // Register endpoint with Zigbee core
  Serial.println("Adding ZigbeeBinary presence endpoint");
  Zigbee.addEndpoint(&zbPresence);

  // Start Zigbee
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
  Serial.println("\nZigbee connected!");
}

void loop() { 
  if (myCodeCell.Run(1)) {  // Run at 1 Hz 
    uint16_t proximity = myCodeCell.Light_ProximityRead();
    bool presence = (proximity > DISTANCE_RANGE);

    // Only report when the state actually changes
    if (presence != lastPresence) {
      lastPresence = presence;

      // Update and report binary input to the Zigbee network
      zbPresence.setBinaryInput(presence);
      zbPresence.reportBinaryInput();

      Serial.print("Presence changed → ");
      Serial.println(presence ? "DETECTED (1)" : "CLEAR (0)");
    }

    // LED feedback
    if (presence) {
      myCodeCell.LED(0xFF, 0, 0);  // Red when object is detected
    } else {
      myCodeCell.LED(0, 0, 0);     // Off when clear
    }
  }
}


