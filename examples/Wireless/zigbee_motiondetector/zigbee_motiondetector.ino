/*
  Example: CodeCell Zigbee Motion Sensor
  Boards: CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Uses the CodeCell motion state (BNO085) as a motion detector.
  - Reports motion as a Zigbee binary input (ON/OFF) to your hub.
  - Can be used as a trigger in Home Assistant / other coordinators.

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

#define ZIGBEE_MOTION_ENDPOINT 1

CodeCell myCodeCell;

// Binary endpoint for motion
ZigbeeBinary zbMotion(ZIGBEE_MOTION_ENDPOINT);

bool lastMoving = false;

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Enable motion state sensing on CodeCell
  myCodeCell.Init(MOTION_STATE);
  myCodeCell.LED_SetBrightness(10);
  myCodeCell.LED(0, 0, 0);

  // Optional: name/model that show up in your Zigbee coordinator
  zbMotion.setManufacturerAndModel("Microbots", "CodeCellC6-Motion");

  // Configure this endpoint as a binary input for motion
  zbMotion.addBinaryInput();
  zbMotion.setBinaryInputApplication(
    BINARY_INPUT_APPLICATION_TYPE_SECURITY_MOTION_DETECTION
  );
  zbMotion.setBinaryInputDescription("Motion Detection");

  // Register endpoint with Zigbee stack
  Serial.println("Adding ZigbeeBinary motion endpoint");
  Zigbee.addEndpoint(&zbMotion);

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
  // 10 Hz update rate, same as your original example
  if (myCodeCell.Run(10)) {
    uint8_t state = myCodeCell.Motion_StateRead();
    bool moving = false;

    switch (state) {
      case MOTION_STATE_MOTION:
        moving = true;
        break;
      default:
        moving = false;
        break;
    }

    // Only notify Zigbee when motion state changes
    if (moving != lastMoving) {
      lastMoving = moving;

      zbMotion.setBinaryInput(moving);
      zbMotion.reportBinaryInput();

      Serial.print("Motion changed → ");
      Serial.println(moving ? "MOTION (1)" : "NO MOTION (0)");

      // Optional LED feedback
      if (moving) {
        myCodeCell.LED(0, 255, 0);   // Green when motion detected
      } else {
        myCodeCell.LED(0, 0, 0);     // Off when no motion
      }
    }
  }
}
