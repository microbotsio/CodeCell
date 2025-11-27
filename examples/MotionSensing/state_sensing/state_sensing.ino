/*
  Example: Motion State Monitor Demo
  Boards: CodeCell C3 / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Demonstrates the CodeCell's Motion State sensing.
  - Continuously reports whether the device is On-Table, In Motion, Stabilizing, or Stationary.  
*/

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200);           // Start USB serial at 115200 baud
  myCodeCell.Init(MOTION_STATE);  // Enable Motion State sensing
}

void loop() {
  if (myCodeCell.Run(10)) {  // Run at 10 Hz
    Serial.print("State: ");
    switch (myCodeCell.Motion_StateRead()) {
      case MOTION_STATE_STABLE:
        Serial.println("Motion Stopped - Stabilizing");
        break;
      case MOTION_STATE_ONTABLE:
        Serial.println("On Table");
        break;
      case MOTION_STATE_STATIONARY:
        Serial.println("Stationary");
        break;
      case MOTION_STATE_MOTION:
        Serial.println("In Motion");
        break;
      default:
        Serial.println("Unknown");
        break;
    }
  }
}
