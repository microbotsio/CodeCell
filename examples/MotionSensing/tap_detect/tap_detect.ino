/*
  Example: CodeCell Tap Detection Demo
  Boards: CodeCell C3 / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Uses the onboard motion sensor to detect a tap gesture.
  - When a tap is detected, the onboard LED shines yellow for 1 second.
*/

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200);                // Start USB serial at 115200 baud
  myCodeCell.Init(MOTION_TAP_DETECTOR); // Enable tap detection on the motion sensor
}

void loop() {
  if (myCodeCell.Run(10)) {            // Run the CodeCell service loop at 10 Hz (every 100 ms)
    if (myCodeCell.Motion_TapRead()) { // Check if a tap event was detected
      Serial.println(">> Tap Detected!");
      myCodeCell.LED(0xA0, 0x60, 0x00); // Turn on LED yellow
      delay(1000);                      // Keep the LED on for 1 second
    }
  }
}
