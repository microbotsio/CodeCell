/*
  Example: CodeCell Depth Gesture Demo
  Boards: CodeCell C3 / CodeCell C3 Light / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Uses the onboard proximity sensor to detect depth gestures.
  - Moving your hand closer to the sensor increases the proximity value.
  - Moving your hand away decreases the value.
  - This example maps proximity readings into gesture states and prints them over Serial.

  Gesture Logic:
  - Near Gesture: Hand detected very close to the sensor
  - Mid Gesture: Hand detected at a medium distance
  - Far Gesture: Hand detected further away
  - No Hand: Nothing detected

  Notes:
  - Adjust the thresholds to match your environment and application.
  - Typical proximity readings range between ~0 (no object) to ~2000+ (very close object),
    but may vary depending on reflectivity and lighting.
*/

#include <CodeCell.h>

CodeCell myCodeCell;

// Thresholds for gesture zones (tweak as needed)
const uint16_t NEAR_THRESHOLD = 700;
const uint16_t MID_THRESHOLD = 100;
const uint16_t FAR_THRESHOLD = 10;

void setup() {
  Serial.begin(115200);    // Start USB serial at 115200 baud
  myCodeCell.Init(LIGHT);  // Enable onboard light + proximity sensing

  myCodeCell.LED_SetBrightness(0);  // Turn off default LED breathing animation

  Serial.println(">> Depth Gesture Demo Started");
}

void loop() {
  if (myCodeCell.Run(2)) {  // Run at 2Hz
    uint16_t proximity = myCodeCell.Light_ProximityRead();

    // Print raw sensor value
    Serial.print("Proximity: ");
    Serial.print(proximity);
    Serial.print("  | Gesture: ");

    // Classify depth gesture
    if (proximity > NEAR_THRESHOLD) {
      Serial.println("NEAR");
      myCodeCell.LED(0xFF, 0x00, 0x00);  // Red LED for near
    } else if (proximity > MID_THRESHOLD) {
      Serial.println("MID");
      myCodeCell.LED(0xFF, 0xA5, 0x00);  // Orange LED for mid
    } else if (proximity > FAR_THRESHOLD) {
      Serial.println("FAR");
      myCodeCell.LED(0x00, 0x00, 0xFF);  // Blue LED for far
    } else {
      Serial.println("NONE");
      myCodeCell.LED(0x00, 0x00, 0x00);  // LED off for no hand
    }
  }
}
