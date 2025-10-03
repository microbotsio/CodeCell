/*
  Example: CodeCell Proximity Sensing Demo
  Boards: CodeCell C3 / CodeCell C3 Light / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Continuously measures proximity using the onboard proximity sensor.
  - Turns the LED red when an object is detected within the threshold distance.
  - Turns the LED green when no object is detected (outside the threshold).
  - Runs the service loop at 1 Hz to keep power use low.
  - Threshold is adjustable via DISTANCE_RANGE.

  Notes:
  - The default breathing effect is disabled so LED feedback matches proximity state.
  - Tune DISTANCE_RANGE to match your environment and desired trigger distance.

*/

#include <CodeCell.h>

CodeCell myCodeCell;

#define DISTANCE_RANGE 100U  // Proximity trigger threshold (higher = requires closer object)

void setup() {
  Serial.begin(115200);              // Start USB serial at 115200 baud
  myCodeCell.Init(LIGHT);            // Enable light + proximity sensing
  myCodeCell.LED_SetBrightness(0);   // Turn off default LED breathing animation
}

void loop() {
  if (myCodeCell.Run(1)) {           // Run the CodeCell service loop at 1 Hz
  
    // Read proximity; values above DISTANCE_RANGE mean "object detected / close"
    if (myCodeCell.Light_ProximityRead() > DISTANCE_RANGE) {
      myCodeCell.LED(0xFF, 0x00, 0x00);  // Red: proximity detected
    } else {
      myCodeCell.LED(0x00, 0xFF, 0x00);  // Green: no proximity detected
    }
  }
}
