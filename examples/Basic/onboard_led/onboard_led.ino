/*
  Example: CodeCell Onboard LED Demo
  Boards: CodeCell C3 / CodeCell C3 Light / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Uses the onboard light/proximity sensor to detect nearby objects.
  - When an object is detected within range, the LED glows red for 1 second.
  - Otherwise, the LED remains off.

  Behavior:
  - Proximity threshold is set at >100 (adjust for your environment).
  - LED turns solid red at max brightness when triggered.
*/

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200);    // Start USB serial at 115200 baud
  myCodeCell.Init(LIGHT);  // Initialize light sensing
}

void loop() {
  if (myCodeCell.Run(10)) {  // Run at 10 Hz (every 100 ms)

    uint16_t proximity = myCodeCell.Light_ProximityRead();  // Read current proximity value

    if (proximity > 100) {               // Check if object is detected within range
      myCodeCell.LED_SetBrightness(10);  // Turn LED on at max brightness
      myCodeCell.LED(0xFF, 0, 0);        // Set LED color to Red
      delay(1000);                       // Keep LED on for 1 second
    } else {
      myCodeCell.LED_SetBrightness(0);  // Otherwise, turn LED off
    }
  }
}
