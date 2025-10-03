/*
  Example: CodeCell Auto-Dimming LED
  Boards: CodeCell C3 / CodeCell C3 Light / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Uses the onboard light sensor to measure white light.
  - Continuously adjusts the onboard LED brightness based on ambient light.
  - Current behavior: the LED gets brighter as the room gets brighter, and dims when the room gets darker.

  Notes:
  - Brightness is scaled from the sensor reading (>> 3) and clamped to 1–254
    to avoid turning the LED completely off or exceeding its max drive.
  - To invert the behavior (brighter in the dark), uncomment the line marked “Invert”.
  - Feel free to tweak the mapping to suit your environment.

*/

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200);             // Start USB serial at 115200 baud
  myCodeCell.Init(LIGHT);           // Enable onboard light/proximity sensing
  myCodeCell.LED_SetBrightness(0);  // Disable the default LED breathing annimation
}

void loop() {
  if (myCodeCell.Run(10)) {         // Run loop at 10 Hz
    
    uint16_t brightness = (myCodeCell.Light_WhiteRead()) >> 3; // Map the raw white-light reading to an 8-bit-like range

    // Clamp to a usable LED range (1–254)
    if (brightness == 0U) {
      brightness = 1U;              // Prevent the LED from fully turning off
    } else if (brightness > 254U) {
      brightness = 254U;            // Cap to avoid over-driving
    }

    // Invert: uncomment to make darker room -> brighter LED
    // brightness = 255U - brightness;  // Invert

    Serial.println(brightness);      // Debug: scaled LED value
    myCodeCell.LED(0, 0, brightness); // Drive onboard LED (blue channel)
  }
}
