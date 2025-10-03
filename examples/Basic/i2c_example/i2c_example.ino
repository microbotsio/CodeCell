/*
  Example: CodeCell I2C Demo - Color Sensing (APDS-9960)
  Boards: CodeCell C3 / CodeCell C3 Light / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Initializes the CodeCell * its light sensor.
  - Sets up an external I2C APDS-9960 color sensor and enables color mode.
  - Streams raw Red/Green/Blue values over USB Serial.
  - Also prints the CodeCell light sensor value for quick debugging.

  Wiring:
  - Connect APDS-9960 to I2C: 3V3, GND, SDA, SCL (default address 0x39).

  Notes:
  - Download the 'Adafruit_APDS9960.h' to try this example out
  - Ensure Tools → USB_CDC_On_Boot is enabled for Serial
*/

#include <CodeCell.h>
#include <Adafruit_APDS9960.h>

CodeCell myCodeCell;
Adafruit_APDS9960 apds;

uint16_t r, g, b, c;  // Raw color channels (R, G, B, and Clear)

void setup() {
  Serial.begin(115200);                // Start USB Serial at 115200 baud

  myCodeCell.Init(LIGHT);              // Initialize CodeCell light sensing

  if (!apds.begin()) {                 // Initialize the APDS-9960 over I2C
    Serial.println("Failed to initialize APDS-9960! Check wiring.");
  } else {
    Serial.println("APDS-9960 initialized.");
  }

  apds.enableColor(true);              // Enable color-sensing mode
}

void loop() {
  if (myCodeCell.Run(10)) {            // Run at 10 Hz (every ~100 ms)
    myCodeCell.PrintSensors();         // Print CodeCell sensors snapshot

    if (apds.colorDataReady()) {       // Read color channels when ready
      apds.getColorData(&r, &g, &b, &c);
      Serial.print(">> Color Sensors: R");
      Serial.print(r);
      Serial.print(",G");
      Serial.print(g);
      Serial.print(",B");
      Serial.println(b);
    }
  }
}
