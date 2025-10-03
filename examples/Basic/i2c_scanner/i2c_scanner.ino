/*
  Example: CodeCell I2C Scanner Demo
  Boards: CodeCell C3 / CodeCell C3 Light / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Initializes the onboard light/proximity sensor.
  - Scans the I2C bus (0x01–0x7E) and prints any detected device addresses.
  - Useful for confirming external sensors are wired correctly.

  How It Works:
  - Probes each I2C address and reports success or errors to the Serial Monitor.
  - Prints a summary count at the end of the scan.

  Notes:
  - Open the Serial Monitor at 115200 baud.
  - Ensure Tools → USB_CDC_On_Boot is enabled 
*/

#include <Wire.h>
#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200);                   // Start USB serial at 115200 baud

  myCodeCell.Init(LIGHT);                 // Initialize light/proximity sensing

  // I2C scanner
  Serial.println("Scanning I2C devices...");
  Wire.begin();                           // Join I2C bus as master
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) {   // Probe 0x01..0x7E
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");        // Pad single hex digit
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found.\n");
  } else {
    Serial.println("Scan complete.\n");
  }
}

void loop() {
  if (myCodeCell.Run(10)) {              // Run at 10 Hz 
    // Add periodic tasks here if needed.
  }
}
