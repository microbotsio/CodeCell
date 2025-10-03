/*
  Example: CodeCell Analog Read Demo
  Boards: CodeCell C3 / CodeCell C3 Light / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Reads an analog value (0–4095) from pin 2 (e.g. a potentiometer or sensor).
  - Prints the reading over USB serial for quick plotting or debugging.
  - Initializes the light/proximity block (optional) 

  Hardware:
  - Connect a potentiometer: wiper → pin 2 (ADC), ends → 3V3 and GND.
*/

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200);           // Start USB serial at 115200 baud
  myCodeCell.Init(LIGHT);         // Initialize light sensing (optional for this demo)
}

void loop() {
  if (myCodeCell.Run(10)) {       // Run the CodeCell service loop at 10 Hz (every 100 ms)
    uint16_t potValue = myCodeCell.pinADC(2);  // Read raw ADC value from pin 2
    Serial.println(potValue);     // Print the value (range typically 0–4095)
  }
}
