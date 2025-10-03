/*
  Example: CodeCell GPIO Demo
  Boards: CodeCell C3 / CodeCell C3 Light / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Demonstrates how to use CodeCell’s GPIO pins for digital input and output.
  - Reads the state of a digital input pin (pin 2).
  - Mirrors the input state on an output pin (pin 1).
  - Great for buttons, switches, or driving LEDs.
*/

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200);               // Start USB serial at 115200 baud
  myCodeCell.Init(LIGHT);             // Initialize the CodeCell with light/proximity sensing enabled
}

void loop() {
  if (myCodeCell.Run(100)) {          // Run the CodeCell service loop at 100 Hz (every 10 ms)

    if (myCodeCell.pinRead(2)) {      // Read pin 5: HIGH means button pressed or signal detected
      myCodeCell.pinWrite(1, HIGH);   // Set pin 1 HIGH (e.g., turn on LED or output signal)
    } else {
      myCodeCell.pinWrite(1, LOW);    // Otherwise set pin 1 LOW (e.g., turn off LED)
    }
  }
}
