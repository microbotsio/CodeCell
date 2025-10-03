/*
  Example: CodeCell PWM Basics
  Boards: CodeCell C3 /CodeCell C3 Light / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Demonstrates generating PWM signals on three GPIO pins.
  - Sets distinct PWM frequencies and duty cycles per pin.

  Pin Setup (this example):
  - Pin 1: 1 kHz @ 50% duty
  - Pin 2: 10 kHz @ 75% duty
  - Pin 3: 20 kHz @ 10% duty
*/

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200);    // Start USB serial at 115200 baud
  myCodeCell.Init(LIGHT);  // Initialize with LIGHT sensing 
}

void loop() {
  if (myCodeCell.Run(10)) {                // Service loop at 10 Hz (every ~100 ms)
    myCodeCell.pinPWM(1, 1000, 50);        // Pin 1: 1 kHz, 50% duty
    myCodeCell.pinPWM(2, 10000, 75);       // Pin 2: 10 kHz, 75% duty
    myCodeCell.pinPWM(3, 20000, 10);       // Pin 3: 20 kHz, 10% duty
  }
}
