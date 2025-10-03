/*
  Example: CodeCell Rotation Sensing Demo
  Boards: CodeCell C3 / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Demonstrates the CodeCell’s onboard motion sensor in rotation mode.
  - Continuously reads angular data (Roll, Pitch, Yaw).
  - Prints the values to the Serial Monitor every 100 ms (10 Hz refresh rate).
*/

#include <CodeCell.h>

CodeCell myCodeCell;

float Roll = 0.0;
float Pitch = 0.0;
float Yaw = 0.0;

void setup() {
  Serial.begin(115200);                     // Start USB serial at 115200 baud
  myCodeCell.Init(MOTION_ROTATION);         // Enable rotation sensing (Roll, Pitch, Yaw)
}

void loop() {
  if (myCodeCell.Run(10)) {                 // Run at 10 Hz (every 100 ms)
    myCodeCell.Motion_RotationRead(Roll, Pitch, Yaw); // Read latest rotation values
    Serial.printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", Roll, Pitch, Yaw); // Print  output
  }
}
