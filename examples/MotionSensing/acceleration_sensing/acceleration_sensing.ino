/*
  Example: Accelerometer Readings Demo
  Boards: CodeCell C3 / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Uses the onboard accelerometer to measure motion on the X, Y, and Z axes.
  - Continuously prints live acceleration data to the Serial Monitor.
  - Can be used to detect shake, vibration, or orientation changes.

  Notes:
  - Acceleration values are in g-units (1 g ≈ 9.81 m/s²).
  - Sampling rate is set to 10 Hz in this example, but you can adjust it as needed.
*/

#include <CodeCell.h>

CodeCell myCodeCell;

float x = 0.0;  // X-axis acceleration
float y = 0.0;  // Y-axis acceleration
float z = 0.0;  // Z-axis acceleration

void setup() {
  Serial.begin(115200);                       // Start USB serial at 115200 baud
  myCodeCell.Init(MOTION_ACCELEROMETER);      // Enable accelerometer sensor
}

void loop() {
  if (myCodeCell.Run(10)) {                   // Run service loop at 10 Hz (every 100 ms)
    myCodeCell.Motion_AccelerometerRead(x, y, z);  // Read acceleration values
    Serial.print("X: "); Serial.print(x);     // Print X value
    Serial.print("  Y: "); Serial.print(y);   // Print Y value
    Serial.print("  Z: "); Serial.println(z); // Print Z value
  }
}
