/*
  Example: CodeCell Digital Compass
  Boards: CodeCell C3 / CodeCell C3 Light / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Reads the onboard magnetometer (X, Y, Z).
  - Computes a 2D compass heading using atan2(Y, X).
  - Outputs a heading in degrees from 0° to 360°.
  - Tip: For best accuracy, keep the board level and away from any magnetic or metal objects.

  Notes:
  - This example is not tilt-compensated; pitch/roll will affect accuracy.
*/

#include <CodeCell.h>
#include <math.h>  // Needed for atan2 and M_PI

CodeCell myCodeCell;

float x = 0.0, y = 0.0, z = 0.0;  // Magnetometer axes 

void setup() {
  Serial.begin(115200);                        // Start USB serial at 115200 baud
  myCodeCell.Init(MOTION_MAGNETOMETER);        // Initialize the magnetometer
}

void loop() {
  if (myCodeCell.Run(10)) {                    // Run the CodeCell service loop at 10 Hz (every 100 ms)

    myCodeCell.Motion_MagnetometerRead(x, y, z);  // Read magnetometer values into x, y, z

    // Compute 2D heading in degrees using the horizontal plane (Y vs X)
    float heading = atan2(y, x) * (180.0 / M_PI);

    // Normalize heading to the range 0°–360°
    if (heading < 0.0) {
      heading += 360.0;
    }

    Serial.print("Compass Heading: ");
    Serial.println(heading);                   // Print heading in degrees
  }
}
