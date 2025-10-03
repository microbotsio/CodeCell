/*
  Example: CodeCell Shake Detection Demo
  Boards: CodeCell C3 / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Uses the onboard accelerometer to detect sudden shake gestures.
  - Continuously reads X, Y, and Z axis acceleration data.
  - Calculates the total movement strength using vector magnitude.
  - If the strength exceeds a defined threshold, a shake is detected and the LED turns red.

  Notes:
  - SHAKE_THRESHOLD defines how sensitive the shake detection is. Lower values = more sensitive.
  - You can experiment with thresholds depending on how strong you want the shake to be.
  - The LED stays red for 0.5 seconds after a shake is detected.
*/

#include <CodeCell.h>
#include <math.h>  // Needed for square root calculations

CodeCell myCodeCell;

float x = 0.0, y = 0.0, z = 0.0;                 // Store acceleration values
const float SHAKE_THRESHOLD = 15.0;              // Adjust sensitivity threshold

void setup() {
  Serial.begin(115200);                          // Start USB serial at 115200 baud
  myCodeCell.Init(MOTION_ACCELEROMETER);         // Enable accelerometer
}

void loop() {
  if (myCodeCell.Run(10)) {                      // Run at 10 Hz (every 100 ms)
    myCodeCell.Motion_AccelerometerRead(x, y, z); // Read accelerometer X, Y, Z data

    // Calculate total movement strength (vector magnitude)
    float totalAcceleration = sqrt(x * x + y * y + z * z);

    Serial.print("Total Acceleration: ");
    Serial.println(totalAcceleration);           // Print acceleration strength to Serial Monitor

    // Check if movement exceeds threshold
    if (totalAcceleration > SHAKE_THRESHOLD) {   
      Serial.println("Shake detected!");
      myCodeCell.LED(255, 0, 0);                 // Turn LED red
      delay(500);                                // Keep LED on for 0.5 seconds
    } else {
      // No shake detected
    }
  }
}
