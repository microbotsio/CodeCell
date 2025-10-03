/*
  Example: CodeCell Gyroscope Rotation Demo
  Boards: CodeCell C3 / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Uses the onboard gyroscope to detect rotational speed.
  - Continuously measures rotation along the Z-axis (yaw).
  - LED feedback:
    * Green = rotating right above threshold
    * Red   = rotating left above threshold
    * Off   = no significant rotation
*/

#include <CodeCell.h>

CodeCell myCodeCell;

float x = 0.0, y = 0.0, z = 0.0;           // Gyroscope readings (°/s)
const float ROTATION_THRESHOLD = 3.0;      // Rotation speed threshold in degrees/sec

void setup() {
  Serial.begin(115200);                    // Start USB serial at 115200 baud
  myCodeCell.Init(MOTION_GYRO);            // Enable gyroscope sensing
}

void loop() {
  if (myCodeCell.Run(10)) {                // Run the CodeCellat 10 Hz 
    
    myCodeCell.Motion_GyroRead(x, y, z);   // Get gyroscope values for X, Y, Z axes

    Serial.print("Gyro Z: ");              
    Serial.println(z);                     // Print Z-axis rotation speed to Serial

    if (z > ROTATION_THRESHOLD) {          
      myCodeCell.LED(0, 255, 0);           // LED green = rotating right
    } 
    else if (z < -ROTATION_THRESHOLD) {    
      myCodeCell.LED(255, 0, 0);           // LED red = rotating left
    } 
    else {
      myCodeCell.LED(0, 0, 0);             // LED off = no rotation
    }
  }
}
