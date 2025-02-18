/*
 * Overview:
 * This code demonstrates the CodeCell's Rotation Sensing features.
 * In this example, the CodeCell continuously reads the angular rotations 
 * Every 100ms it prints the Roll, Pitch, and Yaw angles on the Serial Monitor
 * Feel free to tweak the code with your own creative ideas!
 */

#include <CodeCell.h>

CodeCell myCodeCell;

float Roll = 0.0;
float Pitch = 0.0;
float Yaw = 0.0;

void setup() {
  Serial.begin(115200); // Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial.

  myCodeCell.Init(MOTION_ROTATION); //Initializes Rotation Sensing
}

void loop() {
  if (myCodeCell.Run(10)) {  //Run every 10Hz
    myCodeCell.Motion_RotationRead(Roll, Pitch, Yaw);
    Serial.printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", Roll, Pitch, Yaw);
  }
}
