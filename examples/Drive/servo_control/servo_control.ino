/*
  Example: CodeCell Servo Angle Control Demo
  Boards: CodeCell C3 / CodeCell C3 Light / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Uses the onboard motion sensor to read rotation angles (Roll, Pitch, Yaw).
  - The Pitch angle is mapped to control a servo motor on pin 1.
  - Servo angle is limited to a range of 0–60 degrees for safety.

  Notes:
  - Adjust the mapping logic if you want to use Roll or Yaw instead of Pitch.
*/

#include <CodeCell.h>
#include <ESP32Servo.h>

CodeCell myCodeCell;
Servo myservo;

float Roll = 0.0, Pitch = 0.0, Yaw = 0.0;
int servo_angle = 0;

void setup() {
  Serial.begin(115200);  // Start USB serial at 115200 baud

  myCodeCell.Init(MOTION_ROTATION);  // Enable rotation sensing on the motion sensor
  myservo.attach(1);                 // Attach the servo on pin 1 to the servo object
}

void loop() {
  if (myCodeCell.Run(10)) {  // Run at 10 Hz 

    myCodeCell.Motion_RotationRead(Roll, Pitch, Yaw);  // Read rotation angles from the BNO085 sensor

    servo_angle = abs((int)Pitch);      // Convert Pitch angle to a positive servo angle
    servo_angle = (180 - servo_angle);  // Invert mapping for servo direction

    // Limit the servo angle to the range 0–60 degrees
    if (servo_angle > 60) {
      servo_angle = 60;
    } else if (servo_angle < 0) {
      servo_angle = 0;
    }
    else{
        //Range ok
    }

    Serial.println(servo_angle);  // Print the current servo angle for debugging
    myservo.write(servo_angle);   // Set the servo position based on pitch
  }
}
