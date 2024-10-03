/*
 * Overview:
 * This code demonstrates the CodeCell's Rotation Sensing features.
 * In this example, the CodeCell continuously reads the angular rotations and use these angles to control a servo motor
 * Feel free to tweak the code with your own creative ideas!
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-servo-angle-control
 */

#include <CodeCell.h>
#include <ESP32Servo.h>

CodeCell myCodeCell;
Servo myservo;  

float Roll = 0.0;
float Pitch = 0.0;
float Yaw = 0.0;
int servo_angle = 0;

void setup() {
    Serial.begin(115200); /*Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial*/
    
    myCodeCell.Init(MOTION_ROTATION); /*Initializes rotation sensing */
    myservo.attach(1);  /*Attaches the servo on pin 1 to the servo object*/
}

void loop() {
    if (myCodeCell.Run()) { /*Runs  every 100ms*/
        
        myCodeCell.Motion_RotationRead(Roll, Pitch, Yaw); /* Read rotation angles from the BNO085 sensor*/
        
        
        servo_angle = abs((int)Pitch);/*Convert the pitch angle to a servo angle*/
        servo_angle = (180 - servo_angle);
        
        /*Limit the servo angle to the range 0-60 degrees*/
        if (servo_angle > 60) {
            servo_angle = 60;
        } else if (servo_angle < 0) {
            servo_angle = 0;
        }
        
        Serial.println(servo_angle); /* Print the servo angle for debugging */
        myservo.write(servo_angle);  /* Set the servo position */
    }
}
