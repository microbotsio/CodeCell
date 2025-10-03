/*
  Example: CodeCell MicroLink Demo
  Boards: CodeCell C3 / CodeCell C3 Light / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Connects to the MicroLink app over Bluetooth.
  - Streams live sensor data to the app: battery %, proximity, and yaw angle.
  - Reads app inputs (Buttons A–D and Joystick) and prints them to Serial.

  App Controls:
  - Buttons A–D: When pressed, a message is printed to the Serial Monitor.
  - Joystick (press to read): Shows X/Y values in the Serial Monitor.
*/

#include <CodeCell.h>
#include <MicroLink.h>

CodeCell myCodeCell;    // CodeCell device instance
MicroLink myMicroLink;  // MicroLink app interface

float Roll, Pitch, Yaw;  // Rotation angles (degrees)

void setup() {
  Serial.begin(115200);                      // Start USB serial at 115200 baud
  myCodeCell.Init(LIGHT + MOTION_ROTATION);  // Enable light (incl. proximity) + rotation sensing
  myMicroLink.Init();                        // Start Bluetooth link to the MicroLink app
}

void loop() {
  if (myCodeCell.Run(100)) {  // Run at 100 Hz

    myCodeCell.Motion_RotationRead(Roll, Pitch, Yaw);  // Read Roll/Pitch/Yaw

    // Send battery level, proximity, and yaw to the app’s sensor panel
    myMicroLink.ShowSensors(myCodeCell.BatteryLevelRead(), myCodeCell.Light_ProximityRead(), Yaw);

    // For CodeCell C3 Light use this instead:
    // myMicroLink.ShowSensors(myCodeCell.BatteryLevelRead(), myCodeCell.Light_ProximityRead(), 0);


    // Buttons A–D → print which one was pressed
    if (myMicroLink.ReadButtonA()) {
      Serial.println("ButtonA Pressed");
    }
    if (myMicroLink.ReadButtonB()) {
      Serial.println("ButtonB Pressed");
    }
    if (myMicroLink.ReadButtonC()) {
      Serial.println("ButtonC Pressed");
    }
    if (myMicroLink.ReadButtonD()) {
      Serial.println("ButtonD Pressed");
    }

    // Joystick: only read/print when the stick is pressed
    if (myMicroLink.ReadButtonJoystick()) {
      Serial.print("Joystick X: ");
      Serial.print(myMicroLink.ReadJoystickX());
      Serial.print(", Y: ");
      Serial.println(myMicroLink.ReadJoystickY());
    }

    myMicroLink.Print("Hello World");  // Send a text message to the app console
  }
}
