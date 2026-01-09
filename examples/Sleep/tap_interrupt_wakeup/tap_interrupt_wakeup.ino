/*
  Example: CodeCell Tap Wakeup Demo
  Boards: CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Uses the onboard motion sensor to detect a tap gesture.
  - Lets you put the device into low-power sleep from the MicroLink app.
  - While asleep, a tap on the device will wake it back up.
  - Estimated sleep current consumption: ~860 µA

  App Controls:
  - Button A: Put CodeCell to sleep. It will wake on the next tap.

  Wake Behavior:
  - If the device just woke from sleep because of a tap, the LED will glow yellow for 1 second.
  - Otherwise, we start normally and begin listening for taps.
*/

#include <CodeCell.h>
#include <MicroLink.h>

CodeCell myCodeCell;    
MicroLink myMicroLink;  

void setup() {
  Serial.begin(115200);  // Start USB serial at 115200 baud

  if (myCodeCell.WakeUpCheck()) {                 // Returns true if we woke due to a tap event
    Serial.println(">> Tap Detected on Wake!");
    myCodeCell.LED(0xA0, 0x60, 0x00);             // RGB hex (R,G,B). Here: warm yellow
    delay(1000);                                  // Show the indication for 1 second
  }

  myCodeCell.Init(MOTION_TAP_DETECTOR); // Enable tap detection on the motion sensor
  myMicroLink.Init(); // Enable the MicroLink app so we can read Button A and print messages to the app
}

void loop() {
  if (myCodeCell.Run(10)) { // Run the CodeCell service loop at 10 Hz (every 100 ms)
   
    if (myCodeCell.Motion_TapDetectorRead()) {    // Check for new tap events while we’re awake.
      Serial.println(">> Tap Detected!");
      myCodeCell.LED(0xA0, 0x60, 0x00);           // Brief yellow flash to confirm the tap
      delay(1000);
    }

    if (myMicroLink.ReadButtonA()) {  // If the user presses Button A in the MicroLink app, enter low-power sleep
      myCodeCell.LED_SetBrightness(0);            // Turn off onboard LED to save power
      myMicroLink.Print("Going To Sleep");        // Message visible in the app
      delay(1000);
      myMicroLink.Print("Waiting for Trigger");   // Clarify that a tap will wake the device
      delay(300);
      myCodeCell.SleepTapTrigger(); // Enter sleep and configure tap as the wake source
    }
  }
}
