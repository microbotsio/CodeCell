/*
  Example: CodeCell Proximity-Triggered Alarm
  Boards: CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Monitors the onboard proximity sensor
  - When proximity exceeds a threshold, it triggers an alarm:
    * Red LED turns on
    * DriveCell buzzes the CoilPad
  - You can put the device into low-power sleep from the MicroLink app.
  - While asleep, a proximity event above the threshold will wake it and trigger the alarm.
  - Estimated sleep current consumption: ~425 µA

  App Controls:
  - Button A: Toggle the alarm (Enable/Disable). Also silences an active alarm.
  - Button B: Put CodeCell to sleep. It will wake when proximity exceeds the threshold.

  Wake Behavior:
  - If the device just woke from sleep due to a proximity trigger, the alarm starts immediately
    (LED red + buzzer) until you press Button A to disable it.
*/

#include <CodeCell.h>
#include <DriveCell.h>
#include "MicroLink.h"

// DriveCell driver pins
#define IN1_pin1 2
#define IN1_pin2 3

// Proximity trigger threshold (tune for your setup)
#define Alarm_Threshold 200

DriveCell myDriveCell(IN1_pin1, IN1_pin2);
CodeCell  myCodeCell;
MicroLink myMicroLink;

bool wakeup = 0;        // True if we woke from sleep due to a proximity trigger
bool alarm_enable = 1;  // Global alarm enable/disable flag
char myMessage[18];     // Reserved (not used here)

void setup() {
  if (myCodeCell.WakeUpCheck()) {  // Returns true if last reset was a wake-from-sleep event
    wakeup = 1;                    // Start with alarm active after wake
  } else {
    wakeup = 0;
  }

  Serial.begin(115200);            // Start USB serial at 115200 baud

  myCodeCell.Init(LIGHT);          // Initialize light/proximity sensing
  myCodeCell.LED_SetBrightness(0); // Turn off LED for baseline state

  myMicroLink.Init();              // Start MicroLink (app I/O and logging)

  myDriveCell.Init();              // Initialize DriveCell outputs

  if (wakeup) {
    Serial.println(">> Alarm Triggered!");
    myMicroLink.Print("Alarm Triggered");  // Show on MicroLink app
    myCodeCell.LED(100, 0, 0);             // Solid red LED
    myDriveCell.Tone();                     // Buzz CoilPad

    alarm_enable = 1;                       // Keep alarm latched until user silences
    while (alarm_enable) {
      if (myMicroLink.ReadButtonA()) {      // Button A silences alarm
        wakeup = 0;
        alarm_enable = 0;                   // Turn off alarm
        myMicroLink.Print("Alarm Disabled");
        myDriveCell.Drive(0, 0);            // Stop driver
      }
    }
  } else {
    myMicroLink.Print("Alarm Enabled");     // Default state on fresh boot
  }
}

void loop() {
  if (myCodeCell.Run(10)) {  // Run service loop at 10 Hz (every ~100 ms)
    uint16_t proximity_val = myCodeCell.Light_ProximityRead();  // Read proximity

    // Send live telemetry to the MicroLink app (Battery %, Proximity, placeholder 0)
    myMicroLink.ShowSensors(myCodeCell.BatteryLevelRead(), proximity_val, 0);

    // Trigger if threshold crossed while enabled, or if we’re resuming from a wake event
    if (((proximity_val > Alarm_Threshold) && (alarm_enable)) || (wakeup)) {
      if (!wakeup) {
        Serial.println(">> Alarm Triggered!");
        myMicroLink.Print("Alarm Triggered");
        myCodeCell.LED(100, 0, 0);         // Solid red LED
        myDriveCell.Tone();                 // Buzz CoilPad
      } else {
        // If we came from sleep, ensure the alarm is enabled so it latches
        alarm_enable = 1;
      }

      // Latch alarm until user presses Button A to silence
      while (alarm_enable) {
        if (myMicroLink.ReadButtonA()) {
          wakeup = 0;
          alarm_enable = 0;                 // Turn off alarm
          myMicroLink.Print("Alarm Disabled");
          myDriveCell.Drive(0, 0);          // Stop driver
        }
      }
    }

    // Button A: Toggle Alarm Enabled/Disabled during normal operation
    if (myMicroLink.ReadButtonA()) {
      alarm_enable = !alarm_enable;         // Toggle state
      if (alarm_enable) {
        myMicroLink.Print("Alarm Enabled");
      } else {
        myMicroLink.Print("Alarm Disabled");
        myDriveCell.Drive(0, 0);            // Ensure driver is off
        wakeup = 0;                         // Clear wake latch
      }
      delay(1000);                          // Debounce / user feedback delay
    }

    // Button B: Enter sleep, waking on proximity crossing the threshold
    if (myMicroLink.ReadButtonB()) {
      myDriveCell.Drive(0, 0);              // Ensure outputs are off before sleeping
      myMicroLink.Print("Going To Sleep");
      delay(1000);
      myMicroLink.Print("Waiting for Trigger");
      delay(300);
      myCodeCell.SleepProximityTrigger(Alarm_Threshold); // Sleep with proximity as wake source
    }
  }
}
