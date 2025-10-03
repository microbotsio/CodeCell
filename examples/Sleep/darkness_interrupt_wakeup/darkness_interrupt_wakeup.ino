/*
  Example: CodeCell Darkness-Triggered Alarm
  Boards: CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Monitors ambient light with the onboard sensor.
  - When light drops below a threshold (i.e. gets dark), it triggers an alarm:
    * Red LED turns on
    * DriveCell buzzes the CoilPad
  - You can also put the device into low-power sleep from the MicroLink app.
  - While asleep, a darkness event (light below threshold) will wake it and trigger the alarm.
  - Estimated sleep current consumption: ~280 µA

  App Controls:
  - Button A: Toggle the alarm (Enable/Disable). Also silences an active alarm.
  - Button B: Put CodeCell to sleep. It will wake when light falls below the threshold.

  Wake Behavior:
  - If the device just woke from sleep due to darkness, the alarm starts immediately
    (LED red + buzzer) until you press Button A to disable it.

  Notes:
  - Alarm_Threshold defines the trigger level for DARKNESS. Lower value = darker required.
  - Battery level and live light readings are shown in the MicroLink app.
*/

#include <DriveCell.h>
#include <CodeCell.h>
#include "MicroLink.h"

// DriveCell pins used for buzzing the CoilPad 
#define IN1_pin1 2
#define IN1_pin2 3

#define Alarm_Threshold 50  // Darkness threshold (units: sensor counts) — tune for your environment

DriveCell myDriveCell(IN1_pin1, IN1_pin2);
CodeCell  myCodeCell;
MicroLink myMicroLink;

bool wakeup       = 0;  // True if we woke from sleep due to darkness
bool alarm_enable = 1;  // Alarm is enabled by default
char myMessage[18];

void setup() {

  if (myCodeCell.WakeUpCheck()) {  // Check if we woke from sleep because of a darkness-triggered event
    wakeup = 1;                       // Start in "alarm active" state after wake
  } else {
    wakeup = 0;
  }

  Serial.begin(115200);               // Start USB serial at 115200 baud

  myCodeCell.Init(LIGHT);             // Initialize CodeCell with light sensing enabled
  myCodeCell.LED_SetBrightness(0);    // Turn off LED to save power when idle

  myMicroLink.Init();                 // Enable MicroLink app connection (buttons, logs, sensors)

  myDriveCell.Init();                 // Initialize DriveCell driver (used for tone/buzzer)

  // If we woke due to darkness, immediately present the alarm until user disbales it
  if (wakeup) {
    Serial.println(">> Alarm Triggered!");
    myMicroLink.Print("Alarm Triggered");  // Show status in app
    myCodeCell.LED(100, 0, 0);             // Solid red LED
    myDriveCell.Tone();                    // Buzz CoilPad

    alarm_enable = 1;                      // Ensure alarm is considered enabled
    while (alarm_enable) {                 // Block until user silences
      if (myMicroLink.ReadButtonA()) {
        wakeup = 0;
        alarm_enable = 0;                  // Silence alarm
        myMicroLink.Print("Alarm Disabled");
        myDriveCell.Drive(0, 0);           // Stop driver/buzzer
      }
    }
  } else {
    myMicroLink.Print("Alarm Enabled");    // Initial status when not waking from alarm
  }
}

void loop() {  
  if (myCodeCell.Run(10)) { // Run the CodeCell service loop at 10 Hz (every 100 ms)
    
    uint16_t light_val = myCodeCell.Light_WhiteRead(); // Read current light level (white channel)

    
    myMicroLink.ShowSensors(myCodeCell.BatteryLevelRead(), light_val, 0);// Show battery and light in the MicroLink app 

    if (((light_val < Alarm_Threshold) && (alarm_enable)) || (wakeup)) { // Trigger alarm handler
      if (!wakeup) {
        // Normal (not-from-sleep) trigger path
        Serial.println(">> Alarm Triggered!");
        myMicroLink.Print("Alarm Triggered");
        myCodeCell.LED(100, 0, 0);         // Solid red LED
        myDriveCell.Tone();                // Buzz CoilPad
      } else {
        // If we reached here due to a wake event, ensure alarm is marked enabled
        alarm_enable = 1;
      }

      // Stay in alarm until user presses Button A
      while (alarm_enable) {
        if (myMicroLink.ReadButtonA()) {
          wakeup = 0;
          alarm_enable = 0;                // Silence alarm
          myMicroLink.Print("Alarm Disabled");
          myDriveCell.Drive(0, 0);         // Stop driver/buzzer
        }
      }
    }

    // Button A: Toggle alarm enable/disable (and reset wake state if disabling)
    if (myMicroLink.ReadButtonA()) {
      alarm_enable = !alarm_enable;
      if (alarm_enable) {
        myMicroLink.Print("Alarm Enabled");
      } else {
        myMicroLink.Print("Alarm Disabled");
        myDriveCell.Drive(0, 0);           // Ensure buzzer is off
        wakeup = 0;                        // Clear wake-triggered state
      }
      delay(1000);                         // User feedback delay
    }

    // Button B: Enter low-power sleep and wake on darkness threshold
    if (myMicroLink.ReadButtonB()) {
      myDriveCell.Drive(0, 0);             // Ensure buzzer is off before sleep
      myMicroLink.Print("Going To Sleep");
      delay(1000);
      myMicroLink.Print("Waiting for Trigger");  
      delay(300);
      myCodeCell.SleepDarkTrigger(Alarm_Threshold); // Sleep; wake on darkness trigger
    }
  }
}
