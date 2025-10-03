/*
  Example: CodeCell Motion Alarm (RTC Wakeup Timer)
  Boards: CodeCell C3 / CodeCell C3 Light / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Uses the onboard motion sensor to detect movement and trigger an alarm.
  - When motion is detected, lights the LED red and plays a tone on a DriveCell + CoilPad.
  - Lets you toggle the alarm and go to sleep from the MicroLink app.
  - Estimated sleep current consumption: ~40 µA (for CodeCell C6) and ~861 µA (for CodeCell C3)

  App Controls:
  - Button A: Toggle Alarm ON/OFF.
  - Button B: Put CodeCell to sleep (low power). It will wake on the next timer check.

  Wake Behavior:
  - On a timer wake, the device immediately samples motion:
    * If motion is detected, the alarm is triggered (LED red + buzzer) until you press Button B.
    * If no motion is detected, the device returns to sleep and checks again later.
*/

#include <CodeCell.h>
#include <DriveCell.h>
#include <MicroLink.h>

// DriveCell pins
#define IN1_pin1 2
#define IN1_pin2 3

DriveCell myDriveCell(IN1_pin1, IN1_pin2);
CodeCell myCodeCell;
MicroLink myMicroLink;

bool alarm_enable = 1;  // Alarm enabled by default
bool alarm_notify = 0;  // Alarm currently active
uint8_t motion_state = 0;
char myMessage[18];

void setup() {
  Serial.begin(115200);  // Start USB serial at 115200 baud

  if (myCodeCell.WakeUpCheck()) {          // True if we just woke from sleep
    myCodeCell.Motion_Init(MOTION_STATE);  // Initialize motion-state sensing
    myCodeCell.Motion_Read();              // First read
    Serial.print(">> Reading Sensor.. ");
    motion_state = myCodeCell.Motion_StateRead();  // Get motion state

    while (motion_state == MOTION_STATE_UNKNOWN) {  // Wait for a valid reading
      delay(100);
      myCodeCell.Motion_Read();
      motion_state = myCodeCell.Motion_StateRead();
    }

    if (motion_state != MOTION_STATE_MOTION) {  // No motion on wake → go back to sleep
      Serial.println("Alarm Not Triggered!");
      myCodeCell.SleepTimer(1);  // Sleep; check again in 1 second
    } else {
      // Motion detected on wake → trigger alarm
      Serial.println("Alarm Triggered!");
      alarm_enable = 1;
      alarm_notify = 1;

      myMicroLink.Init();                    // Enable MicroLink app UI
      myDriveCell.Init();                    // Init driver
      myMicroLink.Print("Alarm Triggered");  // App message
      myCodeCell.LED(100, 0, 0);             // LED red
      myDriveCell.Tone();                    // Play alarm tone

      while (!myMicroLink.ReadButtonB()) {  // Stay active until Button B is pressed
        delay(10);
      }

      // Silence and return to sleep
      myCodeCell.LED_SetBrightness(0);  // LED off
      myDriveCell.Drive(0, 0);          // Driver off
      myMicroLink.Print("Going To Sleep");
      delay(1000);
      myMicroLink.Print("Waiting for Trigger");
      delay(300);
      myCodeCell.SleepTimer(3);  // Sleep; check again in 3 seconds
    }
  } else {
    // First power-up (not a wake from sleep)
    alarm_enable = 1;
    alarm_notify = 0;

    myCodeCell.Init(MOTION_STATE);    // Init CodeCell + motion sensing
    myCodeCell.LED_SetBrightness(0);  // LED off
    myMicroLink.Init();               // Enable MicroLink app UI
    myDriveCell.Init();               // Init driver

    myDriveCell.Tone();       // Startup buzz
    myDriveCell.Drive(0, 0);  // Driver off

    myMicroLink.Print("Waiting for Trigger");  // App message
    Serial.println(">> Waiting for Alarm to Trigger");
  }
}

void loop() {
  if (myCodeCell.Run(10)) {  // Run at 10 Hz (every 100 ms)

    myMicroLink.ShowSensors(myCodeCell.BatteryLevelRead(), 0, 0);  // Report battery to app

    motion_state = myCodeCell.Motion_StateRead();  // Latest motion state

    if (alarm_enable) {
      if (alarm_notify) {
        myCodeCell.LED(100, 0, 0);  // Keep LED red while alarm is active
      } else if ((motion_state == MOTION_STATE_MOTION) && (!alarm_notify)) {
        // New motion → trigger alarm
        alarm_notify = 1;
        myCodeCell.LED_SetBrightness(10);
        myCodeCell.LED(100, 0, 0);  // LED red
        Serial.println(">> Alarm Triggered");
        myMicroLink.Print("Alarm Triggered");
        myDriveCell.Tone();  // Play alarm tone
      } else {
        // No motion - do nothing
      }
    }

    if (myMicroLink.ReadButtonA()) {  
      alarm_notify = 0;
      alarm_enable = !alarm_enable; // Toggle alarm enable

      if (alarm_enable) {
        myMicroLink.Print("Waiting for Trigger");
        Serial.println(">> Waiting for Alarm to Trigger");
      } else {
        Serial.println(">> Alarm Disabled");
        myMicroLink.Print("Alarm Disabled");
        myDriveCell.Drive(0, 0);          // Ensure driver off
        myCodeCell.LED_SetBrightness(0);  // LED off
      }
      delay(300);
    }

    if (myMicroLink.ReadButtonB()) {  // Put device to sleep
      alarm_notify = 0;
      myCodeCell.LED_SetBrightness(0);  // LED off
      myDriveCell.Drive(0, 0);          // Driver off
      myMicroLink.Print("Going To Sleep");
      delay(1000);
      myMicroLink.Print("Waiting for Trigger");
      delay(300);
      myCodeCell.SleepTimer(3);  // Go to Sleep & check again in 3 seconds 
    }
  }
}
