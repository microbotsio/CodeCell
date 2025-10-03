/*
  Example: CodeCell Activity + Step Counter Demo
  Boards: CodeCell C3 / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Uses the onboard motion sensor to track steps and activity
  - Sends battery level and yaw angle to the MicroLink app in real time.
  - Lets you check the activity classifier (Driving, Cycling, Walking, Still, Tilting, Running, Climbing).
 
  App Controls:
  - Button A: Guess and print your current activity.
  - Button B: Reset the displayed step count (zero via offset).

  Notes:
  - Step reset is implemented with an offset (the hardware counter keeps increasing).
*/

#include <CodeCell.h>
#include <MicroLink.h>

CodeCell myCodeCell;
MicroLink myMicroLink;

uint16_t step_counter = 0, step_zero = 0;  // step_zero is the offset applied after "reset"
char message[18];                          // Buffer for printing "<N> Steps"
float Roll, Pitch, Yaw;                    // Rotation readings

void setup() {
  Serial.begin(115200);  // Start USB serial at 115200 baud

  // Enable motion rotation, step counter, and activity classifier
  myCodeCell.Init(MOTION_ROTATION + MOTION_STEP_COUNTER + MOTION_ACTIVITY);

  myMicroLink.Init();    // Start MicroLink so we can read buttons and print to the app
}

void loop() {
  if (myCodeCell.Run(10)) {  // Run the CodeCell service loop at 10 Hz (every 100 ms)

    // Update rotation readings (Roll, Pitch, Yaw)
    myCodeCell.Motion_RotationRead(Roll, Pitch, Yaw);

    // Show battery (%, left), dummy proximity (0, middle), and Yaw (right) in the app HUD
    myMicroLink.ShowSensors(myCodeCell.BatteryLevelRead(), 0, Yaw);

    if (myMicroLink.ReadButtonA()) {  // Query and print activity guess
      switch (myCodeCell.Motion_ActivityRead()) {
        case 1:  myMicroLink.Print("Are You Driving?");  break;
        case 2:  myMicroLink.Print("Are You Cycling?");  break;
        case 3:
        case 6:  myMicroLink.Print("Are You Walking?");  break;
        case 4:  myMicroLink.Print("Are You Still?");    break;
        case 5:  myMicroLink.Print("Are You Tilting?");  break;
        case 7:  myMicroLink.Print("Are You Running?");  break;
        case 8:  myMicroLink.Print("Are You Climbing?"); break;
        default: myMicroLink.Print("Still Guessing..");  break;
      }
      delay(2000);  // Give time to read the message in the app
    } else {
      if (myMicroLink.ReadButtonB()) {             // Reset the displayed step count
        step_zero = step_counter + step_zero;      // Advance offset so display returns to zero
      }

       // Read steps and apply offset so resets show as zeroed
      step_counter = myCodeCell.Motion_StepCounterRead() - step_zero;

      sprintf(message, "%u Steps", step_counter);  // Build "N Steps" string
      myMicroLink.Print(message);                  // Print to MicroLink app
    }
  }
}
