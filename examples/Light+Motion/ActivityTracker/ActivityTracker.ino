/*
 * Overview:
 * This code demonstrates the CodeCell's Step Counting & Activity Sensing features.
 * In this example, the CodeCell continuously reads the step counts & motion activity 
 * Every 100ms it prints counter & activity on the Serial Monitor
 * Feel free to tweak the code with your own creative ideas!
 */

#include <CodeCell.h>

CodeCell myCodeCell;

uint16_t step_counter = 0;

void setup() {
  Serial.begin(115200); /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */

  myCodeCell.Init(MOTION_STEP_COUNTER + MOTION_ACTIVITY); /*Initializes Step Counter & Activity Sensing*/
}

void loop() {
  if (myCodeCell.Run()) {   
      myCodeCell.Motion_StepCounterRead(step_counter);
      Serial.print("Steps: ");
      Serial.println(step_counter);

      Serial.print("Activity: ");
      switch (myCodeCell.Motion_ActivityRead()) {
        case 1:
          Serial.println("Driving");
          break;
        case 2:
          Serial.println("Cycling");
          break;
        case 3:
        case 6:
          Serial.println("Walking");
          break;
        case 4:
          Serial.println("Still");
          break;
        case 5:
          Serial.println("Tilting");
          break;
        case 7:
          Serial.println("Running");
          break;
        case 8:
          Serial.println("Climbing Stairs");
          break;
        default:
          Serial.println("Unkown");
          break;
      }
  }
}
