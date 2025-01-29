/*
 * Overview:
 * This code demonstrates the CodeCell's Motion State Sensing feature.
 * In this example, the CodeCell continuously monitors its motion status.
 * On the Serial Monitor, it outputs whether it's On-Table, In Motion, Stabilizing, or Stationary.
 * Feel free to tweak the code with your own creative ideas! 
 */

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200); /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */

  myCodeCell.Init(MOTION_STATE); /*Initializes Motion State Sensing*/
}

void loop() {
  if (myCodeCell.Run(10)) { /*Run every 100ms (10Hz)*/
    Serial.print("State: ");
    switch (myCodeCell.Motion_StateRead()) {
      case MOTION_STATE_STABLE:
        Serial.println("Motion Stopped - Stabilizing");
        break;
      case MOTION_STATE_ONTABLE:
        Serial.println("On Table");
        break;
      case MOTION_STATE_STATIONARY:
        Serial.println("Stationary");
        break;
      case MOTION_STATE_MOTION:
        Serial.println("In Motion");
        break;
      default:
        Serial.println("Unknown");
        break;
    }
  }
}
