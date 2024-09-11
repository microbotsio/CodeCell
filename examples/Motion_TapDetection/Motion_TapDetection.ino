/*
 * Overview:
 * This code demonstrates the CodeCell's Tap Detection feature.
 * In this example, the CodeCell continuously searches for a tap. 
 * When one is detected, it shines the LED yellow for 1 second.
 */

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200); /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */
  
  myCodeCell.Init(MOTION_TAP_DETECTOR); /*Initializes Tap Detection Sensing*/
}

void loop() {
  if (myCodeCell.Run()) {
    /*Runs  every 100ms*/
      if(myCodeCell.Motion_TapRead()){
        /*If Tap is detected shine the LED Yellow for 1 sec*/
        myCodeCell.LED(0XA0, 0x60, 0x00U);
        delay(1000);
      }
  }
}
