/*
 * Overview:
 * This code demonstrates the CodeCell's Light Sensor.
 * In this example, the CodeCell continuously measures the ambient light and updates the brightness of the LED according to how dark it gets.
 * The LED will get dimm in the dark and gets bright when its in sunlight 
 * Feel free to tweak the code with your own creative ideas! 
 */

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200); /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */

  myCodeCell.Init(LIGHT); /*Initializes Light Sensing*/
}

void loop() {
  delay(100);
  uint16_t brightness = (myCodeCell.Light_WhiteRead())>>3; 
  Serial.println(brightness);
  if(brightness == 0U){
    brightness = 1U;
  }
  else if(brightness> 254U){
    brightness = 254U;
  }
  else{
    /*Skip*/
  }
  brightness = 255U - brightness;
  myCodeCell.LED(0,0,brightness);/*Shine all three RGB colors*/
}
