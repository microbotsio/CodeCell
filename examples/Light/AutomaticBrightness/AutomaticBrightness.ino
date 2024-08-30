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
  int brightness = (myCodeCell.Light_AmbientRead()>>8); //Divide by 256 to limit the value to 8-bit
  if(brightness == 0){
    brightness = 1;
  }
  myCodeCell.LED(brightness,brightness,brightness);/*Shine all three RGB colors*/
}
