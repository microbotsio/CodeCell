#include <CodeCell.h>
CodeCell myCodeCell;

void setup() {
  Serial.begin(115200); /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */

  myCodeCell.Init(LIGHT); /*Initializes Light Sensing*/
}

void loop() {
  if (myCodeCell.Run()) {
    /*Runs  every 100ms*/
    uint16_t proximity = myCodeCell.Light_ProximityRead();
    if (proximity > 100) {
      myCodeCell.LED(0xFF, 0, 0); /*Set to Red*/
      delay(1000);
    } else {
      /*Skip*/
    }
  }
}
