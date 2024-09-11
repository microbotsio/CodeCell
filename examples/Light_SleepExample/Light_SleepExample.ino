/*
 * Overview:
 * This code illustrates a CodeCell that wakes-up upon detecting proximity and enters a low-power sleep mode when no proximity is present.
 * During sleep, the CodeCell consumes approximately 750uA and briefly wakes up every second for 100ms to check proximity, consuming around 15mA during this short time.
 * You can modify the code for your specific application needs, or configure it to wake up using a different sensor.
 */



#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200); /*If not enabled, set Serial baud rate to 115200*/

  delay(60);
  if (myCodeCell.WakeUpCheck()) {
    /*Waking up from Sleep - Initialize sensor*/
    while (myCodeCell.Light_Init() == 1) {
      delay(1);
      myCodeCell.LightReset(); /*If sensor not set up, reset it*/
    }
    delay(40);
    if (myCodeCell.Light_ProximityRead() < 10) {
      myCodeCell.Sleep(1); /*If Proxity still not detected go to sleep & check again after 1 sec*/
    }
  }

  myCodeCell.Init(LIGHT); /*Initializes CodeCell*/
}

void loop() {
  if (myCodeCell.Run()) {
    if (myCodeCell.Light_ProximityRead() < 10) {
      myCodeCell.Sleep(1); /*If Proxity not detected go to sleep & check again after 1 sec*/
    }
  }
}
