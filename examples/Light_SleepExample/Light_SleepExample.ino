/*
 * Overview:
 * This code illustrates a CodeCell that wakes-up upon detecting proximity and enters a low-power sleep mode when no proximity is present.
 * During sleep, the CodeCell consumes approximately 750uA and briefly wakes up every second for 100ms to check proximity, consuming around 15mA during this short time.
 * You can modify the code for your specific application needs, or configure it to wake up using a different sensor.
 *
 * Learn more about this example here - https://microbots.io/blogs/codecell/codecell-going-to-sleep
 *
 */

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200); 

  delay(60);//Waking up from Sleep - add a small delay for Serial
  if (myCodeCell.WakeUpCheck()) {
    // Initialize light sensor
    while (myCodeCell.Light_Init() == 1) {
      delay(1);
      myCodeCell.LightReset(); //If sensor not responding, reset it
    }
    delay(40);
    myCodeCell.Light_Read();//Read value from light sensor
    if (myCodeCell.Light_ProximityRead() < 10) {
      myCodeCell.Sleep(1); //If Proxity still not detected go back to sleep & check again after 1 sec
    }
  }

  myCodeCell.Init(LIGHT); //Time to wake up - Initializes all CodeCell peripherals
}

void loop() {
  if (myCodeCell.Run(10)) {  //Run every 10Hz
    if (myCodeCell.Light_ProximityRead() < 10) {
      myCodeCell.Sleep(1); /*If Proxity not detected go to sleep & check again after 1 sec*/
    }
  }
}
