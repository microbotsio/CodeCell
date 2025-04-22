/*
 * Overview:
 * This code shows how easy it is use to use the CodeCell's GPIO pins 
 *
 * Learn more about this example here - https://microbots.io/blogs/codecell/codecell-set-up-a-gpio-in-seconds
 * - Take it a step further and learn how to read an ADC - https://microbots.io/blogs/codecell/codecell-read-analog-values-with-adc
 * - Take it a step further and learn how to create a PWM signal - https://microbots.io/blogs/codecell/codecell-set-up-a-pwm-in-seconds
 */

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200);    // Initialize serial communication at 115200 bps.
                           // Make sure Tools > USB_CDC_On_Boot is enabled to use Serial Monitor.

  myCodeCell.Init(LIGHT);  // Initialize the CodeCell with light and motion sensing enabled.
}

void loop() {
  if (myCodeCell.Run(100)) {  // Check every 100Hz
    
    // Read digital state of pin 5
    if (myCodeCell.pinRead(5)) {  // If pin 5 is HIGH (e.g., button pressed or signal detected)
      myCodeCell.pinWrite(1, HIGH);  // Set pin 1 HIGH (e.g., turn on LED or output)
    } else {
      myCodeCell.pinWrite(1, LOW);   // Set pin 1 LOW (e.g., turn off LED)
    }
  }
}
