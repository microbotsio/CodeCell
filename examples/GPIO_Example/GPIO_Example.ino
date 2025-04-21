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
