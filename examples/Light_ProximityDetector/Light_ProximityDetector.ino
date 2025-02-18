/*
 * Overview:
 * This code demonstrates the CodeCell's prximity sensing.
 * In this example, the CodeCell continuously measures proximity and set the onboard LED to Red when proximity is detected
 * Feel free to tweak the code with your own creative ideas! 
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-proximity-sensing
 */

#include <CodeCell.h>
CodeCell myCodeCell;

void setup() {
    Serial.begin(115200); // Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial
    myCodeCell.Init(LIGHT); // Initializes light sensing
}

void loop() {
    if (myCodeCell.Run(10)) {  //Run every 10Hz
        uint16_t proximity = myCodeCell.Light_ProximityRead();
        
        // Check if an object is within range
        if (proximity > 100) {
            myCodeCell.LED(0xFF, 0, 0); // Set LED to Red when proximity is detected
            delay(1000); // Keep the LED on for 1 second
        } else {
            // No action if the object is out of range
        }
    }
}
