/*
 * Overview:
 * This code demonstrates the CodeCell's Tap Detection feature.
 * In this example, the CodeCell continuously searches for a tap. When one is detected, it shines the LED yellow for 1 second.
 * Feel free to tweak the code with your own creative ideas! 
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-tap-detection
 */

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
    Serial.begin(115200); // Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial
    myCodeCell.Init(MOTION_TAP_DETECTOR); // Initializes tap detection sensing
}

void loop() {
    if (myCodeCell.Run(10)) {  //Run every 10Hz
        if (myCodeCell.Motion_TapRead()) {
            // If a tap is detected, shine the LED yellow for 1 second
            Serial.println("Tap Detected");
            myCodeCell.LED(0xA0, 0x60, 0x00); // Set LED to yellow
            delay(1000); // Keep the LED on for 1 second
        }
    }
}
