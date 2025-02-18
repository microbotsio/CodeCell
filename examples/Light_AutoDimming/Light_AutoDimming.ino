/*
 * Overview:
 * This code demonstrates the CodeCell's Light Sensor.
 * In this example, the CodeCell continuously measures white light and updates the brightness of the LED according to how dark it gets.
 * The LED will get dimmer as the room gets darker - Feel free to tweak the code with your own creative ideas! 
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-auto-dimming
 */

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
    Serial.begin(115200); // Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial.
    myCodeCell.Init(LIGHT); // Initializes light sensing.
}

void loop() {
    delay(100); // Small delay - You can adjust it accordingly
    
    myCodeCell.Light_Read(); //Read light sensor data (only needed when Run() not being used)
    
    // Read white light from the sensor and adjust brightness for 8-bit
    uint16_t brightness = (myCodeCell.Light_WhiteRead()) >> 3; 
    Serial.println(brightness); // Print the brightness value to the serial monitor for debugging.

    // Limit the sensor values to the LED's brightness range (1 to 254)
    if (brightness == 0U) {
        brightness = 1U; // Set a minimum brightness to avoid turning off the LED completely
    } else if (brightness > 254U) {
        brightness = 254U; // Cap the brightness to the LED's maximum level
    }

    brightness = 255U - brightness; // Invert the brightness so the LED dims as it gets brighter
    myCodeCell.LED(0, 0, brightness); // Shine the onboard blue RGB LED with the new adjusted brightness
}
