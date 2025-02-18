/*
 * Overview:
 * In this example we'll explore how to use the CodeCell's onboard motion sensor to measure step counts.
 * These counts are read every 100ms and displayed on an OLED display.
 * Feel free to tweak the code with your own creative ideas! 
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-step-counter
 */
#include <CodeCell.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

CodeCell myCodeCell;

/*Configure up the OLED Displaay*/
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint16_t step_counter = 0;

void setup() {
  Serial.begin(115200); /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */

  myCodeCell.Init(MOTION_STEP_COUNTER); /*Initializes Step Counter Sensing*/

  /*Set up OLED display*/
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("Display Error");
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();
  delay(2000);
}

void loop() {
  if (myCodeCell.Run(10)) {  //Run every 10Hz 
    step_counter = myCodeCell.Motion_StepCounterRead(); //Read step counter
    
    display.clearDisplay();
    display.setCursor(32, 16);  // Start at top-left corner
    display.print(F("Steps: "));
    display.print(step_counter);
    display.display();  //Update Display  
  }
}
