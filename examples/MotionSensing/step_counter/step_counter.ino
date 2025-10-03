/*
  Example: CodeCell Step Counter
  Boards: CodeCell C3 / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Uses the onboard motion sensor to measure step counts.
  - Counts are read every 100 ms (10 Hz) and displayed on an OLED screen.
  - The step count updates in real-time as you move.

  Notes:
  - The OLED screen used here is 128x32 pixels (I2C address 0x3C).
  - Make sure you have the Adafruit SSD1306 and GFX libraries installed.
  - You can easily adapt this example for other displays or output methods.
  - Feel free to tweak the code for your own creative ideas!

*/

#include <CodeCell.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

CodeCell myCodeCell;    

// Configure the OLED display
#define SCREEN_WIDTH 128   // OLED display width, in pixels
#define SCREEN_HEIGHT 32   // OLED display height, in pixels
#define OLED_RESET -1      // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  // I2C address for 128x32 OLED (0x3D for 128x64)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint16_t step_counter = 0;

void setup() {
  Serial.begin(115200);  // Start USB serial at 115200 baud

  myCodeCell.Init(MOTION_STEP_COUNTER);  // Enable step counter sensing on the motion sensor

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("Display Error");  // Print error if OLED not detected
  }

  display.clearDisplay();  
  display.setTextSize(1);  
  display.setTextColor(SSD1306_WHITE);  
  display.display();
  delay(2000);  // Show blank screen briefly on startup
}

void loop() {
  if (myCodeCell.Run(10)) {  // Run CodeCell service loop at 10 Hz (every 100 ms)
    
    step_counter = myCodeCell.Motion_StepCounterRead();  // Read the step count
    
    display.clearDisplay();
    display.setCursor(32, 16);  // Set text starting point
    display.print(F("Steps: "));
    display.print(step_counter);  // Show the step counter value
    display.display();            // Update OLED display  
  }
}
