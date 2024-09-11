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

  myCodeCell.Init(MOTION_STEP_COUNTER); /*Initializes Step Counter & Activity Sensing*/

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
  if (myCodeCell.Run()) {
    myCodeCell.Motion_StepCounterRead(step_counter);

    display.clearDisplay();
    display.setCursor(32, 16);  // Start at top-left corner
    display.print(F("Steps: "));
    display.print(step_counter);
    display.display();    
  }
}