#include <CodeCell.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>#include <CodeCell.h>

CodeCell myCodeCell;

/*Configure up the OLED Displaay*/
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int read_timer = 0;

void setup() {
  Serial.begin(115200); /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */

  myCodeCell.Init(MOTION_ACTIVITY); /*Initializes Step Counter & Activity Sensing*/

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();
  delay(2000);
}

void loop() {
  if (myCodeCell.Run()) {
    if (read_timer < 10) {
      read_timer++;
    } else {
      /*Update every 1 sec*/
      read_timer = 0;
      display.clearDisplay();
      display.setCursor(32, 16);
      display.print(F("Activity: "));
      display.setCursor(32, 24);
      switch (myCodeCell.Motion_ActivityRead()) {
        case 1:
          display.print("Driving");
          break;
        case 2:
          display.print("Cycling");
          break;
        case 3:
        case 6:
          display.print("Walking");
          break;
        case 4:
          display.print("Still");
          break;
        case 5:
          display.print("Tilting");
          break;
        case 7:
          display.print("Running");
          break;
        case 8:
          display.print("Stairs");
          break;
        default:
          display.print("Reading..");
          break;
      }
      display.display();
    }
  }
}
