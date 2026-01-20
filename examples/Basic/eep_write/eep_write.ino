/*
  Example: CodeCell EEPROM (Non-Volatile Memory) Basics
  Boards: CodeCell C3 Light / CodeCell C3 / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Shows how to save a sensor value so it survives power-off and reset
  - Uses the onboard Proximity sensor (LIGHT mode)
  - Reads the last saved proximity value from EEPROM at boot
  - Every few seconds, saves the current proximity value back to EEPROM

  Notes:
  - On ESP32, EEPROM is emulated in Flash memory
  - After writing with EEPROM.put(), you MUST call EEPROM.commit() or the value will not be saved
  - Avoid saving every loop (flash has limited write cycles) - Save only when needed
  - In Arduino IDE, make sure this is set to: Tools → Erase All Flash Before Sketch Upload → DISABLED
    If enabled, all EEPROM / NVM data will be erased on every upload.
*/

#include <CodeCell.h>
#include <EEPROM.h>

CodeCell myCodeCell;

// EEPROM space size (bytes)
#define EEPROM_SIZE 16

// Address where we store our proximity value
#define EEPROM_PROX_ADDR 0

uint16_t proxSaved = 0;

void setup() {
  Serial.begin(115200);
  myCodeCell.Init(LIGHT);  // Initialize CodeCell + enable the light/proximity sensor

  // Initialize EEPROM
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("EEPROM init failed");
    while (1) delay(10);
  }

  // Read previously saved proximity value (if any)
  EEPROM.get(EEPROM_PROX_ADDR, proxSaved);

  Serial.print("Stored Proximity Value: ");
  Serial.println(proxSaved);
}

void loop() {
  if (myCodeCell.Run(2)) {  // Run at 2 Hz (every 500 ms)

    // Read current proximity value
    uint16_t proxNow = myCodeCell.Light_ProximityRead();

    Serial.print("Current Proximity: ");
    Serial.print(proxNow);

    Serial.print(" | Stored Proximity: ");
    Serial.println(proxSaved);

    static unsigned long lastSaveMs = 0;
    if (millis() - lastSaveMs >= 30000) {  //save every 30sec
      lastSaveMs = millis();

      proxSaved = proxNow;

      // Save proximity value to EEPROM
      EEPROM.put(EEPROM_PROX_ADDR, proxSaved);
      EEPROM.commit();

      Serial.print(">> Saved Proximity to EEPROM: ");
      Serial.println(proxSaved);
      Serial.println("   Power off CodeCell, to test it!\n");
    }
  }
}
