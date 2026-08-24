/*
  Example: CodeCell GPIO Wakeup Demo
  Boards: CodeCell C3 / CodeCell C3 Light / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Uses an external GPIO pin to wake the CodeCell from deep sleep.
  - The CodeCell enters sleep when the trigger pin is LOW.
  - While asleep, setting the trigger pin HIGH will wake it back up.
  - CodeCell C3, C3 Light, and C6 can use GPIO 1, 2, or 3.
  - CodeCell C6 Drive can only use GPIO 1.

  Wake Behavior:
  - If the device wakes because of the GPIO trigger, the LED will glow until pin resets
  - After waking from deep sleep, the CodeCell restarts and runs setup() again.
  - If the trigger pin is already HIGH, the CodeCell waits for it to go LOW before entering sleep.

  Important Notes:
  - To trigger a HIGH state, connect the wake-up GPIO to the 3V3 pin.
    Do not connect it to VO, as VO will exceed the ESP32 GPIO voltage rating.
  - Once this example is flashed, the Arduino IDE cannot connect to the CodeCell over USB-C while 
    it is in deep sleep. Keep the wake-up GPIO tied HIGH to keep the board awake before uploading a new sketch.
*/

#include <CodeCell.h>

CodeCell myCodeCell;

#define TRIGGER_PIN 1

void setup() {
  Serial.begin(115200);  // Start USB serial communication

  if (myCodeCell.WakeUpCheck()) { //Check if the CodeCell has just woken up from deep sleep
    Serial.println(">> GPIO Wakeup Detected!"); //SleepGPIOTrigger() will report as a GPIO wake-up event
    delay(1);  // Add 1ms delay
  }

  myCodeCell.Init(LIGHT);  // Initialize the CodeCell and enable the light sensor
}

void loop() {
  if (myCodeCell.Run(10)) { //Run the CodeCell service loop at 10 Hz (every 100 ms)
    if (!myCodeCell.pinRead(TRIGGER_PIN)) {  // Check that the trigger pin is currently LOW before sleeping

      Serial.println(">> Going to sleep. Waiting for GPIO HIGH..");

      myCodeCell.SleepGPIOTrigger(HIGH, TRIGGER_PIN);  // Enter deep sleep and wake when TRIGGER_PIN becomes HIGH

    } else {
      Serial.println(">> Waiting for pin to go LOW..");  // Wait for the pin to return LOW before entering deep sleep
    }
  }
}