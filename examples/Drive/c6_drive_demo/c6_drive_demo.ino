/*
  Example: Proximity-Controlled Motors
  Boards: CodeCell C6 Drive

  Overview:
  - Demonstrates how to use the onboard proximity sensor to control two motors.
  - As your hand or an object gets closer, the motors spin faster.

  Notes:
  - PROXIMITY_RANGE sets the maximum proximity value used for scaling.
  - MOTOR_DEADZONE ensures motors don’t run at very low readings.
  - MOTOR_MAXSPEED limits how fast the motors can go (as a % duty cycle).

  About CodeCell C6 Drive:
  - A tiny all-in-one robotics module. It includes:
      * ESP32-C6 microcontroller with Wi-Fi 6, BLE 5, and Zigbee.
      * Built-in battery charging and USB-C programming.
      * Light/Proximity sensor (VCNL4040) + 9-axis motion sensor (BNO085).
      * Dual H-bridge motor drivers (Drive1 & Drive2) to directly control DC motors,
        actuators, or buzzers (no extra driver board needed).

  Functions:
  * Init()
      Initializes the driver and configures pins/timers.

  * Drive(direction, power_percent)
      Runs the motor with PWM in a direction (true = forward, false = reverse) 
      and a power level from 0–100 (% duty cycle).

  * Tone()
      Generates a built-in-tone by sweeping the drive signal.

  * Pulse(direction, ms_duration)
      Briefly drives in the chosen direction for the specified duration in  
      milliseconds. Useful for clicks/taps on actuators

  * Buzz(us_buzz)
      Toggles rapidly to create a buzzing sound; 

  * Toggle(power_percent)   // ESP32-only optimized PWM
      Toggles polarity at a power level (0–100%) - Handy for vibration effects.

  * Run(smooth, power_percent, flip_speed_ms)   // ESP32-only optimized
      Flips polarity on a timer for actuators (e.g., CoilPad/FlatFlap).
      - smooth = false → square wave flips
      - smooth = true  → smoother ramped/triangular-like drive
      - flip_speed_ms  → how often polarity flips (in milliseconds)
*/



#include <CodeCell.h>

CodeCell myCodeCell;

#define PROXIMITY_RANGE 1000U  // Max proximity value we consider (closeness limit)
#define MOTOR_DEADZONE 30U     // Motors won’t spin until above this %
#define MOTOR_MAXSPEED 90U     // Maximum allowed motor speed (%)

void setup() {
  Serial.begin(115200);        // Open USB serial monitor at 115200 baud
  myCodeCell.Init(LIGHT);      // Enable the onboard light + proximity sensor

  myCodeCell.Drive1.Init();    // Initialize Motor Driver 1
  myCodeCell.Drive2.Init();    // Initialize Motor Driver 2

  myCodeCell.Drive1.Tone();    // Play a short vibration on Motor 1 (feedback)
  myCodeCell.Drive2.Tone();    // Play a short vibration on Motor 2 (feedback)
}

void loop() {
  if (myCodeCell.Run(10)) {    // Run this loop 10 times per second (10 Hz)
    
    // Read proximity sensor (higher = closer)
    uint16_t proximity_level = myCodeCell.Light_ProximityRead();

    // Cap the value so it never goes above PROXIMITY_RANGE
    if (proximity_level > PROXIMITY_RANGE) {
      proximity_level = PROXIMITY_RANGE;
    }

    // Map proximity into usable motor power (0 → MOTOR_DEADZONE, max → MOTOR_MAXSPEED)
    uint8_t speed_level = map(proximity_level, 0, PROXIMITY_RANGE, MOTOR_DEADZONE, MOTOR_MAXSPEED);

    // Drive both motors forward with the mapped speed
    myCodeCell.Drive1.Drive(true, speed_level);
    myCodeCell.Drive2.Drive(true, speed_level);

    // Print values so you can see what’s happening in Serial Monitor
    Serial.print("Proximity Level: ");
    Serial.print(proximity_level);
    Serial.print("  Motors Speed: ");
    Serial.println(speed_level);
  }
}
