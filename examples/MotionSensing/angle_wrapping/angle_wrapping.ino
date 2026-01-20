/*
  Angle Wrapping Explanation

  The IMU outputs Roll, Pitch, and Yaw in the range of -180°  to  +180°
  For many applications (such as compass headings or UI display), it is more convenient to work in a 0° to 360° range.
  This line safely converts the angle without changing its meaning: wrappedYaw = fmod(rawYaw + 360.0f, 360.0f);

  How it works:
  1) Adding 360° shifts all negative angles into a positive range. Example:
       rawYaw =  -45°  →  -45 + 360 = 315°
       rawYaw =  +30°  →  +30 + 360 = 390°

  2) fmod(x, 360) keeps the result within 0°–360° by removing any full rotations
     Example:
       315 % 360 = 315°
       390 % 360 = 30°

  Final result: -180° … +180°  →  0° … 360°

  Important:
  - This does NOT rotate or flip the angle.
  - North, South, East, and West remain in the same physical direction
  - It simply expresses the same orientation using a different range
  - This method is preferred over adding 180°, which would shift the reference direction, causing 180° heading errors
*/

#include <CodeCell.h>

CodeCell myCodeCell;

// Raw angles from IMU (typically -180° to +180°)
float rollRaw = 0.0;
float pitchRaw = 0.0;
float yawRaw = 0.0;

// Wrapped angles (0° to 360°)
float roll360 = 0.0;
float pitch360 = 0.0;
float yaw360 = 0.0;

void setup() {
  Serial.begin(115200);
  myCodeCell.Init(MOTION_ROTATION);  // Enable rotation sensing with magnetometer enabled
}

void loop() {
  if (myCodeCell.Run(10)) {  // Run at 10 Hz (every 100 ms)

    // Read raw IMU angles
    myCodeCell.Motion_RotationRead(rollRaw, pitchRaw, yawRaw);

    // Converts [-180°, +180°] → [0°, 360°] without changing orientation meaning
    roll360 = fmod(rollRaw + 360.0f, 360.0f);
    pitch360 = fmod(pitchRaw + 360.0f, 360.0f);
    yaw360 = fmod(yawRaw + 360.0f, 360.0f);

 
    Serial.println("Rotation Data: ");

    Serial.printf("RAW   | Roll: %7.2f°, Pitch: %7.2f°, Yaw: %7.2f°\n", rollRaw, pitchRaw, yawRaw);

    Serial.printf("WRAP  | Roll: %7.2f°, Pitch: %7.2f°, Yaw: %7.2f°\n", roll360, pitch360, yaw360);

    Serial.println();
  }
}
