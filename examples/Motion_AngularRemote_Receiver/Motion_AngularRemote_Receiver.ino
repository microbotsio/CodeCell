/*
 * Overview:
 * This code demonstrates how to configure the CodeCell's ESP32-C3 to be used as a WiFi remote.
 * In this example, we connect two motors with a two DriveCells to this receiver.
 * The CodeCell (Device 2) will receive angular data from Device 1 and adjusts the speed of two motors based on the received data. 
 * Feel free to tweak the code with your own creative ideas!
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-wifi-remote
 */
 
#include <esp_now.h>
#include <WiFi.h>
#include <CodeCell.h>
#include <DriveCell.h>

#define IN1_pin1 2
#define IN1_pin2 3
#define IN2_pin1 5
#define IN2_pin2 6

CodeCell myCodeCell;
DriveCell Motor1(IN1_pin1, IN1_pin2);
DriveCell Motor2(IN2_pin1, IN2_pin2);

void setup() {
  Serial.begin(115200);
  myCodeCell.Init(LIGHT);  // Initializes Light Sensing
  Motor1.Init();
  Motor2.Init();

  // Initialize WiFi in Station mode
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the receive callback
  esp_now_register_recv_cb(onDataRecv);
}

// Receive callback function
void onDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  int Roll_Speed = 0;
  memcpy(&Roll_Speed, incomingData, sizeof(Roll_Speed));

  if (Roll_Speed > 50) {
    Motor1.Drive(1, Roll_Speed);
    Motor2.Drive(1, Roll_Speed);
  } else {
    Roll_Speed = 100 - Roll_Speed;  //Add Motor Dead Zone
    Motor1.Drive(0, Roll_Speed);
    Motor2.Drive(0, Roll_Speed);
  }
  Serial.println(Roll_Speed);
}

void loop() {
  if (myCodeCell.Run()) {}
}
