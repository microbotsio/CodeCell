/*
 * Overview:
 * This code demonstrates how to configure the CodeCell's ESP32-C3 to be used as a WiFi remote
 * In this example, the CodeCell (Device 1) reads angular data from its motion sensors and sends it to Device 2
 * Feel free to tweak the code with your own creative ideas!
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-wifi-remote
 */

#include <esp_now.h>
#include <WiFi.h>
#include <CodeCell.h>

CodeCell myCodeCell;
uint8_t receiverMAC[] = {0x84, 0xFC, 0xE6, 0xFC, 0xEC, 0xA0};  // Replace with your receiver's MAC address
int Roll_Control = 0;

float Roll = 0.0;
float Pitch = 0.0;
float Yaw = 0.0;

void setup() {
  Serial.begin(115200);

  myCodeCell.Init(MOTION_ROTATION);  // Initializes Light Sensing

  // Initialize WiFi in Station mode
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  if (myCodeCell.Run(10)) {  /*Run every 100ms (10Hz)*/
    myCodeCell.Motion_RotationRead(Roll, Pitch, Yaw);

    Roll = Roll + 180;
    Roll = (Roll * 100)/180;
    if (Roll > 200) {
      Roll = 200;
    } else if (Roll < 0) {
      Roll = 0;
    } else {
      //Skip
    }
    Roll = Roll / 2;
    Roll_Control = (uint8_t)Roll;
    Serial.println(Roll_Control);

    esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&Roll_Control, sizeof(Roll_Control));

    if (result == ESP_OK) {
      Serial.println("Data sent successfully");
    } else {
      Serial.println("Sending Error");
    }
  }
}
