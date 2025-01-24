/*
 * Overview:
 * This code demonstrates how to configure the CodeCell's ESP32-C3 to be used as a WiFi remote
 * In this example, the CodeCell (Device 1) reads the proximity sensor and sends it to Device 2
 * Feel free to tweak the code with your own creative ideas!
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-wifi-remote
 */

#include <esp_now.h>
#include <WiFi.h>
#include <CodeCell.h>

CodeCell myCodeCell;
uint8_t receiverMAC[] = { 0x18, 0x8B, 0x0E, 0x06, 0xB5, 0xCC };  // Replace with your receiver's MAC address

void setup() {
  Serial.begin(115200);

  myCodeCell.Init(LIGHT);  // Initializes Light Sensing

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
  if (myCodeCell.Run(10)) { /*Run every 100ms (10Hz)*/
    uint16_t ProxRead = (myCodeCell.Light_ProximityRead()) >> 4;  //Get Proximity Value and Divide by 16
    Serial.println(ProxRead);
    esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&ProxRead, sizeof(ProxRead));

    if (result == ESP_OK) {
      Serial.println("Data sent successfully");
    } else {
      Serial.println("Sending Error");
    }
  }
}
