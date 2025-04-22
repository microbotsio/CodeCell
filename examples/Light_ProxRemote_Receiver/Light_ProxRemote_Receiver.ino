/*
 * Overview:
 * This code demonstrates how to configure the CodeCell's ESP32-C3 to be used as a WiFi remote.
 * In this example, the CodeCell (Device 2) will receive proximity data from Device 1 and adjusts the LED based on the received data. 
 * Feel free to tweak the code with your own creative ideas!
 * Learn more about this example here - https://microbots.io/blogs/codecell/codecell-wifi-remote
 */

#include <esp_now.h>
#include <WiFi.h>
#include <CodeCell.h>

CodeCell myCodeCell;

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

  // Register the receive callback
  esp_now_register_recv_cb(onDataRecv);
}

// Receive callback function
void onDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  uint16_t Remote_Value;
  memcpy(&Remote_Value, incomingData, sizeof(Remote_Value));

  Serial.println(Remote_Value);
  myCodeCell.LED(0, Remote_Value, 0);
}

void loop() {
  // Nothing to do here
}
