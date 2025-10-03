/*
  Example: ESP-NOW Proximity LED Remote Demo
  Boards: CodeCell C3 / CodeCell C3 Light / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Uses the onboard proximity sensor to broadcast readings over ESP-NOW (Transmitter).
  - A second CodeCell receives the value and maps it to the onboard LED brightness (Receiver).
  - Single-file demo with two roles; compile only one.
  
  Roles:
  - Transmitter (ACTIVE): reads proximity and sends a uint16_t payload.
  - Receiver (COMMENTED): listens for packets and updates LED in onDataRecv().
  
  Setup:
  - Replace RECEIVER_MAC with the MAC address printed by your receiver board.

*/

/* ===== TRANSMITTER ===== */

#include <esp_now.h>
#include <WiFi.h>
#include <CodeCell.h>

CodeCell myCodeCell;

// IMPORTANT: replace with the MAC address printed by your receiver board
uint8_t RECEIVER_MAC[] = { 0x18, 0x8B, 0x0E, 0x06, 0xB5, 0xCC };

void setup() {
  Serial.begin(115200);                       // Start USB serial at 115200 baud
  myCodeCell.Init(LIGHT);                     // Enable light + proximity sensing
  WiFi.mode(WIFI_STA);                        // ESP-NOW requires Station mode
  Serial.print("TX MAC: "); Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {             // Initialize ESP-NOW stack
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_peer_info_t peer{};                 // Configure receiver peer
  memcpy(peer.peer_addr, RECEIVER_MAC, 6);
  peer.channel  = 0;                          // Use current STA channel
  peer.encrypt  = false;                      // No encryption for simple demo
  if (esp_now_add_peer(&peer) != ESP_OK) {    // Add peer to peer list
    Serial.println("Add peer failed");
    return;
  }
}

void loop() {
  if (myCodeCell.Run(10)) {                   // Run at 10 Hz
    uint16_t prox = myCodeCell.Light_ProximityRead() >> 4;  // Map 4095 → 255
    if (esp_now_send(RECEIVER_MAC, (uint8_t*)&prox, sizeof(prox)) == ESP_OK) {
      Serial.print("Sent: "); Serial.println(prox);         // Log sent value
    } else {
      Serial.println("Send error");                         // Log if send failed
    }
  }
}

/* ===== RECEIVER ===== */
// #include <esp_now.h>
// #include <WiFi.h>
// #include <CodeCell.h>

// CodeCell myCodeCell;

// void onDataRecv(const esp_now_recv_info*, const uint8_t* data, int len) {
//   if (len < (int)sizeof(uint16_t)) return;    // Ignore malformed packets
//   uint16_t v; memcpy(&v, data, sizeof(v));    // Extract 16-bit payload
//   myCodeCell.LED(0, v, 0);                    // Map value to green LED 0..255
//   Serial.print("Recv: "); Serial.println(v);  // Log received value
// }

// void setup() {
//   Serial.begin(115200);                       // Start USB serial at 115200 baud
//   myCodeCell.Init(LIGHT);                     // Optional for RX; keeps API consistent
//   WiFi.mode(WIFI_STA);                        // ESP-NOW requires Station mode
//   Serial.print("RX MAC: "); Serial.println(WiFi.macAddress());

//   if (esp_now_init() != ESP_OK) {             // Initialize ESP-NOW stack
//     Serial.println("ESP-NOW init failed");
//     return;
//   }
//   esp_now_register_recv_cb(onDataRecv);       // Handle packets in callback
// }

// void loop() {}                                 // All work done in the callback

