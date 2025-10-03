/*
  Example: ESPNOW Tilt Remote Demo
  Boards: CodeCell C3 / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Transmit the CodeCell’s Roll angle (−180..+180°) and translate it to a motor speed value.
  - Sends the value wirelessly via ESP-NOW from a Transmitter to a Receiver.
  - The Receiver robot has two motors; the Transmitter controls both their speed and direction.

  Roles:
  - Transmitter (TX): measures Roll and broadcasts a 0–100 command.
  - Receiver (RX): receives the command and drives both motors forward or reverse.

  How to Use:
  - Upload the TX block to your sending device (default active below).
  - Replace RECEIVER_MAC with your receiver’s MAC address.
  - To run as a Receiver, comment out the TX block and uncomment the RX block.
  - The RX logic is simple: values above 50 drive forward, below 50 drive in reverse.
*/


/* ===== TRANSMITTER ===== */
#include <esp_now.h>
#include <WiFi.h>
#include <CodeCell.h>

CodeCell myCodeCell;

// IMPORTANT: replace with the MAC address printed by your receiver board
uint8_t RECEIVER_MAC[] = { 0x84, 0xFC, 0xE6, 0xFC, 0xEC, 0xA0 };

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
  if (myCodeCell.Run(10)) {  //Run every 10Hz
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


/* ===== RECEIVER ===== */
// #include <esp_now.h>
// #include <WiFi.h>
// #include <CodeCell.h>
// #include <DriveCell.h>

// #define IN1_pin1 2
// #define IN1_pin2 3
// #define IN2_pin1 5
// #define IN2_pin2 6

// CodeCell myCodeCell;
// DriveCell Motor1(IN1_pin1, IN1_pin2);
// DriveCell Motor2(IN2_pin1, IN2_pin2);

// void setup() {
//   Serial.begin(115200);
//   myCodeCell.Init(LIGHT);  // Initializes Light Sensing
//   Motor1.Init();
//   Motor2.Init();

//   // Initialize WiFi in Station mode
//   WiFi.mode(WIFI_STA);
//   Serial.println(WiFi.macAddress());

//   // Initialize ESP-NOW
//   if (esp_now_init() != ESP_OK) {
//     Serial.println("Error initializing ESP-NOW");
//     return;
//   }

//   // Register the receive callback
//   esp_now_register_recv_cb(onDataRecv);
// }

// // Receive callback function
// void onDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
//   int Roll_Speed = 0;
//   memcpy(&Roll_Speed, incomingData, sizeof(Roll_Speed));

//   if (Roll_Speed > 50) {
//     Motor1.Drive(1, Roll_Speed);
//     Motor2.Drive(1, Roll_Speed);
//   } else {
//     Roll_Speed = 100 - Roll_Speed;  //Add Motor Dead Zone
//     Motor1.Drive(0, Roll_Speed);
//     Motor2.Drive(0, Roll_Speed);
//   }
//   Serial.println(Roll_Speed);
// }

// void loop() {
//   if (myCodeCell.Run(10)) { //Run every 10Hz
//   }
// }
