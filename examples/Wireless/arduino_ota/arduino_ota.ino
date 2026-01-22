/*
  Example: ArduinoOTA Wireless Upload (Over-The-Air)
  Boards: CodeCell C3 / CodeCell C3 Light / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Connects CodeCell to your Wi-Fi network 
  - Starts ArduinoOTA so you can upload new sketches wirelessly (no USB cable needed after first flash)
  - Prints the board’s IP address in the Serial Monitor for quick confirmation

  Notes:
  - Works on CodeCell C3 (4MB), but CodeCell C6 (8MB) is recommended because ArduinoOTA increases sketch size; 
    typical minimum program size is around ~1.1MB, so extra flash headroom helps (especially as your project grows)

  How to Use:
  1) Fill in ssid/password below with your Wi-Fi credentials
  2) Upload this sketch over USB (first flash)
  3) Open Serial Monitor @ 115200 baud and confirm the board connects to Wi-Fi (IP prints)
  4) For future uploads (wireless):
     - In Arduino IDE go to Tools → Port
     - Select the Network Port (the one showing your CodeCell / IP)
     - Click Upload to flash the new sketch over Wi-Fi.
*/


#include <CodeCell.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>

CodeCell myCodeCell;

const char *ssid = "GOINTERNET-2234A0";//"..........";
const char *password = "fzaSd5faE4";//"..........";
uint32_t last_ota_time = 0;

void setup() {
  Serial.begin(115200);
  myCodeCell.Init(LIGHT); 

  Serial.println(">> ArduinoOTA Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println(">> ArduinoOTA Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println(">> ArduinoOTA Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      if (millis() - last_ota_time > 500) {
        Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
        last_ota_time = millis();
      }
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });

  ArduinoOTA.begin();

  Serial.print(">> ArduinoOTA Connected ");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  ArduinoOTA.handle();
  if (myCodeCell.Run(10)) {  // Run loop at 10 Hz
  }
}