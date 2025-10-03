/*
  Example: BLE Sensor Broadcast Demo
  Boards: CodeCell C3 / CodeCell C3 Light / CodeCell C6 / CodeCell C6 Drive

  Overview:
  - Initializes CodeCell’s light sensor.
  - Creates a BLE service with a characteristic that clients can READ and get NOTIFY updates from.
  - Advertise the proximity reading to a connected BLE client in real time 

  BLE Details:
  - Service UUID: 12345678-1234-5678-1234-56789abcdef0
  - Characteristic UUID (SENSOR_UUID): abcd5678-abcd-5678-abcd-56789abcdef0
    * PROPERTY_READ   → Client can manually read the current value.
    * PROPERTY_NOTIFY → Device pushes updates when new data is available.

  How It Works:
  - CodeCell.Run(1) drives a 1Hz loop.
  - Light_ProximityRead() reads the onboard proximity sensor.
  - The value is converted to a string and sent via BLE notify.

  Testing with Mobile Apps:
  - Install a BLE scanner app on your phone like the nRF Connect (iOS / Android, by Nordic Semiconductor)
  - After connecting, look for the advertised service UUID and open the characteristic.
  - You’ll see the proximity values updating in real-time.

  Notes:
  - The BLE device advertises under the name "CodeCell_BLE".
  - If the client disconnects, advertising restarts automatically.
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <CodeCell.h>
CodeCell myCodeCell;

BLECharacteristic* pSensorCharacteristic = nullptr;

#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define SENSOR_UUID  "abcd5678-abcd-5678-abcd-56789abcdef0"

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    Serial.println("BLE Connected");
    delay(1000); // Small pause for stability after connect
  }

  void onDisconnect(BLEServer* pServer) override {
    Serial.println("BLE Disconnected");
    delay(500);
    BLEDevice::startAdvertising(); // Restart advertising so clients can reconnect
  }
};

void setup() {
  Serial.begin(115200);                  // Start USB serial at 115200 baud
  myCodeCell.Init(LIGHT);                // Enable light + proximity sensing

  BLEDevice::init("CodeCell_BLE");       // Set the BLE device name
  BLEServer* bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new MyServerCallbacks());

  BLEService* bleService = bleServer->createService(BLEUUID(SERVICE_UUID));

  // Create BLE characteristic for proximity data (readable + notifiable)
  pSensorCharacteristic = bleService->createCharacteristic(
    SENSOR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pSensorCharacteristic->addDescriptor(new BLE2902()); // Enables notifications on most clients

  bleService->start();                   // Start the service

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID); // Advertise the service UUID
  BLEDevice::startAdvertising();         // Begin advertising
}

void loop() {
  if (myCodeCell.Run(1)) {              // Run at 1Hz 
    uint16_t proximity = myCodeCell.Light_ProximityRead();
    Serial.print("Proximity: ");
    Serial.println(proximity);

    // Send proximity value as a UTF-8 string
    String proximityStr = String(proximity);
    pSensorCharacteristic->setValue(proximityStr.c_str());
    pSensorCharacteristic->notify();     // Push update to connected client
  }
}
