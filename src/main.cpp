#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define DEVICE_NAME "LibrePulse Bike"
#define FITNESSMACHINE_SERVICE_UUID        "1826"
#define INDOOR_BIKE_DATA_CHARACTERISTIC    "2AD2"
#define RESISTANCE_LEVEL_CHARACTERISTIC    "2AD6"

// Simulated values
float rpm = 80.0;          // Starting at 80 RPM
int resistance = 50;       // 50% resistance
float power = 150.0;       // 150W starting power
bool deviceConnected = false;

// Direction of value change for simulation
bool rpmIncreasing = true;
bool powerIncreasing = true;

BLEServer* pServer = NULL;
BLECharacteristic* pIndoorBikeDataCharacteristic = NULL;
BLECharacteristic* pResistanceCharacteristic = NULL;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Client Connected");
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Client Disconnected");
        BLEDevice::startAdvertising();
    }
};

void updateSimulatedMetrics() {
    // Simulate realistic RPM changes (60-100 RPM range)
    if (rpmIncreasing) {
        rpm += random(1, 3) * 0.5;
        if (rpm >= 100) rpmIncreasing = false;
    } else {
        rpm -= random(1, 3) * 0.5;
        if (rpm <= 60) rpmIncreasing = true;
    }

    // Simulate resistance changes (30-70% range)
    if (random(100) < 10) { // 10% chance to change resistance
        resistance = constrain(resistance + random(-5, 6), 30, 70);
    }

    // Simulate power based on RPM and resistance (100-300W range)
    power = (rpm * resistance * 0.15) + random(-10, 11);
    power = constrain(power, 100, 300);
}

void sendBLEData() {
    if (deviceConnected) {
        uint8_t bikeData[8];
        
        // Flags (first 2 bytes)
        uint16_t flags = 0x0045;  // Speed, Cadence, Power
        bikeData[0] = flags & 0xFF;
        bikeData[1] = (flags >> 8) & 0xFF;
        
        // Speed (static bike)
        bikeData[2] = 0;
        bikeData[3] = 0;
        
        // Instantaneous Cadence (resolution 0.5)
        uint16_t cadence = rpm * 2;
        bikeData[4] = cadence & 0xFF;
        bikeData[5] = (cadence >> 8) & 0xFF;
        
        // Instantaneous Power
        uint16_t powerValue = (uint16_t)power;
        bikeData[6] = powerValue & 0xFF;
        bikeData[7] = (powerValue >> 8) & 0xFF;
        
        pIndoorBikeDataCharacteristic->setValue(bikeData, 8);
        pIndoorBikeDataCharacteristic->notify();
    }
}

void setup() {
    Serial.begin(115200);
    randomSeed(analogRead(0));  // Initialize random number generator

    // Create the BLE Device
    BLEDevice::init(DEVICE_NAME);

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(BLEUUID(FITNESSMACHINE_SERVICE_UUID));

    // Create BLE Characteristics
    pIndoorBikeDataCharacteristic = pService->createCharacteristic(
        BLEUUID(INDOOR_BIKE_DATA_CHARACTERISTIC),
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pIndoorBikeDataCharacteristic->addDescriptor(new BLE2902());

    // Start the service
    pService->start();

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(FITNESSMACHINE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    BLEDevice::startAdvertising();

    Serial.println("BLE Indoor Bike Test Mode Ready!");
}

void loop() {
    updateSimulatedMetrics();
    
    if (deviceConnected) {
        sendBLEData();
    }
    
    Serial.printf("Power: %.1fW, Cadence: %.1frpm, Resistance: %d%%\n",
                  power, rpm, resistance);
    
    delay(100);  // 10Hz update rate
}