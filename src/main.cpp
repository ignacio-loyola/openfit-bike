#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define DEVICE_NAME "LibrePulse Bike"
#define FITNESSMACHINE_SERVICE_UUID        "1826"
#define INDOOR_BIKE_DATA_CHARACTERISTIC    "2AD2"
#define RESISTANCE_LEVEL_CHARACTERISTIC    "2AD6"

// Simulated values matching Auuki app ranges
float power = 124.0;        // Starting power like shown in image
float cadence = 85.0;       // Starting cadence
float speed = 30.6;         // Speed shown in image
int powerLap = 160;         // Power lap value from image
int resistance = 50;        // Middle resistance value
bool deviceConnected = false;

// Simulation control
unsigned long lastUpdate = 0;
bool isHighIntensity = false;
unsigned long intervalStart = 0;
const unsigned long INTERVAL_DURATION = 60000; // 1-minute intervals

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
    unsigned long currentTime = millis();
    
    // Update interval state every INTERVAL_DURATION
    if (currentTime - intervalStart >= INTERVAL_DURATION) {
        intervalStart = currentTime;
        isHighIntensity = !isHighIntensity;
    }

    // Simulate interval training pattern
    if (isHighIntensity) {
        // High intensity phase
        power = 160.0 + random(-10, 11);
        cadence = 85.0 + random(-5, 6);
        speed = 35.0 + random(-2, 3);
        resistance = 70 + random(-5, 6);
    } else {
        // Recovery phase
        power = 124.0 + random(-10, 11);
        cadence = 60.0 + random(-5, 6);
        speed = 30.6 + random(-2, 3);
        resistance = 50 + random(-5, 6);
    }

    // Ensure values stay in realistic ranges
    power = constrain(power, 50, 300);
    cadence = constrain(cadence, 0, 120);
    speed = constrain(speed, 15, 45);
    resistance = constrain(resistance, 0, 100);
}

void sendBLEData() {
    if (deviceConnected) {
        // Create Indoor Bike Data packet according to FTMS specification
        uint8_t bikeData[12]; // Increased size to accommodate more fields
        
        // Flags (first 2 bytes) - Indicate which fields are present
        // Bit 0: More Data
        // Bit 1: Average Speed Present
        // Bit 2: Instantaneous Cadence Present
        // Bit 3: Average Cadence Present
        // Bit 4: Total Distance Present
        // Bit 5: Resistance Level Present
        // Bit 6: Instantaneous Power Present
        // Bit 7: Average Power Present
        // Bit 8: Expended Energy Present
        // Bit 9: Heart Rate Present
        // Bit 10: Metabolic Equivalent Present
        // Bit 11: Elapsed Time Present
        // Bit 12: Remaining Time Present
        uint16_t flags = 0x44; // Instantaneous Speed (bit 2) and Power (bit 6) present
        bikeData[0] = flags & 0xFF;
        bikeData[1] = (flags >> 8) & 0xFF;
        
        // Instantaneous Speed (km/h with resolution of 0.01)
        uint16_t speedValue = speed * 100;
        bikeData[2] = speedValue & 0xFF;
        bikeData[3] = (speedValue >> 8) & 0xFF;
        
        // Instantaneous Cadence (rpm with resolution of 0.5)
        uint16_t cadenceValue = cadence * 2;
        bikeData[4] = cadenceValue & 0xFF;
        bikeData[5] = (cadenceValue >> 8) & 0xFF;
        
        // Instantaneous Power (watts with resolution of 1)
        uint16_t powerValue = (uint16_t)power;
        bikeData[6] = powerValue & 0xFF;
        bikeData[7] = (powerValue >> 8) & 0xFF;
        
        pIndoorBikeDataCharacteristic->setValue(bikeData, 8);
        pIndoorBikeDataCharacteristic->notify();
    }
}

void setup() {
    Serial.begin(115200);
    randomSeed(analogRead(0));
    intervalStart = millis();

    // Create the BLE Device
    BLEDevice::init(DEVICE_NAME);
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
    
    Serial.printf("Power: %.1fW, Cadence: %.1frpm, Speed: %.1fkm/h, Resistance: %d%%\n",
                  power, cadence, speed, resistance);
    
    delay(100);  // 10Hz update rate
}