#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define DEVICE_NAME "LibrePulse Bike"
#define FITNESSMACHINE_SERVICE_UUID        "1826"
#define INDOOR_BIKE_DATA_CHARACTERISTIC    "2AD2"
#define RESISTANCE_LEVEL_CHARACTERISTIC    "2AD6"

// Pin definitions
const int MAGNET_SENSOR_PIN = 12;
const int RESISTANCE_PIN = 34;
const int READINGS_COUNT = 10;

// Calibrated resistance values from testing
const int RESISTANCE_MIN = 1863;  // No contact value
const int RESISTANCE_MAX = 2267;  // Full contact value

// Simulated values matching Auuki app ranges
float power = 124.0;        // Starting power like shown in image
float cadence = 85.0;       // Starting cadence
float speed = 30.6;         // Speed shown in image
int resistance = 50;        // Middle resistance value
bool deviceConnected = false;

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

int getSmoothedResistance() {
    long sum = 0;
    for(int i = 0; i < READINGS_COUNT; i++) {
        sum += analogRead(RESISTANCE_PIN);
        delay(1);
    }
    int rawResistance = sum / READINGS_COUNT;
    
    // Map using calibrated values, resistance increases as value increases
    int percentage = map(rawResistance, RESISTANCE_MIN, RESISTANCE_MAX, 0, 100);
    return constrain(percentage, 0, 100);
}

float calculatePower(float cadence, int resistancePercent) {
    // Adjust power calculation based on calibrated resistance
    float baseResistance = map(resistancePercent, 0, 100, 5, 40);  // 5-40 Nm torque range
    float angularVelocity = cadence * 0.104720;  // Convert RPM to rad/s
    return baseResistance * angularVelocity;
}


void updateMetrics() {
    static unsigned long lastMagnetTime = 0;
    static unsigned long currentMagnetTime = 0;
    static unsigned long lastValidMagnetTime = 0;
    static const unsigned long DEBOUNCE_TIME = 20;  // 20ms debounce
    
    unsigned long currentTime = millis();
    
    if (digitalRead(MAGNET_SENSOR_PIN) == LOW) {  // Magnet detected
        if (currentTime - lastMagnetTime > DEBOUNCE_TIME) {  // Basic debounce
            currentMagnetTime = currentTime;
            
            if (lastMagnetTime != 0) {
                unsigned long timeDiff = currentMagnetTime - lastMagnetTime;
                
                // Map timeDiff to beginner cadence range (40-90 RPM)
                if (timeDiff > 222 && timeDiff < 1500) {  // Between 90 and 40 RPM range  
                    cadence = map(timeDiff, 222, 1500, 90, 40);
                    lastValidMagnetTime = currentTime;
                }
            }
            lastMagnetTime = currentMagnetTime;
        }
    }

    // Reset cadence if no valid updates for 2 seconds
    if (currentTime - lastValidMagnetTime > 2000) {
        cadence = 0;
    }
    
    // Cap cadence at 90 RPM for beginner range
    cadence = min(cadence, 90.0f);

    // Get resistance and calculate power
    resistance = getSmoothedResistance();
    power = calculatePower(cadence, resistance);

    // Calculate speed based on cadence (adjusted for beginner range)
    speed = (cadence * 0.25);  // Adjusted conversion factor
}

void sendBLEData() {
    if (deviceConnected) {
        uint8_t bikeData[8];
        
        // Flags (first 2 bytes)
        uint16_t flags = 0x44;  // Speed and Power present
        bikeData[0] = flags & 0xFF;
        bikeData[1] = (flags >> 8) & 0xFF;
        
        // Speed (km/h with resolution of 0.01)
        uint16_t speedValue = speed * 100;
        bikeData[2] = speedValue & 0xFF;
        bikeData[3] = (speedValue >> 8) & 0xFF;
        
        // Instantaneous Cadence (rpm with resolution of 0.5)
        uint16_t cadenceValue = cadence * 2;
        bikeData[4] = cadenceValue & 0xFF;
        bikeData[5] = (cadenceValue >> 8) & 0xFF;
        
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
    pinMode(MAGNET_SENSOR_PIN, INPUT_PULLUP);
    pinMode(RESISTANCE_PIN, INPUT);

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

    Serial.println("BLE Indoor Bike Ready!");
}

void loop() {
    updateMetrics();
    
    if (deviceConnected) {
        sendBLEData();
    }
    
    Serial.printf("Power: %.1fW, Cadence: %.1frpm, Speed: %.1fkm/h, Resistance: %d%%\n",
                  power, cadence, speed, resistance);
    
    delay(100);  // 10Hz update rate
}