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

// Variables for bike metrics
float power = 0.0;
float cadence = 0.0;
float lastValidCadence = 0.0;  // Store last valid cadence
float speed = 0.0;
int resistance = 50;
bool deviceConnected = false;

// Timing constants for beginner-intermediate
const unsigned long MAX_SPEED_HOLD_TIME = 1000;    // Time to trigger max speed when magnet held
const unsigned long MIN_HALF_REV_TIME = 250;       // 120 RPM max
const unsigned long MAX_HALF_REV_TIME = 1000;      // 30 RPM min
const float MAX_CADENCE = 120.0;                   // Maximum cadence for beginner-intermediate (RPM)
const float MIN_CADENCE = 30.0;                    // Minimum cadence (RPM)

// Validation constants
const unsigned long ZERO_TIMEOUT = 3000;           // Time before resetting to zero (increased from 2000)
const unsigned long MIN_VALID_TIME = 100;          // Minimum time between valid readings
const float CADENCE_CHANGE_THRESHOLD = 30.0;       // Maximum allowed sudden change in cadence

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
    
    int percentage = map(rawResistance, RESISTANCE_MIN, RESISTANCE_MAX, 0, 100);
    return constrain(percentage, 0, 100);
}

float calculatePower(float cadence, int resistancePercent) {
    float baseResistance = map(resistancePercent, 0, 100, 5, 40);
    float angularVelocity = cadence * 0.104720;
    return baseResistance * angularVelocity;
}

bool isValidCadenceChange(float newCadence, float oldCadence) {
    // Check if the change in cadence is within reasonable limits
    return abs(newCadence - oldCadence) <= CADENCE_CHANGE_THRESHOLD;
}

void updateMetrics() {
    static unsigned long lastTriggerTime = 0;
    static unsigned long lastValidTime = 0;
    static unsigned long magnetStartTime = 0;
    static bool wasTriggered = false;
    static const unsigned long DEBOUNCE_TIME = 20;  // 20ms debounce
    static float cadenceBuffer[5] = {0};  // Buffer for averaging
    static int bufferIndex = 0;
    
    unsigned long currentTime = millis();
    bool isMagnetTriggered = (digitalRead(MAGNET_SENSOR_PIN) == LOW);
    
    // Track when magnet first triggered
    if (isMagnetTriggered && !wasTriggered) {
        magnetStartTime = currentTime;
    }
    
    // Check for maximum speed condition (magnet held down)
    if (isMagnetTriggered && (currentTime - magnetStartTime >= MAX_SPEED_HOLD_TIME)) {
        cadence = MAX_CADENCE;
        lastValidTime = currentTime;
        lastValidCadence = cadence;
        Serial.println("Maximum speed triggered!");
    }
    // Normal speed detection
    else if (isMagnetTriggered) {
        // Check if enough time has passed since last trigger (debounce)
        if (currentTime - lastTriggerTime >= DEBOUNCE_TIME && 
            currentTime - lastTriggerTime >= MIN_VALID_TIME) {
            
            if (lastTriggerTime != 0) {
                unsigned long timeDiff = currentTime - lastTriggerTime;
                
                // Calculate cadence based on half revolution (two magnets)
                if (timeDiff >= MIN_HALF_REV_TIME && timeDiff <= MAX_HALF_REV_TIME) {
                    float calculatedCadence = (60.0f * 1000.0f) / (timeDiff * 2.0f);
                    
                    // Validate the calculated cadence
                    if (calculatedCadence >= MIN_CADENCE && 
                        calculatedCadence <= MAX_CADENCE && 
                        isValidCadenceChange(calculatedCadence, lastValidCadence)) {
                        
                        // Update buffer for moving average
                        cadenceBuffer[bufferIndex] = calculatedCadence;
                        bufferIndex = (bufferIndex + 1) % 5;
                        
                        // Calculate average cadence
                        float sum = 0;
                        for (int i = 0; i < 5; i++) {
                            sum += cadenceBuffer[i];
                        }
                        float averageCadence = sum / 5;
                        
                        // Apply additional smoothing
                        cadence = cadence * 0.7 + averageCadence * 0.3;
                        
                        // Update validation variables
                        lastValidTime = currentTime;
                        lastValidCadence = cadence;
                        
                        Serial.printf("New cadence: %.1f RPM (raw: %.1f)\n", 
                                    cadence, calculatedCadence);
                    }
                }
            }
            lastTriggerTime = currentTime;
        }
    }
    
    wasTriggered = isMagnetTriggered;

    // More gradual cadence reduction instead of immediate zero
    if (currentTime - lastValidTime > ZERO_TIMEOUT) {
        if (cadence > 0) {
            // Gradually reduce cadence
            cadence = max(0.0, cadence * 0.95);
            if (cadence < 1.0) {
                cadence = 0;
                Serial.println("Cadence reset to 0 (timeout)");
            }
        }
    }

    // Update resistance and calculate power
    resistance = getSmoothedResistance();
    power = calculatePower(cadence, resistance);

    // Calculate speed based on cadence
    speed = cadence * 0.25;
}

void sendBLEData() {
    if (deviceConnected) {
        uint8_t bikeData[8];
        
        uint16_t flags = 0x44;  // Speed and Power present
        bikeData[0] = flags & 0xFF;
        bikeData[1] = (flags >> 8) & 0xFF;
        
        uint16_t speedValue = speed * 100;  // 0.01 resolution
        bikeData[2] = speedValue & 0xFF;
        bikeData[3] = (speedValue >> 8) & 0xFF;
        
        uint16_t cadenceValue = cadence * 2;  // 0.5 resolution
        bikeData[4] = cadenceValue & 0xFF;
        bikeData[5] = (cadenceValue >> 8) & 0xFF;
        
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

    Serial.println("Starting LibrePulse Bike...");
    Serial.printf("Magnet sensor pin: %d\n", MAGNET_SENSOR_PIN);
    Serial.printf("Resistance pin: %d\n", RESISTANCE_PIN);

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