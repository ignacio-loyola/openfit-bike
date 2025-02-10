#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Basic configuration
#define DEVICE_NAME "LibrePulse Bike"

// Pin definitions
const int MAGNET_SENSOR_PIN = 12;
const int RESISTANCE_PIN = 34;

// Bike metrics
float cadence = 0.0;
float speed = 0.0;
float power = 0.0;
int resistance = 0;

// Connected status
bool deviceConnected = false;

// BLE objects
BLEServer* pServer = NULL;
BLECharacteristic* pBikeDataCharacteristic = NULL;

class ServerCallbacks: public BLEServerCallbacks {
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

void updateCadence() {
    static unsigned long lastTriggerTime = 0;
    static unsigned long lastValidTime = 0;
    static bool lastMagnetState = HIGH;
    static const unsigned long TIMEOUT = 3000;  // Increased timeout to 3 seconds
    static float lastValidCadence = 0;
    static const unsigned long MIN_TRIGGER_TIME = 100;  // Minimum time between triggers (ms)
    
    unsigned long currentTime = millis();
    bool currentMagnetState = digitalRead(MAGNET_SENSOR_PIN);
    
    // Detect magnet trigger (falling edge)
    if (currentMagnetState == LOW && lastMagnetState == HIGH) {
        unsigned long timeSinceLastTrigger = currentTime - lastTriggerTime;
        
        // Only process if enough time has passed (debounce)
        if (timeSinceLastTrigger >= MIN_TRIGGER_TIME) {
            if (lastTriggerTime > 0) {
                // Calculate time for half revolution (since we have 2 magnets)
                // Convert to RPM: (60 seconds * 1000ms) / (halfRevTime * 2 for full revolution)
                float newCadence = (30000.0f) / timeSinceLastTrigger;
                
                // More tolerant range check (5-130 RPM)
                if (newCadence >= 5.0 && newCadence <= 130.0) {
                    // If we had a previous valid cadence, use stronger smoothing
                    if (cadence > 0) {
                        // More weight to current cadence to prevent drops
                        cadence = (cadence * 0.6) + (newCadence * 0.4);
                    } else {
                        // If starting from zero, trust the new reading more
                        cadence = newCadence;
                    }
                    lastValidTime = currentTime;
                    lastValidCadence = cadence;
                }
            }
            lastTriggerTime = currentTime;
        }
    }
    
    // Gradual decay instead of immediate zero
    if ((currentTime - lastValidTime) > TIMEOUT) {
        if (cadence > 0) {
            // Gradually reduce cadence
            cadence = cadence * 0.95;  // Reduce by 5% each update
            if (cadence < 5.0) {  // Only zero out if really slow
                cadence = 0;
            }
        }
    }
    
    lastMagnetState = currentMagnetState;
}

void updateResistance() {
    // Read and smooth resistance value
    const int numReadings = 5;
    int total = 0;
    
    for (int i = 0; i < numReadings; i++) {
        total += analogRead(RESISTANCE_PIN);
        delay(1);
    }
    
    int rawValue = total / numReadings;
    
    // Map to 0-100% range
    resistance = map(rawValue, 1863, 2267, 0, 100);
    resistance = constrain(resistance, 0, 100);
}

void updateMetrics() {
    // Update cadence first
    updateCadence();
    
    // Update resistance
    updateResistance();
    
    // Calculate speed (simple conversion)
    speed = cadence * 0.25;  // Example conversion factor
    
    // Calculate power based on cadence and resistance
    float basePower = map(resistance, 0, 100, 5, 40);  // 5-40 base power range
    power = (basePower * cadence) / 30.0;  // Simple power calculation
}

void sendBLEData() {
    if (!deviceConnected) return;
    
    uint8_t data[8];
    
    // Flags
    uint16_t flags = 0x44;  // Speed and power present
    data[0] = flags & 0xFF;
    data[1] = (flags >> 8) & 0xFF;
    
    // Speed (0.01 resolution)
    uint16_t speedValue = speed * 100;
    data[2] = speedValue & 0xFF;
    data[3] = (speedValue >> 8) & 0xFF;
    
    // Cadence (0.5 resolution)
    uint16_t cadenceValue = cadence * 2;
    data[4] = cadenceValue & 0xFF;
    data[5] = (cadenceValue >> 8) & 0xFF;
    
    // Power (watts)
    uint16_t powerValue = (uint16_t)power;
    data[6] = powerValue & 0xFF;
    data[7] = (powerValue >> 8) & 0xFF;
    
    pBikeDataCharacteristic->setValue(data, 8);
    pBikeDataCharacteristic->notify();
}

void setup() {
    Serial.begin(115200);
    
    // Configure pins
    pinMode(MAGNET_SENSOR_PIN, INPUT_PULLUP);
    pinMode(RESISTANCE_PIN, INPUT);
    
    // Initialize BLE
    BLEDevice::init(DEVICE_NAME);
    
    // Create server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    // Create service (Fitness Machine Service)
    BLEService *pService = pServer->createService(BLEUUID((uint16_t)0x1826));
    
    // Create characteristic (Indoor Bike Data)
    pBikeDataCharacteristic = pService->createCharacteristic(
        BLEUUID((uint16_t)0x2AD2),
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pBikeDataCharacteristic->addDescriptor(new BLE2902());
    
    // Start service and advertising
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID((uint16_t)0x1826);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
    
    Serial.println("Bike Ready!");
}

void loop() {
    // Update all measurements
    updateMetrics();
    
    // Send BLE update
    sendBLEData();
    
    // Print debug info
    Serial.printf("Cadence: %.1f RPM, Speed: %.1f km/h, Power: %.1f W, Resistance: %d%%\n",
                  cadence, speed, power, resistance);
    
    // Small delay to prevent overwhelming the serial output
    delay(100);
}