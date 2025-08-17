#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <math.h>

/* ===================== Pins & Names ===================== */
#define DEVICE_NAME "LibrePulse Bike"
static const int MAGNET_SENSOR_PIN = 27;  // avoid strapping pins like GPIO12
static const int RESISTANCE_PIN    = 34;  // ADC1 only

/* ===================== Model Params ===================== */
// Magnet on the CRANK: cadence_rpm is CRANK RPM.
// Speed is derived from cadence * (effective gear ratio) * wheel circumference.
static const uint8_t MAGNETS_PER_REV = 2;      // magnets per crank revolution
static const float   CADENCE_MIN     = 5.0f;   // rpm sanity
static const float   CADENCE_MAX     = 130.0f;
static const uint32_t DEBOUNCE_US    = 6000;   // reed/hall debounce
static const uint32_t NOTIFY_INTERVAL_MS = 100; // FTMS @10 Hz

/* ===================== Calibration & Prefs ===================== */
struct Cal {
  // 3-point capture for resistance
  int adc_free   = 2600;  // slack / free spin (knob fully loose)
  int adc_engage = 3000;  // point where resistance just starts
  int adc_high   = 2200;  // hardest you'd still pedal

  bool   invert        = true;  // derived (ADC decreases as resistance increases)
  uint8_t deadzone_pct = 5;     // info only (computed from free→engage)
  float  gamma         = 1.0f;  // response shaping (1.0 = linear)

  // Motion model
  int   wheel_circ_mm  = 2148;  // 27.5" MTB ~2148 mm
  float gear_ratio     = 2.2f;  // wheel revs per crank rev (typical mid-gear)

  // Resistance-coupled speed factor (0..1). 0 = off (classic).
  // ratio_eff = gear_ratio * (1 - speed_res_factor * r_norm)  [clamped ≥ 0.2]
  float speed_res_factor = 0.0f;

  // Torque model for power (Nm): T = Tmin + (Tmax - Tmin) * r_norm^1.0
  // power_w = T * omega (omega = 2π * cadence/60)
  float torque_min_nm = 0.5f;   // small baseline drag
  float torque_max_nm = 25.0f;  // heavy-ish brake at 100%
};

Preferences prefs;
Cal cal;

/* ===================== Runtime State ===================== */
volatile uint32_t lastPulseUs = 0;
volatile uint32_t lastValidIntervalUs = 0;

float cadence_rpm = 0.0f;  // CRANK RPM
float speed_kmh   = 0.0f;
float power_w     = 0.0f;
int   resistance_pct = 0;  // 0..100%

bool deviceConnected = false;

// BLE
BLEServer*         pServer = nullptr;
BLECharacteristic* pBikeDataCharacteristic = nullptr;

/* ===================== ISR ===================== */
void IRAM_ATTR magnetISR() {
  uint32_t now = micros();
  uint32_t dt  = now - lastPulseUs;   // unsigned handles wrap
  if (dt >= DEBOUNCE_US) {
    lastPulseUs = now;
    lastValidIntervalUs = dt;
  }
}

/* ===================== Utils ===================== */
static inline float ema(float prev, float sample, float alpha) {
  return prev + alpha * (sample - prev);
}

static inline int readADCavg(uint8_t samples = 32) {
  int sum = 0;
  for (uint8_t i = 0; i < samples; ++i) sum += analogRead(RESISTANCE_PIN);
  return sum / samples;
}

// wait for an entire line (Enter). Trim CR/LF.
String readLine() {
  String s = "";
  while (true) {
    while (!Serial.available()) { delay(5); }
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') break;
    s += c;
  }
  s.trim();
  return s;
}

static void loadPrefs() {
  prefs.begin("bike", true);
  cal.adc_free         = prefs.getInt("adc_free",   cal.adc_free);
  cal.adc_engage       = prefs.getInt("adc_engage", cal.adc_engage);
  cal.adc_high         = prefs.getInt("adc_high",   cal.adc_high);
  cal.invert           = prefs.getBool("invert",    cal.invert);
  cal.deadzone_pct     = prefs.getUChar("dz",       cal.deadzone_pct);
  cal.gamma            = prefs.getFloat("gamma",    cal.gamma);
  cal.wheel_circ_mm    = prefs.getInt("circ_mm",    cal.wheel_circ_mm);
  cal.gear_ratio       = prefs.getFloat("ratio",    cal.gear_ratio);
  cal.speed_res_factor = prefs.getFloat("srf",      cal.speed_res_factor);
  cal.torque_min_nm    = prefs.getFloat("tmin",     cal.torque_min_nm);
  cal.torque_max_nm    = prefs.getFloat("tmax",     cal.torque_max_nm);
  prefs.end();
}

static void savePrefs() {
  prefs.begin("bike", false);
  prefs.putInt("adc_free",   cal.adc_free);
  prefs.putInt("adc_engage", cal.adc_engage);
  prefs.putInt("adc_high",   cal.adc_high);
  prefs.putBool("invert",    cal.invert);
  prefs.putUChar("dz",       cal.deadzone_pct);
  prefs.putFloat("gamma",    cal.gamma);
  prefs.putInt("circ_mm",    cal.wheel_circ_mm);
  prefs.putFloat("ratio",    cal.gear_ratio);
  prefs.putFloat("srf",      cal.speed_res_factor);
  prefs.putFloat("tmin",     cal.torque_min_nm);
  prefs.putFloat("tmax",     cal.torque_max_nm);
  prefs.end();
}

static void printCalibration() {
  Serial.printf("[CAL] free=%d, engage=%d, high=%d | invert=%s | deadzone=%u%% | gamma=%.2f\n",
      cal.adc_free, cal.adc_engage, cal.adc_high,
      cal.invert ? "true":"false", cal.deadzone_pct, cal.gamma);
  Serial.printf("[CFG] wheel=%d mm | ratio=%.2f | speed_res_factor=%.2f | torque_min=%.2f Nm | torque_max=%.2f Nm\n",
      cal.wheel_circ_mm, cal.gear_ratio, cal.speed_res_factor, cal.torque_min_nm, cal.torque_max_nm);
}

/* Map raw ADC -> 0..100% (anchor 0% at ENGAGE, 100% at HIGH; auto-invert; gamma) */
static float mapCalibrated(int raw) {
  int lo = min(cal.adc_engage, cal.adc_high);
  int hi = max(cal.adc_engage, cal.adc_high);
  if (hi == lo) hi = lo + 1;

  float x = (float)(raw - lo) / (float)(hi - lo);
  if (x < 0.0f) x = 0.0f;
  if (x > 1.0f) x = 1.0f;

  // If ADC decreases as resistance increases, ENGAGE > HIGH → invert
  bool needInvert = (cal.adc_high < cal.adc_engage);
  if (needInvert) x = 1.0f - x;

  float g = cal.gamma;
  if (g < 0.3f) g = 0.3f;
  if (g > 3.0f) g = 3.0f;
  x = powf(x, g);

  float pct = x * 100.0f;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}

/* ===================== Cadence / Resistance / Speed / Power ===================== */
void updateCadence() {
  static uint32_t lastSeenPulseUs = 0;
  uint32_t nowMs = millis();

  noInterrupts();
  uint32_t intervalUs = lastValidIntervalUs;
  uint32_t pulseUs    = lastPulseUs;
  interrupts();

  if (intervalUs > 0 && pulseUs != 0 && pulseUs != lastSeenPulseUs) {
    lastSeenPulseUs = pulseUs;

    // CRANK RPM (magnet on crank; MAGNETS_PER_REV pulses per crank rev)
    float crankRevPerSec = 1e6f / (float(intervalUs) * MAGNETS_PER_REV);
    float rpm = crankRevPerSec * 60.0f;

    if (rpm >= CADENCE_MIN && rpm <= CADENCE_MAX) {
      cadence_rpm = ema(cadence_rpm, rpm, 0.35f);
    }
  }

  // Decay if no pulses for ~3s
  if ((nowMs - (lastPulseUs / 1000U)) > 3000U) {
    if (cadence_rpm > 0.0f) {
      cadence_rpm *= 0.95f;
      if (cadence_rpm < CADENCE_MIN) cadence_rpm = 0.0f;
    }
  }
}

void updateResistance() {
  static bool  adcInit = false;
  static float resEMA  = 0.0f;

  if (!adcInit) {
    analogReadResolution(12);                           // 0..4095
    analogSetPinAttenuation(RESISTANCE_PIN, ADC_11db);  // wider range
    adcInit = true;
  }
  int raw = readADCavg(16);
  float pct = mapCalibrated(raw);
  resEMA = ema(resEMA, pct, 0.2f);
  resistance_pct = (int)(resEMA + 0.5f);
}

static inline float speedPerCadence_kmh() {
  // ratio_eff = base_ratio * (1 - srf * r_norm), clamped to ≥ 0.2
  float r_norm = resistance_pct / 100.0f;
  float ratio_eff = cal.gear_ratio * (1.0f - cal.speed_res_factor * r_norm);
  if (ratio_eff < 0.2f) ratio_eff = 0.2f;

  // km/h per CRANK RPM = ratio_eff * (wheel_circ_mm * 60 / 1e6)
  return ratio_eff * (cal.wheel_circ_mm * 60.0f / 1e6f);
}

static inline float torqueFromResistance_Nm() {
  // Linear in r_norm (you can switch to exponential if you like):
  float r_norm = resistance_pct / 100.0f;
  float T = cal.torque_min_nm + (cal.torque_max_nm - cal.torque_min_nm) * r_norm;
  if (T < 0.0f) T = 0.0f;
  return T;
}

void updateMetrics() {
  updateCadence();
  updateResistance();

  speed_kmh = cadence_rpm * speedPerCadence_kmh();

  // Torque-based power: P = T * omega (omega = 2π * cadence / 60)
  const float omega = (2.0f * (float)M_PI) * (cadence_rpm / 60.0f);
  const float T = torqueFromResistance_Nm();
  power_w = T * omega;
}

/* ===================== BLE: FTMS Indoor Bike Data ===================== */
void sendBLEData() {
  static uint32_t lastNotifyMs = 0;
  uint32_t nowMs = millis();
  if (!deviceConnected) return;
  if ((nowMs - lastNotifyMs) < NOTIFY_INTERVAL_MS) return;
  lastNotifyMs = nowMs;

  // Include: Instantaneous Speed, Instantaneous Cadence, Resistance Level, Instantaneous Power
  const uint16_t flags = 0x0064; // bits 2(cadence),5(resistance),6(power); MoreData=0 => include speed

  const uint16_t speedValue    = (uint16_t)(speed_kmh   * 100.0f + 0.5f); // 0.01 km/h
  const uint16_t cadenceValue  = (uint16_t)(cadence_rpm * 2.0f   + 0.5f); // 0.5 rpm
  const int16_t  resistLevel   = (int16_t)resistance_pct;                 // SINT16
  const int16_t  powerValue    = (int16_t)(power_w + 0.5f);               // SINT16 W

  uint8_t data[10];
  data[0] = flags & 0xFF;           data[1] = (flags >> 8) & 0xFF;
  data[2] = speedValue & 0xFF;      data[3] = (speedValue >> 8) & 0xFF;
  data[4] = cadenceValue & 0xFF;    data[5] = (cadenceValue >> 8) & 0xFF;
  data[6] = resistLevel & 0xFF;     data[7] = (resistLevel >> 8) & 0xFF;
  data[8] = powerValue & 0xFF;      data[9] = (powerValue >> 8) & 0xFF;

  pBikeDataCharacteristic->setValue(data, sizeof(data));
  pBikeDataCharacteristic->notify();
}

/* ===================== BLE callbacks ===================== */
class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    deviceConnected = true;
    Serial.println("Client Connected");
  }
  void onDisconnect(BLEServer* s) override {
    deviceConnected = false;
    Serial.println("Client Disconnected");
    BLEDevice::startAdvertising();
  }
};

/* ===================== Calibration Flow ===================== */
void runCalibration() {
  Serial.println("\n=== Resistance Calibration (3-point) ===");
  Serial.println("1) FREE: knob fully loose (slack). Press ENTER to capture...");
  (void)readLine(); delay(200);
  int freeVal = readADCavg(64);
  Serial.printf("FREE  = %d\n", freeVal);

  Serial.println("2) ENGAGE: turn knob until resistance JUST STARTS. Press ENTER...");
  (void)readLine(); delay(200);
  int engageVal = readADCavg(64);
  Serial.printf("ENGAGE= %d\n", engageVal);

  Serial.println("3) HIGH: hardest you'd still pedal. Press ENTER...");
  (void)readLine(); delay(200);
  int highVal = readADCavg(64);
  Serial.printf("HIGH  = %d\n", highVal);

  cal.adc_free   = freeVal;
  cal.adc_engage = engageVal;
  cal.adc_high   = highVal;

  // Derived settings
  cal.invert = (cal.adc_high < cal.adc_engage);
  int span_total = abs(cal.adc_high - cal.adc_free);
  int span_gap   = abs(cal.adc_engage - cal.adc_free);
  float dz = (span_total > 0) ? (float)span_gap / (float)span_total : 0.0f;
  if (dz < 0.0f) dz = 0.0f;
  if (dz > 0.6f) dz = 0.6f;
  cal.deadzone_pct = (uint8_t)roundf(dz * 100.0f);

  // Gamma input (waits for Enter)
  Serial.printf("Gamma shaping (0.5..2.0). Current=%.2f. Type value and press ENTER: ", cal.gamma);
  String gammaStr = readLine();
  if (gammaStr.length()) {
    float g = gammaStr.toFloat();
    if (g < 0.5f) g = 0.5f;
    if (g > 2.0f) g = 2.0f;
    cal.gamma = g;
  }

  savePrefs();
  Serial.println("Saved calibration:");
  printCalibration();
  Serial.println("=== Done ===\n");
}

/* ===================== Commands ===================== */
void printHelp() {
  Serial.println("Commands:");
  Serial.println("  cal                 - run 3-point resistance calibration (FREE, ENGAGE, HIGH)");
  Serial.println("  show                - print current calibration/params");
  Serial.println("  raw                 - stream raw ADC -> % (Ctrl+C to stop monitor)");
  Serial.println("  setcirc <mm>        - set wheel circumference in mm (1500..3000), e.g. setcirc 2148");
  Serial.println("  setratio <x>        - set base gear ratio (wheel revs per crank rev), e.g. setratio 2.2");
  Serial.println("  setspeedres <f>     - set resistance→ratio factor 0..1 (0=off). e.g. setspeedres 0.3");
  Serial.println("  settorque <min> <max> - set torque model in Nm, e.g. settorque 0.5 25");
  Serial.println("  setgamma <x>        - set resistance gamma shaping (0.5..2.0)");
  Serial.println("  help                - show this help");
}

void handleCommand(const String& cmdLine) {
  if (!cmdLine.length()) return;

  if (cmdLine.equalsIgnoreCase("cal")) { runCalibration(); return; }
  if (cmdLine.equalsIgnoreCase("show")) { printCalibration(); return; }
  if (cmdLine.equalsIgnoreCase("raw")) {
    Serial.println("Raw ADC stream (Ctrl+C to stop monitor):");
    for (int i=0;i<2000;i++) {
      int raw = readADCavg(8);
      float pct = mapCalibrated(raw);
      Serial.printf("raw=%d -> %5.1f%%\n", raw, pct);
      delay(50);
      if (Serial.available()) break;
    }
    return;
  }
  if (cmdLine.startsWith("setcirc")) {
    int sp = cmdLine.indexOf(' ');
    if (sp > 0) {
      int mm = cmdLine.substring(sp+1).toInt();
      if (mm >= 1500 && mm <= 3000) {
        cal.wheel_circ_mm = mm; savePrefs();
        Serial.printf("Wheel circumference set to %d mm\n", cal.wheel_circ_mm);
      } else {
        Serial.println("Enter a value between 1500 and 3000 mm.");
      }
    } else {
      Serial.printf("Current circumference: %d mm\n", cal.wheel_circ_mm);
      Serial.println("Usage: setcirc 2148");
    }
    return;
  }
  if (cmdLine.startsWith("setratio")) {
    int sp = cmdLine.indexOf(' ');
    if (sp > 0) {
      float r = cmdLine.substring(sp+1).toFloat();
      if (r > 0.3f && r < 5.0f) {
        cal.gear_ratio = r; savePrefs();
        Serial.printf("Base gear ratio set to %.2f (wheel revs per crank rev)\n", cal.gear_ratio);
      } else {
        Serial.println("Enter a sensible ratio (0.3..5.0). Typical MTB mid-gear ~2.2");
      }
    } else {
      Serial.printf("Current base ratio: %.2f\nUsage: setratio 2.2\n", cal.gear_ratio);
    }
    return;
  }
  if (cmdLine.startsWith("setspeedres")) {
    int sp = cmdLine.indexOf(' ');
    if (sp > 0) {
      float f = cmdLine.substring(sp+1).toFloat();
      if (f < 0.0f) f = 0.0f; if (f > 1.0f) f = 1.0f;
      cal.speed_res_factor = f; savePrefs();
      Serial.printf("Resistance→ratio factor set to %.2f (0=off). ratio_eff = ratio*(1 - %.2f*r)\n",
                    cal.speed_res_factor, cal.speed_res_factor);
    } else {
      Serial.printf("Current speed_res_factor: %.2f\nUsage: setspeedres 0.3\n", cal.speed_res_factor);
    }
    return;
  }
  if (cmdLine.startsWith("settorque")) {
    // settorque <min> <max>
    int sp = cmdLine.indexOf(' ');
    if (sp > 0) {
      int sp2 = cmdLine.indexOf(' ', sp+1);
      if (sp2 > 0) {
        float tmin = cmdLine.substring(sp+1, sp2).toFloat();
        float tmax = cmdLine.substring(sp2+1).toFloat();
        if (tmin < 0.0f) tmin = 0.0f;
        if (tmax < tmin + 0.5f) tmax = tmin + 0.5f;
        cal.torque_min_nm = tmin;
        cal.torque_max_nm = tmax;
        savePrefs();
        Serial.printf("Torque model set: Tmin=%.2f Nm, Tmax=%.2f Nm\n", cal.torque_min_nm, cal.torque_max_nm);
      } else {
        Serial.println("Usage: settorque 0.5 25");
      }
    } else {
      Serial.printf("Current torque: Tmin=%.2f Nm, Tmax=%.2f Nm\n", cal.torque_min_nm, cal.torque_max_nm);
      Serial.println("Usage: settorque 0.5 25");
    }
    return;
  }
  if (cmdLine.startsWith("setgamma")) {
    int sp = cmdLine.indexOf(' ');
    if (sp > 0) {
      float g = cmdLine.substring(sp+1).toFloat();
      if (g < 0.5f) g = 0.5f; if (g > 2.0f) g = 2.0f;
      cal.gamma = g; savePrefs();
      Serial.printf("Gamma set to %.2f\n", cal.gamma);
    } else {
      Serial.printf("Current gamma: %.2f\nUsage: setgamma 1.0\n", cal.gamma);
    }
    return;
  }
  if (cmdLine.equalsIgnoreCase("help")) { printHelp(); return; }

  Serial.println("Unknown command. Type 'help'.");
}

/* ===================== Setup / Loop ===================== */
void setup() {
  Serial.begin(115200);

  pinMode(MAGNET_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MAGNET_SENSOR_PIN), magnetISR, FALLING);

  pinMode(RESISTANCE_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(RESISTANCE_PIN, ADC_11db);

  loadPrefs();
  printCalibration();
  Serial.println("Type 'help' for commands.");

  // BLE init
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(185);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // FTMS (0x1826) with Indoor Bike Data (0x2AD2)
  BLEService* pService = pServer->createService(BLEUUID((uint16_t)0x1826));
  pBikeDataCharacteristic = pService->createCharacteristic(
    BLEUUID((uint16_t)0x2AD2),
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pBikeDataCharacteristic->addDescriptor(new BLE2902()); // CCCD
  pService->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID((uint16_t)0x1826);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("Bike Ready!");
}

void loop() {
  // Serial command handler (line-based)
  if (Serial.available()) {
    String cmd = readLine();
    handleCommand(cmd);
  }

  updateMetrics();
  sendBLEData();

  // Human-readable log (2 Hz)
  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (now - lastPrint >= 500) {
    lastPrint = now;
    int raw = analogRead(RESISTANCE_PIN);
    // Show ratio_eff for visibility
    float r_norm = resistance_pct / 100.0f;
    float ratio_eff = cal.gear_ratio * (1.0f - cal.speed_res_factor * r_norm);
    if (ratio_eff < 0.2f) ratio_eff = 0.2f;
    Serial.printf("Cad: %.1f RPM | Spd: %.2f km/h | Pwr: %.1f W | Res: %d%% | raw=%d | ratio_eff=%.2f\n",
                  cadence_rpm, speed_kmh, power_w, resistance_pct, raw, ratio_eff);
  }
}
