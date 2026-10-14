/**
 * ============================================================================
 * M5StickC Plus2 Aviation Avionics Simulator & Variometer
 * 
 * Hardware:
 *   - Device: M5StickC Plus2 (ESP32-PICO-V3-02, 240x135 ST7789v2 LCD)
 *   - Internal Sensors: 6-Axis IMU (MPU6886/BMI270), Buzzer, RTC, Battery
 *   - External Sensor: GY-63 module (MS5611 Barometric Pressure Sensor)
 *     connected to Grove Port (SDA=G32, SCL=G33, 5V, GND) or I2C bus
 *   - Bluetooth Low Energy (BLE): Transmits LK8EX1 sentences at 10Hz to
 *     XCTrack, XCSoar, Flyme, LK8000, SeeYou Navigator
 * 
 * Visuals & Modes:
 *   - Mode 0: Primary Flight Display (PFD Glass Cockpit)
 *   - Mode 1: Fullscreen Horizon with HUD overlay
 *   - Mode 2: Classic Altimeter Dial
 *   - Mode 3: Interactive Manual Altitude Calibration Menu (with NVS Flash save)
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <M5Unified.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include "kalmanfilter4d.h"

// ============================================================================
// COLOR DEFINITIONS (Mirko Pavleski Aviation Palette)
// ============================================================================
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

#define COL_BLACK        0x0000
#define COL_WHITE        0xFFFF
#define COL_YELLOW       0xFFE0
#define COL_ORANGE       0xFD20
#define COL_GREEN        0x07E0
#define COL_RED          0xF800
#define COL_CYAN         0x07FF
#define COL_DARK_GRAY    rgb565(35, 40, 45)
#define COL_PANEL_BG     rgb565(16, 20, 24)
#define COL_TAPE_BG      rgb565(10, 14, 18)
#define COL_SKY          rgb565(20, 165, 225)  // Vibrant Aviation Sky Blue
#define COL_GROUND       rgb565(150, 95, 45)   // Cockpit Earth Brown
#define COL_RUNWAY       rgb565(75, 38, 25)    // Runway Surface Brown
#define COL_RING_OUTER   rgb565(170, 170, 170) // Outer metallic bezel
#define COL_RING_INNER   rgb565(45, 45, 45)    // Inner dark bezel

// Base initial calibration altitude requested: 883m
#define CALIBRATED_BASE_ALT_M  883.0f

// ============================================================================
// PERSISTENT STORAGE (NVS FLASH) FOR CALIBRATION
// ============================================================================
static Preferences prefs;
static float cachedCalibratedAltitude = CALIBRATED_BASE_ALT_M;

static float loadCalibratedAltitude() {
    prefs.begin("avionics", false);
    if (prefs.isKey("calAlt")) {
        cachedCalibratedAltitude = prefs.getFloat("calAlt", CALIBRATED_BASE_ALT_M);
    } else {
        cachedCalibratedAltitude = CALIBRATED_BASE_ALT_M;
        prefs.putFloat("calAlt", CALIBRATED_BASE_ALT_M);
    }
    prefs.end();
    return cachedCalibratedAltitude;
}

static void saveCalibratedAltitude(float altM) {
    cachedCalibratedAltitude = altM;
    prefs.begin("avionics", false);
    prefs.putFloat("calAlt", altM);
    prefs.end();
    Serial.printf("[NVS] Altitude calibrada salva na Flash: %.1fm\r\n", altM);
}

// ============================================================================
// BLUETOOTH LOW ENERGY (BLE) FOR XCTRACK (Nordic UART Service + LK8EX1)
// ============================================================================
#define BLE_SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static NimBLEServer* pBleServer = nullptr;
static NimBLECharacteristic* pBleTxChar = nullptr;
static bool bleConnected = false;
static uint32_t lastBleTransmitMs = 0;

class BleServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        bleConnected = true;
        pServer->updateConnParams(desc->conn_handle, 6, 12, 0, 100);
        Serial.println("[BLE] XCTrack conectado com sucesso!");
    }
    void onDisconnect(NimBLEServer* pServer) override {
        bleConnected = false;
        Serial.println("[BLE] Cliente desconectado. Reiniciando advertising...");
        NimBLEDevice::startAdvertising();
    }
};

static void ble_init() {
    NimBLEDevice::init("M5-VARIO");
    NimBLEDevice::setMTU(185); // Critical: allows 32+ byte NMEA sentences without truncation
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    pBleServer = NimBLEDevice::createServer();
    pBleServer->setCallbacks(new BleServerCallbacks());

    NimBLEService* pService = pBleServer->createService(BLE_SERVICE_UUID);
    pBleTxChar = pService->createCharacteristic(BLE_CHARACTERISTIC_UUID_TX, NIMBLE_PROPERTY::NOTIFY);
    pService->createCharacteristic(BLE_CHARACTERISTIC_UUID_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);

    pService->start();

    // Critical for XCTrack discovery: advertise the Nordic UART Service UUID in beacon
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(BLE_SERVICE_UUID);
    pAdv->setScanResponse(true);
    pAdv->setMinPreferred(0x06); // 7.5ms
    pAdv->setMaxPreferred(0x12);
    pAdv->start();
    Serial.println("[BLE] Advertising started as 'M5-VARIO' (Nordic UART LK8EX1, MTU=185)");
}

static void ble_transmit_LK8EX1(int32_t rawPressurePa, int32_t altM, int32_t cps, int32_t tempC, float batVoltage) {
    if (pBleTxChar == nullptr) return;
    if (pBleServer == nullptr || pBleServer->getConnectedCount() == 0) return;

    // Sentence: $LK8EX1,pressure,altitude,vario_cps,temp,battery*checksum\r\n
    char szmsg[64];
    snprintf(szmsg, sizeof(szmsg), "$LK8EX1,%d,%d,%d,%d,%.1f*",
             (int)rawPressurePa, (int)altM, (int)cps, (int)tempC, batVoltage);

    uint8_t cksum = 0;
    for (char* p = szmsg + 1; *p != '*'; p++) {
        cksum ^= (uint8_t)*p;
    }
    char szcksum[8];
    snprintf(szcksum, sizeof(szcksum), "%02X\r\n", cksum);
    strcat(szmsg, szcksum);

    pBleTxChar->setValue((const uint8_t*)szmsg, strlen(szmsg));
    pBleTxChar->notify();

    static uint32_t lastDbg = 0;
    if (millis() - lastDbg > 2000) {
        lastDbg = millis();
        Serial.printf("[BLE-TX] %s", szmsg);
    }
}

// ============================================================================
// GY-63 (MS5611) I2C DRIVER (With Moving-Average Noise Damping)
// ============================================================================
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_CONV_D1_4096 0x48
#define MS5611_CMD_CONV_D2_4096 0x58
#define MS5611_CMD_ADC_READ     0x00
#define MS5611_CMD_PROM_READ    0xA0

#define PRESS_FILTER_SIZE 8

class MS5611_I2C {
public:
    TwoWire* i2c = nullptr;
    uint8_t  addr = 0x77;
    uint16_t C[8];
    bool     isDetected = false;
    bool     isSimulated = false; // NEVER starts as true automatically!
    float    seaLevelPressurePa = 101325.0f; // Calibrated QNH reference

    enum State { IDLE, WAIT_D1, WAIT_D2 } state = IDLE;
    uint32_t lastConvStart = 0;
    uint32_t D1 = 0;
    uint32_t D2 = 0;

    float rawPressurePa = 101325.0f;
    float rawTemperatureC = 20.0f;
    float rawAltitudeM = CALIBRATED_BASE_ALT_M;

    // Moving average filter to eliminate sensor ADC jitter
    float pressHistory[PRESS_FILTER_SIZE];
    int   pressHistIdx = 0;
    bool  pressHistFilled = false;

    // Smooth demo simulation variables
    float simAlt = CALIBRATED_BASE_ALT_M;
    float simPhase = 0.0f;

    int activeSda = 33;
    int activeScl = 32;

    bool begin() {
        isDetected = false;
        isSimulated = false;
        state = IDLE;

        struct PinCandidate {
            int sda;
            int scl;
            const char* name;
        };

        const PinCandidate candidates[] = {
            { 33, 32, "Grove Port A (SDA=33, SCL=32)" },
            { 32, 33, "Grove Port A Invertido (SDA=32, SCL=33)" },
            { 21, 22, "Barramento Interno (SDA=21, SCL=22)" },
            {  0, 26, "Header Topo (SDA=0, SCL=26)" },
            { 26,  0, "Header Topo Invertido (SDA=26, SCL=0)" }
        };

        for (const auto& p : candidates) {
            Serial.printf("[I2C Scan] Testando %s...\r\n", p.name);
            Wire.end();
            delay(10);
            Wire.begin(p.sda, p.scl, 100000UL);

            const uint8_t addrs[] = { 0x77, 0x76 };
            for (uint8_t a : addrs) {
                Wire.beginTransmission(a);
                if (Wire.endTransmission() == 0) {
                    Wire.beginTransmission(a);
                    Wire.write(MS5611_CMD_RESET);
                    Wire.endTransmission();
                    delay(15);

                    addr = a;
                    i2c = &Wire;
                    activeSda = p.sda;
                    activeScl = p.scl;
                    readProm();

                    if (C[1] != 0 && C[1] != 0xFFFF) {
                        isDetected = true;
                        Serial.printf("[GY-63] MS5611 DETECTADO em 0x%02X via %s! C1=%u, C2=%u\r\n",
                                      addr, p.name, C[1], C[2]);
                        Wire.setClock(400000UL);
                        triggerD1();
                        return true;
                    } else {
                        Serial.printf("[GY-63] Respondeu em 0x%02X mas PROM invalida (C1=0x%04X)\r\n", a, C[1]);
                    }
                }
            }
        }

        // Return to standard Grove (SDA=33, SCL=32)
        Wire.end();
        delay(10);
        Wire.begin(33, 32, 100000UL);
        i2c = &Wire;
        activeSda = 33;
        activeScl = 32;

        // Sensor not detected: do NOT auto-enable demo mode
        isSimulated = false;
        rawAltitudeM = CALIBRATED_BASE_ALT_M;
        simAlt = CALIBRATED_BASE_ALT_M;
        Serial.println("[GY-63] Sensor nao detectado nos pinos testados. Aguardando conexao...");
        return false;
    }

    bool probeAddress(TwoWire &w, uint8_t a) {
        w.beginTransmission(a);
        if (w.endTransmission() == 0) {
            w.beginTransmission(a);
            w.write(MS5611_CMD_RESET);
            w.endTransmission();
            delay(10);
            return true;
        }
        return false;
    }

    void readProm() {
        for (uint8_t i = 0; i < 8; i++) {
            i2c->beginTransmission(addr);
            i2c->write(MS5611_CMD_PROM_READ + (i * 2));
            i2c->endTransmission();
            i2c->requestFrom(addr, (uint8_t)2);
            if (i2c->available() >= 2) {
                C[i] = (i2c->read() << 8) | i2c->read();
            } else {
                C[i] = 0;
            }
        }
    }

    void calibrateToAltitude(float targetAltM) {
        if (rawPressurePa > 50000.0f && rawPressurePa < 120000.0f) {
            // P0 = P / (1 - targetAlt / 44330)^5.255
            seaLevelPressurePa = rawPressurePa / powf(1.0f - (targetAltM / 44330.0f), 5.255f);
            rawAltitudeM = targetAltM;
        } else {
            seaLevelPressurePa = 101325.0f;
            rawAltitudeM = targetAltM;
            simAlt = targetAltM;
        }
    }

    void triggerD1() {
        if (!isDetected) return;
        i2c->beginTransmission(addr);
        i2c->write(MS5611_CMD_CONV_D1_4096);
        i2c->endTransmission();
        lastConvStart = micros();
        state = WAIT_D1;
    }

    void triggerD2() {
        if (!isDetected) return;
        i2c->beginTransmission(addr);
        i2c->write(MS5611_CMD_CONV_D2_4096);
        i2c->endTransmission();
        lastConvStart = micros();
        state = WAIT_D2;
    }

    uint32_t readADC() {
        i2c->beginTransmission(addr);
        i2c->write(MS5611_CMD_ADC_READ);
        i2c->endTransmission();
        i2c->requestFrom(addr, (uint8_t)3);
        if (i2c->available() >= 3) {
            return ((uint32_t)i2c->read() << 16) | ((uint32_t)i2c->read() << 8) | i2c->read();
        }
        return 0;
    }

    bool update(float &altitudeOut) {
        if (isSimulated) {
            // Demo thermal simulation only when user explicitly turned it ON
            simPhase += 0.025f;
            float climb = sinf(simPhase) * 1.2f;
            simAlt += climb * 0.02f;
            rawAltitudeM = simAlt;
            rawPressurePa = seaLevelPressurePa * powf(1.0f - (simAlt / 44330.0f), 5.255f);
            altitudeOut = rawAltitudeM;
            return true;
        }

        if (!isDetected) {
            // Sensor not yet found: keep altitude locked at calibrated reference
            rawAltitudeM = cachedCalibratedAltitude;
            altitudeOut = rawAltitudeM;

            // Probe in background every 2 seconds without hanging UI
            static uint32_t lastProbeMs = 0;
            if (millis() - lastProbeMs > 2000) {
                lastProbeMs = millis();
                if (probeAddress(Wire1, 0x77) || probeAddress(Wire1, 0x76) ||
                    probeAddress(Wire, 0x77)  || probeAddress(Wire, 0x76)) {
                    if (begin()) {
                        calibrateToAltitude(cachedCalibratedAltitude);
                    }
                }
            }
            return false;
        }

        uint32_t now = micros();
        if (state == WAIT_D1 && (now - lastConvStart >= 9500)) {
            D1 = readADC();
            triggerD2();
        } else if (state == WAIT_D2 && (now - lastConvStart >= 9500)) {
            D2 = readADC();
            calculate();
            triggerD1();
            altitudeOut = rawAltitudeM;
            return true;
        }
        return false;
    }

    void calculate() {
        int64_t dT = (int64_t)D2 - ((int64_t)C[5] << 8);
        int64_t TEMP = 2000 + ((dT * (int64_t)C[6]) >> 23);

        int64_t OFF  = ((int64_t)C[2] << 16) + (((int64_t)C[4] * dT) >> 7);
        int64_t SENS = ((int64_t)C[1] << 15) + (((int64_t)C[3] * dT) >> 8);

        if (TEMP < 2000) {
            int64_t T2 = (dT * dT) >> 31;
            int64_t OFF2 = (5 * (TEMP - 2000) * (TEMP - 2000)) >> 1;
            int64_t SENS2 = (5 * (TEMP - 2000) * (TEMP - 2000)) >> 2;
            if (TEMP < -1500) {
                OFF2 += 7 * (TEMP + 1500) * (TEMP + 1500);
                SENS2 += (11 * (TEMP + 1500) * (TEMP + 1500)) >> 1;
            }
            TEMP -= T2;
            OFF  -= OFF2;
            SENS -= SENS2;
        }

        int64_t P = (((D1 * SENS) >> 21) - OFF) >> 15;
        float currentP = (float)P;

        pressHistory[pressHistIdx] = currentP;
        pressHistIdx = (pressHistIdx + 1) % PRESS_FILTER_SIZE;
        if (pressHistIdx == 0) pressHistFilled = true;

        int count = pressHistFilled ? PRESS_FILTER_SIZE : pressHistIdx;
        float sumP = 0.0f;
        for (int i = 0; i < count; i++) sumP += pressHistory[i];
        rawPressurePa = sumP / (float)count;
        rawTemperatureC = (float)TEMP / 100.0f;

        // Altitude with calibrated seaLevelPressurePa
        rawAltitudeM = 44330.0f * (1.0f - powf(rawPressurePa / seaLevelPressurePa, 0.1902949f));
    }
};

static MS5611_I2C GY63;

// ============================================================================
// SYSTEM & FLIGHT STATE
// ============================================================================
static M5Canvas canvas(&M5.Display);

enum DisplayMode {
    MODE_PFD = 0,         // Glass Cockpit Primary Flight Display (Horizon + Tapes)
    MODE_FULL_HORIZON,    // Fullscreen Artificial Horizon with HUD overlay
    MODE_ALTIMETER_DIAL,  // Classic 3-Pointer Altimeter Dial
    MODE_CALIB_MENU,      // Interactive Altitude Calibration Menu
    MODE_COUNT
};

static DisplayMode currentMode = MODE_PFD;

static float pitchDeg = 0.0f;
static float rollDeg = 0.0f;

static float pitchOffset = 0.0f;
static float rollOffset = 0.0f;

static float AltitudeM = CALIBRATED_BASE_ALT_M;
static float ClimbrateCps = 0.0f; // cm/s
static float ClimbrateMs = 0.0f;  // m/s
static float ZeroAltReference = CALIBRATED_BASE_ALT_M;

static bool  isAudioMuted = false;
static int   batteryPct = 100;
static uint32_t lastKfTimeUs = 0;

// Interactive Menu Variables
static float menuTargetAlt = CALIBRATED_BASE_ALT_M;
static int   menuStepM = 10; // 10m, 1m, 50m

// On-screen notification Toast banner
static char toastMsg[36] = "";
static uint16_t toastColor = COL_YELLOW;
static uint32_t toastUntilMs = 0;

static void showToast(const char* msg, uint16_t color = COL_YELLOW, uint32_t durationMs = 1600) {
    strncpy(toastMsg, msg, sizeof(toastMsg) - 1);
    toastMsg[sizeof(toastMsg) - 1] = '\0';
    toastColor = color;
    toastUntilMs = millis() + durationMs;
}

// Toggle Demo Mode Helper
static void toggleDemoMode() {
    GY63.isSimulated = !GY63.isSimulated;
    if (GY63.isSimulated) {
        GY63.simAlt = AltitudeM;
        GY63.simPhase = 0.0f;
        M5.Speaker.tone(900, 80);
        delay(90);
        M5.Speaker.tone(1200, 100);
        showToast("MODO DEMO LIGADO", COL_ORANGE, 2000);
        Serial.println("[MODE] Modo DEMO LIGADO.");
    } else {
        M5.Speaker.stop();
        ClimbrateMs = 0.0f;
        ClimbrateCps = 0.0f;
        M5.Speaker.tone(1200, 100);
        delay(90);
        M5.Speaker.tone(1600, 120);
        if (GY63.isDetected) {
            showToast("MODO SENSOR REAL", COL_GREEN, 2000);
        } else {
            showToast("MODO DEMO DESLIGADO", COL_WHITE, 2000);
        }
        Serial.println("[MODE] Modo DEMO DESLIGADO.");
    }
}

// ============================================================================
// AUDIO VARIOMETER GENERATOR
// ============================================================================
static void handleAudioVario(float vsiMs) {
    if (isAudioMuted || currentMode == MODE_CALIB_MENU || (!GY63.isDetected && !GY63.isSimulated)) {
        M5.Speaker.stop();
        return;
    }

    // Variometer deadband: perfectly silent between -0.20 m/s and +0.20 m/s
    if (fabsf(vsiMs) < 0.20f) {
        M5.Speaker.stop();
        return;
    }

    static uint32_t lastBeepTick = 0;
    static bool isBeeping = false;
    uint32_t now = millis();

    // Climb threshold: +0.22 m/s
    if (vsiMs >= 0.22f) {
        int freq = (int)(540.0f + (vsiMs * 250.0f));
        if (freq > 2200) freq = 2200;

        int period = (int)(440.0f - (vsiMs * 70.0f));
        if (period < 85) period = 85;
        int onTime = (int)(period * 0.52f);

        uint32_t elapsed = now - lastBeepTick;
        if (elapsed < (uint32_t)onTime) {
            if (!isBeeping) {
                M5.Speaker.tone(freq, onTime);
                isBeeping = true;
            }
        } else if (elapsed < (uint32_t)period) {
            if (isBeeping) {
                M5.Speaker.stop();
                isBeeping = false;
            }
        } else {
            lastBeepTick = now;
            M5.Speaker.tone(freq, onTime);
            isBeeping = true;
        }
    }
    // Sink warning: -1.40 m/s
    else if (vsiMs <= -1.40f) {
        int sinkFreq = (int)(300.0f + (vsiMs * 25.0f));
        if (sinkFreq < 180) sinkFreq = 180;
        M5.Speaker.tone(sinkFreq, 80);
        isBeeping = true;
    } else {
        if (isBeeping) {
            M5.Speaker.stop();
            isBeeping = false;
        }
    }
}

// ============================================================================
// ARTIFICIAL HORIZON RENDERING (Mirko Pavleski Visual Style)
// ============================================================================
static void drawArtificialHorizon(int CX, int CY, int R) {
    const int r2 = R * R;
    const float pitchScale = 1.6f;

    float p = pitchDeg - pitchOffset;
    float r = rollDeg - rollOffset;

    if (p > 50.0f)  p = 50.0f;
    if (p < -50.0f) p = -50.0f;

    float rollRad = r * (PI / 180.0f);
    float cr = cosf(rollRad);
    float sr = sinf(rollRad);
    float pitchPix = p * pitchScale;

    // 1. DYNAMIC SKY & GROUND DISC FILL
    for (int y = -R; y <= R; y++) {
        int y2 = y * y;
        for (int x = -R; x <= R; x++) {
            if (x * x + y2 <= r2) {
                float yr = x * sr + y * cr;
                if (yr + pitchPix < 0.0f) {
                    canvas.drawPixel(CX + x, CY + y, COL_SKY);
                } else {
                    canvas.drawPixel(CX + x, CY + y, COL_GROUND);
                }
            }
        }
    }

    auto H = [&](float lx, float ly, int &sx, int &sy) {
        sx = CX + (int)(lx * cr + ly * sr);
        sy = CY - (int)(lx * sr - ly * cr);
    };

    // 2. VIRTUAL RUNWAY
    {
        int x1, y1, x2, y2, x3, y3;
        float apexY = 4.0f - pitchPix;
        float baseY = 42.0f - pitchPix;

        H(0.0f,   apexY, x1, y1);
        H(-22.0f, baseY, x2, y2);
        H(22.0f,  baseY, x3, y3);

        canvas.fillTriangle(x1, y1, x2, y2, x3, y3, COL_RUNWAY);

        int xa, ya, xb, yb;
        H(0.0f,   apexY, xa, ya);
        H(-22.0f, baseY, xb, yb);
        canvas.drawWideLine(xa, ya, xb, yb, 1.5f, COL_ORANGE);

        H(0.0f,   apexY, xa, ya);
        H(22.0f,  baseY, xb, yb);
        canvas.drawWideLine(xa, ya, xb, yb, 1.5f, COL_ORANGE);

        H(0.0f,   apexY, xa, ya);
        H(0.0f,   baseY, xb, yb);
        canvas.drawWideLine(xa, ya, xb, yb, 1.0f, COL_WHITE);
    }

    // 3. MAIN HORIZON LINE
    {
        int x1, y1, x2, y2;
        H(-R, -pitchPix, x1, y1);
        H( R, -pitchPix, x2, y2);
        canvas.drawWideLine(x1, y1, x2, y2, 2.0f, COL_WHITE);
    }

    // 4. PITCH LADDER (-20°, -10°, -5°, +5°, +10°, +20°)
    const int stepDeg = 5;
    for (int deg = -20; deg <= 20; deg += stepDeg) {
        if (deg == 0) continue;

        int len = (abs(deg) == 10 || abs(deg) == 20) ? 32 : 16;
        float localY = -(deg * pitchScale) - pitchPix;

        if (fabsf(localY) > (float)R - 4.0f) continue;

        int x1, y1, x2, y2;
        H(-len / 2.0f, localY, x1, y1);
        H( len / 2.0f, localY, x2, y2);

        canvas.drawWideLine(x1, y1, x2, y2, 1.5f, COL_WHITE);

        if (abs(deg) == 10 || abs(deg) == 20) {
            int xl, yl, xr, yr;
            H(-len / 2.0f - 11.0f, localY - 3.0f, xl, yl);
            H( len / 2.0f + 5.0f,  localY - 3.0f, xr, yr);

            canvas.setTextColor(COL_WHITE);
            canvas.setTextDatum(middle_center);
            canvas.setFont(&fonts::Font0);
            canvas.setTextSize(1);
            canvas.drawNumber(abs(deg), xl, yl);
            canvas.drawNumber(abs(deg), xr, yr);
        }
    }

    // 5. ROLL SCALE ON RING
    const int rollAngles[] = {-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60};
    for (int a : rollAngles) {
        float rad = (a - 90) * (PI / 180.0f);
        int r1 = R - 3;
        int r2 = R + 2;
        if (a % 30 == 0) r1 = R - 6;

        int x1 = CX + (int)(cosf(rad) * r1);
        int y1 = CY + (int)(sinf(rad) * r1);
        int x2 = CX + (int)(cosf(rad) * r2);
        int y2 = CY + (int)(sinf(rad) * r2);
        canvas.drawWideLine(x1, y1, x2, y2, 1.2f, COL_WHITE);
    }

    canvas.fillTriangle(CX, CY - R + 2, CX - 4, CY - R - 5, CX + 4, CY - R - 5, COL_ORANGE);

    // Moving white roll pointer triangle
    {
        auto ROLL = [&](float lx, float ly, int &sx, int &sy) {
            sx = CX + (int)(lx * cr + ly * sr);
            sy = CY - (int)(lx * sr - ly * cr);
        };
        int px1, py1, px2, py2, px3, py3;
        ROLL(0.0f,  -(float)R + 5.0f, px1, py1);
        ROLL(-4.0f, -(float)R + 11.0f, px2, py2);
        ROLL( 4.0f, -(float)R + 11.0f, px3, py3);
        canvas.fillTriangle(px1, py1, px2, py2, px3, py3, COL_WHITE);
    }

    // 6. METALLIC BEZEL RINGS
    canvas.drawCircle(CX, CY, R,     COL_RING_INNER);
    canvas.drawCircle(CX, CY, R + 1, COL_RING_OUTER);
    canvas.drawCircle(CX, CY, R + 2, COL_BLACK);

    // 7. FIXED AIRCRAFT SYMBOL (Yellow Waterline Crosshair)
    canvas.drawWideLine(CX - 36, CY, CX - 10, CY, 3.0f, COL_YELLOW);
    canvas.drawWideLine(CX + 10, CY, CX + 36, CY, 3.0f, COL_YELLOW);
    canvas.drawWideLine(CX - 10, CY, CX, CY + 7, 3.0f, COL_YELLOW);
    canvas.drawWideLine(CX, CY + 7, CX + 10, CY, 3.0f, COL_YELLOW);
    canvas.fillCircle(CX, CY, 2, COL_YELLOW);
}

// ============================================================================
// AVIATION FORMAT ALTIMETER & VARIOMETER TAPES (PFD Style with LARGE ALTITUDE)
// ============================================================================
static void drawAviationAltimeterAndVsi(int startX) {
    const int tapeW = 46;
    const int vsiW  = 26;
    const int tapeX = startX + 1;
    const int vsiX  = tapeX + tapeW + 1;
    const int centerY = 67;

    // Background Panels
    canvas.fillRect(tapeX, 0, tapeW, 135, COL_TAPE_BG);
    canvas.fillRect(vsiX, 0, vsiW, 135, COL_PANEL_BG);

    canvas.drawFastVLine(tapeX - 1, 0, 135, COL_DARK_GRAY);
    canvas.drawFastVLine(vsiX - 1, 0, 135, COL_DARK_GRAY);

    // ALTIMETER TAPE
    float currentAlt = AltitudeM;
    int intAlt = (int)currentAlt;
    int offsetPix = (int)((currentAlt - (float)intAlt) * 1.0f);

    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(1);
    canvas.setTextColor(COL_WHITE);

    for (int y = -65; y <= 65; y++) {
        int markAlt = intAlt + y;
        if (markAlt < 0) continue;

        int screenY = centerY - (y - offsetPix);
        if (screenY < 14 || screenY > 120) continue;

        if (markAlt % 20 == 0) {
            canvas.drawFastHLine(tapeX, screenY, 7, COL_WHITE);
            canvas.setTextDatum(middle_left);
            canvas.drawNumber(markAlt, tapeX + 9, screenY);
        } else if (markAlt % 10 == 0) {
            canvas.drawFastHLine(tapeX, screenY, 4, COL_DARK_GRAY);
        }
    }

    // Top Header: ALT m
    canvas.fillRect(tapeX, 0, tapeW, 14, COL_PANEL_BG);
    canvas.setTextDatum(top_center);
    canvas.setTextColor(COL_CYAN);
    canvas.drawString("ALT m", tapeX + tapeW / 2, 2);

    // LARGE CURRENT ALTITUDE DRUM / POINTER BUG
    const int bugH = 26;
    const int bugY = centerY - bugH / 2;
    canvas.fillTriangle(tapeX - 4, centerY, tapeX, centerY - 6, tapeX, centerY + 6, COL_WHITE);
    canvas.fillRect(tapeX, bugY, tapeW, bugH, COL_BLACK);
    canvas.drawRect(tapeX, bugY, tapeW, bugH, COL_GREEN);
    canvas.drawRect(tapeX + 1, bugY + 1, tapeW - 2, bugH - 2, COL_WHITE);

    // LARGE BOLD ALTITUDE NUMBER (Font4)
    canvas.setTextColor(COL_GREEN, COL_BLACK);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font4);
    canvas.setTextSize(1);
    canvas.drawNumber((int)currentAlt, tapeX + (tapeW / 2), centerY + 1);

    // Bottom Info: AGL / Relative Altitude
    canvas.fillRect(tapeX, 121, tapeW, 14, COL_PANEL_BG);
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(bottom_center);
    canvas.setTextColor(COL_YELLOW);
    canvas.printf("AGL:%+d", (int)(currentAlt - ZeroAltReference));

    // VERTICAL SPEED INDICATOR (VSI)
    const float vsiScale = 12.0f;
    float vsi = ClimbrateMs;

    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(1);
    canvas.setTextColor(COL_WHITE);
    canvas.setTextDatum(middle_right);

    int vsiMarks[] = {4, 2, 0, -2, -4};
    for (int m : vsiMarks) {
        int y = centerY - (int)(m * vsiScale);
        canvas.drawFastHLine(vsiX + 1, y, 4, COL_WHITE);
        if (m != 0) {
            canvas.drawNumber(abs(m), vsiX + vsiW - 2, y);
        }
    }

    canvas.drawFastHLine(vsiX, centerY, 8, COL_WHITE);

    int barHeight = (int)(vsi * vsiScale);
    if (barHeight > 50)  barHeight = 50;
    if (barHeight < -50) barHeight = -50;

    if (barHeight > 1) {
        canvas.fillRect(vsiX + 2, centerY - barHeight, 6, barHeight, COL_GREEN);
        canvas.drawRect(vsiX + 2, centerY - barHeight, 6, barHeight, COL_WHITE);
    } else if (barHeight < -1) {
        canvas.fillRect(vsiX + 2, centerY, 6, -barHeight, COL_ORANGE);
        canvas.drawRect(vsiX + 2, centerY, 6, -barHeight, COL_WHITE);
    }

    canvas.fillRect(vsiX, 0, vsiW, 14, (vsi >= 0.15f) ? COL_GREEN : (vsi <= -1.3f ? COL_RED : COL_DARK_GRAY));
    canvas.setTextDatum(top_center);
    canvas.setTextColor(COL_BLACK);
    canvas.setFont(&fonts::Font0);
    canvas.printf("%+.1f", vsi);

    canvas.fillRect(vsiX, 121, vsiW, 14, COL_PANEL_BG);
    canvas.setTextDatum(bottom_center);
    canvas.setTextColor(isAudioMuted ? COL_RED : COL_GREEN);
    canvas.drawString(isAudioMuted ? "MUT" : "SND", vsiX + vsiW / 2, 134);
}

// ============================================================================
// TOP & BOTTOM STATUS HEADER OVERLAYS
// ============================================================================
static void drawStatusHeader() {
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(1);

    canvas.setTextDatum(top_left);
    canvas.setTextColor(COL_WHITE, COL_BLACK);
    canvas.printf(" P:%+02d R:%+02d", (int)(pitchDeg - pitchOffset), (int)(rollDeg - rollOffset));

    canvas.setTextDatum(top_center);
    if (bleConnected) {
        canvas.setTextColor(COL_GREEN, COL_BLACK);
        canvas.drawString("BLE:XCT", 116, 2);
    } else {
        canvas.setTextColor(COL_CYAN, COL_BLACK);
        canvas.drawString("BLE:ADV", 116, 2);
    }

    canvas.setTextDatum(top_right);
    if (GY63.isSimulated) {
        canvas.setTextColor(COL_ORANGE, COL_BLACK);
        canvas.drawString("DEMO ", 164, 2);
    } else if (GY63.isDetected) {
        canvas.setTextColor(COL_GREEN, COL_BLACK);
        canvas.drawString("GY-63 ", 164, 2);
    } else {
        canvas.setTextColor(COL_DARK_GRAY, COL_BLACK);
        canvas.drawString("S/BARO", 164, 2);
    }

    canvas.setTextDatum(bottom_left);
    canvas.setTextColor(COL_WHITE, COL_BLACK);
    canvas.printf(" BAT:%d%%", batteryPct);
}

// ============================================================================
// FULLSCREEN HORIZON MODE (HUD Overlay with Large Altitude)
// ============================================================================
static void drawFullscreenHorizon() {
    drawArtificialHorizon(120, 67, 65);

    canvas.setTextColor(COL_GREEN, COL_BLACK);

    canvas.setFont(&fonts::Font2);
    canvas.setTextDatum(top_left);
    canvas.printf("P:%+02d\n", (int)(pitchDeg - pitchOffset));
    canvas.printf("R:%+02d", (int)(rollDeg - rollOffset));

    canvas.setFont(&fonts::Font4);
    canvas.setTextDatum(top_right);
    canvas.printf("%dm\n", (int)AltitudeM);

    canvas.setFont(&fonts::Font2);
    canvas.setTextColor((ClimbrateMs >= 0.22f) ? COL_GREEN : (ClimbrateMs <= -1.40f ? COL_RED : COL_DARK_GRAY), COL_BLACK);
    canvas.printf("%+.1fm/s", ClimbrateMs);

    canvas.setTextDatum(bottom_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COL_YELLOW, COL_BLACK);
    canvas.printf("AGL:%+dm | %s | %s | %s",
                  (int)(AltitudeM - ZeroAltReference),
                  isAudioMuted ? "MUTE" : "AUDIO",
                  bleConnected ? "BLE:OK" : "BLE:WAIT",
                  GY63.isSimulated ? "DEMO" : (GY63.isDetected ? "GY-63" : "S/BARO"));
}

// ============================================================================
// CLASSIC ALTIMETER DIAL MODE (Adapted from Mirko Pavleski drawScreen_Altimeter)
// ============================================================================
static void drawClassicAltimeterDial() {
    canvas.fillScreen(COL_PANEL_BG);

    const int CX = 95;
    const int CY = 67;
    const int R  = 62;

    canvas.drawCircle(CX, CY, R, COL_RING_OUTER);
    canvas.drawCircle(CX, CY, R - 1, COL_RING_INNER);
    canvas.fillCircle(CX, CY, R - 2, rgb565(20, 24, 28));

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COL_WHITE);
    canvas.setTextDatum(middle_center);
    for (int i = 0; i < 10; i++) {
        float angle = (i * 36 - 90) * (PI / 180.0f);
        int tx = CX + (int)(cosf(angle) * (R - 12));
        int ty = CY + (int)(sinf(angle) * (R - 12));
        canvas.drawNumber(i, tx, ty);

        int x1 = CX + (int)(cosf(angle) * (R - 3));
        int y1 = CY + (int)(sinf(angle) * (R - 3));
        int x2 = CX + (int)(cosf(angle) * (R - 7));
        int y2 = CY + (int)(sinf(angle) * (R - 7));
        canvas.drawLine(x1, y1, x2, y2, COL_WHITE);
    }

    int altInt = (int)AltitudeM;
    float hand100Deg = fmodf(AltitudeM, 1000.0f) * 0.36f;
    float hand1000Deg = fmodf(AltitudeM, 10000.0f) * 0.036f;

    auto drawHand = [&](float deg, int len, int w, uint16_t col) {
        float rad = (deg - 90.0f) * (PI / 180.0f);
        int hx = CX + (int)(cosf(rad) * len);
        int hy = CY + (int)(sinf(rad) * len);
        canvas.drawWideLine(CX, CY, hx, hy, (float)w, col);
    };

    drawHand(hand1000Deg, R - 22, 3, COL_YELLOW);
    drawHand(hand100Deg,  R - 8,  1, COL_WHITE);
    canvas.fillCircle(CX, CY, 3, COL_YELLOW);

    canvas.fillRect(CX - 25, CY + 20, 50, 18, COL_BLACK);
    canvas.drawRect(CX - 25, CY + 20, 50, 18, COL_WHITE);
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(COL_GREEN);
    canvas.setFont(&fonts::Font2);
    canvas.drawNumber(altInt, CX, CY + 29);

    drawAviationAltimeterAndVsi(166);
}

// ============================================================================
// MODE 3: INTERACTIVE ALTITUDE CALIBRATION MENU
// Allows adjusting target altitude manually and saving permanently to Flash NVS
// ============================================================================
static void drawCalibrationMenu() {
    canvas.fillScreen(COL_PANEL_BG);

    // Title Bar
    canvas.fillRect(0, 0, 240, 20, rgb565(26, 32, 40));
    canvas.drawFastHLine(0, 20, 240, COL_CYAN);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COL_CYAN, rgb565(26, 32, 40));
    canvas.drawString("MENU: AJUSTE DE ALTITUDE", 120, 10);

    // Large Target Altitude Drum Box
    const int boxX = 40;
    const int boxY = 25;
    const int boxW = 160;
    const int boxH = 42;

    canvas.fillRect(boxX, boxY, boxW, boxH, COL_BLACK);
    canvas.drawRect(boxX, boxY, boxW, boxH, COL_GREEN);
    canvas.drawRect(boxX + 1, boxY + 1, boxW - 2, boxH - 2, COL_WHITE);

    // Large Digits for Altitude (Font4)
    canvas.setFont(&fonts::Font4);
    canvas.setTextColor(COL_GREEN, COL_BLACK);
    canvas.setTextDatum(middle_center);
    canvas.drawNumber((int)menuTargetAlt, 108, 46);

    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COL_WHITE, COL_BLACK);
    canvas.drawString("m", 168, 46);

    // Live Baro Sensor Pressure & Estimated QNH
    float curP = GY63.rawPressurePa;
    float calcQNH = (curP > 50000.0f) ? (curP / powf(1.0f - (menuTargetAlt / 44330.0f), 5.255f)) : 101325.0f;

    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(COL_YELLOW, COL_PANEL_BG);
    canvas.setCursor(12, 73);
    canvas.printf("P.Sensor: %.1f hPa | QNH: %.1f hPa", curP / 100.0f, calcQNH / 100.0f);

    // Step Size Pill Indicator (Strictly 1 meter per click)
    canvas.fillRect(55, 87, 130, 18, rgb565(30, 36, 44));
    canvas.drawRect(55, 87, 130, 18, COL_CYAN);
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(COL_CYAN, rgb565(30, 36, 44));
    canvas.drawString("Passo Fixo: [ +/- 1m ]", 120, 96);

    // Bottom Action Bar
    canvas.fillRect(0, 112, 240, 23, rgb565(20, 24, 30));
    canvas.drawFastHLine(0, 112, 240, COL_DARK_GRAY);
    canvas.setTextColor(COL_WHITE, rgb565(20, 24, 30));
    canvas.drawString("[A]: +1m   [B]: -1m   [Segurar A]: SALVAR", 120, 123);
}

// Draw notification popup toast
static void drawToast() {
    if (millis() < toastUntilMs) {
        canvas.fillRect(14, 48, 212, 38, COL_BLACK);
        canvas.drawRect(14, 48, 212, 38, toastColor);
        canvas.drawRect(15, 49, 210, 36, toastColor);
        canvas.setTextDatum(middle_center);
        canvas.setFont(&fonts::Font0);
        canvas.setTextSize(1);
        canvas.setTextColor(toastColor, COL_BLACK);
        canvas.drawString(toastMsg, 120, 67);
    }
}

// ============================================================================
// MAIN SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n\n==================================================");
    Serial.println("  M5StickC Plus2 Aviation Avionics & Variometer  ");
    Serial.println("==================================================");

    // Initialize M5StickC Plus2 hardware (Display, IMU, Power, Buttons, Speaker)
    auto cfg = M5.config();
    M5.begin(cfg);

    // Power external Grove Port A (5V rail) for GY-63 module
    M5.Power.setExtOutput(true);
    delay(30);

    M5.Speaker.setVolume(200);

    // Setup landscape display (240x135)
    M5.Display.setRotation(1);
    M5.Display.fillScreen(COL_BLACK);

    canvas.setColorDepth(16);
    canvas.createSprite(240, 135);

    // Load calibrated altitude from NVS Flash (defaults to 883m if never saved)
    float baseAltitude = loadCalibratedAltitude();
    menuTargetAlt = baseAltitude;
    Serial.printf("[SETUP] Altitude carregada da Flash: %.1fm\r\n", baseAltitude);

    // Show Boot Screen
    canvas.fillScreen(COL_BLACK);
    canvas.setTextColor(COL_CYAN);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.drawString("AVIONICS SIMULATOR", 120, 30);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COL_WHITE);
    canvas.drawString("M5-Watch StickC Plus2", 120, 52);
    canvas.drawString("Nivelando Horizonte Inicial...", 120, 68);
    canvas.setTextColor(COL_GREEN);
    canvas.printf("Calibrando Altitude: %dm", (int)baseAltitude);
    canvas.setTextColor(COL_CYAN);
    canvas.drawString("Iniciando BLE (M5-VARIO)...", 120, 102);
    canvas.pushSprite(0, 0);

    // ------------------------------------------------------------------------
    // TARE INITIAL HORIZON ORIENTATION AT POWER-ON
    // Captures the resting posture of the device as level flight (0° pitch, 0° roll)
    // ------------------------------------------------------------------------
    float sumPitch = 0.0f;
    float sumRoll  = 0.0f;
    int validSamples = 0;
    for (int i = 0; i < 25; i++) {
        if (M5.Imu.update()) {
            auto imuData = M5.Imu.getImuData();
            float ax = imuData.accel.x;
            float ay = imuData.accel.y;
            float az = imuData.accel.z;
            float rawRoll  = atan2f(ay, az) * (180.0f / PI);
            float rawPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / PI);
            sumRoll  += rawRoll;
            sumPitch += rawPitch;
            validSamples++;
        }
        delay(10);
    }
    if (validSamples > 0) {
        pitchOffset = sumPitch / (float)validSamples;
        rollOffset  = sumRoll  / (float)validSamples;
        pitchDeg = pitchOffset;
        rollDeg  = rollOffset;
    }

    // Initialize Bluetooth Low Energy (BLE) for XCtrack
    ble_init();

    // Initialize GY-63 Sensor and Calibrate to base altitude (883m)
    bool baroOk = GY63.begin();
    GY63.calibrateToAltitude(baseAltitude);

    if (baroOk) {
        M5.Speaker.tone(1200, 100);
        delay(120);
        M5.Speaker.tone(1600, 150);
    } else {
        M5.Speaker.tone(600, 100);
        delay(110);
        M5.Speaker.tone(850, 150);
    }

    // Configure 4D Kalman Filter initialized at base altitude
    kalmanFilter4d_configure(100.0f, 1.0f, baseAltitude * 100.0f, 0.0f, 0.0f);
    AltitudeM = baseAltitude;
    ZeroAltReference = baseAltitude;
    lastKfTimeUs = micros();

    char bootMsg[36];
    snprintf(bootMsg, sizeof(bootMsg), "NIVELADO | %dm | BLE ATIVO", (int)baseAltitude);
    showToast(bootMsg, COL_GREEN, 2200);
    delay(800);
}

// ============================================================================
// MAIN FLIGHT LOOP
// ============================================================================
void loop() {
    M5.update();

    // 1. BUTTON CONTROLS (Standard StickC Plus 2 buttons)
    // ------------------------------------------------------------------------
    if (currentMode == MODE_CALIB_MENU) {
        // ====================================================================
        // CONTROLES QUANDO DENTRO DO MENU DE AJUSTE DE ALTITUDE
        // Ajuste estritamente fixo de 1 metro por toque
        // ====================================================================
        if (M5.BtnA.wasHold()) {
            // Segurar A (>1s): Salva na Flash, recalibra a pressão barométrica e volta ao PFD
            saveCalibratedAltitude(menuTargetAlt);
            GY63.calibrateToAltitude(menuTargetAlt);
            AltitudeM = menuTargetAlt;
            ZeroAltReference = menuTargetAlt;
            kalmanFilter4d_configure(100.0f, 1.0f, menuTargetAlt * 100.0f, 0.0f, 0.0f);
            currentMode = MODE_PFD;

            M5.Speaker.tone(1000, 100);
            delay(110);
            M5.Speaker.tone(1500, 150);

            char msg[36];
            snprintf(msg, sizeof(msg), "ALT SALVA: %dm", (int)menuTargetAlt);
            showToast(msg, COL_GREEN, 2000);
        } else if (M5.BtnA.wasClicked()) {
            // Toque em A: Aumenta altitude em exatamente 1 metro (+1m)
            menuTargetAlt += 1.0f;
            M5.Speaker.tone(1200, 30);
        }

        if (M5.BtnB.wasClicked()) {
            // Toque em B: Diminui altitude em exatamente 1 metro (-1m)
            menuTargetAlt -= 1.0f;
            if (menuTargetAlt < -500.0f) menuTargetAlt = -500.0f;
            M5.Speaker.tone(900, 30);
        }

        if (M5.BtnPWR.wasClicked()) {
            // Cancelar sem salvar e voltar ao voo
            currentMode = MODE_PFD;
            showToast("AJUSTE CANCELADO", COL_ORANGE, 1500);
            M5.Speaker.tone(600, 80);
        }

    } else {
        // ====================================================================
        // CONTROLES EM MODO NORMAL DE VOO (PFD / HUD / DIAL)
        // ====================================================================
        // Botão A (Frontal M5 - GPIO 37)
        if (M5.BtnA.wasHold()) {
            // Segurar > 1s: Nivelar Horizonte na posição atual do braço/relógio
            pitchOffset = pitchDeg;
            rollOffset  = rollDeg;
            M5.Speaker.tone(1000, 80);
            delay(90);
            M5.Speaker.tone(1500, 120);
            showToast("HORIZONTE NIVELADO", COL_YELLOW, 1800);
            Serial.printf("[CALIB] Nivel zero: P=%.1f, R=%.1f\r\n", pitchOffset, rollOffset);
        } else if (M5.BtnA.wasDoubleClicked()) {
            // Duplo Clique: Alterna Modo Demo <-> Normal
            toggleDemoMode();
        } else if (M5.BtnA.wasClicked()) {
            // Toque Curto: Zera altitude relativa de decolagem (AGL = 0m)
            ZeroAltReference = AltitudeM;
            M5.Speaker.tone(880, 80);
            showToast("AGL ZERADO (0m)", COL_CYAN, 1600);
            Serial.printf("[ALT] AGL zerado: Referencia = %.1fm\r\n", ZeroAltReference);
        }

        // Botão B (Lateral Inferior - GPIO 39)
        if (M5.BtnB.wasHold()) {
            // Segurar > 1s: Alterna entre os modos de exibição
            currentMode = (DisplayMode)((currentMode + 1) % MODE_COUNT);
            M5.Speaker.tone(1200, 100);
            if (currentMode == MODE_CALIB_MENU) {
                menuTargetAlt = roundf(AltitudeM); // Inicia a edição na altitude inteira atual
                showToast("MENU AJUSTE ALTITUDE", COL_CYAN, 1800);
            } else {
                const char* modeNames[] = {"MODO 0: PFD COCKPIT", "MODO 1: HORIZONTE HUD", "MODO 2: ALTIMETRO DIAL"};
                showToast(modeNames[currentMode], COL_WHITE, 1800);
            }
            Serial.printf("[MODE] Switched to Mode %d\r\n", currentMode);
        } else if (M5.BtnB.wasDoubleClicked()) {
            // Duplo Clique: Alterna Modo Demo <-> Normal (Atalho alternativo)
            toggleDemoMode();
        } else if (M5.BtnB.wasClicked()) {
            // Toque Curto: LIGA / DESLIGA o áudio do variômetro (Mute / Unmute)
            isAudioMuted = !isAudioMuted;
            if (isAudioMuted) {
                M5.Speaker.stop();
                M5.Speaker.tone(350, 150);
                showToast("VARIO AUDIO: DESLIGADO", COL_RED, 1500);
            } else {
                M5.Speaker.tone(1000, 80);
                delay(90);
                M5.Speaker.tone(1400, 100);
                showToast("VARIO AUDIO: LIGADO", COL_GREEN, 1500);
            }
            Serial.printf("[AUDIO] Mute = %d\r\n", isAudioMuted);
        }

        // Botão C / Power (Lateral Superior - GPIO 35)
        if (M5.BtnPWR.wasClicked()) {
            toggleDemoMode();
        }
    }

    // 2. READ IMU FOR ARTIFICIAL HORIZON (PITCH & ROLL)
    if (M5.Imu.update()) {
        auto imuData = M5.Imu.getImuData();
        float ax = imuData.accel.x;
        float ay = imuData.accel.y;
        float az = imuData.accel.z;

        float rawRoll  = atan2f(ay, az) * (180.0f / PI);
        float rawPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / PI);

        pitchDeg = (pitchDeg * 0.85f) + (rawPitch * 0.15f);
        rollDeg  = (rollDeg  * 0.85f) + (rawRoll  * 0.15f);
    }

    // 3. READ BAROMETRIC PRESSURE & KALMAN FILTER UPDATE
    float sampleAltM = 0.0f;
    if (GY63.update(sampleAltM)) {
        uint32_t nowUs = micros();
        float dtSec = (nowUs > lastKfTimeUs) ? (float)(nowUs - lastKfTimeUs) / 1000000.0f : 0.02f;
        lastKfTimeUs = nowUs;

        kalmanFilter4d_predict(dtSec);
        float kfAltCm = 0.0f;
        float kfVsiCps = 0.0f;
        kalmanFilter4d_update(sampleAltM * 100.0f, 0.0f, &kfAltCm, &kfVsiCps);

        AltitudeM    = kfAltCm / 100.0f;
        ClimbrateCps = kfVsiCps;
        ClimbrateMs  = kfVsiCps / 100.0f;

        // Variometer deadband: perfectly silent and zeroed at rest on table
        if (fabsf(ClimbrateMs) < 0.20f) {
            ClimbrateMs = 0.0f;
            ClimbrateCps = 0.0f;
        }
    } else if (!GY63.isSimulated && !GY63.isDetected) {
        // Sensor disconnected and not demo: keep stationary and completely silent
        AltitudeM = cachedCalibratedAltitude;
        ClimbrateCps = 0.0f;
        ClimbrateMs = 0.0f;
    }

    // 4. TRANSMIT BLE LK8EX1 SENTENCES TO XCTRACK @ 10Hz (100ms)
    if (millis() - lastBleTransmitMs >= 100) {
        lastBleTransmitMs = millis();
        float batVolt = (float)M5.Power.getBatteryVoltage() / 1000.0f;
        if (batVolt < 3.0f) batVolt = 3.7f;
        ble_transmit_LK8EX1((int32_t)GY63.rawPressurePa, (int32_t)AltitudeM, (int32_t)ClimbrateCps, (int32_t)GY63.rawTemperatureC, batVolt);
    }

    // 5. AUDIO VARIOMETER TICK
    handleAudioVario(ClimbrateMs);

    // 6. BATTERY LEVEL
    static uint32_t lastBatRead = 0;
    if (millis() - lastBatRead > 5000) {
        lastBatRead = millis();
        batteryPct = M5.Power.getBatteryLevel();
        if (batteryPct < 0) batteryPct = 0;
        if (batteryPct > 100) batteryPct = 100;
    }

    // 7. RENDER DISPLAY
    canvas.fillScreen(COL_PANEL_BG);

    switch (currentMode) {
        case MODE_PFD:
        default:
            drawArtificialHorizon(82, 67, 56);
            drawAviationAltimeterAndVsi(166);
            drawStatusHeader();
            break;

        case MODE_FULL_HORIZON:
            drawFullscreenHorizon();
            break;

        case MODE_ALTIMETER_DIAL:
            drawClassicAltimeterDial();
            break;

        case MODE_CALIB_MENU:
            drawCalibrationMenu();
            break;
    }

    // Draw notification toast on top if active
    drawToast();

    // Blit to screen via hardware SPI DMA (flicker-free)
    canvas.pushSprite(0, 0);

    delay(20);
}
