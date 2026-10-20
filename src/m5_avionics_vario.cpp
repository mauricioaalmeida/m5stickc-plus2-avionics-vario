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

// Base initial calibration altitude requested for home ramp: 853m
#define CALIBRATED_BASE_ALT_M  853.0f

// ============================================================================
// PERSISTENT STORAGE (NVS FLASH) FOR BAROMETER QNH CALIBRATION
// Standard Sea-Level Pressure is 101325.0 Pa (1013.25 hPa)
// Once calibrated initially (or via Mode 3 Menu), the calculated QNH is saved.
// On normal boots, the variometer uses this QNH to calculate true altitude:
// Altitude = 44330 * (1 - (P / QNH)^0.190295)
// So when taken to other takeoff locations, it shows the ACTUAL altitude of that place!
// ============================================================================
static Preferences prefs;
static float savedQnhSeaLevelPa = 101325.0f;
static bool  hasSavedQnh = false;

static float loadSavedQNH() {
    prefs.begin("avionics", false);
    if (prefs.isKey("qnhPa")) {
        savedQnhSeaLevelPa = prefs.getFloat("qnhPa", 101325.0f);
        if (savedQnhSeaLevelPa >= 85000.0f && savedQnhSeaLevelPa <= 115000.0f) {
            hasSavedQnh = true;
        } else {
            savedQnhSeaLevelPa = 101325.0f;
            hasSavedQnh = false;
        }
    } else {
        hasSavedQnh = false;
        savedQnhSeaLevelPa = 101325.0f;
    }
    prefs.end();
    return savedQnhSeaLevelPa;
}

static void saveQNH(float qnhPa) {
    if (qnhPa < 85000.0f || qnhPa > 115000.0f) return;
    savedQnhSeaLevelPa = qnhPa;
    hasSavedQnh = true;
    prefs.begin("avionics", false);
    prefs.putFloat("qnhPa", qnhPa);
    prefs.end();
    Serial.printf("[NVS] QNH calibrado e salvo na Flash: %.1f hPa\r\n", qnhPa / 100.0f);
}

// ============================================================================
// BLUETOOTH LOW ENERGY (BLE) PARA XCTRACK, XCSOAR & APPS DE VOO
// Suporta:
// 1. Serviço XC Tracer (UUID 0xFFE0 / Char 0xFFE1) - Padrão universal do XCTrack
// 2. Serviço Nordic UART (NUS 6E400001 / TX 6E400003) - Padrão DIY Vario
// 3. Serviço Environmental Sensing (0x181A / Baro 0x2A6D) - Pressão padrão BLE SIG
// 4. Serviço Bateria (0x180F / Bat 0x2A19) - Status de bateria padrão BLE SIG
// Transmissão fragmentada em blocos seguros (20 bytes) para compatibilidade MTU total.
// ============================================================================
#define BLE_UUID_XCTRACER_SERVICE        "0000ffe0-0000-1000-8000-00805f9b34fb"
#define BLE_UUID_XCTRACER_DATA_CHAR      "0000ffe1-0000-1000-8000-00805f9b34fb"

#define BLE_UUID_NUS_SERVICE             "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_UUID_NUS_CHAR_TX             "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_UUID_NUS_CHAR_RX             "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

#define BLE_UUID_ESS_SERVICE             0x181A
#define BLE_UUID_ESS_PRESSURE_CHAR       0x2A6D
#define BLE_UUID_ESS_TEMP_CHAR           0x2A6E
#define BLE_UUID_BATTERY_SERVICE         0x180F
#define BLE_UUID_BATTERY_LEVEL_CHAR      0x2A19

static NimBLEServer* pBleServer = nullptr;
static NimBLECharacteristic* pXctChar = nullptr;
static NimBLECharacteristic* pNusTxChar = nullptr;
static NimBLECharacteristic* pNusRxChar = nullptr;
static NimBLECharacteristic* pBlePressChar = nullptr;
static NimBLECharacteristic* pBleTempChar = nullptr;
static NimBLECharacteristic* pBleBatChar = nullptr;

static bool bleConnected = false;
static uint32_t lastBleTransmitMs = 0;
static int batteryPct = 100;
static float batteryVolt = 4.0f;
static void showToast(const char* msg, uint16_t color = COL_YELLOW, uint32_t durationMs = 1600);

static inline bool isBleConnected() {
    return (pBleServer != nullptr && pBleServer->getConnectedCount() > 0);
}

static void updateBatteryStatus() {
    int lvl = M5.Power.getBatteryLevel();
    int16_t mv = M5.Power.getBatteryVoltage();
    bool isChg = M5.Power.isCharging();

    if (mv > 0) {
        batteryVolt = (float)mv / 1000.0f;
    }
    // Clamp voltage reading if charging from USB rail
    if (batteryVolt > 4.25f || isChg) {
        if (batteryVolt > 4.25f) batteryVolt = 4.20f;
    }

    if (lvl < 0 || lvl > 100) {
        if (mv > 3200 && mv <= 4250) {
            lvl = (int)((mv - 3300) * 100 / (4150 - 3350));
        } else if (isChg) {
            lvl = 100;
        } else {
            lvl = 85;
        }
    }
    if (lvl < 0) lvl = 0;
    if (lvl > 100) lvl = 100;
    batteryPct = lvl;
}

class BleServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        bleConnected = true;
        // Intervalo de conexão de 15ms a 30ms para resposta rápida do variômetro com baixo consumo
        pServer->updateConnParams(desc->conn_handle, 12, 24, 0, 400);
        Serial.printf("[BLE] XCTrack conectado com sucesso via BLE! ConnID=%u\r\n", desc->conn_handle);
        showToast("BLE XCTRACK CONECTADO", COL_GREEN, 2000);
    }
    void onDisconnect(NimBLEServer* pServer) override {
        bleConnected = false;
        Serial.println("[BLE] Cliente desconectado. Reiniciando advertising...");
        showToast("BLE DESCONECTADO", COL_RED, 2000);
        NimBLEDevice::startAdvertising();
    }
};

static uint8_t ble_nmea_checksum(const char *szNMEA) {
    const char* sz = &szNMEA[1]; // skip leading '$'
    uint8_t cksum = 0;
    while (*sz != 0 && *sz != '*') {
        cksum ^= (uint8_t)*sz;
        sz++;
    }
    return cksum;
}

// Transmissão de pacotes fragmentados em blocos de até 20 bytes para evitar truncation por MTU default
static void ble_send_chunked(NimBLECharacteristic* pChar, const char* str) {
    if (pChar == nullptr) return;
    int len = strlen(str);
    int offset = 0;
    while (offset < len) {
        int chunkLen = len - offset;
        if (chunkLen > 20) chunkLen = 20; // 20 bytes cabe em qualquer MTU ATT (default 23 - 3)
        pChar->setValue((const uint8_t*)(str + offset), chunkLen);
        pChar->notify();
        offset += chunkLen;
        if (offset < len) {
            delay(4); // Pequeno intervalo para esvaziar a fila do rádio BLE
        }
    }
}

static void ble_init() {
    NimBLEDevice::init("M5-VARIO");
    // Potência +3dBm: alcance ideal para cockpit (5-10m) economizando muita bateria
    NimBLEDevice::setPower(ESP_PWR_LVL_P3);
    NimBLEDevice::setMTU(185);

    pBleServer = NimBLEDevice::createServer();
    pBleServer->setCallbacks(new BleServerCallbacks());

    // 1. Serviço XC Tracer (0xFFE0 / 0xFFE1) - Driver nativo padrão ouro no XCTrack
    NimBLEService* pXctService = pBleServer->createService(NimBLEUUID(BLE_UUID_XCTRACER_SERVICE));
    pXctChar = pXctService->createCharacteristic(
        NimBLEUUID(BLE_UUID_XCTRACER_DATA_CHAR),
        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pXctService->start();

    // 2. Serviço Nordic UART (NUS 6E400001 / TX 6E400003)
    NimBLEService* pNusService = pBleServer->createService(NimBLEUUID(BLE_UUID_NUS_SERVICE));
    pNusTxChar = pNusService->createCharacteristic(
        NimBLEUUID(BLE_UUID_NUS_CHAR_TX),
        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ
    );
    pNusRxChar = pNusService->createCharacteristic(
        NimBLEUUID(BLE_UUID_NUS_CHAR_RX),
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pNusService->start();

    // 3. Serviço Environmental Sensing (0x181A) - Barômetro e Temperatura padrão BLE SIG
    NimBLEService* pEnvService = pBleServer->createService(NimBLEUUID((uint16_t)BLE_UUID_ESS_SERVICE));
    pBlePressChar = pEnvService->createCharacteristic(
        NimBLEUUID((uint16_t)BLE_UUID_ESS_PRESSURE_CHAR),
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    pBleTempChar = pEnvService->createCharacteristic(
        NimBLEUUID((uint16_t)BLE_UUID_ESS_TEMP_CHAR),
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    pEnvService->start();

    // 4. Serviço de Bateria (0x180F / 0x2A19) - Status de bateria padrão BLE SIG
    NimBLEService* pBatService = pBleServer->createService(NimBLEUUID((uint16_t)BLE_UUID_BATTERY_SERVICE));
    pBleBatChar = pBatService->createCharacteristic(
        NimBLEUUID((uint16_t)BLE_UUID_BATTERY_LEVEL_CHAR),
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    pBatService->start();

    // 5. Configuração do Advertising BLE (UUIDs 16-bit cabem com folga nos 31 bytes)
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setName("M5-VARIO");
    pAdvertising->addServiceUUID(NimBLEUUID((uint16_t)0xFFE0));
    pAdvertising->addServiceUUID(NimBLEUUID((uint16_t)0x181A));
    pAdvertising->addServiceUUID(NimBLEUUID((uint16_t)0x180F));
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // 7.5ms
    pAdvertising->setMaxPreferred(0x12); // 22.5ms
    pAdvertising->start();

    Serial.println("[BLE] Servidor BLE 'M5-VARIO' Iniciado com Sucesso!");
    Serial.println("  -> Perfil XC Tracer (0xFFE0 / 0xFFE1)");
    Serial.println("  -> Perfil Nordic UART (NUS 6E400001 / TX 6E400003)");
    Serial.println("  -> Perfil Barômetro SIG (0x181A / 0x2A6D)");
    Serial.println("  -> Perfil Bateria SIG (0x180F / 0x2A19)");
}

static void ble_transmit_telemetry(int32_t rawPressurePa, float altM, float vsiMs, int32_t tempC, float batVoltage) {
    if (!isBleConnected()) return;

    if (rawPressurePa < 30000 || rawPressurePa > 120000) rawPressurePa = 101325;
    if (batVoltage < 3.0f || batVoltage > 4.5f) batVoltage = 4.0f;

    int32_t cps = (int32_t)roundf(vsiMs * 100.0f);
    if (cps < -3000) cps = -3000;
    if (cps > 3000) cps = 3000;

    // 1. Sentença $LK8EX1 a 10 Hz com PRESSÃO REAL em Pascal (ex: 92430 = 924.30 hPa)
    char szmsg[64];
    snprintf(szmsg, sizeof(szmsg), "$LK8EX1,%d,%d,%d,%d,%.1f*",
             (int)rawPressurePa, (int)roundf(altM), (int)cps, (int)tempC, batVoltage);
    uint8_t cksum = ble_nmea_checksum(szmsg);
    char szcksum[8];
    snprintf(szcksum, sizeof(szcksum), "%02X\r\n", cksum);
    strcat(szmsg, szcksum);

    // Envia tanto no canal XC Tracer (0xFFE1) quanto no Nordic UART (6E400003)
    ble_send_chunked(pXctChar, szmsg);
    ble_send_chunked(pNusTxChar, szmsg);

    // 2. Sentença $XCTRC a 2 Hz (padrão XC Tracer para altitude/climb)
    static uint32_t lastXctrcMs = 0;
    if (millis() - lastXctrcMs >= 500) {
        lastXctrcMs = millis();
        char szXctrc[128];
        float pressHpa = (float)rawPressurePa / 100.0f;
        float vsiClimb = (float)cps / 100.0f;
        snprintf(szXctrc, sizeof(szXctrc),
                 "$XCTRC,2026,10,20,12,0,0,0,0.000000,0.000000,%.2f,0.00,0.0,%.2f,,,,%.2f,%d*",
                 altM, vsiClimb, pressHpa, batteryPct);
        uint8_t ckXctrc = ble_nmea_checksum(szXctrc);
        char szckXctrc[8];
        snprintf(szckXctrc, sizeof(szckXctrc), "%02X\r\n", ckXctrc);
        strcat(szXctrc, szckXctrc);

        ble_send_chunked(pXctChar, szXctrc);
        ble_send_chunked(pNusTxChar, szXctrc);

        // Atualiza características BLE SIG padrão (Pressão 0x2A6D em 0.1 Pa e Bateria 0x2A19 em %)
        if (pBlePressChar != nullptr) {
            uint32_t pressVal = (uint32_t)(rawPressurePa * 10);
            pBlePressChar->setValue((uint8_t*)&pressVal, 4);
            pBlePressChar->notify();
        }
        if (pBleBatChar != nullptr) {
            uint8_t batLvl = (uint8_t)batteryPct;
            pBleBatChar->setValue(&batLvl, 1);
            pBleBatChar->notify();
        }
    }

    static uint32_t lastDbg = 0;
    if (millis() - lastDbg > 2000) {
        lastDbg = millis();
        Serial.printf("[BLE-TX] XCTrack Conectado | P=%.1fhPa Alt=%dm Bat=%d%% (%.2fV) | LK8EX1+XCTRC\r\n",
                      rawPressurePa / 100.0f, (int)roundf(altM), batteryPct, batVoltage);
    }
}

// ============================================================================
// ON-SCREEN NOTIFICATION TOAST
// ============================================================================
static char toastMsg[36] = "";
static uint16_t toastColor = COL_YELLOW;
static uint32_t toastUntilMs = 0;

static void showToast(const char* msg, uint16_t color, uint32_t durationMs) {
    strncpy(toastMsg, msg, sizeof(toastMsg) - 1);
    toastMsg[sizeof(toastMsg) - 1] = '\0';
    toastColor = color;
    toastUntilMs = millis() + durationMs;
}

// ============================================================================
// GY-63 (MS5611) I2C DRIVER (High-Precision 100Hz OSR4096 Continuous Baro)
// ============================================================================
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_CONV_D1_4096 0x48
#define MS5611_CMD_CONV_D2_4096 0x58
#define MS5611_CMD_ADC_READ     0x00
#define MS5611_CMD_PROM_READ    0xA0

#define PRESS_FILTER_SIZE 4

class MS5611_I2C {
public:
    uint8_t  addr = 0x77;
    uint16_t C[8];
    bool     isDetected = false;
    bool     isSimulated = false; // NEVER starts as true automatically!
    float    seaLevelPressurePa = 101325.0f; // Calibrated QNH reference

    enum State { IDLE, WAIT_D1, WAIT_D2 } state = IDLE;
    uint32_t lastConvStart = 0;
    uint32_t D1 = 0;
    uint32_t D2 = 0;

    int64_t dT = 0;
    int64_t TEMP = 2000;
    int64_t OFF = 0;
    int64_t SENS = 0;
    uint32_t sampleCounter = 0;

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

    int activeSda = 32;
    int activeScl = 33;

    bool begin() {
        isDetected = false;
        isSimulated = false;
        state = IDLE;
        sampleCounter = 0;

        struct PinPair {
            int sda;
            int scl;
            const char* name;
        };

        const PinPair pairs[] = {
            { 32, 33, "Grove Port A Padrao (SDA=32, SCL=33)" },
            { 33, 32, "Grove Port A Invertido (SDA=33, SCL=32)" },
            { 26, 0,  "Top Header G26/G0 (SDA=26, SCL=0)" },
            { 0, 26,  "Top Header G0/G26 (SDA=0, SCL=26)" },
            { 25, 26, "Top Header G25/G26 (SDA=25, SCL=26)" },
            { 26, 25, "Top Header G26/G25 (SDA=26, SCL=25)" }
        };

        const uint8_t addrs[] = { 0x77, 0x76 };

        for (const auto& p : pairs) {
            activeSda = p.sda;
            activeScl = p.scl;
            pinMode(activeSda, INPUT_PULLUP);
            pinMode(activeScl, INPUT_PULLUP);
            delay(5);
            int sdaLvl = digitalRead(activeSda);
            int sclLvl = digitalRead(activeScl);
            Serial.printf("[PIN DIAG] %s -> SDA=%d, SCL=%d\r\n", p.name, sdaLvl, sclLvl);

            if (sdaLvl == 0 && sclLvl == 0) {
                // Both lines held low, cannot communicate on these pins
                continue;
            }

            M5.Ex_I2C.begin(I2C_NUM_0, activeSda, activeScl);
            delay(10);

            // Log any device found on these pins
            for (uint8_t a = 8; a < 120; a++) {
                if (M5.Ex_I2C.scanID(a, 100000)) {
                    Serial.printf("[I2C SCAN] Dispositivo encontrado em 0x%02X nos pinos %s\r\n", a, p.name);
                }
            }

            for (uint8_t a : addrs) {
                if (M5.Ex_I2C.scanID(a, 100000)) {
                    addr = a;

                    // Send MS5611 Reset
                    if (M5.Ex_I2C.start(addr, false, 100000)) {
                        M5.Ex_I2C.write(MS5611_CMD_RESET);
                        M5.Ex_I2C.stop();
                    }
                    delay(30);

                    readProm();

                    if (C[1] != 0 && C[1] != 0xFFFF && C[2] != 0) {
                        isDetected = true;
                        Serial.printf("[GY-63] MS5611 DETECTADO COM SUCESSO em 0x%02X via %s [M5.Ex_I2C]!\r\n",
                                      addr, p.name);

                        // Reset moving average filter
                        for (int i = 0; i < PRESS_FILTER_SIZE; i++) pressHistory[i] = 0;
                        pressHistIdx = 0;
                        pressHistFilled = false;

                        // Direct synchronous settling of D2 (temperature) and D1 (pressure)
                        for (int k = 0; k < 8; k++) {
                            triggerD2();
                            delay(15);
                            D2 = readADC();
                            updateTempCoefficients();

                            triggerD1();
                            delay(15);
                            D1 = readADC();
                            calculatePressure();
                        }

                        state = IDLE;
                        triggerD1();

                        Serial.printf("[GY-63] Leitura inicial estabilizada: D1=%u, D2=%u -> Pressao: %.1f hPa (Alt: %.1fm)\r\n",
                                      D1, D2, rawPressurePa / 100.0f, rawAltitudeM);
                        return true;
                    }
                }
            }
        }

        // Also check internal In_I2C bus just in case
        for (uint8_t a : addrs) {
            if (M5.In_I2C.scanID(a, 100000)) {
                Serial.printf("[GY-63] MS5611 encontrado no In_I2C interno em 0x%02X!\r\n", a);
            }
        }

        // Sensor not detected: do NOT auto-enable demo mode
        isSimulated = false;
        rawAltitudeM = CALIBRATED_BASE_ALT_M;
        simAlt = CALIBRATED_BASE_ALT_M;
        Serial.println("[GY-63] Sensor nao detectado nos pinos testados. Aguardando conexao...");
        return false;
    }

    void readProm() {
        for (uint8_t i = 0; i < 8; i++) {
            C[i] = 0;
            delay(5);
            uint8_t reg = MS5611_CMD_PROM_READ + (i * 2);
            uint8_t buf[2] = {0, 0};
            if (M5.Ex_I2C.readRegister(addr, reg, buf, 2, 100000)) {
                C[i] = ((uint16_t)buf[0] << 8) | buf[1];
            }
        }
        Serial.printf("[GY-63 PROM] C1=%u C2=%u C3=%u C4=%u C5=%u C6=%u\r\n",
                      C[1], C[2], C[3], C[4], C[5], C[6]);
    }

    bool isCalibrated = false;

    void calibrateToAltitude(float targetAltM) {
        if (rawPressurePa > 40000.0f && rawPressurePa < 115000.0f) {
            seaLevelPressurePa = rawPressurePa / powf(1.0f - (targetAltM / 44330.0f), 5.255f);
            saveQNH(seaLevelPressurePa);
            rawAltitudeM = targetAltM;
            for (int i = 0; i < PRESS_FILTER_SIZE; i++) {
                pressHistory[i] = rawPressurePa;
            }
            pressHistIdx = 0;
            pressHistFilled = true;
            isCalibrated = true;
            Serial.printf("[CALIBRATE] TargetAlt=%.1fm | SensorP=%.1f hPa -> QNH Salvo=%.1f hPa\r\n",
                          targetAltM, rawPressurePa / 100.0f, seaLevelPressurePa / 100.0f);
        } else {
            seaLevelPressurePa = 101325.0f;
            rawAltitudeM = targetAltM;
            simAlt = targetAltM;
            isCalibrated = false;
        }
    }

    bool triggerD1() {
        if (!isDetected) return false;
        if (M5.Ex_I2C.start(addr, false, 100000)) {
            M5.Ex_I2C.write(MS5611_CMD_CONV_D1_4096);
            M5.Ex_I2C.stop();
            lastConvStart = millis();
            state = WAIT_D1;
            return true;
        }
        return false;
    }

    bool triggerD2() {
        if (!isDetected) return false;
        if (M5.Ex_I2C.start(addr, false, 100000)) {
            M5.Ex_I2C.write(MS5611_CMD_CONV_D2_4096);
            M5.Ex_I2C.stop();
            lastConvStart = millis();
            state = WAIT_D2;
            return true;
        }
        return false;
    }

    uint32_t readADC() {
        uint8_t buf[3] = {0, 0, 0};
        if (M5.Ex_I2C.readRegister(addr, MS5611_CMD_ADC_READ, buf, 3, 100000)) {
            return ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
        }
        // Fallback with explicit write command then read
        if (M5.Ex_I2C.start(addr, false, 100000)) {
            M5.Ex_I2C.write(MS5611_CMD_ADC_READ);
            M5.Ex_I2C.stop();
            delayMicroseconds(50);
            if (M5.Ex_I2C.start(addr, true, 100000)) {
                if (M5.Ex_I2C.read(buf, 3, true)) {
                    M5.Ex_I2C.stop();
                    return ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
                }
                M5.Ex_I2C.stop();
            }
        }
        return 0;
    }

    void updateTempCoefficients() {
        if (D2 < 100000) return;
        static int64_t smoothedD2 = 0;
        if (smoothedD2 == 0) smoothedD2 = (int64_t)D2;
        else smoothedD2 = (smoothedD2 * 7 + (int64_t)D2) >> 3; // Smooth temperature to avoid step jumps

        dT = smoothedD2 - ((int64_t)C[5] << 8);
        TEMP = 2000 + ((dT * (int64_t)C[6]) >> 23);

        OFF  = ((int64_t)C[2] << 16) + (((int64_t)C[4] * dT) >> 7);
        SENS = ((int64_t)C[1] << 15) + (((int64_t)C[3] * dT) >> 8);

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
        rawTemperatureC = (float)TEMP / 100.0f;
    }

    void calculatePressure() {
        if (D1 < 100000 || SENS == 0) return;

        // High precision floating point calculation without integer bitshift truncation
        float pPa = (((float)((int64_t)D1 * SENS)) / 2097152.0f - (float)OFF) / 32768.0f;

        if (pPa < 40000.0f || pPa > 120000.0f) return;

        // 4-sample moving average filter (~56ms group delay) to eliminate MS5611 ADC noise
        pressHistory[pressHistIdx] = pPa;
        pressHistIdx = (pressHistIdx + 1) % PRESS_FILTER_SIZE;
        if (pressHistIdx == 0) pressHistFilled = true;

        float sumP = 0.0f;
        int count = pressHistFilled ? PRESS_FILTER_SIZE : pressHistIdx;
        if (count == 0) count = 1;
        for (int i = 0; i < count; i++) {
            sumP += pressHistory[i];
        }
        rawPressurePa = sumP / (float)count;

        // Altitude with calibrated seaLevelPressurePa
        float alt = 44330.0f * (1.0f - powf(rawPressurePa / seaLevelPressurePa, 0.1902949f));
        if (!isnan(alt) && !isinf(alt)) {
            rawAltitudeM = alt;
        }

        static uint32_t lastCalcLog = 0;
        if (millis() - lastCalcLog > 2000) {
            lastCalcLog = millis();
            Serial.printf("[GY-63 LIVE] P=%.1f hPa | Temp=%.1f C | Alt=%.1fm | QNH=%.1f hPa\r\n",
                          rawPressurePa / 100.0f, rawTemperatureC, rawAltitudeM, seaLevelPressurePa / 100.0f);
        }
    }

    bool update(float &altitudeOut) {
        if (isSimulated) {
            simPhase += 0.035f;
            float climb = sinf(simPhase) * 1.5f;
            simAlt += climb * 0.015f;
            rawAltitudeM = simAlt;
            rawPressurePa = seaLevelPressurePa * powf(1.0f - (simAlt / 44330.0f), 5.255f);
            altitudeOut = rawAltitudeM;
            return true;
        }

        if (!isDetected) {
            altitudeOut = rawAltitudeM;

            static uint32_t lastProbeMs = 0;
            if (millis() - lastProbeMs > 2500) {
                lastProbeMs = millis();
                if (begin()) {
                    showToast("GY-63 CONECTADO!", COL_GREEN, 2000);
                }
            }
            return false;
        }

        uint32_t now = millis();
        if (state == IDLE) {
            triggerD1();
            return false;
        }

        // OSR 4096 requires 9.04ms max conversion time. 14ms wait ensures 100% completion without NACK/timeout.
        if (state == WAIT_D1 && (now - lastConvStart >= 14)) {
            uint32_t adc = readADC();
            if (adc > 100000) {
                D1 = adc;
                calculatePressure();
                if (!hasSavedQnh && rawPressurePa > 40000.0f && rawPressurePa < 115000.0f) {
                    calibrateToAltitude(CALIBRATED_BASE_ALT_M);
                }
                altitudeOut = rawAltitudeM;
                sampleCounter++;

                // Interleave 1 temperature reading every 100 pressure readings (~1.5s)
                if (sampleCounter % 100 == 0) {
                    triggerD2();
                } else {
                    triggerD1(); // Next pressure reading immediately (~70Hz)
                }
                return true;
            } else {
                triggerD1();
            }
        } else if (state == WAIT_D2 && (now - lastConvStart >= 14)) {
            uint32_t adc = readADC();
            if (adc > 100000) {
                D2 = adc;
                updateTempCoefficients();
            }
            triggerD1();
        }
        return false;
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

// Interactive Menu Variables
static float menuTargetAlt = CALIBRATED_BASE_ALT_M;


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
// 2D KINEMATIC KALMAN FILTER FOR VARIOMETER
// Mathematically optimal estimation of altitude (cm) and climb/sink rate (cm/s).
// Eliminates all IMU tilt leakage, providing rock-solid stillness at rest
// and natural, responsive climb/sink detection without lag or overshoot.
// ============================================================================
class VarioKalman2D {
public:
    float z = 85300.0f; // Altitude in cm
    float v = 0.0f;     // Vertical climb rate in cm/s

    // Covariance matrix P
    float Pzz = 1500.0f;
    float Pzv = 0.0f;
    float Pvv = 1500.0f;

    // Process noise: vertical acceleration variance in cm^2/s^4
    // 1200 cm^2/s^4 provides instantaneous ~200ms response without lagging behind movements
    const float AccelVariance = 1200.0f;

    // Measurement noise: MS5611 filtered altitude variance in cm^2
    const float ZMeasVariance = 120.0f;

    void init(float initialAltM) {
        z = initialAltM * 100.0f;
        v = 0.0f;
        Pzz = 1500.0f;
        Pzv = 0.0f;
        Pvv = 1500.0f;
    }

    void predict(float dt) {
        if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

        z += v * dt;
        // Velocity damping: eliminates momentum overshoot and prevents runaway drift
        v *= 0.965f;

        float dt2 = dt * dt;
        float dt3 = dt2 * dt;
        float dt4 = dt2 * dt2;

        float qzz = AccelVariance * dt4 * 0.25f;
        float qzv = AccelVariance * dt3 * 0.5f;
        float qvv = AccelVariance * dt2;

        Pzz += (2.0f * dt * Pzv) + (dt2 * Pvv) + qzz;
        Pzv += (dt * Pvv) + qzv;
        Pvv += qvv;
    }

    void update(float measuredAltM) {
        float zm = measuredAltM * 100.0f;
        float z_err = zm - z;

        // Adaptive responsiveness: if moving swiftly, track faster
        float r_z = ZMeasVariance;
        float abs_err = fabsf(z_err);
        if (abs_err > 10.0f) {
            r_z /= (1.0f + 0.04f * (abs_err - 10.0f));
            if (r_z < 30.0f) r_z = 30.0f;
        }

        float S = Pzz + r_z;
        if (S < 1.0f) S = 1.0f;

        float Kz = Pzz / S;
        float Kv = Pzv / S;

        z += Kz * z_err;
        v += Kv * z_err;

        float prevPzz = Pzz;
        float prevPzv = Pzv;

        Pzz -= Kz * prevPzz;
        Pzv -= Kz * prevPzv;
        Pvv -= Kv * prevPzv;

        if (Pzz < 1.0f) Pzz = 1.0f;
        if (Pvv < 1.0f) Pvv = 1.0f;

        if (isnan(z) || isinf(z) || fabsf(z) > 1000000.0f) {
            z = zm;
            v = 0.0f;
            Pzz = 1500.0f;
            Pvv = 1500.0f;
            Pzv = 0.0f;
        }
    }

    float getAltitudeM() const {
        return z * 0.01f;
    }

    float getClimbrateMs() const {
        return v * 0.01f;
    }
};

static VarioKalman2D kfVario;

// ============================================================================
// AUDIO VARIOMETER WITH THERMAL PURR ("RONRONAR DE TÉRMICA")
// ============================================================================
static void handleAudioVario(float vsiMs) {
    if (isAudioMuted || currentMode == MODE_CALIB_MENU || (!GY63.isDetected && !GY63.isSimulated)) {
        M5.Speaker.stop();
        return;
    }

    // Deadband at rest: perfectly silent when stationary or within deadband (+/- 0.12 m/s)
    if (fabsf(vsiMs) < 0.12f) {
        M5.Speaker.stop();
        return;
    }

    static uint32_t lastBeepTick = 0;
    static bool isBeeping = false;
    uint32_t now = millis();

    // 1. CLIMB TONES (Subida Franca): >= +0.22 m/s (Fast, immediate response)
    if (vsiMs >= 0.22f) {
        int freq = (int)(600.0f + (vsiMs * 250.0f));
        if (freq > 2300) freq = 2300;

        int period = (int)(420.0f - (vsiMs * 75.0f));
        if (period < 75) period = 75;
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
    // 2. RONRONAR DE TÉRMICA (Thermal Sniffer / Buoyant Air Purr): +0.12 m/s to +0.22 m/s
    // Gentle low-pitch rhythmic chirp informing the pilot of buoyant air / thermal entry
    else if (vsiMs >= 0.12f && vsiMs < 0.22f) {
        const int purrFreq = 300;   // Soft low buzz
        const int purrPeriod = 140; // 140ms cycle
        const int purrOnTime = 16;  // Short 16ms pulse

        uint32_t elapsed = now - lastBeepTick;
        if (elapsed < (uint32_t)purrOnTime) {
            if (!isBeeping) {
                M5.Speaker.tone(purrFreq, purrOnTime);
                isBeeping = true;
            }
        } else if (elapsed < (uint32_t)purrPeriod) {
            if (isBeeping) {
                M5.Speaker.stop();
                isBeeping = false;
            }
        } else {
            lastBeepTick = now;
            M5.Speaker.tone(purrFreq, purrOnTime);
            isBeeping = true;
        }
    }
    // 3. SINK ALARM (Descendente Forte): <= -1.30 m/s
    else if (vsiMs <= -1.30f) {
        int sinkFreq = (int)(320.0f + (vsiMs * 30.0f));
        if (sinkFreq < 160) sinkFreq = 160;
        M5.Speaker.tone(sinkFreq, 80);
        isBeeping = true;
    }
    // 4. NORMAL GLIDE SINK (-1.30 m/s to -0.10 m/s): SILENT
    else {
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

    // VSI BAR: GREEN FOR CLIMB (ASCENDENTES), VIBRANT RED FOR SINK (DESCENDENTES)
    if (barHeight > 1) {
        // Ascendente: VERDE vibrante
        canvas.fillRect(vsiX + 2, centerY - barHeight, vsiW - 4, barHeight, COL_GREEN);
        canvas.drawRect(vsiX + 2, centerY - barHeight, vsiW - 4, barHeight, COL_WHITE);
    } else if (barHeight < -1) {
        // Descendente: VERMELHO vibrante com destaque total
        canvas.fillRect(vsiX + 2, centerY, vsiW - 4, -barHeight, COL_RED);
        canvas.drawRect(vsiX + 2, centerY, vsiW - 4, -barHeight, COL_WHITE);
    }

    // Top VSI numeric pill: GREEN for climb, RED for sink, DARK GRAY for flat
    uint16_t vsiPillColor = COL_DARK_GRAY;
    uint16_t vsiTextColor = COL_WHITE;
    if (vsi >= 0.08f) {
        vsiPillColor = COL_GREEN;
        vsiTextColor = COL_BLACK;
    } else if (vsi <= -0.08f) {
        vsiPillColor = COL_RED;
        vsiTextColor = COL_WHITE;
    }
    canvas.fillRect(vsiX, 0, vsiW, 14, vsiPillColor);
    canvas.setTextDatum(top_center);
    canvas.setTextColor(vsiTextColor, vsiPillColor);
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

    // 1. Top bar background
    canvas.fillRect(0, 0, 166, 14, COL_PANEL_BG);
    canvas.drawFastHLine(0, 14, 166, COL_DARK_GRAY);

    // Pitch & Roll (Top Left)
    canvas.setTextDatum(top_left);
    canvas.setTextColor(COL_WHITE, COL_PANEL_BG);
    char prBuf[24];
    snprintf(prBuf, sizeof(prBuf), "P:%+02d R:%+02d", (int)(pitchDeg - pitchOffset), (int)(rollDeg - rollOffset));
    canvas.drawString(prBuf, 2, 3);

    // Bluetooth Status (Top Center)
    canvas.setTextDatum(top_center);
    if (isBleConnected()) {
        canvas.setTextColor(COL_GREEN, COL_PANEL_BG);
        canvas.drawString("BLE:OK", 92, 3);
    } else {
        canvas.setTextColor(COL_CYAN, COL_PANEL_BG);
        canvas.drawString("BLE:DISC", 92, 3);
    }

    // Baro Sensor Status (Top Right)
    canvas.setTextDatum(top_right);
    if (GY63.isSimulated) {
        canvas.setTextColor(COL_ORANGE, COL_PANEL_BG);
        canvas.drawString("DEMO", 163, 3);
    } else if (GY63.isDetected) {
        canvas.setTextColor(COL_GREEN, COL_PANEL_BG);
        canvas.drawString("GY-63", 163, 3);
    } else {
        canvas.setTextColor(COL_DARK_GRAY, COL_PANEL_BG);
        canvas.drawString("S/BARO", 163, 3);
    }

    // 2. Bottom status bar (below artificial horizon: x=0..166, y=121..134)
    canvas.fillRect(0, 121, 166, 14, COL_PANEL_BG);
    canvas.drawFastHLine(0, 120, 166, COL_DARK_GRAY);

    bool isChg = M5.Power.isCharging();
    uint16_t batCol = isChg ? COL_CYAN : ((batteryPct >= 40) ? COL_GREEN : ((batteryPct >= 20) ? COL_YELLOW : COL_RED));

    // Battery icon outline at x=4, y=123 (width 16, height 9)
    canvas.drawRect(4, 123, 16, 9, COL_WHITE);
    canvas.fillRect(20, 125, 2, 5, COL_WHITE); // battery terminal pip

    int innerW = (batteryPct * 12) / 100;
    if (innerW > 0) {
        canvas.fillRect(6, 125, innerW, 5, batCol);
    }

    // Battery percentage and voltage text
    canvas.setTextDatum(middle_left);
    canvas.setTextColor(batCol, COL_PANEL_BG);
    char batBuf[36];
    if (isChg) {
        snprintf(batBuf, sizeof(batBuf), "CHG: %d%% (%.2fV)", batteryPct, batteryVolt);
    } else {
        snprintf(batBuf, sizeof(batBuf), "BAT: %d%% (%.2fV)", batteryPct, batteryVolt);
    }
    canvas.drawString(batBuf, 26, 128);
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
    uint16_t hudVsiCol = (ClimbrateMs >= 0.08f) ? COL_GREEN : (ClimbrateMs <= -0.08f ? COL_RED : COL_WHITE);
    canvas.setTextColor(hudVsiCol, COL_BLACK);
    canvas.printf("%+.1fm/s", ClimbrateMs);

    // Bottom Status Bar
    canvas.fillRect(0, 121, 240, 14, COL_PANEL_BG);
    canvas.drawFastHLine(0, 120, 240, COL_DARK_GRAY);

    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COL_YELLOW, COL_PANEL_BG);
    char hudStr[64];
    snprintf(hudStr, sizeof(hudStr), "AGL:%+dm | BAT:%d%% (%.2fV) | %s | %s",
             (int)(AltitudeM - ZeroAltReference),
             batteryPct, batteryVolt,
             isBleConnected() ? "BLE:OK" : "BLE:WAIT",
             isAudioMuted ? "MUT" : "SND");
    canvas.drawString(hudStr, 120, 128);
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

    // Top-left Battery info outside circular dial
    bool isChg = M5.Power.isCharging();
    uint16_t batCol = isChg ? COL_CYAN : ((batteryPct >= 40) ? COL_GREEN : ((batteryPct >= 20) ? COL_YELLOW : COL_RED));
    canvas.drawRect(4, 4, 16, 9, COL_WHITE);
    canvas.fillRect(20, 6, 2, 5, COL_WHITE);
    int innerW = (batteryPct * 12) / 100;
    if (innerW > 0) {
        canvas.fillRect(6, 6, innerW, 5, batCol);
    }
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(middle_left);
    canvas.setTextColor(batCol, COL_PANEL_BG);
    char dialBat[32];
    snprintf(dialBat, sizeof(dialBat), "%s%d%%", isChg ? "CHG:" : "BAT:", batteryPct);
    canvas.drawString(dialBat, 25, 9);

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
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(COL_WHITE, rgb565(20, 24, 30));
    char calAction[64];
    snprintf(calAction, sizeof(calAction), "[A]:+1m  [B]:-1m  [A 1s]:SALVAR | BAT:%d%%", batteryPct);
    canvas.drawString(calAction, 120, 123);
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

    // Initialize Battery readings immediately with voltage clamping
    updateBatteryStatus();

    // Power external Grove Port A (5V rail) for GY-63 module
    M5.Power.setExtOutput(true);
    delay(150);

    Serial.printf("[I2C Info] Ex_I2C Port=%d, SDA=%d, SCL=%d, Enabled=%d\r\n",
                  M5.Ex_I2C.getPort(), M5.Ex_I2C.getSDA(), M5.Ex_I2C.getSCL(), M5.Ex_I2C.isEnabled());
    Serial.printf("[I2C Info] In_I2C Port=%d, SDA=%d, SCL=%d, Enabled=%d\r\n",
                  M5.In_I2C.getPort(), M5.In_I2C.getSDA(), M5.In_I2C.getSCL(), M5.In_I2C.isEnabled());

    M5.Speaker.setVolume(200);

    // Setup landscape display (240x135)
    M5.Display.setRotation(1);
    M5.Display.fillScreen(COL_BLACK);

    canvas.setColorDepth(16);
    canvas.createSprite(240, 135);

    // Carrega QNH de referencia salvo da Flash NVS
    loadSavedQNH();
    GY63.seaLevelPressurePa = savedQnhSeaLevelPa;
    Serial.printf("[SETUP] QNH carregado da Flash: %.1f hPa | Bat: %d%% (%.2fV)\r\n",
                  savedQnhSeaLevelPa / 100.0f, batteryPct, batteryVolt);

    // Show Boot Screen
    canvas.fillScreen(COL_BLACK);
    canvas.setTextColor(COL_CYAN);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.drawString("AVIONICS SIMULATOR", 120, 24);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COL_WHITE);
    canvas.drawString("M5-Watch StickC Plus2", 120, 44);
    canvas.drawString("Nivelando Horizonte Inicial...", 120, 60);
    canvas.setTextColor(COL_GREEN);
    canvas.printf("QNH Referencia: %.1f hPa", savedQnhSeaLevelPa / 100.0f);
    canvas.setTextColor(COL_CYAN);
    canvas.drawString("Iniciando BLE (M5-VARIO)...", 120, 94);
    canvas.setTextColor(COL_YELLOW);
    canvas.printf("Bateria: %d%% (%.2fV)", batteryPct, batteryVolt);
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

    // Inicializa BLE Multi-Profile (XC Tracer + Nordic UART + SIG ESS + Bat) para XCTrack
    ble_init();

    // Inicializa sensor GY-63 com QNH salvo
    bool baroOk = GY63.begin();
    GY63.seaLevelPressurePa = savedQnhSeaLevelPa;

    // Se nunca foi calibrado antes (primeiro boot da vida), calibra para 853m inicialmente e salva
    if (!hasSavedQnh) {
        GY63.calibrateToAltitude(CALIBRATED_BASE_ALT_M);
    }

    if (baroOk) {
        M5.Speaker.tone(1200, 100);
        delay(120);
        M5.Speaker.tone(1600, 150);
    } else {
        M5.Speaker.tone(600, 100);
        delay(110);
        M5.Speaker.tone(850, 150);
    }

    // Initialize altitude references
    AltitudeM = GY63.rawAltitudeM;
    ZeroAltReference = AltitudeM;
    menuTargetAlt = roundf(AltitudeM);

    char bootMsg[36];
    snprintf(bootMsg, sizeof(bootMsg), "QNH %.1f | BT ATIVO", GY63.seaLevelPressurePa / 100.0f);
    showToast(bootMsg, COL_GREEN, 2200);
    delay(800);

    // Consume any button click generated by pressing the power button to boot up
    M5.update();
    GY63.isSimulated = false;
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
            // Segurar A (>1s): Salva na Flash o novo QNH e atualiza a altitude
            GY63.calibrateToAltitude(menuTargetAlt);
            AltitudeM = menuTargetAlt;
            ZeroAltReference = menuTargetAlt;
            currentMode = MODE_PFD;

            M5.Speaker.tone(1000, 100);
            delay(110);
            M5.Speaker.tone(1500, 150);

            char msg[36];
            snprintf(msg, sizeof(msg), "QNH SALVO: %dm", (int)menuTargetAlt);
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
        // Botão Principal A (Frontal M5 - GPIO 37)
        // - 1 clique rápido: Desativar / Ativar som do variômetro (Mute / Unmute)
        // - Clique mantido por 3s: Zerar altitude relativa (AGL = 0m) e nivelar horizonte
        // - 2 cliques rápidos: Alternar Modo Demo (simulação)
        static uint32_t btnAPressStartMs = 0;
        static bool btnAHandled3s = false;

        if (M5.BtnA.wasPressed()) {
            btnAPressStartMs = millis();
            btnAHandled3s = false;
        }

        if (M5.BtnA.isPressed()) {
            if (!btnAHandled3s && (millis() - btnAPressStartMs >= 3000)) {
                btnAHandled3s = true;
                // Zerar altitude relativa (AGL) e nivelar horizonte na posição atual do braço
                ZeroAltReference = AltitudeM;
                pitchOffset = pitchDeg;
                rollOffset  = rollDeg;
                M5.Speaker.tone(1000, 90);
                delay(100);
                M5.Speaker.tone(1600, 160);
                showToast("ZERADO! (AGL 0m / NIVEL)", COL_CYAN, 2000);
                Serial.printf("[ZERAR] AGL zerado em %.1fm, Horizonte nivelado\r\n", ZeroAltReference);
            }
        }

        if (M5.BtnA.wasReleased()) {
            if (!btnAHandled3s && (millis() - btnAPressStartMs < 2000)) {
                // UM CLIQUE: Desativar / Ativar áudio do variômetro
                isAudioMuted = !isAudioMuted;
                if (isAudioMuted) {
                    M5.Speaker.stop();
                    M5.Speaker.tone(350, 150);
                    showToast("VARIO: MUDO", COL_RED, 1500);
                } else {
                    M5.Speaker.tone(1000, 80);
                    delay(90);
                    M5.Speaker.tone(1400, 100);
                    showToast("VARIO: ATIVO", COL_GREEN, 1500);
                }
                Serial.printf("[AUDIO] Mute = %d\r\n", isAudioMuted);
            }
        } else if (M5.BtnA.wasDoubleClicked()) {
            toggleDemoMode();
        }

        // Botão B (Lateral Inferior - GPIO 39)
        if (M5.BtnB.wasHold()) {
            // Segurar B (> 1s): Entra no Menu de Ajuste Fino de Altitude
            currentMode = MODE_CALIB_MENU;
            menuTargetAlt = roundf(AltitudeM);
            M5.Speaker.tone(1200, 100);
            showToast("MENU AJUSTE ALTITUDE", COL_CYAN, 1800);
            Serial.println("[MODE] Menu Ajuste Altitude aberto");
        } else if (M5.BtnB.wasDoubleClicked()) {
            toggleDemoMode();
        } else if (M5.BtnB.wasClicked()) {
            // Toque Curto B: Alterna entre os modos de exibição (PFD, HUD, Dial)
            currentMode = (DisplayMode)((currentMode + 1) % 3);
            M5.Speaker.tone(1100, 60);
            const char* modeNames[] = {"MODO 0: PFD COCKPIT", "MODO 1: HORIZONTE HUD", "MODO 2: ALTIMETRO DIAL"};
            showToast(modeNames[currentMode], COL_WHITE, 1800);
            Serial.printf("[MODE] Switched to Mode %d\r\n", currentMode);
        }

        // Botão C / Power (Lateral Superior - GPIO 35)
        if (M5.BtnPWR.wasClicked()) {
            ZeroAltReference = AltitudeM;
            showToast("AGL ZERADO (0m)", COL_CYAN, 1400);
            M5.Speaker.tone(1200, 60);
        }
    }

    // ------------------------------------------------------------------------
    // 2. READ IMU FOR ARTIFICIAL HORIZON
    // ------------------------------------------------------------------------
    if (M5.Imu.update()) {
        auto imuData = M5.Imu.getImuData();
        float ax = imuData.accel.x; // in G
        float ay = imuData.accel.y; // in G
        float az = imuData.accel.z; // in G

        float rawRoll  = atan2f(ay, az) * (180.0f / PI);
        float rawPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / PI);

        // Filter artificial horizon orientation smoothly
        pitchDeg = (pitchDeg * 0.85f) + (rawPitch * 0.15f);
        rollDeg  = (rollDeg  * 0.85f) + (rawRoll  * 0.15f);
    }

    // ------------------------------------------------------------------------
    // 3. BAROMETER UPDATE & SENSOR FUSION (2D Kinematic Kalman Filter)
    // ------------------------------------------------------------------------
    float sampleAltM = 0.0f;
    static uint32_t lastBaroSampleTimeUs = 0;

    if (GY63.update(sampleAltM)) {
        uint32_t nowUs = micros();
        float dt = (lastBaroSampleTimeUs > 0) ? (float)(nowUs - lastBaroSampleTimeUs) * 1e-6f : 0.01f;
        lastBaroSampleTimeUs = nowUs;
        if (dt < 0.002f || dt > 0.08f) dt = 0.01f;

        // Inicializa o filtro de Kalman na primeira leitura real (sem forçar 853m!)
        static bool kfReady = false;
        if (!kfReady && sampleAltM > -500.0f && sampleAltM < 9000.0f) {
            kfReady = true;
            kfVario.init(sampleAltM);
            AltitudeM = sampleAltM;
            ZeroAltReference = sampleAltM;
            menuTargetAlt = roundf(sampleAltM);
            ClimbrateMs = 0.0f;
            ClimbrateCps = 0.0f;
            Serial.printf("[VARIO] Inicializado com Altitude Real: %.1fm (QNH=%.1f hPa)\r\n",
                          sampleAltM, GY63.seaLevelPressurePa / 100.0f);
        }

        // Kinematic Kalman prediction and measurement update
        kfVario.predict(dt);
        kfVario.update(sampleAltM);

        AltitudeM = kfVario.getAltitudeM();
        float rawVsi = kfVario.getClimbrateMs();

        // Deadband filter: +/- 0.08 m/s (8 cm/s) for stability at rest
        if (fabsf(rawVsi) < 0.08f) {
            ClimbrateMs = 0.0f;
        } else {
            ClimbrateMs = rawVsi;
        }

        ClimbrateCps = ClimbrateMs * 100.0f;
    } else if (!GY63.isSimulated && !GY63.isDetected) {
        // Sensor disconnected and not demo: keep stationary and completely silent
        AltitudeM = ZeroAltReference;
        ClimbrateCps = 0.0f;
        ClimbrateMs = 0.0f;
    }

    // ------------------------------------------------------------------------
    // 4. TRANSMIT TELEMETRY TO XCTRACK VIA BLE @ 10Hz (100ms)
    // ------------------------------------------------------------------------
    if (millis() - lastBleTransmitMs >= 100) {
        lastBleTransmitMs = millis();
        int32_t txPressure = (GY63.rawPressurePa > 50000.0f) ? (int32_t)GY63.rawPressurePa : 101325;
        int32_t txTemp = (GY63.isDetected) ? (int32_t)GY63.rawTemperatureC : 22;
        float batVolt = (float)M5.Power.getBatteryVoltage() / 1000.0f;
        if (batVolt < 3.2f || batVolt > 4.35f) batVolt = 4.0f;
        ble_transmit_telemetry(txPressure, AltitudeM, ClimbrateMs, txTemp, batVolt);
    }

    // 5. AUDIO VARIOMETER TICK
    handleAudioVario(ClimbrateMs);

    // 6. BATTERY LEVEL
    static uint32_t lastBatRead = 0;
    if (millis() - lastBatRead > 2000) {
        lastBatRead = millis();
        updateBatteryStatus();
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

    delay(5);
}
