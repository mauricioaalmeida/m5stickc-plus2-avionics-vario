#include <Arduino.h>
#include "config.h"
#include "ble_uart.h"

#ifdef ESP32_WROOM_BMP280
// ==============================================================================
// ESP-WROOM-32: Bluetooth Classic SPP (Serial Port Profile)
// Totalmente compatível com pareamento do Android, XCSoar, XCTrack, LK8000
// ==============================================================================
#include <BluetoothSerial.h>

static BluetoothSerial SerialBT;

static uint8_t bt_nmea_checksum(const char *szNMEA) {
    const char* sz = &szNMEA[1]; // skip leading '$'
    uint8_t cksum = 0;
    while ((*sz) != 0 && (*sz != '*')) {
        cksum ^= (uint8_t) *sz;
        sz++;
    }
    return cksum;
}

void ble_uart_init() {
    SerialBT.begin("ESP32-BT-Vario");
    dbg_println(("Bluetooth Classic SPP iniciado como 'ESP32-BT-Vario'"));
}

bool ble_uart_is_connected() {
    return SerialBT.hasClient();
}

void ble_uart_transmit_LK8EX1(int32_t rawPressurePa, int32_t altm, int32_t cps, int32_t tempC, float batVoltage) {
    char szmsg[64];
    sprintf(szmsg, "$LK8EX1,%d,%d,%d,%d,%.1f*", (int)rawPressurePa, (int)altm, (int)cps, (int)tempC, batVoltage);
    uint8_t cksum = bt_nmea_checksum(szmsg);
    char szcksum[8];
    sprintf(szcksum, "%02X\r\n", cksum);
    strcat(szmsg, szcksum);

#ifdef BLE_DEBUG
    dbg_printf(("%s", szmsg));
#endif

    if (SerialBT.hasClient()) {
        SerialBT.print(szmsg);
    }
}

void ble_uart_transmit_LK8EX1(int32_t altm, int32_t cps, float batVoltage) {
    ble_uart_transmit_LK8EX1(999999, altm, cps, 99, batVoltage);
}

#else
// ==============================================================================
// ESP32-C3: NimBLE (Bluetooth Low Energy)
// ==============================================================================
#include <NimBLEDevice.h>
#include <esp_gap_ble_api.h>
#include <esp_gattc_api.h>
#include <esp_gatt_defs.h>
#include <esp_bt_main.h>
#include <esp_gatt_common_api.h>

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLEServer* pBLEServer                = NULL;
NimBLEService* pService                 = NULL;
NimBLECharacteristic* pTxCharacteristic = NULL;
NimBLECharacteristic* pRxCharacteristic = NULL;

static uint8_t ble_uart_nmea_checksum(const char *szNMEA);

void ble_uart_init() {
    NimBLEDevice::init("BLE-Vario");
    NimBLEDevice::setMTU(46);
    NimBLEDevice::setPower(ESP_PWR_LVL_N0);

    // Sem autenticação forçada para permitir conexão transparente com apps de voo
    pBLEServer = NimBLEDevice::createServer();
    pService = pBLEServer->createService(SERVICE_UUID);
    pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, NIMBLE_PROPERTY::NOTIFY);
    pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        NIMBLE_PROPERTY::WRITE);

    pService->start();
    pBLEServer->getAdvertising()->start();
}

bool ble_uart_is_connected() {
    return (pBLEServer != NULL && pBLEServer->getConnectedCount() > 0);
}

static uint8_t ble_uart_nmea_checksum(const char *szNMEA) {
    const char* sz = &szNMEA[1];
    uint8_t cksum = 0;
    while ((*sz) != 0 && (*sz != '*')) {
        cksum ^= (uint8_t) *sz;
        sz++;
    }
    return cksum;
}

void ble_uart_transmit_LK8EX1(int32_t rawPressurePa, int32_t altm, int32_t cps, int32_t tempC, float batVoltage) {
    char szmsg[64];
    sprintf(szmsg, "$LK8EX1,%d,%d,%d,%d,%.1f*", rawPressurePa, altm, cps, tempC, batVoltage);
    uint8_t cksum = ble_uart_nmea_checksum(szmsg);
    char szcksum[8];
    sprintf(szcksum, "%02X\r\n", cksum);
    strcat(szmsg, szcksum);

#ifdef BLE_DEBUG
    dbg_printf(("%s", szmsg));
#endif

    if (pTxCharacteristic != NULL) {
        pTxCharacteristic->setValue((const uint8_t*)szmsg, strlen(szmsg));
        pTxCharacteristic->notify();
    }
}

void ble_uart_transmit_LK8EX1(int32_t altm, int32_t cps, float batVoltage) {
    ble_uart_transmit_LK8EX1(999999, altm, cps, 99, batVoltage);
}

#endif
