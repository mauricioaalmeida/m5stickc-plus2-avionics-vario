#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include "config.h"
#include "bmp280.h"

#define BMP280_REG_CALIB00   0x88
#define BMP280_REG_CHIPID    0xD0
#define BMP280_REG_RESET     0xE0
#define BMP280_REG_STATUS    0xF3
#define BMP280_REG_CONTROL   0xF4
#define BMP280_REG_CONFIG    0xF5
#define BMP280_REG_PRESSURE  0xF7

static SPIClass BmpSPI(HSPI);
static SPISettings SpiSettings(4000000, MSBFIRST, SPI_MODE0);

BMP280::BMP280() {
    pressurePa = 101325.0f;
    altitudeCm = 0.0f;
    altitudeCmAvg = 0.0f;
    temperatureC = 20;
    _t_fine = 0;
    _cs = 13;
    _sck = 15;
    _mosi = 26;
    _miso = 25;
}

uint8_t BMP280::read8(uint8_t reg) {
    BmpSPI.beginTransaction(SpiSettings);
    digitalWrite(_cs, LOW);
    BmpSPI.transfer(reg | 0x80); // Bit 7 set for read
    uint8_t val = BmpSPI.transfer(0x00);
    digitalWrite(_cs, HIGH);
    BmpSPI.endTransaction();
    return val;
}

void BMP280::write8(uint8_t reg, uint8_t val) {
    BmpSPI.beginTransaction(SpiSettings);
    digitalWrite(_cs, LOW);
    BmpSPI.transfer(reg & 0x7F); // Bit 7 cleared for write
    BmpSPI.transfer(val);
    digitalWrite(_cs, HIGH);
    BmpSPI.endTransaction();
}

void BMP280::read_buf(uint8_t reg, uint8_t* buf, uint8_t len) {
    BmpSPI.beginTransaction(SpiSettings);
    digitalWrite(_cs, LOW);
    BmpSPI.transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = BmpSPI.transfer(0x00);
    }
    digitalWrite(_cs, HIGH);
    BmpSPI.endTransaction();
}

bool BMP280::init(int pinCS, int pinSCK, int pinMOSI, int pinMISO) {
    _cs = pinCS;
    _sck = pinSCK;
    _mosi = pinMOSI;
    _miso = pinMISO;

    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);

    BmpSPI.begin(_sck, _miso, _mosi, _cs);
    delay(20);

    // Soft reset BMP280
    write8(BMP280_REG_RESET, 0xB6);
    delay(50);

    // Verify chip ID
    if (!check_id()) {
        dbg_println(("BMP280: ID mismatch!"));
        return false;
    }

    // Read 24 calibration bytes (0x88 to 0x9F)
    uint8_t calib[24];
    read_buf(BMP280_REG_CALIB00, calib, 24);

    _dig_T1 = (uint16_t)(calib[0] | (calib[1] << 8));
    _dig_T2 = (int16_t)(calib[2] | (calib[3] << 8));
    _dig_T3 = (int16_t)(calib[4] | (calib[5] << 8));

    _dig_P1 = (uint16_t)(calib[6] | (calib[7] << 8));
    _dig_P2 = (int16_t)(calib[8] | (calib[9] << 8));
    _dig_P3 = (int16_t)(calib[10] | (calib[11] << 8));
    _dig_P4 = (int16_t)(calib[12] | (calib[13] << 8));
    _dig_P5 = (int16_t)(calib[14] | (calib[15] << 8));
    _dig_P6 = (int16_t)(calib[16] | (calib[17] << 8));
    _dig_P7 = (int16_t)(calib[18] | (calib[19] << 8));
    _dig_P8 = (int16_t)(calib[20] | (calib[21] << 8));
    _dig_P9 = (int16_t)(calib[22] | (calib[23] << 8));

    // Configure BMP280 for Variometer:
    // Standby: 0.5ms (000), Filter: 16 (100) -> 0x10
    write8(BMP280_REG_CONFIG, 0x10);

    // osrs_t x2 (010), osrs_p x16 (101), Normal mode (11) -> 0x57
    write8(BMP280_REG_CONTROL, 0x57);
    delay(30);

    // Take initial samples to establish baseline altitude
    float sumAlt = 0.0f;
    for (int i = 0; i < 8; i++) {
        read_sample();
        sumAlt += altitudeCm;
        delay(15);
    }
    altitudeCmAvg = sumAlt / 8.0f;

    dbg_printf(("BMP280 OK! P: %.1f Pa, T: %d C, Z: %.0f cm\n", pressurePa, (int)temperatureC, altitudeCmAvg));
    return true;
}

bool BMP280::check_id() {
    uint8_t id = read8(BMP280_REG_CHIPID);
    dbg_printf(("BMP280 ID = 0x%02X (expected 0x58 or 0x60)\n", id));
    return (id == 0x58 || id == 0x60);
}

bool BMP280::read_sample() {
    uint8_t buf[6];
    read_buf(BMP280_REG_PRESSURE, buf, 6);

    int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
    int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((int32_t)buf[5] >> 4);

    if (adc_P == 0x80000 || adc_T == 0x80000) {
        return false; // Conversion not ready or SPI error
    }

    // 1. Temperature Compensation (Bosch 64-bit integer formula)
    int64_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int64_t)_dig_T1 << 1))) * ((int64_t)_dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int64_t)_dig_T1)) * ((adc_T >> 4) - ((int64_t)_dig_T1))) >> 12) * ((int64_t)_dig_T3)) >> 14;
    _t_fine = (int32_t)(var1 + var2);

    int32_t t = (_t_fine * 5 + 128) >> 8;
    temperatureC = t / 100;

    // 2. Pressure Compensation (Bosch 64-bit integer formula)
    int64_t p;
    var1 = ((int64_t)_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)_dig_P6;
    var2 = var2 + ((var1 * (int64_t)_dig_P5) << 17);
    var2 = var2 + (((int64_t)_dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)_dig_P3) >> 8) + ((var1 * (int64_t)_dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_dig_P1) >> 33;

    if (var1 == 0) {
        return false; // Avoid divide by zero
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)_dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)_dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)_dig_P7) << 4);

    pressurePa = (float)p / 256.0f;
    altitudeCm = pa_to_cm(pressurePa);
    return true;
}

// Hypsometric formula conversion: Pa -> cm
float BMP280::pa_to_cm(float paf) {
    if (paf <= 0.0f) return 0.0f;
    // Standard atmosphere: 44330 * (1 - (P / 101325)^0.190295) in meters * 100 for cm
    float altM = 44330.0f * (1.0f - powf(paf / 101325.0f, 0.190295f));
    return altM * 100.0f;
}
