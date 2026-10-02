#ifndef BMP280_H_
#define BMP280_H_

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

class BMP280 {
public:
    BMP280();

    // Initialize SPI bus and sensor (CS=13, SCK=15, MOSI=26, MISO=25)
    bool init(int pinCS = 13, int pinSCK = 15, int pinMOSI = 26, int pinMISO = 25);

    // Verify chip ID (0x58 for BMP280, 0x60 for BME280)
    bool check_id();

    // Trigger & read samples
    bool read_sample();

    // Calculate altitude in cm from pressure in Pa
    static float pa_to_cm(float paf);

    float pressurePa;
    float altitudeCm;
    float altitudeCmAvg;
    int32_t temperatureC;

private:
    int _cs;
    int _sck;
    int _mosi;
    int _miso;

    int32_t _t_fine;

    // Calibration coefficients
    uint16_t _dig_T1;
    int16_t  _dig_T2, _dig_T3;
    uint16_t _dig_P1;
    int16_t  _dig_P2, _dig_P3, _dig_P4, _dig_P5, _dig_P6, _dig_P7, _dig_P8, _dig_P9;

    uint8_t read8(uint8_t reg);
    void write8(uint8_t reg, uint8_t val);
    void read_buf(uint8_t reg, uint8_t* buf, uint8_t len);
    void spi_transfer_bytes(const uint8_t* out, uint8_t* in, size_t len);
};

#endif
