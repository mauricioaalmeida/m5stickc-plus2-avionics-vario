#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "display.h"

#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64
#define SSD1306_ADDR_1 0x3C
#define SSD1306_ADDR_2 0x3D

#ifndef OLED_PIN_SDA
#define OLED_PIN_SDA 18
#endif
#ifndef OLED_PIN_SCL
#define OLED_PIN_SCL 19
#endif

bool IsDisplayDetected = false;
static uint8_t DisplayI2CAddr = SSD1306_ADDR_1;
static uint8_t FrameBuffer[SSD1306_WIDTH * (SSD1306_HEIGHT / 8)];

// Basic 5x7 ASCII font table
static const uint8_t Font5x7[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, // (space)
    0x00, 0x00, 0x5F, 0x00, 0x00, // !
    0x00, 0x07, 0x00, 0x07, 0x00, // "
    0x14, 0x7F, 0x14, 0x7F, 0x14, // #
    0x24, 0x2A, 0x7F, 0x2A, 0x12, // $
    0x23, 0x13, 0x08, 0x64, 0x62, // %
    0x36, 0x49, 0x55, 0x22, 0x50, // &
    0x00, 0x05, 0x03, 0x00, 0x00, // '
    0x00, 0x1C, 0x22, 0x41, 0x00, // (
    0x00, 0x41, 0x22, 0x1C, 0x00, // )
    0x08, 0x2A, 0x1C, 0x2A, 0x08, // *
    0x08, 0x08, 0x3E, 0x08, 0x08, // +
    0x00, 0x50, 0x30, 0x00, 0x00, // ,
    0x08, 0x08, 0x08, 0x08, 0x08, // -
    0x00, 0x60, 0x60, 0x00, 0x00, // .
    0x20, 0x10, 0x08, 0x04, 0x02, // /
    0x3E, 0x51, 0x49, 0x45, 0x3E, // 0
    0x00, 0x42, 0x7F, 0x40, 0x00, // 1
    0x42, 0x61, 0x51, 0x49, 0x46, // 2
    0x21, 0x41, 0x45, 0x4B, 0x31, // 3
    0x18, 0x14, 0x12, 0x7F, 0x10, // 4
    0x27, 0x45, 0x45, 0x45, 0x39, // 5
    0x3C, 0x4A, 0x49, 0x49, 0x30, // 6
    0x01, 0x71, 0x09, 0x05, 0x03, // 7
    0x36, 0x49, 0x49, 0x49, 0x36, // 8
    0x06, 0x49, 0x49, 0x29, 0x1E, // 9
    0x00, 0x36, 0x36, 0x00, 0x00, // :
    0x00, 0x56, 0x36, 0x00, 0x00, // ;
    0x00, 0x08, 0x14, 0x22, 0x41, // <
    0x14, 0x14, 0x14, 0x14, 0x14, // =
    0x41, 0x22, 0x14, 0x08, 0x00, // >
    0x02, 0x01, 0x51, 0x09, 0x06, // ?
    0x32, 0x49, 0x79, 0x41, 0x3E, // @
    0x7E, 0x11, 0x11, 0x11, 0x7E, // A
    0x7F, 0x49, 0x49, 0x49, 0x36, // B
    0x3E, 0x41, 0x41, 0x41, 0x22, // C
    0x7F, 0x41, 0x41, 0x22, 0x1C, // D
    0x7F, 0x49, 0x49, 0x49, 0x41, // E
    0x7F, 0x09, 0x09, 0x01, 0x01, // F
    0x3E, 0x41, 0x41, 0x51, 0x32, // G
    0x7F, 0x08, 0x08, 0x08, 0x7F, // H
    0x00, 0x41, 0x7F, 0x41, 0x00, // I
    0x20, 0x40, 0x41, 0x3F, 0x01, // J
    0x7F, 0x08, 0x14, 0x22, 0x41, // K
    0x7F, 0x40, 0x40, 0x40, 0x40, // L
    0x7F, 0x02, 0x04, 0x02, 0x7F, // M
    0x7F, 0x04, 0x08, 0x10, 0x7F, // N
    0x3E, 0x41, 0x41, 0x41, 0x3E, // O
    0x7F, 0x09, 0x09, 0x09, 0x06, // P
    0x3E, 0x41, 0x51, 0x21, 0x5E, // Q
    0x7F, 0x09, 0x19, 0x29, 0x46, // R
    0x46, 0x49, 0x49, 0x49, 0x31, // S
    0x01, 0x01, 0x7F, 0x01, 0x01, // T
    0x3F, 0x40, 0x40, 0x40, 0x3F, // U
    0x1F, 0x20, 0x40, 0x20, 0x1F, // V
    0x7F, 0x20, 0x18, 0x20, 0x7F, // W
    0x63, 0x14, 0x08, 0x14, 0x63, // X
    0x03, 0x04, 0x78, 0x04, 0x03, // Y
    0x61, 0x51, 0x49, 0x45, 0x43, // Z
    0x00, 0x7F, 0x41, 0x41, 0x00, // [
    0x02, 0x04, 0x08, 0x10, 0x20, // \
    0x00, 0x41, 0x41, 0x7F, 0x00, // ]
    0x04, 0x02, 0x01, 0x02, 0x04, // ^
    0x40, 0x40, 0x40, 0x40, 0x40, // _
    0x00, 0x01, 0x02, 0x04, 0x00, // `
    0x20, 0x54, 0x54, 0x54, 0x78, // a
    0x7F, 0x48, 0x44, 0x44, 0x38, // b
    0x38, 0x44, 0x44, 0x44, 0x20, // c
    0x38, 0x44, 0x44, 0x48, 0x7F, // d
    0x38, 0x54, 0x54, 0x54, 0x18, // e
    0x08, 0x7E, 0x09, 0x01, 0x02, // f
    0x08, 0x14, 0x54, 0x54, 0x3C, // g
    0x7F, 0x08, 0x04, 0x04, 0x78, // h
    0x00, 0x44, 0x7D, 0x40, 0x00, // i
    0x20, 0x40, 0x44, 0x3D, 0x00, // j
    0x00, 0x7F, 0x10, 0x28, 0x44, // k
    0x00, 0x41, 0x7F, 0x40, 0x00, // l
    0x7C, 0x04, 0x18, 0x04, 0x78, // m
    0x7C, 0x08, 0x04, 0x04, 0x78, // n
    0x38, 0x44, 0x44, 0x44, 0x38, // o
    0x7C, 0x14, 0x14, 0x14, 0x08, // p
    0x08, 0x14, 0x14, 0x18, 0x7C, // q
    0x7C, 0x08, 0x04, 0x04, 0x08, // r
    0x48, 0x54, 0x54, 0x54, 0x20, // s
    0x04, 0x3F, 0x44, 0x40, 0x20, // t
    0x3C, 0x40, 0x40, 0x20, 0x7C, // u
    0x1C, 0x20, 0x40, 0x20, 0x1C, // v
    0x3C, 0x40, 0x30, 0x40, 0x3C, // w
    0x44, 0x28, 0x10, 0x28, 0x44, // x
    0x0C, 0x50, 0x50, 0x50, 0x3C, // y
    0x44, 0x64, 0x54, 0x4C, 0x44  // z
};

static void oled_command(uint8_t cmd) {
    Wire.beginTransmission(DisplayI2CAddr);
    Wire.write(0x00); // Command stream
    Wire.write(cmd);
    Wire.endTransmission();
}

static void oled_data(const uint8_t* data, size_t length) {
    Wire.beginTransmission(DisplayI2CAddr);
    Wire.write(0x40); // Data stream
    for (size_t i = 0; i < length; i++) {
        Wire.write(data[i]);
    }
    Wire.endTransmission();
}

void display_clear() {
    memset(FrameBuffer, 0, sizeof(FrameBuffer));
}

static void draw_pixel(int x, int y, uint8_t color) {
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) return;
    int index = x + (y / 8) * SSD1306_WIDTH;
    if (color) {
        FrameBuffer[index] |= (1 << (y % 8));
    } else {
        FrameBuffer[index] &= ~(1 << (y % 8));
    }
}

static void draw_line_h(int x, int y, int w, uint8_t color) {
    for (int i = 0; i < w; i++) draw_pixel(x + i, y, color);
}

static void draw_line_v(int x, int y, int h, uint8_t color) {
    for (int i = 0; i < h; i++) draw_pixel(x, y + i, color);
}

static void draw_rect(int x, int y, int w, int h, uint8_t color, bool fill = false) {
    if (fill) {
        for (int i = 0; i < h; i++) draw_line_h(x, y + i, w, color);
    } else {
        draw_line_h(x, y, w, color);
        draw_line_h(x, y + h - 1, w, color);
        draw_line_v(x, y, h, color);
        draw_line_v(x + w - 1, y, h, color);
    }
}

static void draw_char(int x, int y, char c, uint8_t size = 1, uint8_t color = 1) {
    if (c < 32 || c > 122) c = ' ';
    const uint8_t* charData = &Font5x7[(c - 32) * 5];
    for (int col = 0; col < 5; col++) {
        uint8_t line = pgm_read_byte(charData + col);
        for (int row = 0; row < 7; row++) {
            if (line & (1 << row)) {
                if (size == 1) {
                    draw_pixel(x + col, y + row, color);
                } else {
                    for (int sx = 0; sx < size; sx++) {
                        for (int sy = 0; sy < size; sy++) {
                            draw_pixel(x + col * size + sx, y + row * size + sy, color);
                        }
                    }
                }
            }
        }
    }
}

static void draw_string(int x, int y, const char* str, uint8_t size = 1, uint8_t color = 1) {
    int curX = x;
    while (*str) {
        draw_char(curX, y, *str, size, color);
        curX += (5 * size) + size; // 5 width + spacing
        str++;
    }
}

static void display_flush() {
    if (!IsDisplayDetected) return;
    for (uint8_t page = 0; page < 8; page++) {
        oled_command(0xB0 + page);
        oled_command(0x00);
        oled_command(0x10);
        // Transmit page in 16-byte chunks to fit within I2C buffer
        const uint8_t* p = &FrameBuffer[page * SSD1306_WIDTH];
        for (int chunk = 0; chunk < SSD1306_WIDTH; chunk += 16) {
            oled_data(p + chunk, 16);
        }
    }
}

bool display_init() {
    // Candidate I2C pin pairs and reset pin for integrated OLED boards
    struct I2CPair {
        int sda;
        int scl;
        int rst;
    };

    const I2CPair candidatePins[] = {
        { OLED_PIN_SDA, OLED_PIN_SCL, 16 }, // Configured pins
        { 5, 4, 16 },                       // Wemos Lolin ESP32 OLED standard (SDA 5, SCL 4, RST 16)
        { 21, 22, -1 },                     // Standard ESP32 I2C / TTGO OLED
        { 4, 15, 16 }                       // Heltec WiFi Kit 32 (SDA 4, SCL 15, RST 16)
    };

    IsDisplayDetected = false;

    for (size_t i = 0; i < sizeof(candidatePins) / sizeof(candidatePins[0]); i++) {
        int sda = candidatePins[i].sda;
        int scl = candidatePins[i].scl;
        int rst = candidatePins[i].rst;

        // Hardware Reset pulse if OLED has a dedicated reset pin (common on integrated boards)
        if (rst >= 0) {
            pinMode(rst, OUTPUT);
            digitalWrite(rst, LOW);
            delay(15);
            digitalWrite(rst, HIGH);
            delay(15);
        }

        Wire.begin(sda, scl, (uint32_t)OLED_I2C_FREQ);
        delay(25);

        // Probe 0x3C
        Wire.beginTransmission(SSD1306_ADDR_1);
        if (Wire.endTransmission() == 0) {
            DisplayI2CAddr = SSD1306_ADDR_1;
            IsDisplayDetected = true;
            break;
        }

        // Probe 0x3D
        Wire.beginTransmission(SSD1306_ADDR_2);
        if (Wire.endTransmission() == 0) {
            DisplayI2CAddr = SSD1306_ADDR_2;
            IsDisplayDetected = true;
            break;
        }
    }

    if (!IsDisplayDetected) {
        return false;
    }

    // SSD1306 Initialization sequence
    static const uint8_t initSeq[] = {
        0xAE,       // Display OFF
        0xD5, 0x80, // Set Display Clock Divide Ratio
        0xA8, 0x3F, // Set Multiplex Ratio (64 lines)
        0xD3, 0x00, // Set Display Offset
        0x40,       // Set Start Line
        0x8D, 0x14, // Charge Pump Setting (enable 7.5V)
        0x20, 0x00, // Memory Addressing Mode: Horizontal
        0xA1,       // Set Segment Re-map (A0/A1 flip)
        0xC8,       // Set COM Output Scan Direction
        0xDA, 0x12, // Set COM Pins Hardware Configuration
        0x81, 0xCF, // Set Contrast Control
        0xD9, 0xF1, // Set Pre-charge Period
        0xDB, 0x40, // Set VCOMH Deselect Level
        0xA4,       // Entire Display ON (resume)
        0xA6,       // Normal Display (not inverted)
        0xAF        // Display ON
    };

    for (size_t i = 0; i < sizeof(initSeq); i++) {
        oled_command(initSeq[i]);
    }

    display_clear();
    display_flush();
    return true;
}

void display_power_off() {
    if (!IsDisplayDetected) return;
    display_clear();
    draw_string(25, 28, "POWER OFF", 2, 1);
    display_flush();
    delay(500);
    oled_command(0xAE); // Display OFF
}

void display_show_boot(const char* fwRev) {
    if (!IsDisplayDetected) return;
    display_clear();
    draw_rect(0, 0, 128, 64, 1);
    draw_string(14, 12, "ESP32-C3 VARIO", 1, 1);
    draw_string(24, 28, "KALMAN 4D+", 1, 1);
    char buf[20];
    sprintf(buf, "FW Rev %s", fwRev);
    draw_string(30, 44, buf, 1, 1);
    display_flush();
}

void display_show_calibration(const char* title, int countdownSecs, int progressPercent) {
    if (!IsDisplayDetected) return;
    display_clear();
    draw_string(10, 8, title, 1, 1);
    
    char buf[16];
    sprintf(buf, "%d sec", countdownSecs);
    draw_string(45, 24, buf, 2, 1);

    // Progress bar
    draw_rect(14, 46, 100, 10, 1, false);
    int fillW = (progressPercent * 96) / 100;
    if (fillW > 96) fillW = 96;
    if (fillW > 0) {
        draw_rect(16, 48, fillW, 6, 1, true);
    }
    display_flush();
}

void display_show_message(const char* title, const char* msg) {
    if (!IsDisplayDetected) return;
    display_clear();
    draw_rect(0, 0, 128, 64, 1);
    draw_string(14, 16, title, 1, 1);
    draw_string(14, 34, msg, 1, 1);
    display_flush();
}

void display_update(int32_t altM, int32_t climbrateCps, float batteryV, bool bleActive, bool isMuted, float avg20sCps, float peakClimbCps) {
    if (!IsDisplayDetected) return;
    display_clear();

    // 1. TOP STATUS BAR (Y: 0..9)
    // Battery icon
    draw_rect(2, 1, 16, 8, 1, false);
    draw_rect(18, 3, 2, 4, 1, true);
    int bars = 0;
    if (batteryV >= 4.05f) bars = 3;
    else if (batteryV >= 3.85f) bars = 2;
    else if (batteryV >= 3.65f) bars = 1;
    for (int b = 0; b < bars; b++) {
        draw_rect(4 + (b * 4), 3, 3, 4, 1, true);
    }
    char vbuf[10];
    sprintf(vbuf, "%.2fV", batteryV);
    draw_string(24, 2, vbuf, 1, 1);

    // Audio / Mute indicator
    if (isMuted) {
        draw_string(68, 2, "MUT", 1, 1);
    } else {
        draw_string(68, 2, "SND", 1, 1);
    }

    // BLE indicator
    if (bleActive) {
        draw_string(96, 2, "BLE", 1, 1);
    }

    draw_line_h(0, 11, 112, 1); // Separator line

    // 2. MAIN CLIMBRATE (LARGE FONT) (Y: 15..35)
    char crBuf[16];
    float crMs = (float)climbrateCps / 100.0f;
    if (climbrateCps >= 0) {
        sprintf(crBuf, "+%.1f", crMs);
    } else {
        sprintf(crBuf, "%.1f", crMs);
    }
    draw_string(8, 16, crBuf, 2, 1);
    draw_string(76, 22, "m/s", 1, 1);

    // 3. ALTITUDE & THERMAL STATS (Y: 38..62)
    char altBuf[20];
    sprintf(altBuf, "ALT %dm", (int)altM);
    draw_string(4, 38, altBuf, 1, 1);

    // 20s integrated thermal average & peak climb
    char thBuf[24];
    float avgMs = avg20sCps / 100.0f;
    float peakMs = peakClimbCps / 100.0f;
    sprintf(thBuf, "A: %+.1f Pk:%+.1f", avgMs, peakMs);
    draw_string(4, 52, thBuf, 1, 1);

    // 4. GRAPHICAL VARIO TAPE (RIGHT EDGE: X=114..127)
    draw_line_v(114, 0, 64, 1);
    draw_line_h(112, 32, 5, 1); // 0 m/s zero center mark
    // Scale: +/- 5 m/s across +/- 30 pixels (6 pixels per m/s)
    int varioPixels = (int)(crMs * 6.0f);
    if (varioPixels > 30) varioPixels = 30;
    if (varioPixels < -30) varioPixels = -30;

    if (varioPixels > 0) {
        // Climb: fill bar going up from center Y=32
        draw_rect(118, 32 - varioPixels, 8, varioPixels, 1, true);
    } else if (varioPixels < 0) {
        // Sink: fill bar going down from center Y=32
        draw_rect(118, 32, 8, -varioPixels, 1, true);
    }

    display_flush();
}
