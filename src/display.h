#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

extern bool IsDisplayDetected;

// Initialize display (auto-detects SSD1306/SH1106 at 0x3C / 0x3D)
// Returns true if an I2C OLED display was detected and initialized
bool display_init();

// Update display with flight telemetry (called at ~10Hz)
void display_update(int32_t altM, int32_t climbrateCps, float batteryV, bool bleActive, bool isMuted, float avg20sCps, float peakClimbCps);

// Calibration / Status notification screens
void display_show_boot(const char* fwRev);
void display_show_calibration(const char* title, int countdownSecs, int progressPercent);
void display_show_message(const char* title, const char* msg);
void display_clear();
void display_power_off();

#endif
