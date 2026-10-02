#ifndef KALMAN_FILTER4D_H_
#define KALMAN_FILTER4D_H_

#include <stdint.h>
#include <stdbool.h>

// Thermal statistics and flight analysis structure
typedef struct {
    float avg10s;       // 10-second integrated climb rate (cm/s)
    float avg20s;       // 20-second integrated climb rate (cm/s, full 360 circle average)
    float peakClimb;    // Peak climb rate achieved in current thermal (cm/s)
    float peakSink;     // Peak sink rate (cm/s)
    float nettoVario;   // Netto vario: airmass vertical speed compensating glider polar sink (cm/s)
    bool  isThermalling; // True when climbing consistently in a thermal
} THERMAL_STATS_t;

void kalmanFilter4d_configure(float aVariance, float kAdapt, float zInitial, float vInitial, float aInitial);
void kalmanFilter4d_predict(float dt);
void kalmanFilter4d_update(float zm, float am, float* pz, float* pv);

// Thermal assistant and diagnostic accessors
void kalmanFilter4d_get_thermal_stats(THERMAL_STATS_t* pStats);
void kalmanFilter4d_reset_thermal_stats();
void kalmanFilter4d_get_all_states(float* pz, float* pv, float* pa, float* pb);

#endif


