#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "kalmanfilter4d.h"

static const char* TAG = "kalmanfilter4d";

typedef struct KF4_STATE_ {
	float z; // altitude in cm
	float v; // climb/sink rate in cm/s
	float a; // gravity-compensated net earth-z axis acceleration in cm/s^2
	float b; // acceleration residual bias (post-calibration) in cm/s^2
} KF4_STATE;

static KF4_STATE  State;

// 4x4 process model state covariance estimate P 
// Note : P is symmetric
static float Pzz, Pzv, Pza, Pzb;
static float Pvz, Pvv, Pva, Pvb;
static float Paz, Pav, Paa, Pab;
static float Pbz, Pbv, Pba, Pbb;

// 4x4 Process model noise covariance parameters
static float AccelVariance; // environmental acceleration variance, depends on conditions
static float ABiasVariance; // acceleration bias noise variance

// measurement model sensor noise variance
static float ZMeasVariance; // altitude measurement noise variance
static float AMeasVariance; // acceleration measurement noise variance

// Adaptive uncertainty injection factor
static float KAdapt;

// ============================================================================
// THERMAL ASSISTANT & INTEGRATOR STATE
// ============================================================================
#define THERMAL_RINGBUF_SIZE 100 // 100 samples @ 5Hz = 20 seconds window
static float ThermalHistory[THERMAL_RINGBUF_SIZE];
static int   ThermalHead = 0;
static int   ThermalCount = 0;
static uint32_t LastThermalSampleMs = 0;

static THERMAL_STATS_t ThermalStats = {
    .avg10s = 0.0f,
    .avg20s = 0.0f,
    .peakClimb = 0.0f,
    .peakSink = 0.0f,
    .nettoVario = 0.0f,
    .isThermalling = false
};

// Typical paraglider hands-up still-air sink rate (~ -1.1 m/s = -110 cm/s)
#define POLAR_SINK_CPS -110.0f

void kalmanFilter4d_configure(float aVariance, float kAdapt, float zInitial, float vInitial, float aInitial) {
	ZMeasVariance = KF_Z_MEAS_VARIANCE;
	AMeasVariance = KF_A_MEAS_VARIANCE;
    ABiasVariance = KF_ACCELBIAS_VARIANCE;
	AccelVariance = aVariance;
	KAdapt = kAdapt;

	State.z = zInitial;
	State.v = vInitial;
    State.a = aInitial;
	State.b = 0.0f; // assume residual acceleration bias = 0 initially

	Pzz = 1500.0f;
    Pzv = 0.0f;
	Pza = 0.0f;
	Pzb = 0.0f;
	
	Pvz = Pzv; 
	Pvv = 1500.0f;
	Pva = 0.0f;
	Pvb = 0.0f;
	
	Paz = Pza;
	Pav = Pva;
	Paa = 100000.0f;
	Pab = 0.0f;

	Pbz = Pzb;
	Pbv = Pvb;
	Pba = Pab;
	Pbb = 1500.0f;

    kalmanFilter4d_reset_thermal_stats();
}

// Process model state transition matrix F  (4x4)
//  | 1   dt  dt^2/2 -dt^2/2 |
//  | 0   1   dt     -dt     |
//  | 0   0   1       0      |
//  | 0   0   0       1      |
//
void kalmanFilter4d_predict(float dt) {
	if (dt > 0.1f || dt <= 0.0f) dt = 0.02f;
#ifdef BARO_ONLY_MODE
	// 2D Process model:
	// z_k = z_k-1 + v_k-1 * dt
	// v_k = v_k-1
	State.z += State.v * dt;

	// Process noise covariance based on vertical acceleration variance
	float dt2 = dt * dt;
	float dt3 = dt2 * dt;
	float dt4 = dt2 * dt2;
	float qzz = AccelVariance * dt4 * 0.25f;
	float qzv = AccelVariance * dt3 * 0.5f;
	float qvv = AccelVariance * dt2;

	Pzz += 2.0f * dt * Pzv + dt2 * Pvv + qzz;
	Pzv += dt * Pvv + qzv;
	Pvv += qvv;
#else
	// Predicted (a priori) state vector estimate x_k- = F * x_k-1+
	float accel_true = State.a - State.b; // true acceleration = measured acceleration minus bias
	State.z = State.z + (State.v * dt) + (accel_true * dt * dt * 0.5f);
	State.v = State.v + (accel_true * dt);

	// Predicted (a priori) state covariance estimate P_k- = (F * P_k-1+ * F_t) + Qk
	float dt2 = dt * dt;
	float dt3 = dt2 * dt; 
	float dt4 = dt2 * dt2;
	float dt2div2 = dt2 * 0.5f;
	float dt3div2 = dt3 * 0.5f;
	float dt4div2 = dt4 * 0.5f;
	float dt4div4 = dt4 * 0.25f;
	
	float p00 = Pzz + 2.0f*Pzv*dt + (Pza - Pzb)*dt2  + Pvv*dt2div2 + (Pva - Pvb)*dt3 + (Paa+Pbb)*dt4div4 - Pab*dt4div2;
	float p01 = Pzv + dt*(Pza - Pzb + Pvv) + 3.0f*dt2div2*(Pva - Pvb) - Pab*dt3 + (Paa + Pbb)*dt3div2;
	float p02 = Pza + Pva*dt + (Paa - Pba)*dt2div2;
	float p03 = Pzb + Pvb*dt + (Pab - Pbb)*dt2div2;

	float p11 = Pvv + 2.0f*dt*(Pva - Pvb) + dt2*(Paa - 2.0f*Pab + Pbb);
	float p12 = Pva + dt*(Paa - Pba);
	float p13 = Pvb + dt*(Pab - Pbb);

	float p22 = Paa;
	float p23 = Pab;
	float p33 = Pbb;

	// Add Process Noise Q (kinematically coupled continuous Wiener acceleration noise)
	// Propagate acceleration perturbation into altitude and velocity uncertainty
	p00 += dt4div4 * AccelVariance;
	p01 += dt3div2 * AccelVariance;
	p02 += dt2div2 * AccelVariance;

	p11 += dt2 * AccelVariance;
	p12 += dt * AccelVariance;

	p22 += AccelVariance;
	p33 += ABiasVariance;

	Pzz = p00;
	Pzv = p01;
	Pza = p02;
	Pzb = p03;

	Pvz = Pzv;
	Pvv = p11;
	Pva = p12;
	Pvb = p13;

	Paz = Pza;
	Pav = Pva;
	Paa = p22;
	Pab = p23;

	Pbz = Pzb;
	Pbv = Pvb;
	Pba = Pab;
	Pbb = p33; 
#endif
}

// Update state estimate and state covariance estimate, given new z and a measurements
void kalmanFilter4d_update(float zm, float am, float* pz, float* pv) {
#ifdef BARO_ONLY_MODE
	float z_err = zm - State.z;

	// Adaptive measurement noise: adjust trust based on innovation
	// During steady thermal climb or sharp sink, increase responsiveness
	float r_z = ZMeasVariance;
	float abs_err = fabsf(z_err);
	if (abs_err > 15.0f) {
		r_z /= (1.0f + 0.03f * KAdapt * (abs_err - 15.0f));
		if (r_z < 60.0f) r_z = 60.0f; // Prevent oversensitivity to noise
	}

	float S = Pzz + r_z;
	float Kz = Pzz / S;
	float Kv = Pzv / S;

	State.z += Kz * z_err;
	State.v += Kv * z_err;

	Pzz -= Kz * Pzz;
	Pzv -= Kz * Pzv;
	Pvv -= Kv * Pzv;

	if (Pzz < 1.0f) Pzz = 1.0f;
	if (Pvv < 1.0f) Pvv = 1.0f;

	if (isnan(State.z) || isinf(State.z) || fabsf(State.z) > 5000000.0f) {
		State.z = zm;
		State.v = 0.0f;
		Pzz = 1500.0f;
		Pvv = 1500.0f;
		Pzv = 0.0f;
	}

	*pz = State.z;
	*pv = State.v;
#else
	// Innovation Error y_k = measurement minus apriori estimate
	float z_err = zm - State.z;
	float a_err = am - State.a;

	// Innovation covariance S_k = (H * P_k- * H_t) + R_k
	float s00 = Pzz + ZMeasVariance;
	float s01 = Pza;
	float s10 = s01;
	float s11 = Paa + AMeasVariance;

	// Advanced Thermal Updraft & Maneuver Detection:
	// Calculate deviation from estimated acceleration bias
	float accel_ext = fabsf(am - State.b);

	// Distinguish real thermal entry from random turbulence/flutter:
	// A genuine thermal lift produces positive acceleration consistent with barometric trend (z_err > -15.0cm)
	bool isThermalLift = ((am - State.b) > 15.0f && z_err > -15.0f);

	if (isThermalLift) {
		// When entering lift, maintain maximum accelerometer trust for zero-lag audio response
		s11 += (0.15f * KAdapt * accel_ext);
	} else {
		// In pure mechanical vibration or rapid negative flutter, apply adaptive damping
		s11 += (0.6f * KAdapt * accel_ext);
	}

	// Update acceleration bias estimate primarily when in steady, non-accelerating conditions
	if (accel_ext < 40.0f) {
		ABiasVariance = KF_ACCELBIAS_VARIANCE;
	} else {
		ABiasVariance = KF_ACCELBIAS_VARIANCE / (1.0f + accel_ext * 0.1f);
	}

	// Compute S_k_inv (2x2 matrix inversion)
	float sdetinv = 1.0f / (s00 * s11 - s10 * s01);
	float sinv00 =  sdetinv * s11;
	float sinv01 = -sdetinv * s10;
	float sinv10 =  sinv01;
	float sinv11 =  sdetinv * s00;

	// Kalman gain K_k [4x2] matrix
	// K_k = P_k- * H_t * S_k_inv
	float k00 = Pzz*sinv00 + Pza*sinv10;  
	float k01 = Pzz*sinv01 + Pza*sinv11;
	float k10 = Pvz*sinv00 + Pva*sinv10;
	float k11 = Pvz*sinv01 + Pva*sinv11;
	float k20 = Paz*sinv00 + Paa*sinv10;
	float k21 = Paz*sinv01 + Paa*sinv11;
	float k30 = Pbz*sinv00 + Pba*sinv10;
	float k31 = Pbz*sinv01 + Pba*sinv11;

	// Updated (a posteriori) state estimate x_k+ 
	State.z = State.z + (k00*z_err + k01*a_err);
	State.v = State.v + (k10*z_err + k11*a_err);
	State.a = State.a + (k20*z_err + k21*a_err);
	State.b = State.b + (k30*z_err + k31*a_err);

	// Updated (a posteriori) state covariance estimate P_k+
	float tmp = 1.0f - k00;
	float p00 = tmp*Pzz - k01*Paz;
	float p01 = tmp*Pzv - k01*Pav;
	float p02 = tmp*Pza - k01*Paa;
	float p03 = tmp*Pzb - k01*Pab;

	float p11 = -k10*Pzv + Pvv - k11*Pav;
	float p12 = -k10*Pza + Pva - k11*Paa;
	float p13 = -k10*Pzb + Pvb - k11*Pab;

	float p22 = -k20*Pza + (1.0f-k21)*Paa;
	float p23 = -k20*Pzb + (1.0f-k21)*Pab;

	float p33 = -k30*Pzb - k31*Pab + Pbb;

	Pzz = p00;
	Pzv = p01;
	Pza = p02;
	Pzb = p03;

	Pvz = Pzv;
	Pvv = p11;
	Pva = p12;
	Pvb = p13;

	Paz = Pza;
	Pav = Pva;
	Paa = p22;
	Pab = p23;

	Pbz = Pzb;
	Pbv = Pvb;
	Pba = Pab;
	Pbb = p33; 

	// Return state variables of interest (z and v)
	*pz = State.z;
	*pv = State.v;
#endif

    // ========================================================================
    // UPDATE THERMAL ASSISTANT & INTEGRATOR
    // ========================================================================
    uint32_t nowMs = millis();
    if (nowMs - LastThermalSampleMs >= 200) { // Sample at 5 Hz
        LastThermalSampleMs = nowMs;
        ThermalHistory[ThermalHead] = State.v;
        ThermalHead = (ThermalHead + 1) % THERMAL_RINGBUF_SIZE;
        if (ThermalCount < THERMAL_RINGBUF_SIZE) {
            ThermalCount++;
        }

        // Calculate 10-second (last 50 samples) and 20-second (last 100 samples) averages
        float sum10 = 0.0f;
        int count10 = ThermalCount < 50 ? ThermalCount : 50;
        for (int i = 0; i < count10; i++) {
            int idx = (ThermalHead - 1 - i + THERMAL_RINGBUF_SIZE) % THERMAL_RINGBUF_SIZE;
            sum10 += ThermalHistory[idx];
        }
        ThermalStats.avg10s = count10 > 0 ? (sum10 / (float)count10) : State.v;

        float sum20 = 0.0f;
        for (int i = 0; i < ThermalCount; i++) {
            sum20 += ThermalHistory[i];
        }
        ThermalStats.avg20s = ThermalCount > 0 ? (sum20 / (float)ThermalCount) : State.v;

        // Peak climb rate in current thermal
        if (State.v > ThermalStats.peakClimb) {
            ThermalStats.peakClimb = State.v;
        }
        if (State.v < ThermalStats.peakSink) {
            ThermalStats.peakSink = State.v;
        }

        // Netto vario: actual vertical airmass speed = vario climbrate minus glider still-air sink rate
        ThermalStats.nettoVario = State.v - POLAR_SINK_CPS;

        // Thermalling status: climb average > 30 cm/s
        ThermalStats.isThermalling = (ThermalStats.avg10s > 30.0f);
    }
}

void kalmanFilter4d_get_thermal_stats(THERMAL_STATS_t* pStats) {
    if (pStats != NULL) {
        *pStats = ThermalStats;
    }
}

void kalmanFilter4d_reset_thermal_stats() {
    ThermalHead = 0;
    ThermalCount = 0;
    LastThermalSampleMs = millis();
    ThermalStats.avg10s = 0.0f;
    ThermalStats.avg20s = 0.0f;
    ThermalStats.peakClimb = 0.0f;
    ThermalStats.peakSink = 0.0f;
    ThermalStats.nettoVario = 0.0f;
    ThermalStats.isThermalling = false;
    for (int i = 0; i < THERMAL_RINGBUF_SIZE; i++) {
        ThermalHistory[i] = 0.0f;
    }
}

void kalmanFilter4d_get_all_states(float* pz, float* pv, float* pa, float* pb) {
    if (pz) *pz = State.z;
    if (pv) *pv = State.v;
    if (pa) *pa = State.a - State.b;
    if (pb) *pb = State.b;
}
