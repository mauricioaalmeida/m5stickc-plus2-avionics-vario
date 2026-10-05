#include <Arduino.h>
#include <Wire.h>
#include <FS.h>
#include <LittleFS.h>
#include "config.h"
#include "hw_spi.h"
#include "util.h"
#include "imu.h"
#include "mpu9250.h"
#include "ms5611.h"
#include "kalmanfilter4d.h"
#include "audio.h"
#include "vaudio.h"
#include "adc.h"
#include "nvd.h"
#include "ringbuf.h"
#include "wifi_cfg.h"
#include "ui.h"
#include "ble_uart.h"
#include "display.h"
#include "bmp280.h"

const char* FwRevision = "1.00";

#ifdef USE_BMP280
BMP280 BaroBMP;
#else
MS5611 Baro;
#endif

#ifndef BARO_ONLY_MODE
MPU9250 Imu;
volatile SemaphoreHandle_t DrdySemaphore;
static void IRAM_ATTR drdy_interrupt_handler();
#endif

boolean	bWebConfigure = false;

volatile int AltitudeM;
volatile int ClimbrateCps;
int LEDState;

static void vario_task(void * pvParameter);
static void wifi_config_task(void * pvParameter);
static void pwr_ctrl_task(void* pvParameter);
static void ble_task(void* pvParameter);
static void display_task(void* pvParameter);
static void power_off();

// PCCA button (GPIO9) has an external 10K pullup resistor to VCC
// This button has  different functions : program, configure, calibrate & audio toggle 
// 1. Program
//    Power on the unit with PCCA button pressed. Or with power on, keep 
//    PCCA pressed and momentarily press the reset button.
//    This will put the ESP32-C3 into programming mode, and you can flash 
//    the application code.
// 2. WiFi Configuration
//    After normal power on, immediately press PCCA and keep it pressed. 
//    Wait until you hear a low tone, then release. The unit will now be in WiFi configuration
//    configuration mode. 
// 3. Calibrate Accelerometer
//    After normal power on, wait until you hear the battery voltage feedback beeps and
//    then the countdown to gyroscope calibration. If you press the PCCA button
//    during the gyro calibration countdown, the unit will start accelerometer calibration first. 
//    Accelerometer re-calibration is required if the acceleration calibration values in 
//    flash were never written, or if the entire flash has been erased.
// 4. Audio Mute/Unmute  
//    When the vario is in normal operation, pressing PCCA will toggle the
//    vario audio feedback.


#ifndef BARO_ONLY_MODE
// handles data ready interrupt from MPU9250 (every 2ms)
static void IRAM_ATTR drdy_interrupt_handler() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(DrdySemaphore, &xHigherPriorityTaskWoken);
    if( xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(); // this wakes up vario_task immediately instead of on next FreeRTOS tick
		}	
	}	
#endif


void setup() {
#ifdef TOP_DEBUG   
#ifdef HW_REV_B 
	Serial.begin();
#else 
	Serial.begin(115200);
#endif
	delay(100);
	Serial.println("\n\n========================================");
	Serial.println("   ESP32 OLED BMP280 VARIO STARTING    ");
	Serial.println("========================================");
#endif

#if pinPCCA >= 0
	pinMode(pinPCCA, INPUT_PULLUP); //  Program/Configure/Calibrate/Audio Mute Button
#endif
#if pinPwrCtrl >= 0
	pinMode(pinPwrCtrl, OUTPUT); // soft-switch power on/off
	digitalWrite(pinPwrCtrl, LOW);
#endif
#if pinPwrSens >= 0
	pinMode(pinPwrSens, INPUT); // to detect  power-off button press
#endif

#if pinLED >= 0
	pinMode(pinLED, OUTPUT); // power/bluetooth LED
	LED_OFF();
#endif
#if pinAudioEn >= 0
	pinMode(pinAudioEn, OUTPUT_OPEN_DRAIN); // output enable for 74HC240, active low
	digitalWrite(pinAudioEn, HIGH);
#endif

	wifi_off(); // turn off radio to save power

#if pinPwrCtrl >= 0
	// PWR button needs to be pressed for one second to switch on
	delay(1000);
	digitalWrite(pinPwrCtrl, HIGH);
#endif
	LED_ON();
#if pinAudioEn >= 0
	digitalWrite(pinAudioEn, LOW);
#endif
#ifndef USE_BMP280
	spi_init();
#endif
	display_init(); // Auto-detect I2C SSD1306/SH1106 display
	if (IsDisplayDetected) {
		display_show_boot(FwRevision);
	}

	dbg_printf(("\nESP32 BLUETOOTH VARIO compiled on %s at %s\n", __DATE__, __TIME__));
	dbg_printf(("Firmware Revision %s\n", FwRevision));

	dbg_println(("\nLoad non-volatile configuration and calibration data from flash"));  
	nvd_config_load(Config);
	nvd_calib_load(Calib);
	adc_init();
	int adcVal = adc_sample_average();
	BatteryVoltage = adc_battery_voltage(adcVal);

	bWebConfigure = false;
	dbg_println(("To start web configuration mode, press and hold the PCC button"));
	dbg_println(("until you hear a low-frequency tone. Then release the button"));
	for (int cnt = 0; cnt < 6; cnt++) {
		dbg_println((8-cnt));
		delay(500);
		if (BTN_PCCA()) {
			bWebConfigure = true;
			break;
			}
		}

  setCpuFrequencyMhz(80);
  uint32_t Freq = getCpuFrequencyMhz();
  Serial.print("CPU Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getXtalFrequencyMhz();
  Serial.print("XTAL Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getApbFrequency();
  Serial.print("APB Freq = ");
  Serial.print(Freq);
  Serial.println(" Hz");		
   	
#if pinPwrSens >= 0 && pinPwrCtrl >= 0
	xTaskCreate( pwr_ctrl_task, "pwr_ctrl_task", 1024, NULL, PWR_CTRL_TASK_PRIORITY, NULL );
#endif
	if (bWebConfigure == true) {
		dbg_println(("Web configuration mode"));
		audio_generate_tone(200, 3000);
    	xTaskCreate( wifi_config_task, "wifi_config_task", 4096, NULL, WIFI_CFG_TASK_PRIORITY, NULL );
		}
  	else {
		dbg_println(("Vario mode"));
		dbg_println(("\nAudio indication of battery voltage"));
		ui_indicate_battery_voltage(BatteryVoltage);
		if (IsDisplayDetected) {
			xTaskCreate(display_task, "display_task", 2048, NULL, DISPLAY_TASK_PRIORITY, NULL);
		}
    	xTaskCreate( vario_task, "vario_task", 4096, NULL, VARIO_TASK_PRIORITY, NULL );
		}
	// delete the loopTask which called setup() from arduino app_main()
	vTaskDelete(NULL);
	}


static void power_off() {
#if pinPwrCtrl >= 0
	digitalWrite(pinPwrCtrl, LOW);
#endif
	LED_OFF();
	display_power_off();
	audio_generate_tone(200, 1000); // when you hear the tone, you can release the power button.
	audio_off();
	esp_deep_sleep_start(); // required as button is still pressed
	}


static void ble_task(void* pvParameter){
	ble_uart_init();
	int counter = 0;
	dbg_println(("\nBluetooth LE LK8EX1 messages @ 10Hz\n"));
	while (1) {
		counter++;
		if (counter > 9) {
			counter = 0;
			LEDState = !LEDState;
#if (pinLED >= 0)
			digitalWrite(pinLED, LEDState);
#endif
			}
		BatteryVoltage = adc_battery_voltage();
#ifdef USE_BMP280
		ble_uart_transmit_LK8EX1((int32_t)BaroBMP.pressurePa, AltitudeM, ClimbrateCps, BaroBMP.temperatureC, BatteryVoltage);				
#else
		ble_uart_transmit_LK8EX1((int32_t)Baro.pressurePa, AltitudeM, ClimbrateCps, Baro.temperatureC, BatteryVoltage);				
#endif
		vTaskDelay(100/portTICK_PERIOD_MS);
		}
	}


static void display_task(void* pvParameter) {
	THERMAL_STATS_t thStats;
	while (1) {
		kalmanFilter4d_get_thermal_stats(&thStats);
		display_update(AltitudeM, ClimbrateCps, BatteryVoltage, Config.misc.bleEnable, IsMuted, thStats.avg20s, thStats.peakClimb);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}


static void pwr_ctrl_task(void* pvParameter){
	int state = 0;
	int counter = 0;
	int timeoutCounter = (int)(PWR_OFF_DELAY_MS/(10UL*portTICK_PERIOD_MS));

	while (1) {
		switch (state) {
			case 0 :
			default :  
					if (digitalRead(pinPwrSens) == HIGH){
						counter = 0;
						state = 1;
						}
			break;
				
			case 1 : 
					if (digitalRead(pinPwrSens) == HIGH){
						counter++;
						if (counter >= timeoutCounter) {
							dbg_println(("Switching off power!"));
							power_off();
							}
						}
					else {
						state = 0;
						counter = 0;
						}
			break;
				}
		vTaskDelay(10 / portTICK_PERIOD_MS);
		}
	}


static void wifi_config_task(void * pvParameter) {
	if (!LittleFS.begin(true)){
		dbg_println(("Warning: Error mounting LittleFS with formatOnFail"));
	} else {
		dbg_println(("LittleFS mounted successfully"));
	}
	wificfg_ap_server_init(); 
	if (IsDisplayDetected) {
		display_show_wifi("ESP32-VARIO", "192.168.4.1", "/update");
	}
	while (1) {
		wificfg_loop();
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}


static void vario_task(void * pvParameter) {
#ifdef USE_BMP280
	float kfAltitudeCm = 0.0f;
	float kfClimbrateCps = 0.0f;
	uint32_t timePreviousUs, timeNowUs;
	float kfTimeDeltaUSecs = 0.0f;
	int pwrOffCounter = 0;
	int pwrOffTimeoutSecs = 0;

	dbg_println(("\nCheck communication with BMP280 (SPI)"));
	if (!BaroBMP.init(pinBMP280_CS, pinBMP280_SCK, pinBMP280_MOSI, pinBMP280_MISO)) {
		dbg_println(("BMP280 communication failed!"));
		dbg_flush();
		ui_indicate_fault_BMP280();
	} else {
		dbg_println(("BMP280 OK"));
	}

	dbg_println(("\nKalmanFilter config (Baro-only)"));
	kalmanFilter4d_configure(1000.0f*(float)Config.kf.accelVariance, ((float)Config.kf.adapt)/100.0f, BaroBMP.altitudeCmAvg, 0.0f, 0.0f);

	if (Config.misc.bleEnable) {
		xTaskCreate(ble_task, "ble_task", 4096, NULL, BLE_TASK_PRIORITY, NULL );
	}

	vaudio_config();  
	timeNowUs = timePreviousUs = micros();
	ui_btn_init();	

	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xPeriod = pdMS_TO_TICKS(20); // 50 Hz loop (20ms)

	while (1) {
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
		timeNowUs = micros();
		kfTimeDeltaUSecs = timeNowUs > timePreviousUs ? (float)(timeNowUs - timePreviousUs) : 20000.0f;
		timePreviousUs = timeNowUs;

		if (BaroBMP.read_sample()) {
			float dtKF = kfTimeDeltaUSecs / 1000000.0f;
			kalmanFilter4d_predict(dtKF);
			kalmanFilter4d_update(BaroBMP.altitudeCm, 0.0f, (float*)&kfAltitudeCm, (float*)&kfClimbrateCps);

			AltitudeM = F_TO_I(kfAltitudeCm / 100.0f);
			ClimbrateCps = F_TO_I(kfClimbrateCps);
			vaudio_tick_handler(ClimbrateCps);

			if (ABS(ClimbrateCps) > PWR_OFF_THRESHOLD_CPS) { 
				pwrOffTimeoutSecs = 0;
			} else {
				pwrOffCounter++;
				if (pwrOffCounter >= 50) { // 1 second
					pwrOffCounter = 0;
					pwrOffTimeoutSecs++;
#if pinPwrCtrl >= 0
					if (pwrOffTimeoutSecs >= (Config.misc.pwrOffTimeoutMinutes * 60)) {
						dbg_println(("Timed out with no significant climb/sink, power down"));
						ui_indicate_power_off();
						power_off(); 
					}
#endif
				}
			}
		}

		if (BtnPCCAPressed) {
			BtnPCCAPressed = false;
			IsMuted = !IsMuted;
			if (IsMuted) audio_off();
		}
	}
	vTaskDelete(NULL);
	}
#else
	float accelmG[3];  // milli-Gs
	float gyroDps[3];  // degrees/second
	float mag[3];      // unitless vector  
	float kfAltitudeCm = 0.0f; // kalman filtered altitude in cm
	float kfClimbrateCps = 0.0f;  // kalman filtered climb/sink rate in cm/s

	uint32_t timePreviousUs, timeNowUs; // time markers
	float imuTimeDeltaUSecs; // time between imu samples, in microseconds
	float kfTimeDeltaUSecs; // time between kalman filter updates, in microseconds

	int pwrOffCounter, baroCounter, drdyCounter;
	int pwrOffTimeoutSecs;

	dbg_println(("\nCheck communication with MS5611"));
	if (!Baro.read_prom()) {
		dbg_println(("Bad CRC read from MS5611 calibration PROM"));
		dbg_flush();
		ui_indicate_fault_MS5611(); 
		power_off();
		}
	dbg_println(("MS5611 OK"));
  
	dbg_println(("\nCheck communication with MPU9250"));
	if (!Imu.check_id()) {
		dbg_println(("Error reading Mpu9250 WHO_AM_I register"));
		dbg_flush();
		ui_indicate_fault_MPU9250();
		power_off();
		}
	dbg_println(("MPU9250 OK"));

#ifdef USE_9DOF_AHRS    
	// configure MPU9250 to start generating gyro, accel and mag data  
	Imu.config_accel_gyro_mag();
	// calibrate gyro (accel + mag if required)
	ui_calibrate_accel_gyro_mag();
#else
	// configure MPU9250 to start generating gyro, accel data  
	Imu.config_accel_gyro();
	// calibrate gyro (accel  if required)
	ui_calibrate_accel_gyro();
#endif
	delay(50);  
	  
	dbg_println(("\nMS5611 config"));
	Baro.reset();
	Baro.get_calib_coefficients(); // load MS5611 factory programmed calibration data
	Baro.averaged_sample(4); // get an estimate of starting altitude
	Baro.init_sample_state_machine(); // start the pressure & temperature sampling cycle

	dbg_println(("\nKalmanFilter config"));
	// initialize kalman filter with MS5611 estimated altitude, estimated initial climbrate = 0.0
	kalmanFilter4d_configure(1000.0f*(float)Config.kf.accelVariance, ((float)Config.kf.adapt)/100.0f, Baro.altitudeCmAvg, 0.0f, 0.0f);

	if (Config.misc.bleEnable) {
		xTaskCreate(ble_task, "ble_task", 4096, NULL, BLE_TASK_PRIORITY, NULL );
		}

	vaudio_config();  
	timeNowUs = timePreviousUs = micros();
	ringbuf_init(); 
	ui_btn_init();	
	// interrupt output of MPU9250 is configured as push-pull, active high pulse. This is connected to
	// pinDRDYInt which has an external 10K pull-down resistor
	pinMode(pinDRDYInt, INPUT); 
    DrdySemaphore = xSemaphoreCreateBinary();
	baroCounter = pwrOffCounter = drdyCounter = 0;
	pwrOffTimeoutSecs = 0;
	kfTimeDeltaUSecs = imuTimeDeltaUSecs = 0.0f;
	attachInterrupt(pinDRDYInt, drdy_interrupt_handler, RISING);

	while (1) {
		// MPU9250 500Hz ODR => 2mS sample interval
		// wait for data ready interrupt from MPU9250 
		xSemaphoreTake(DrdySemaphore, portMAX_DELAY); 
		timeNowUs = micros();
		imuTimeDeltaUSecs = timeNowUs > timePreviousUs ? (float)(timeNowUs - timePreviousUs) : 2000.0f; // if rollover use expected time difference
		timePreviousUs = timeNowUs;
		drdyCounter++;
		uint32_t marker = micros(); // set marker for estimating the time taken to read and process the data
#ifdef USE_9DOF_AHRS
		// accelerometer samples (ax,ay,az) in milli-Gs, gyroscope samples (gx,gy,gz) in degrees/second, mag samples are unitless
		Imu.get_accel_gyro_mag_data(accelmG, gyroDps, mag); 
#else
		// accelerometer samples (ax,ay,az) in milli-Gs, gyroscope samples (gx,gy,gz) in degrees/second
		Imu.get_accel_gyro_data(accelmG, gyroDps); 
#endif
		// W.r.t. the CJMCU-117 board mounting on the VhARIO-ESPC3 PCB layout,
		// we arbitrarily decide that the CJMCU-117 board silkscreen -X points "forward" or "north", 
		// silkscreen -Y points "right" or "east", and silkscreen +Z points down. This is the North-East-Down (NED) 
		// right-handed coordinate frame used in the AHRS algorithm implementation.
		// The mapping from sensor frame to NED frame is : 
		// gn = gx, ge = -gy, gd = gz  : clockwise rotations about the + ned axis must result in +ve readings
		// an = ax, ae = ay, ad = -az   : when the + ned axis points down, axis reading must be +ve max
		// mn = -my, me = -mx, md = -mz   : when the + ned axis points magnetic north, axis reading must be +ve max
		// The AHRS algorithm expects rotation rates in radians/second
		// Acceleration data is only used for orientation correction when the acceleration magnitude is between 0.75G and 1.25G
		float dtIMU = imuTimeDeltaUSecs/1000000.0f;

		float gn = DEG_TO_RAD*gyroDps[0];
		float ge = -DEG_TO_RAD*gyroDps[1];
		float gd = DEG_TO_RAD*gyroDps[2];
		float an = accelmG[0];
		float ae = accelmG[1];
		float ad = -accelmG[2];

		// Centripetal acceleration compensation during thermalling / circling:
		// Paraglider estimated trim speed ~10 m/s (~36 km/h).
		// Lateral centripetal acceleration in mG: ac = (V * yawRate / 9.8) * 1000 ~= 1020 * gd.
		float ae_for_ahrs = ae - (1020.0f * gd);

		// Dynamic G-window for thermalling turns:
		// When yaw rate indicates circling, allow higher G-load (up to 1.6G) without losing gravity tracking
		float maxGAllowedSq = (fabsf(gd) > 0.15f) ? 2560000.0f : 1562500.0f; // 1.6G vs 1.25G
		float accelMagnitudeSquared = an*an + ae_for_ahrs*ae_for_ahrs + ad*ad;
		int bUseAccel = ((accelMagnitudeSquared > 490000.0f) && (accelMagnitudeSquared < maxGAllowedSq)) ? 1 : 0;

#ifdef USE_9DOF_AHRS		
		float mn = -mag[1];
		float me = -mag[0];
		float md = -mag[2];
		imu_mahonyAHRS_update9DOF(bUseAccel, true, dtIMU, gn, ge, gd, an, ae_for_ahrs, ad, mn, me, md);
#else
		imu_mahonyAHRS_update6DOF(bUseAccel, dtIMU, gn, ge, gd, an, ae_for_ahrs, ad);
#endif		
		float gCompensatedAccel = imu_gravity_compensated_accel(an, ae, ad, Q0, Q1, Q2, Q3);
		ringbuf_add_sample(gCompensatedAccel);  
		baroCounter++;
		kfTimeDeltaUSecs += imuTimeDeltaUSecs;
		if (baroCounter >= 5) { // 5*2mS = 10mS elapsed, this is the sampling period for MS5611, 
			baroCounter = 0;    // alternating between pressure and temperature samples
			// one altitude sample is calculated for every new pair of pressure & temperature samples
			int zMeasurementAvailable = Baro.sample_state_machine(); 
			if ( zMeasurementAvailable ) { 
				// average earth-z acceleration over the 20mS interval between z samples
				// is used in the kf algorithm update phase
				float zAccelAverage = ringbuf_average_newest_samples(10); 
				float dtKF = kfTimeDeltaUSecs/1000000.0f;
				kalmanFilter4d_predict(dtKF);
				kalmanFilter4d_update(Baro.altitudeCm, zAccelAverage, (float*)&kfAltitudeCm, (float*)&kfClimbrateCps);
				// reset time elapsed between kalman filter algorithm updates
				kfTimeDeltaUSecs = 0.0f;
				AltitudeM = F_TO_I(kfAltitudeCm/100.0f);
				ClimbrateCps = F_TO_I(kfClimbrateCps);
				vaudio_tick_handler(ClimbrateCps); // audio feedback handler
				if (ABS(ClimbrateCps) > PWR_OFF_THRESHOLD_CPS) { 
					// reset power-off timeout watchdog if there is significant vertical motion
					pwrOffTimeoutSecs = 0;
					}
				else
				if (pwrOffTimeoutSecs >= (Config.misc.pwrOffTimeoutMinutes*60)) {
					dbg_println(("Timed out with no significant climb/sink, power down"));
					ui_indicate_power_off();
					power_off(); 
					}   
				}
			}
			
		if (BtnPCCAPressed) {
			BtnPCCAPressed = false;
			IsMuted = !IsMuted;
			if (IsMuted) audio_off();
			}	
		uint32_t elapsedUs =  micros() - marker; // calculate time  taken to read and process the data, must be less than 2mS
		if (drdyCounter >= 500) {
			drdyCounter = 0; // 1 second elapsed
			pwrOffTimeoutSecs++;
			#ifdef IMU_DEBUG
			//float yaw, pitch, roll;
			//imu_quaternion_to_yaw_pitch_roll(Q0,Q1,Q2,Q3, &yaw, &pitch, &roll);
			// Pitch is positive for clockwise rotation about the NED frame +Y axis
			// Roll is positive for clockwise rotation about the NED frame +X axis
			// Yaw is positive for clockwise rotation about the NED frame +Z axis
			// If magnetometer isn't used, yaw is initialized to 0 on power up.
			//dbg_printf(("\nY = %d P = %d R = %d\n", (int)yaw, (int)pitch, (int)roll));
			//dbg_printf(("kv = %d, timeout_counter = %d\n", (int)kfClimbrateCps, pwrOffTimeoutSecs));
			//dbg_printf(("ax = %.1f ay = %.1f az = %.1f\n",accelmG[0], accelmG[1], accelmG[2]));
			//dbg_printf(("gx = %.1f gy = %.1f gz = %.1f\n",gyroDps[0], gyroDps[1], gyroDps[2]));
			//dbg_printf(("mx = %.1f my = %.1f mz = %.1f\n",mag[0], mag[1], mag[2]));
			//dbg_printf(("Elapsed %dus\n", (int)elapsedUs)); 
			#endif     
			}
		}
	vTaskDelete(NULL);
	}
#endif


void loop(){
	}
