#ifndef _ANALOG_H
#define _ANALOG_H
#include <inttypes.h>
#include "configuration.h"
#include "Vector3.h"

/*
 GYRO_HW_FACTOR is the relation between rotation rate and ADCValue:
 ADCValue [units] =
 rotational speed [rad/s] *
 gyro sensitivity [V/rad/s] *
 amplifier gain [units] *
 1024 [units] /
 3V full range [V]

 GYRO_HW_FACTOR is:
 gyro sensitivity [V/rad/s] *
 amplifier gain [units] *
 1024 [units] /
 3V full range [V]

 Examples:
 FC1.3 has 38.39 mV/rad/s gyros and amplifiers with a gain of 5.7:
 GYRO_HW_FACTOR = 0.03839 V/deg/s*5.7*1024/3V = 74.688 units/rad/s.

 FC2.0 has 6*(3/5) mV/rad/s gyros (they are ratiometric) and no amplifiers:
 GYRO_HW_FACTOR = 0.006 V/rad/s*1*1024*3V/5V/3V = 70.405 units/(rad/s).

 My InvenSense copter has 2mV/deg/s gyros and no amplifiers:
 GYRO_HW_FACTOR = 0.002 V/rad/s*1*1024/3V = 39.1139 units/(rad/s)
 (only about half as sensitive as V1.3. But it will take about twice the rotation rate!)

 GYRO_HW_FACTOR is given in the makefile.
*/

#define ACCEL_HW_FACTOR 204.8f

// If defined, acceleration is scaled to m/s^2. Else it is scaled to g's.
#define USE_MSSQUARED 1

/*
 * How many samples are added in one ADC loop, for pitch&roll and yaw,
 * respectively. This is = the number of occurences of each channel in the 
 * channelsForStates array in analog.c.
 */
#define GYRO_OVERSAMPLING_XY 8
#define GYRO_RATE_FACTOR_XY (GYRO_HW_FACTOR * GYRO_OVERSAMPLING_XY * GYRO_XY_CORRECTION)
#define GYRO_RATE_FACTOR_XY_MS (GYRO_RATE_FACTOR_XY * 1000.0)

#define GYRO_OVERSAMPLING_Z 4
#define GYRO_RATE_FACTOR_Z (GYRO_HW_FACTOR * GYRO_OVERSAMPLING_Z * GYRO_Z_CORRECTION)
#define GYRO_RATE_FACTOR_Z_MS (GYRO_RATE_FACTOR_Z * 1000.0)

#define ACCEL_OVERSAMPLING_XY 4
#define ACCEL_G_FACTOR_XY (ACCEL_HW_FACTOR * ACCEL_OVERSAMPLING_XY)
#define ACCEL_M_SSQUARED_FACTOR_XY (ACCEL_G_FACTOR_XY / 9.82f)

#define ACCEL_OVERSAMPLING_Z 2

// number of counts per g
#define ACCEL_G_FACTOR_Z (ACCEL_HW_FACTOR * ACCEL_OVERSAMPLING_Z)
// number of counts per m/s^2
#define ACCEL_M_SSQUARED_FACTOR_Z (ACCEL_G_FACTOR_Z / 9.82f)

#ifdef USE_MSSQUARED
#define ACCEL_FACTOR_XY ACCEL_M_SSQUARED_FACTOR_XY
#define ACCEL_FACTOR_Z ACCEL_M_SSQUARED_FACTOR_Z
#else
#define ACCEL_FACTOR_XY ACCEL_G_FACTOR_XY
#define ACCEL_FACTOR_Z ACCEL_G_FACTOR_Z
#endif
/*
 * Gyro saturation prevention.
 */
// How far from the end of its range a gyro is considered near-saturated.
#define SENSOR_MIN_XY (32 * GYRO_OVERSAMPLING_XY)
// Other end of the range (calculated)
#define SENSOR_MAX_XY (GYRO_OVERSAMPLING_XY * 1023 - SENSOR_MIN_XY)
// Max. boost to add "virtually" to gyro signal at total saturation.
#define EXTRAPOLATION_LIMIT 2500
// Slope of the boost (calculated)
#define EXTRAPOLATION_SLOPE (EXTRAPOLATION_LIMIT/SENSOR_MIN_XY)

#define X 0
#define Y 1
#define Z 2

// ADC channels
#define AD_GYRO_Z          0
#define AD_GYRO_X          1
#define AD_GYRO_Y          2
#define AD_AIRPRESSURE     3
#define AD_UBAT            4
#define AD_ACCEL_Z         5
#define AD_ACCEL_Y         6
#define AD_ACCEL_X         7

/*
 * The values that this module outputs
 * These first 2 exported arrays are zero-offset. The "PID" ones are used
 * in the attitude control as rotation rates. The "ATT" ones are for
 * integration to angles. For the same axis, the PID and ATT variables 
 * generally have about the same values. There are just some differences 
 * in filtering, and when a gyro becomes near saturated. 
 * Maybe this distinction is not really necessary.
 */
extern Vector3f gyro_attitude;

/*
 * The acceleration values that this module outputs. They are zero based.
 */
extern Vector3f accel;

#define GYRO_D_WINDOW_LENGTH 8

extern int16_t gyro_control[3];
extern int16_t gyroD[2];

extern int16_t UBat;

// 1:11 voltage divider, 1024 counts per 3V, and result is divided by 3.
#define UBAT_AT_5V (int16_t)((5.0 * (1.0/11.0)) * 1024 / (3.0 * 3))

extern sensorOffset_t gyroOffset;
extern sensorOffset_t accelOffset;
extern sensorOffset_t gyroAmplifierOffset;

typedef enum {
    CONTROL_SENSOR_SAMPLING_DATA,
    CONTROL_SENSOR_DATA_READY,
} ControlSensorDataStatus;

typedef enum {
    ATTITUDE_SENSOR_NO_DATA,
    ATTITUDE_SENSOR_DATA_READY,
    ATTITUDE_SENSOR_READING_DATA
} AttitudeSensorDataStatus;

extern volatile uint8_t analog_controlDataStatus;
extern volatile uint8_t analog_attitudeDataStatus;

/*
 * Air pressure.
 * The sensor has a sensitivity of 45 mV/kPa.
 * An approximate p(h) formula is = p(h[m])[kPa] = p_0 - 11.95 * 10^-3 * h
 * p(h[m])[kPa] = 101.3 - 11.95 * 10^-3 * h
 * 11.95 * 10^-3 * h = 101.3 - p[kPa]
 * h = (101.3 - p[kPa])/0.01195
 * That is: dV = -45 mV * 11.95 * 10^-3 dh = -0.53775 mV / m.
 * That is, with 38.02 * 1.024 / 3 steps per mV: -7 steps / m

Display pressures
4165 mV-->1084.7
4090 mV-->1602.4   517.7
3877 mV-->3107.8  1503.4

4165 mV-->1419.1
3503 mV-->208.1
Diff.:   1211.0

Calculated  Vout = 5V(.009P-0.095) --> 5V .009P = Vout + 5V 0.095 --> P = (Vout + 5V 0.095)/(5V 0.009)
4165 mV = 5V(0.009P-0.095)  P = 103.11 kPa  h = -151.4m
4090 mV = 5V(0.009P-0.095)  P = 101.44 kPa  h = -11.7m   139.7m
3877 mV = 5V(0.009P-0.095)  P = 96.7   kPa  h = 385m     396.7m

4165 mV = 5V(0.009P-0.095)  P = 103.11 kPa  h = -151.4m
3503 mV = 5V(0.009P-0.095)  P = 88.4   kPa  h = 384m  Diff: 1079.5m
Pressure at sea level: 101.3 kPa. voltage: 5V * (0.009P-0.095) = 4.0835V
This is OCR2 = 143.15 at 1.5V in --> simple pressure = 
*/

#define AIRPRESSURE_OVERSAMPLING 28
#define AIRPRESSURE_FILTER 8
// Minimum A/D value before a range change is performed.
#define MIN_RAWPRESSURE (200 * 2)
// Maximum A/D value before a range change is performed.
#define MAX_RAWPRESSURE (1023 * 2 - MIN_RAWPRESSURE)

#define MIN_RANGES_EXTRAPOLATION 15
#define MAX_RANGES_EXTRAPOLATION 240

#define PRESSURE_EXTRAPOLATION_COEFF 25L
#define AUTORANGE_WAIT_FACTOR 1

#define ABS_ALTITUDE_OFFSET 108205

void analog_init(void);

/*
 * This is really only for use for the ENC-03 code module, which needs to get the raw value
 * for its calibration. The raw value should not be used for anything else.
 */
uint16_t gyroValueForFC13DACCalibration(uint8_t axis);

/*
 * Start the conversion cycle. It will stop automatically.
 */
void startADCCycle(void);
void waitADCCycle(uint16_t delay);

/*
 */
void analog_updateControlData(void);
void analog_sumAttitudeData(void);
void analog_updateAttitudeData(void);

/*
 * Read gyro and acc.meter calibration from EEPROM.
 */
void analog_setNeutral(void);

/*
 * Zero-offset gyros and write the calibration data to EEPROM.
 */
void analog_calibrateGyros(void);

/*
 * Zero-offset accelerometers and write the calibration data to EEPROM.
 */
void analog_calibrateAcc(void);


void analog_setGround(void);
int32_t analog_getHeight(void);
int16_t analog_getDHeight(void);

#endif //_ANALOG_H
