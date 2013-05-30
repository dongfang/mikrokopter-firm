/*********************************************************************************/
/* Flight Control                                                                */
/*********************************************************************************/

#ifndef _FLIGHT_H
#define _FLIGHT_H

#include <inttypes.h>
#include "timer0.h"

typedef enum {
  FM_UNINITALIZED,
  FM_RETURN_TO_LEVEL,
  FM_HEADING_HOLD,
  FM_RATE
} FlightMode_t;

// Max, yaw error to correct. If error is greater, the target heading is pulled towards current heading.
#define YAW_I_LIMIT (int)(PI / 3.0f * INT16DEG_PI_FACTOR)

// A PI/4 attitude error with average PFactor (PID_NORMAL_VALUE) should make a 1024<<16 diff.
// PI/4 * INT16DEG_PI_FACTOR * PID_NORMAL_VALUE * x = (CONTROL_INPUT_HARDWARE_RANGE<<16)/2 --> 
// x = (CONTROL_INPUT_HARDWARE_RANGE<<15)*4 / (PID_NORMAL_VALUE * PI * INT16DEG_PI_FACTOR)
// x = CONTROL_INPUT_HARDWARE_RANGE<<17 / (PID_NORMAL_VALUE * PI * (1<<15) / PI)
// x = CONTROL_INPUT_HARDWARE_RANGE<<2 / PID_NORMAL_VALUE = about 82
#define ATT_P_SCALER_LSHIFT16 (int)(((1L<<(LOG_CONTROL_RANGE+2)) / PID_NORMAL_VALUE) + 0.5) 

// A full control input (=half range) with average PFactor (PID_NORMAL_VALUE) should get a rate to drive gyros to PI/sec (say).
// PI/s * x * PID_NORMAL_VALUE = CONTROL_INPUT_HARDWARE_RANGE/2
// PI/s is PI * GYRO_RATE_FACTOR_XY units.
// PI * GYRO_RATE_FACTOR_XY * x * PID_NORMAL_VALUE = CONTROL_INPUT_HARDWARE_RANGE/2
// x = CONTROL_INPUT_HARDWARE_RANGE/(2*PI*GYRO_RATE_FACTOR_XY*PID_NORMAL_VALUE) = about 379
#define RATE_P_SCALER_LSHIFT16 (int)((1L<<(LOG_CONTROL_RANGE+16)) / (2*PI*GYRO_RATE_FACTOR_XY*PID_NORMAL_VALUE) + 0.5)

// A full control input (=half range) with average PFactor (PID_NORMAL_VALUE) should get a rate to drive gyros to PI/2sec (say).
// PI/2s * x * PID_NORMAL_VALUE = CONTROL_INPUT_HARDWARE_RANGE/2
// PI/2/s is PI/2 * GYRO_RATE_FACTOR_Z units.
// PI/2 * GYRO_RATE_FACTOR_Z * x * PID_NORMAL_VALUE = CONTROL_INPUT_HARDWARE_RANGE/2
// PI * GYRO_RATE_FACTOR_Z * x * PID_NORMAL_VALUE = CONTROL_INPUT_HARDWARE_RANGE
// x = CONTROL_INPUT_HARDWARE_RANGE / (PI * GYRO_RATE_FACTOR_Z * PID_NORMAL_VALUE)
// x<<16 is about 1517
#define YAW_RATE_SCALER_LSHIFT16 (int)((1L<<(LOG_CONTROL_RANGE+16)) / (PI*GYRO_RATE_FACTOR_Z*PID_NORMAL_VALUE) + 0.5)

// This is where control affects the target heading. It also (later) directly controls yaw.
// We want, at a normal yaw stick P, a PI/2s rate at full stick.
// That is, f_control * x * CONTROL_INPUT_HARDWARE_RANGE/2 = PI/2 * INT16DEG_PI_FACTOR= 1<<14
// f_control * x * CONTROL_INPUT_HARDWARE_RANGE = 1<<15
// x = 1<<15 / (CONTROL_INPUT_HARDWARE_RANGE * F_CONTROL) 
// 4194
#define YAW_STICK_INTEGRATION_SCALER_LSHIFT16 (int)(((1L<<(15+16-LOG_CONTROL_RANGE)) / F_CONTROL) + 0.5)

void flight_setMode(FlightMode_t _flightMode);
void flight_setParameters(void);

void flight_setGround(void);
void flight_takeOff(void);

void flight_control(void);

#endif //_FLIGHT_H
