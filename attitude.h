/*********************************************************************************/
/* Attitude sense system (processing of gyro, accelerometer and altimeter data)  */
/*********************************************************************************/

#ifndef _ATTITUDE_H
#define _ATTITUDE_H

#include <inttypes.h>
#include <math.h>

#include "analog.h"
//#include "timer0.h"
//#define _PI 3.1415926535897932384626433832795

#define ACC_MS2_FACTOR (1.0f / ACC_G_FACTOR_XY)

// 57.3
#define RAD_DEG_FACTOR (180.0 / M_PI)

/*
 * Attitudes calculated by numerical integration of gyro rates
 */
extern int16_t attitude[3];

/*
 * Re-init flight attitude, setting all angles to 0 (or to whatever can be derived from acc. sensor).
 * To be called when the pilot commands gyro calibration (eg. by moving the left stick up-left or up-right).
 */
void attitude_setNeutral(void);

/*
 * Main routine, called from the flight loop.
 */
void attitude_update(void);

#endif //_ATTITUDE_H
