#ifndef _SENSORS_H
#define _SENSORS_H

#include <inttypes.h>
#include "configuration.h"

extern sensorOffset_t gyroAmplifierOffset;

/*
 * Common procedures for all gyro types.
 * FC 1.3 hardware: Searching the DAC values that return neutral readings.
 * FC 2.0 hardware: Nothing to do.
 * InvenSense hardware: Output a pulse on the AUTO_ZERO line.
 */
void gyro_calibrate(void);

/*
 * FC 1.3: Output data in gyroAmplifierOffset to DAC. All other versions: Do nothing.
 */
void gyro_init(void);

/*
 * Set some default FC parameters, depending on gyro type: Drift correction etc.
 */
void gyro_setDefaultParameters(void);

#endif
