#include "configuration.h"

void gyro_calibrate(void) {
  // Nothing to calibrate.
}

void gyro_init(void) {
  // No amplifiers, no DAC.
}

void gyro_setDefaultParameters(void) {
  IMUConfig.gyroQuadrant = 4;
  IMUConfig.accQuadrant = 4;
  IMUConfig.imuReversedFlags = IMU_REVERSE_ACCEL_Z;
  // staticParams.gyroD = 5;
}
