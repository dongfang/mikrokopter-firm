#include "sensors.h"
#include "analog.h"
#include "twimaster.h"
#include "configuration.h"
#include "eeprom.h"
#include "timer0.h"
#include "output.h"

#include <stdio.h>

void I2C_OutputAmplifierOffsets(void) {
	uint16_t timeout = setDelay(2000);
	I2C_Start(TWI_STATE_GYRO_OFFSET_TX); // initiate data transmission
	// Wait for I2C to finish transmission.
	while (twi_state) {
		// Did it take too long?
		if (checkDelay(timeout)) {
			printf("\r\n DAC or I2C Error! check I2C, 3Vref, DAC, and BL-Ctrl");
			break;
		}
	}
}

uint8_t DACChannelFor(uint8_t axis) {
	switch (axis) {
	case X:
		return 1;
    case Y:
        return 0;
	case Z:
		return 2;
	default:
		return -1; // should never happen.
	}
}

void gyro_calibrate(void) {				
	printf("gyro_calibrate");
	uint8_t i, axis, factor, numberOfAxesInRange = 0;
	// GyroDefectNick = 0; GyroDefectRoll = 0; GyroDefectYaw = 0;

	for (i = 200; i != 0; i--) {
		waitADCCycle(i <= 10 ? 10 : 2);
		// If all 3 axis are in range, shorten the remaining number of iterations.
		if (numberOfAxesInRange == 3 && i > 10) i = 10;
		numberOfAxesInRange = 0;
		for (axis=X; axis<=Z; axis++) {
			uint8_t dacChannel = DACChannelFor(axis);
			if (axis==Z)
				factor=GYRO_OVERSAMPLING_Z;
			else
				factor=GYRO_OVERSAMPLING_XY;

			if (gyroValueForFC13DACCalibration(axis) < (uint16_t)(510*factor))
				gyroAmplifierOffset.offsets[dacChannel]--;
			else if (gyroValueForFC13DACCalibration(axis) > (uint16_t)(515 * factor))
				gyroAmplifierOffset.offsets[dacChannel]++;
			else
				numberOfAxesInRange++;

			/* Gyro is defective. But do keep DAC within bounds (it's an op amp not a differential amp). */
			if (gyroAmplifierOffset.offsets[dacChannel] < 10) {
				gyroAmplifierOffset.offsets[dacChannel] = 10;
				versionInfo.hardwareErrors[0] |= (FC_ERROR0_GYRO_X << axis);
			} else if (gyroAmplifierOffset.offsets[dacChannel] > 245) {
				gyroAmplifierOffset.offsets[dacChannel] = 245;
				versionInfo.hardwareErrors[0] |= (FC_ERROR0_GYRO_X << axis);
			}
		}
		
		I2C_OutputAmplifierOffsets();
	}
	gyroAmplifierOffset_writeToEEProm();
	waitADCCycle(70);
}

void gyro_init(void) {
  if (gyroAmplifierOffset_readFromEEProm()) {
    printf("gyro amp invalid, recalibrate.");
	gyroAmplifierOffset.offsets[X] =
		gyroAmplifierOffset.offsets[Y] =
		gyroAmplifierOffset.offsets[Z] = (uint8_t)(255 * 1.2089 / 3.0);
  } else {
	I2C_OutputAmplifierOffsets();
  }
}

void gyro_setDefaultParameters(void) {
  IMUConfig.gyroQuadrant = 4;
  IMUConfig.accQuadrant = 4;
  IMUConfig.imuReversedFlags = IMU_REVERSE_ACCEL_Z | IMU_REVERSE_GYRO_Z;
  staticParams.gyroD = 3;
}
