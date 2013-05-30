#include "timer0.h"
#include "configuration.h"

#include <avr/io.h>

/*
 * Configuration for my prototype board with InvenSense gyros.
 * The FC 1.3 board is installed upside down, therefore Z acc is reversed but not roll.
 */

// The special auto-zero feature of this gyro needs a port.
#define AUTOZERO_PORT PORTD
#define AUTOZERO_DDR DDRD
#define AUTOZERO_BIT 5

void gyro_calibrate(void) {
	// If port not already set to output and high, do it.
	if (!(AUTOZERO_DDR & (1 << AUTOZERO_BIT)) || !(AUTOZERO_PORT & (1 << AUTOZERO_BIT))) {
		AUTOZERO_PORT |= (1 << AUTOZERO_BIT);
		AUTOZERO_DDR |= (1 << AUTOZERO_BIT);
		delay_ms(100);
	}

	// Make a pulse on the auto-zero output line.
	AUTOZERO_PORT &= ~(1 << AUTOZERO_BIT);
	delay_ms(1);
	AUTOZERO_PORT |= (1 << AUTOZERO_BIT);
	// Delay_ms(10);
	delay_ms_with_adc_measurement(100, 0);
}

void gyro_init(void) {
	gyro_calibrate();
}

void gyro_setDefaultParameters(void) {
  staticParams.gyroD = 3;
  IMUConfig.driftCompDivider = 2;
  IMUConfig.driftCompLimit = 5;
  IMUConfig.zerothOrderCorrection = 1;
  IMUConfig.imuReversedFlags = IMU_REVERSE_ACC_Z;
}
