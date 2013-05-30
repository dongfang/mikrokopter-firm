#include <avr/boot.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#include "timer0.h"
#include "timer2.h"
#include "uart0.h"
//#include "uart1.h"
#include "output.h"
#include "attitude.h"
//#include "commands.h"
//#include "flight.h"
//#include "profiler.h"
#include "rc.h"
#include "analog.h"
#include "configuration.h"
#include "twimaster.h"
#include "controlMixer.h"
#include "eeprom.h"
#include "beeper.h"
#ifdef USE_MK3MAG
#include "mk3mag.h"
#endif

int16_t main(void) {
	uint16_t timer;

	// disable interrupts global
	cli();

	// analyze hardware environment
	setCPUType();
	setBoardRelease();

	fdevopen(uart_putchar, NULL);

	// disable watchdog
	MCUSR &= ~(1 << WDRF);
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	WDTCSR = 0;

	// initalize modules
	output_init();
	timer0_init();
	// timer2_init();
	usart0_init();
	//if (CPUType == ATMEGA644P);// usart1_Init();
	RC_Init();
	I2C_init();
	analog_init();
#ifdef USE_MK3MAG
	MK3MAG_init();
#endif
#ifdef USE_DIRECT_GPS
	usart1_init();
#endif

	// Parameter Set handling
	IMUConfig_readOrDefault();
	channelMap_readOrDefault();
	outputMixer_readOrDefault();
	paramSet_readOrDefault();

	// enable interrupts global
	sei();

	printf("\n\r===================================");
	printf("\n\rFlightControl");
	printf("\n\rHardware: Custom");
	printf("\n\r     CPU: Atmega644");
	if (CPUType == ATMEGA644P)
		printf("p");
	printf("\n\rSoftware: V%d.%d%c", VERSION_MAJOR, VERSION_MINOR,
			VERSION_PATCH + 'a');
	printf("\n\r===================================");

	// Wait for a short time (otherwise the RC channel check won't work below)
	// timer = SetDelay(500);
	// while(!CheckDelay(timer));

	// Instead, while away the time by flashing the 2 outputs:
	// First J16, then J17. Makes it easier to see which is which.
	timer = setDelay(200);
	output_setLED(0, 1);
	GRN_OFF;
	RED_ON;
	while (!checkDelay(timer))
		;

	timer = setDelay(200);
	output_setLED(0, 0);
	output_setLED(1, 1);
	RED_OFF;
	GRN_ON;
	while (!checkDelay(timer))
		;

	timer = setDelay(200);
	while (!checkDelay(timer))
		;
	output_setLED(1, 0);
	GRN_OFF;

	twi_diagnostics();

	printf("\n\r===================================");

#ifdef USE_NAVICTRL
	printf("\n\rSupport for NaviCtrl");
#endif

#ifdef USE_DIRECT_GPS
	printf("\n\rDirect (no NaviCtrl) navigation");
#endif

	controlMixer_setNeutral();

	I2CTimeout = 5000;

	// Cal. attitude sensors and reset integrals.
	attitude_setNeutral();

	// Init flight parameters
	// flight_setNeutral();

	beep(2000);
	printf("\n\n\r");

	timer2_init();
	
	/*
	 * Main loop updates attitude and does some nonessential tasks
	 * like beeping alarms and computing servo values.
	 */
	while (1) {
		attitude_update();

        // This is fair to leave here - servo values only need update after a change in attitude anyway.
        // calculateServoValues();

		if (UBat <= UBAT_AT_5V || UBat >= staticParams.batteryVoltageWarning) {
			// Do nothing. The voltage on the input side of the regulator is <5V;
			// we must be running off USB power. Keep it quiet.
          MKFlags &= ~MKFLAG_LOWBAT;
		} else {
			beepBatteryAlarm();
			MKFlags |= MKFLAG_LOWBAT;
		}
	}
	return 1;
}
