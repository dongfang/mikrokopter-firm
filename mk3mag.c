#include <avr/io.h>
#include <stdlib.h>
#include <inttypes.h>
#include "timer0.h"
#include "output.h"
#include "eeprom.h"
#include "mk3mag.h"

uint8_t PWMTimeout = 12;
// This will updated in interrupt handler. Should not be processed by main, other than just atomic copy.
volatile uint16_t volatileMagneticHeading;
ToMk3Mag_t toMk3Mag;

/*********************************************/
/*  Initialize Interface to MK3MAG Compass   */
/*********************************************/
void MK3MAG_init(void) {
	// Port PC4 connected to PWM output from compass module
	DDRC &= ~(1 << DDC4); // set as input
	PORTC |= (1 << PORTC4); // pull up  to increase PWM counter also if nothing is connected
	PWMTimeout = 0;
	toMk3Mag.calState = 0;
	toMk3Mag.orientation = 1;
}

/*********************************************/
/*  Get PWM from MK3MAG                      */
/*********************************************/
void MK3MAG_periodicTask(void) {// called every 102.4 us by timer 0 ISR
	static uint16_t PWMCount = 0;
	static uint16_t beepDelay = 0;
	// static uint16_t debugCounter = 0;
	// The pulse width varies from 1ms (0°) to 36.99ms (359.9°)
	// in other words 100us/° with a +1ms offset.
	// The signal goes low for 65ms between pulses,
	// so the cycle time is 65mS + the pulse width.
	// pwm is high

	// if (debugCounter++ == 5000) {
    // DebugOut.Digital[0] ^= DEBUG_MK3MAG;
	// debugCounter = 0;
	// }

	if (PINC & (1 << PINC4)) {
		// If PWM signal is high increment PWM high counter
		// This counter is incremented by a periode of 102.4us,
		// i.e. the resoluton of pwm coded heading is approx. 1 deg.
		PWMCount++;
		// pwm overflow?
		if (PWMCount > 400) {
			if (PWMTimeout)
				PWMTimeout--; // decrement timeout
			volatileMagneticHeading = -1; // unknown heading
			PWMCount = 0; // reset PWM Counter
		}
	} else { // pwm is low
		// ignore pwm values values of 0 and higher than 37 ms;
		if ((PWMCount) && (PWMCount < 362)) { // 362 * 102.4us = 37.0688 ms
			if (PWMCount < 10)
			  volatileMagneticHeading = 0;
			else {
			  volatileMagneticHeading = ((uint32_t) (PWMCount - 10) * 1049L) / 1024; // correct timebase and offset
				//DebugOut.Digital[1] ^= DEBUG_MK3MAG; // correct signal recd.
			}
			/*
			 compassHeading - compassCourse on a -180..179 range.
			 compassHeading 20 compassCourse 30 --> ((540 - 10)%360) - 180 = -10
			 compassHeading 30 compassCourse 20 --> ((540 + 10)%360) - 180 = 10
			 compassHeading 350 compassCourse 10 --> ((540 + 340)%360) - 180 = -20
			 compassHeading 10 compassCourse 350 --> ((540 - 340)%360) - 180 = 20
			 */
			//compassOffCourse = ((540 + compassHeading - compassCourse) % 360) - 180;
			PWMTimeout = 12; // if 12 periodes long no valid PWM was detected the data are invalid
			// 12 * 362 counts * 102.4 us
		}
		PWMCount = 0; // reset pwm counter
	}

	if (!PWMTimeout) {
		if (checkDelay(beepDelay)) {
			if (!beepTime)
				beep(50); // make noise with 10Hz to signal the compass problem
			beepDelay = setDelay(100);
		}
	}
}

