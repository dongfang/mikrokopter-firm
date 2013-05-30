#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
//#include "eeprom.h"
#include "profiler.h"
#include "controlMixer.h"
#include "configuration.h"
#include "analog.h"
#include "timer0.h"
#include "debug.h"
#include "beeper.h"
#include "output.h"
#include "commands.h"
#include "flight.h"
#include "uart0.h"
#include "twimaster.h"

#ifdef USE_MK3MAG
#include "mk3mag.h"
#endif

#define MILLIS_DIVIDER 10

volatile uint32_t jiffiesClock;
volatile uint32_t millisClock;
volatile uint8_t  loopJiffiesClock;
volatile uint16_t beepTime;
volatile uint16_t beepModulation = BEEP_MODULATION_NONE;

volatile uint8_t flightControlStatus;

/*****************************************************
 * Initialize Timer 0                   
 *****************************************************/
// timer 0 is used for the PWM generation to control the offset voltage at the air pressure sensor
// Its overflow interrupt routine is used to generate the beep signal and the flight control motor update rate
void timer0_init(void) {
	uint8_t sreg = SREG;

	// disable all interrupts before reconfiguration
	cli();

	// Configure speaker port as output.
	if (boardRelease == 10) { // Speaker at PD2
		DDRD |= (1 << DDD2);
		PORTD &= ~(1 << PORTD2);
	} else { // Speaker at PC7
		DDRC |= (1 << DDC7);
		PORTC &= ~(1 << PORTC7);
	}

	// set PB3 and PB4 as output for the PWM used as offset for the pressure sensor
	DDRB |= (1 << DDB4) | (1 << DDB3);
	PORTB &= ~((1 << PORTB4) | (1 << PORTB3));

	// Timer/Counter 0 Control Register A

	// Waveform Generation Mode is Fast PWM (Bits WGM02 = 0, WGM01 = 1, WGM00 = 1)
	// Clear OC0A on Compare Match, set OC0A at BOTTOM, noninverting PWM (Bits COM0A1 = 1, COM0A0 = 0)
	// Clear OC0B on Compare Match, set OC0B at BOTTOM, (Bits COM0B1 = 1, COM0B0 = 0)
	TCCR0A &= ~((1 << COM0A0) | (1 << COM0B0));
	TCCR0A |= (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);

	// Timer/Counter 0 Control Register B
	// set clock divider for timer 0 to SYSCLOCK/8 = 20MHz/8 = 2.5MHz
	// i.e. the timer increments from 0x00 to 0xFF with an update rate of 2.5 MHz
	// hence the timer overflow interrupt frequency is 2.5 MHz/256 = 9.765 kHz
	// divider 8 (Bits CS02 = 0, CS01 = 1, CS00 = 0)
	TCCR0B &= ~((1 << FOC0A) | (1 << FOC0B) | (1 << WGM02));
	TCCR0B = (TCCR0B & 0xF8) | (0 << CS02) | (1 << CS01) | (0 << CS00);

	// initialize the Output Compare Register A & B used for PWM generation on port PB3 & PB4
	OCR0A = 0; // for PB3
	OCR0B = 120; // for PB4

	// init Timer/Counter 0 Register
	TCNT0 = 0;

	// Timer/Counter 0 Interrupt Mask Register
	// enable timer overflow interrupt only
	TIMSK0 &= ~((1 << OCIE0B) | (1 << OCIE0A));
	TIMSK0 |= (1 << TOIE0);

	SREG = sreg;
}

void runFlightControlTask(void) {
	if (flightControlStatus != NOT_RUNNING) {
		// Previous execution not completed! It is dangerous to start another.
		debugOut.digital[0] |= DEBUG_MAINLOOP_TIMER;
		return;
	}

	debugOut.digital[0] &= ~DEBUG_MAINLOOP_TIMER;

	controlMixer_periodicTask();
	commands_handleCommands();
	flightControlStatus = RUNNING;

    if (!--I2CTimeout || missingMotor) { // try to reset the i2c if motor is missing or timeout
         if (!I2CTimeout) {
             I2C_reset();
             I2CTimeout = 5;
         }
         beepI2CAlarm();
     }
	
	if (analog_controlDataStatus != CONTROL_SENSOR_DATA_READY) {
		// Analog data should have been ready but is not!!
		debugOut.digital[1] |= DEBUG_MAINLOOP_TIMER;
	} else {
		debugOut.digital[1] &= ~DEBUG_MAINLOOP_TIMER;
		J4HIGH;
		analog_sumAttitudeData();
		analog_updateControlData();
		flight_control();
		output_applyMulticopterMixer();
		I2C_start(TWI_STATE_MOTOR_TX);
		J4LOW;
	}

	flightControlStatus = NOT_RUNNING;
}

/*****************************************************/
/*          Interrupt Routine of Timer 0             */
/*****************************************************/
ISR (TIMER0_OVF_vect) { // 9765.625 Hz
	static uint8_t millisDivider = MILLIS_DIVIDER;
	static uint8_t controlLoopDivider = CONTROLLOOP_DIVIDER;
	static uint8_t serialLoopDivider = SERIALLOOP_DIVIDER /2;
    static uint8_t outputLoopDivider = OUTPUTLOOP_DIVIDER /3;
	uint8_t beeperOn = 0;

	jiffiesClock++;
	loopJiffiesClock++;
	// profiler_scoreTimerHit();

	sei();

	if (!--millisDivider) {
		millisClock++;
		millisDivider = MILLIS_DIVIDER;
	}

	if (!--controlLoopDivider) {
	    //sei();
		controlLoopDivider= CONTROLLOOP_DIVIDER;
		runFlightControlTask();
		//cli();
	}

    if (!--serialLoopDivider) {
        //sei();
        serialLoopDivider= SERIALLOOP_DIVIDER;
        // Allow serial data transmission if there is still time, or if we are not flying anyway.
        usart0_transmitTxData();
        usart0_processRxData();
        //cli();
    }

    if (!--outputLoopDivider) {
        //sei();
        outputLoopDivider= OUTPUTLOOP_DIVIDER;
        output_update();
        //cli();
    }

	// beeper on if duration is not over
	if (beepTime) {
		beepTime--; // decrement BeepTime
		if (beepTime & beepModulation)
			beeperOn = 1;
		else
			beeperOn = 0;
	} else { // beeper off if duration is over
		beeperOn = 0;
		beepModulation = BEEP_MODULATION_NONE;
	}

	if (beeperOn) {
		// set speaker port to high.
		if (boardRelease == 10)
			PORTD |= (1 << PORTD2); // Speaker at PD2
		else
			PORTC |= (1 << PORTC7); // Speaker at PC7
	} else { // beeper is off
		// set speaker port to low
		if (boardRelease == 10)
			PORTD &= ~(1 << PORTD2);// Speaker at PD2
		else
			PORTC &= ~(1 << PORTC7);// Speaker at PC7
	}

#ifdef USE_MK3MAG
	// update compass value if this option is enabled in the settings
	if (staticParams.bitConfig & CFG_COMPASS_ENABLED) {
		MK3MAG_periodicTask(); // read out mk3mag pwm
	}
#endif
}

// -----------------------------------------------------------------------
uint16_t setDelay(uint16_t t) {
	return (millisClock + t - 1);
}

// -----------------------------------------------------------------------
int8_t checkDelay(uint16_t t) {
	return (((t - millisClock) & 0x8000) >> 8); // check sign bit
}

// -----------------------------------------------------------------------
void delay_ms(uint16_t w) {
	uint16_t t_stop = setDelay(w);
	while (!checkDelay(t_stop))
		;
}
