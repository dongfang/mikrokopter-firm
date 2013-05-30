#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer2.h"

#if (SERVO_RESOLUTION == COARSE)
#define CS2 ((1<<CS21)|(1<<CS20))
#else
#define CS2 (1<<CS21)
#endif

#define FRAMELEN (PULSELENGTH_2200 * staticParams.servoCount + 128)

// volatile uint8_t servoActive = 0;
// volatile uint8_t recalculateServoTimes = 0;
volatile uint16_t pwmChannels[MAX_PWMCHANNELS];

#define HEF4017R_ON     PORTC |=  (1<<PORTC6)
#define HEF4017R_OFF    PORTC &= ~(1<<PORTC6)

/*****************************************************
 *              Initialize Timer 2
 *****************************************************/
void timer2_init(void) {
    uint8_t sreg = SREG;

    // disable all interrupts before reconfiguration
    cli();

    // set PD7 as output of the PWM for pitch servo
    DDRD |= (1 << DDD7);
    PORTD &= ~(1 << PORTD7); // set PD7 to low

    DDRC |= (1 << DDC6); // set PC6 as output (Reset for HEF4017)
    HEF4017R_ON; // enable reset

    // Timer/Counter 2 Control Register A
    // Timer Mode is CTC (Bits: WGM22 = 0, WGM21 = 1, WGM20 = 0)
    // PD7: Output OCR2 match, (Bits: COM2A1 = 1, COM2A0 = 0)
    // PD6: Normal port operation, OC2B disconnected, (Bits: COM2B1 = 0, COM2B0 = 0)
    TCCR2A &= ~((1 << COM2A0) | (1 << COM2B1) | (1 << COM2B0) | (1 << WGM20) | (1 << WGM22));
    TCCR2A |= (1 << COM2A1) | (1 << WGM21);

    // Timer/Counter 2 Control Register B

    // Set clock divider for timer 2 to 20MHz / 8 = 2.5 MHz
    // The timer increments from 0x00 to 0xFF with an update rate of 2.5 kHz or 0.4 us
    // hence the timer overflow interrupt frequency is 625 kHz / 256 = 9.765 kHz or 0.1024ms

    TCCR2B &= ~((1 << FOC2A) | (1 << FOC2B) | (1 << CS20) | (1 << CS21) | (1 << CS22));
    TCCR2B |= CS2;

    // Initialize the Timer/Counter 2 Register
    TCNT2 = 0;

    // Initialize the Output Compare Register A used for signal generation on port PD7.
    OCR2A = 255;

    // Timer/Counter 2 Interrupt Mask Register
    // Enable timer output compare match A Interrupt only
    TIMSK2 &= ~((1 << OCIE2B) | (1 << TOIE2));
    TIMSK2 |= (1 << OCIE2A);

    SREG = sreg;
}

/*
void servo_On(void) {
    servoActive = 1;
}
void servo_Off(void) {
    servoActive = 0;
    HEF4017R_ON; // enable reset
}
*/

/*****************************************************
 * Control Servo Position
 *****************************************************/

/*
void calculateServoValues(void) {
  if (!recalculateServoTimes) return;
  for (uint8_t axis=0; axis<MAX_SERVOS; axis++) {
    servoValues[axis] = servoValue(axis);
  }
  recalculateServoTimes = 0;
}
*/

ISR(TIMER2_COMPA_vect) {
  static uint16_t remainingPulseTime;
  static uint8_t servoIndex = 0;
  static uint16_t sumOfPulseTimes = 0;

  if (!remainingPulseTime) {
    // Pulse is over, and the next pulse has already just started. Calculate length of next pulse.
    if (servoIndex < staticParams.servoCount) {
      // There are more signals to output.
      sumOfPulseTimes += (remainingPulseTime = pwmChannels[servoIndex]); //pwmChannels[servoMap[servoIndex]]);
      servoIndex++;
    } else {
      // There are no more signals. Reset the counter and make this pulse cover the missing frame time.
      remainingPulseTime = FRAMELEN - sumOfPulseTimes;
      sumOfPulseTimes = servoIndex = 0;
      HEF4017R_ON;
    }
  }
  
  // Schedule the next OCR2A event. The counter is already reset at this time.
  if (remainingPulseTime > 256+128) {
    // Set output to reset to zero at next OCR match. It does not really matter when the output is set low again,
    // as long as it happens once per pulse. This will, because all pulses are > 255+128 long.
    OCR2A = 255;
    TCCR2A &= ~(1<<COM2A0);
    remainingPulseTime -= 256;
  } else if (remainingPulseTime > 256) {
    // Remaining pulse lengths in the range [256..256+128] might cause trouble if handled the standard
    // way, which is in chunks of 256. The remainder would be very small, possibly causing an interrupt on interrupt
    // condition. Instead we now make a chunk of 128. The remaining chunk will then be in [128..255] which is OK.
    OCR2A=127;
    remainingPulseTime -= 128;
  } else {
    // Set output to high at next OCR match. This is when the 4017 counter will advance by one. Also set reset low
    TCCR2A |= (1<<COM2A0);
    OCR2A = remainingPulseTime-1;
    remainingPulseTime = 0;
    HEF4017R_OFF; // implement servo-disable here, by only removing the reset signal if ServoEnabled!=0.
  }
}
