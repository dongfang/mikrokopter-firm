#ifndef _OUTPUT_H
#define _OUTPUT_H

#include <avr/io.h>
#include "configuration.h"

#define OUTPUT_HIGH(num)        {PORTC |=  (4 << (num));}
#define OUTPUT_LOW(num)         {PORTC &= ~(4 << (num));}
#define OUTPUT_TOGGLE(num) (    {PORTC ^=  (4 << (num));}

// Control terms (multicopter)
// TODO: Multicopter stuff separate?
extern int16_t throttleTerm;
extern int32_t yawTerm, term[2];

// I2C and PWM motor and servo outputs.
// extern int16_t outputs[NUM_OUTPUTS];


/*
 * Set to 0 for using outputs as the usual flashing lights.
 * Set to one of the DEBUG_... defines h for using the outputs as debug lights.
 */
#define DIGITAL_DEBUG_MASK 0

void output_init(void);
void output_setLED(uint8_t num, uint8_t state);
void output_update(void);
void output_applyMulticopterMixer(void);
void output_setParameters(void);

#endif //_output_H
