#ifndef _TIMER2_H
#define _TIMER2_H

#include <inttypes.h>
#include <math.h>
#include "configuration.h"

extern volatile uint16_t pwmChannels[MAX_PWMCHANNELS];

#define SERVO_RESOLUTION FINE

#if (SERVO_RESOLUTION == COARSE)
#define F_TIMER2 (F_CPU/32)
#define CS2 ((1<<CS21)|(1<<CS20))
#else
#define F_TIMER2(F_CPU/8)
#define CS2 (1<<CS21)
#endif

//#define ABSOLUTE_MIN ((float)F_TIMER2*0.00075 + 0.5)
//#define ABSOLUTE_MAX ((float)F_TIMER2*0.00225 + 0.5)

#define PULSELENGTH_800  ((float)F_TIMER2*0.0008)
#define PULSELENGTH_1000 ((float)F_TIMER2*0.0010)
#define PULSELENGTH_1500 ((float)F_TIMER2*0.0015)
#define PULSELENGTH_2000 ((float)F_TIMER2*0.0020)
#define PULSELENGTH_2200 ((float)F_TIMER2*0.0022)

#define PWM_CONTROL_SCALE_FACTOR (PULSELENGTH_1000/(float)CONTROL_RANGE)
#define PWM_BYTE_SCALE_FACTOR (PULSELENGTH_1000/256.0)

// Prevent damage even if grossly misconfigured:
// Multiply this by angle in int16-degrees and again by 100 to get servo deflection to same angle 
// (assuming 90 degrees over 1 ms, which is the norm):

void timer2_init(void);
//void calculateServoValues(void);
#endif //_TIMER2_H
