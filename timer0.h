#ifndef _TIMER0_H
#define _TIMER0_H

#include <inttypes.h>

// Normally it is 20MHz/2048 = 9765.625 Hz
#define F_TIMER0IRQ ((float)F_CPU/2048.0)
#define T_TIMER0IRQ ((float)2048.0/(float)F_CPU)
// About 250 Hz
#define F_CONTROL 250
#define CONTROLLOOP_DIVIDER (F_TIMER0IRQ/F_CONTROL)
#define SERIALLOOP_DIVIDER 61
#define OUTPUTLOOP_DIVIDER 100

extern volatile uint32_t jiffiesClock;
extern volatile uint32_t millisClock;
extern volatile uint8_t  loopJiffiesClock;

extern void timer0_init(void);
extern void delay_ms(uint16_t w);
extern uint16_t setDelay(uint16_t t);
extern int8_t checkDelay(uint16_t t);

typedef enum {
  NOT_RUNNING = 0,
  RUNNING,
  BLOCKED_FOR_CALIBRATION
} FlightControlRunStatus;

extern volatile uint8_t flightControlStatus;

#endif //_TIMER0_H
