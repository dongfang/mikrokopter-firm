#ifndef _PROFILER_H
#define _PROFILER_H

#include <inttypes.h>
#include <avr/pgmspace.h>

#define UNACCOUNTED 0

#define ANALOG_UPDATE 1
#define MATRIX_UPDATE1 2
#define MATRIX_UPDATE2 3
#define MATRIX_NORMALIZE1 4
#define MATRIX_NORMALIZE2 5
#define DRIFT_CORRECTION 6
#define CHECK_MATRIX 7
#define EULER_ANGLES 8
#define ANGLESOUTPUT 9

#define CONTROLMIXER 10
#define COMMANDS 11
#define FLIGHTCONTROL 12
#define UART 13
#define OUTPUTS 14

//extern uint8_t currentProfiledActivity;
extern volatile uint16_t activitiesTimerHits[16];
extern volatile uint32_t totalProfilerHits;
void setCurrentProfiledActivity(uint8_t what);
void reset(void);
void profiler_scoreTimerHit(void);

extern PGM_P PROFILER_LABELS[] PROGMEM;

#endif

