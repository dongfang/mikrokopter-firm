#ifndef _RC_H
#define _RC_H

#include <inttypes.h>
#include "configuration.h"

// Number of cycles a command must be repeated before commit.
#define COMMAND_TIMER 100

extern void RC_Init(void);
// the RC-Signal. todo: Not export any more.
extern volatile int16_t PPM_in[MAX_CONTROLCHANNELS];
// extern volatile int16_t PPM_diff[MAX_CHANNELS];	// the differentiated RC-Signal. Should that be exported??
extern volatile uint8_t NewPpmData; // 0 indicates a new recieved PPM Frame
extern volatile uint8_t RCQuality;  // rc signal quality indicator (0 to 200)

// defines for lookup staticParams.ChannelAssignment
#define CH_PITCH       	0
#define CH_ROLL		    1
#define CH_THROTTLE	    2
#define CH_YAW		    3
#define CH_VARIABLES    4
#define VARIABLES_OFFSET 120

// void RC_periodicTask(void);
void RC_periodicTaskAndRPTY(int16_t* RPTY);
uint8_t RC_getArgument(void);
uint8_t RC_getCommand(void);
int16_t RC_getVariable(uint8_t varNum);
void RC_calibrate(void);
uint8_t RC_getSignalQuality(void);
uint8_t RC_getLooping(uint8_t looping);
#ifdef USE_MK3MAG
uint8_t RC_testCompassCalState(void);
#endif
#endif //_RC_H
