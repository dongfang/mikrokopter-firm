// Or does this simply belong in uart0.h??
#ifndef _EXTERNALCONTROL_H
#define _EXTERNALCONTROL_H

#include<inttypes.h>

typedef struct {
	uint8_t digital[2];
	uint8_t remoteButtons;
	int8_t  pitch;
	int8_t  roll;
	int8_t  yaw;
	uint8_t throttle;
	int8_t  height;
	uint8_t command; // Let's use that for commands now.
	uint8_t frame;
	uint8_t argument; // Let's use that for arguemnts.
}__attribute__((packed)) ExternalControl_t;

extern ExternalControl_t externalControl;
extern volatile uint8_t externalControlActive;

void EC_periodicTaskAndRPTY(int16_t* RPTY);
uint8_t EC_getArgument(void);
uint8_t EC_getCommand(void);
int16_t EC_getVariable(uint8_t varNum);
void EC_calibrate(void);
uint8_t EC_getSignalQuality(void);
void EC_setNeutral(void);

#endif
