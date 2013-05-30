#ifndef _MK3MAG_H
#define _MK3MAG_H

typedef struct {
	int16_t attitude[2];
	uint8_t userParam[2];
	uint8_t calState;
	uint8_t orientation;
} ToMk3Mag_t;

extern ToMk3Mag_t toMk3Mag;
extern volatile uint16_t volatileMagneticHeading;

// Initialization
void MK3MAG_init(void);

// should be called cyclic to get actual compass heading
void MK3MAG_periodicTask(void);

#endif //_MK3MAG_H
