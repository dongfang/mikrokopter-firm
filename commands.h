#ifndef _COMMANDS_H
#define _COMMANDS_H

#include <inttypes.h>
/*
 * An enumeration over the  start motors, stop motors, calibrate gyros
 * and calibreate acc. meters commands.
 */
#define COMMAND_NONE            0
#define COMMAND_GYROCAL         1
#define COMMAND_ACCCAL          2
#define COMMAND_GYRO_ACC_CAL    3
#define COMMAND_STOP            10
#define COMMAND_START           11

extern uint8_t compassCalState;

void commands_handleCommands(void);

#ifdef USE_MK3MAG
uint8_t commands_isCalibratingCompass(void);
#endif

#endif
