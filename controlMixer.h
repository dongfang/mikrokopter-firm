#ifndef _CONTROLMIXER_H
#define _CONTROLMIXER_H

#include <inttypes.h>
/*
 * An attempt to define a generic control. That could be an R/C receiver, an external control
 * (serial over Bluetooth, Wi232, XBee, whatever) or the NaviCtrl.
 * This resembles somewhat an object-oriented class definition (except that there are no virtuals).
 * The idea is that the combination of different control inputs, of the way they superimpose upon
 * each other, the priorities between them and the behavior in case that one fails is simplified, 
 * and all in one place.
 */

/*
 * Our output.
 */
extern int16_t controls[4];
extern uint16_t controlActivity;
//extern uint16_t maxControl[2];

void controlMixer_initVariables(void);
//void controlMixer_updateVariables(void);

void controlMixer_setNeutral(void);

/*
 * Update the exported variables. Called at every flight control cycle.
 */
void controlMixer_periodicTask(void);

/*
 * Get the current command. See the COMMAND_.... define's
 */
uint8_t controlMixer_getCommand(void);

void controlMixer_performCalibrationCommands(uint8_t command);

uint8_t controlMixer_getSignalQuality(void);
extern uint8_t controlMixer_didReceiveSignal;


/*
 * Gets the argument for the current command (a number).
 * 
 * Stick position to argument values (for stick control):
 * 2--3--4
 * |     |  +
 * 1  9  5  ^ 0
 * |     |  |  
 * 8--7--6
 *    
 * + <--
 *    0
 * 
 * Not in any of these positions: 0
 */
uint8_t controlMixer_getArgument(void);
uint8_t controlMixer_isCommandRepeated(void);

#endif
