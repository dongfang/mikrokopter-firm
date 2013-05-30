#include <stdlib.h>
#include "controlMixer.h"
#include "rc.h"
#include "heightControl.h"
#include "externalControl.h"
#include "compassControl.h"
#include "failsafeControl.h"
#include "naviControl.h"
#include "configuration.h"
//#include "attitude.h"
#include "commands.h"
#include "debug.h"
#include "definitions.h"

// uint16_t maxControl[2] = { 0, 0 };
// uint16_t controlActivity = 0;
int16_t controls[4];

// Internal variables for reading commands made with an R/C stick.
uint8_t lastCommand;
uint8_t lastArgument;

uint8_t isCommandRepeated;
uint8_t controlMixer_didReceiveSignal;

/*
 * This could be expanded to take arguments from ohter sources than the RC 
 * (read: Custom MK RC project)
 */
uint8_t controlMixer_getArgument(void) {
  return lastArgument;
}

/*
 * This could be expanded to take calibrate / start / stop commands from ohter sources 
 * than the R/C (read: Custom MK R/C project)
 */
uint8_t controlMixer_getCommand(void) {
  return lastCommand;
}

uint8_t controlMixer_isCommandRepeated(void) {
  return isCommandRepeated;
}

/*
 * TODO: This assumes R/C as source. Not necessarily true.
 */
void controlMixer_updateVariables(void) {
  uint8_t i;
  for (i=0; i < VARIABLE_COUNT; i++) {
    int16_t targetvalue = RC_getVariable(i);
    if (targetvalue < 0)
      variables[i] = 0;
    else if (targetvalue > 255)
      variables[i] = 255;
    else variables[i] = targetvalue;
  }
}

void controlMixer_setNeutral() {
  controlMixer_updateVariables();
  EC_setNeutral();
  HC_setGround();
  FC_setNeutral();  // FC is FailsafeControl, not FlightCtrl.

  // This is to set the home pos in navi.
  // MKFlags |= MKFLAG_CALIBRATE;
}

uint8_t controlMixer_getSignalQuality(void) {
  uint8_t rcQ = RC_getSignalQuality();
  uint8_t ecQ = EC_getSignalQuality();
  
  // This needs not be the only correct solution...
  return rcQ > ecQ ? rcQ : ecQ;
}

/*
void updateControlAndMeasureControlActivity(uint8_t index, int16_t newValue) {
  int16_t tmp = controls[index];
  controls[index] = newValue;
  
  tmp -= newValue;
  tmp /= 2;
  tmp = tmp * tmp;
  // tmp += (newValue >= 0) ? newValue : -newValue;

  / *
  if (controlActivity + (uint16_t)tmp >= controlActivity)
    controlActivity += tmp;
  else controlActivity = 0xffff;
  * /
  if (controlActivity + (uint16_t)tmp < 0x8000)
    controlActivity += tmp;
}

#define CADAMPING 10
void dampenControlActivity(void) {
  uint32_t tmp = controlActivity;
  tmp *= ((1<<CADAMPING)-1);
  tmp >>= CADAMPING;
  controlActivity = tmp;
}
*/

/*
 * Update the variables indicating stick position from the sum of R/C, GPS and external control
 * and whatever other controls we invented in the meantime...
 * Update variables.
 * Decode commands but do not execute them.
 */

void controlMixer_periodicTask(void) {
  int16_t tempRPTY[4] = { 0, 0, 0, 0 };

  // Decode commands.
  uint8_t rcCommand = (RC_getSignalQuality() >= SIGNAL_OK) ? RC_getCommand()
    : COMMAND_NONE;

  uint8_t ecCommand = (EC_getSignalQuality() >= SIGNAL_OK) ? EC_getCommand()
    : COMMAND_NONE;

  // Update variables ("potis").
  if (controlMixer_getSignalQuality() >= SIGNAL_GOOD) {
    controlMixer_updateVariables();
    controlMixer_didReceiveSignal = 1;
  } else { // Signal is not OK
    // Could handle switch to emergency flight here.
    // throttle is handled elsewhere.
  }

  if (rcCommand != COMMAND_NONE) {
    isCommandRepeated = (lastCommand == rcCommand);
    lastCommand = rcCommand;
    lastArgument = RC_getArgument();
  } else if (ecCommand != COMMAND_NONE) {
    isCommandRepeated = (lastCommand == ecCommand);
    lastCommand = ecCommand;
    lastArgument = EC_getArgument();
    if (ecCommand == COMMAND_GYROCAL) debugOut.digital[0] |= DEBUG_COMMAND;
    if (ecCommand == COMMAND_ACCCAL) debugOut.digital[1] |= DEBUG_COMMAND;
  } else {
    // Both sources have no command, or one or both are out.
    // Just set to false. There is no reason to check if the none-command was repeated anyway.
    isCommandRepeated = 0;
    lastCommand = COMMAND_NONE;
  }
  
  // This will init the values (not just add to them).
  RC_periodicTaskAndRPTY(tempRPTY);

  // Add external control to RC
  EC_periodicTaskAndRPTY(tempRPTY);

#ifdef USE_DIRECT_GPS
  if (staticParams.bitConfig & (CFG_NAVI_ENABLED))
    navigation_periodicTaskAndRPTY(tempRPTY);
#endif

  // Add compass control (could also have been before navi, they are independent)
  // CC_periodicTaskAndRPTY(tempRPTY);

  FC_periodicTaskAndRPTY(tempRPTY);
  
  // This is temporary. There might be some emergency height control also.
  if (!(MKFlags & MKFLAG_EMERGENCY_FLIGHT)) {
    // Add height control (could also have been before navi and/or compass, they are independent)
    HC_periodicTaskAndRPTY(tempRPTY);

    // Add attitude control (could also have been before navi and/or compass, they are independent)
    // AC_getRPTY(tempRPTY);
  }

  // Commit results to global variable and also measure control activity.
  controls[CONTROL_ROLL]     = tempRPTY[CONTROL_ROLL];
  controls[CONTROL_PITCH]    = tempRPTY[CONTROL_PITCH];
  controls[CONTROL_THROTTLE] = tempRPTY[CONTROL_THROTTLE];
  controls[CONTROL_YAW]      = tempRPTY[CONTROL_YAW];

  debugOut.analog[14] =controls[CONTROL_ROLL]; 
  debugOut.analog[15] =controls[CONTROL_PITCH];
  
  // We can safely do this even with a bad signal - the variables will not have been updated then.
  configuration_applyVariablesToParams();
 }
