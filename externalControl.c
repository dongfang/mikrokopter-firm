#include "externalControl.h"
#include "configuration.h"
//#include "controlMixer.h"
#include "definitions.h"

ExternalControl_t externalControl;
volatile uint8_t externalControlActive;
// TODO: Who is going to call this

void EC_setNeutral(void) {
  // if necessary. From main.c.
  externalControl.command = 0;
  externalControl.argument = 0;
  externalControl.pitch = 0;
  externalControl.roll = 0;
  externalControl.yaw = 0;
  externalControl.throttle = 0;

  // From main.c. What does it do??
  externalControl.digital[0] = 0x55;
}

void EC_periodicTaskAndRPTY(int16_t* RPTY) {
  if (externalControlActive) {
    externalControlActive--;
    RPTY[CONTROL_PITCH] += externalControl.pitch * (int16_t) staticParams.stickP;
    RPTY[CONTROL_ROLL] += externalControl.roll * (int16_t) staticParams.stickP;
    RPTY[CONTROL_THROTTLE] += externalControl.throttle;
    RPTY[CONTROL_YAW] += externalControl.yaw; // No stickP or similar??????
  }
}

uint8_t EC_getArgument(void) {
  return externalControl.argument;
}

uint8_t EC_getCommand(void) {
  return externalControl.command;
}

// not implemented.
int16_t EC_getVariable(uint8_t varNum) {
  return 0;
}

uint8_t EC_getSignalQuality(void) {
  if (externalControlActive > 40)
    // Configured and heard from recently
    return SIGNAL_GOOD;

  if (externalControlActive)
    // Configured and heard from
    return SIGNAL_OK;

  if (dynamicParams.externalControl < 128)
    // External control is not even configured.
    return NO_SIGNAL;

  // Configured but expired.
  return SIGNAL_LOST;
}
