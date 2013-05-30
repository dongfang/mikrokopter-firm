#include <stdlib.h>
#include "commands.h"
#include "controlMixer.h"
#include "flight.h"
#include "eeprom.h"
#include "analog.h"
#include "attitude.h"
#include "configuration.h"
#include "beeper.h"

#ifdef USE_MK3MAG
// TODO: Kick that all outa here!
uint8_t compassCalState;
#endif

void commands_handleCommands(void) {
  /*
   * Get the current command (start/stop motors, calibrate), if any.
   */
  uint8_t command = controlMixer_getCommand();
  uint8_t repeated = controlMixer_isCommandRepeated();

  if (!(MKFlags & MKFLAG_MOTOR_RUN)) {
    uint8_t argument = controlMixer_getArgument();
    if ((command==COMMAND_GYROCAL || command==COMMAND_GYRO_ACC_CAL) && !repeated) {
      // Run gyro calibration but do not repeat it.
      // TODO: out of here. Anyway, MKFLAG_MOTOR_RUN is cleared. Not enough?
      // isFlying = 0;
      // check roll/pitch stick position
      // if pitch stick is top or roll stick is left or right --> change parameter setting
      // according to roll/pitch stick position

      if (argument < 6) {
        // Gyro calinbration, with or without selecting a new parameter-set.
        if (argument > 0 && argument < 6) {
          // A valid parameter-set (1..5) was chosen - use it.
          setActiveParamSet(argument);
        }
        paramSet_readFromEEProm(getActiveParamSet());
        analog_calibrateGyros();
        attitude_setNeutral();
        controlMixer_setNeutral();
        flight_setGround();
        beepNumber(getActiveParamSet());
      }
#ifdef USE_MK3MAG
      else if ((staticParams.bitConfig & CFG_COMPASS_ENABLED) && argument == 7) {
        // If right stick is centered and down
        compassCalState = 1;
        beep(1000);
      }
#endif
    }

    // save the ACC neutral setting to eeprom
    if ((command==COMMAND_ACCCAL || command==COMMAND_GYRO_ACC_CAL)  && !repeated) {
      // Run gyro and acc. meter calibration but do not repeat it.
      analog_calibrateAcc();
      attitude_setNeutral();
      controlMixer_setNeutral();
      beepNumber(getActiveParamSet());
    }
  } // end !MOTOR_RUN condition.
  if (command == COMMAND_START) {
    isFlying = 1; // TODO: Really????
    // if (!controlMixer_isCommandRepeated()) {
    // attitude_startDynamicCalibration(); // Try sense the effect of the motors on sensors.
    MKFlags |= MKFLAG_MOTOR_RUN;
    // } else { // Pilot is holding stick, ever after motor start. Continue to sense the effect of the motors on sensors.
    // attitude_continueDynamicCalibration();
    // setPointYaw = 0;
    // IPartPitch = 0;
    // IPartRoll = 0;
    // }
  } else if (command == COMMAND_STOP) {
    isFlying = 0;
    MKFlags &= ~(MKFLAG_MOTOR_RUN);
  }
}

/*
 *     if (controlMixer_testCompassCalState()) {
 compassCalState++;
 if (compassCalState < 5)
 beepNumber(compassCalState);
 else
 beep(1000);
 }
 *
 */

#ifdef USE_MK3MAG
uint8_t commands_isCalibratingCompass(void) {
  return RC_testCompassCalState();
}
#endif
