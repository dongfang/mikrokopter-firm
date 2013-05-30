#include "controlMixer.h"
#include "timer0.h"
#include "configuration.h"
//#include "twimaster.h"
#include "flight.h"
//#include "output.h"
#include "definitions.h"
#include "beeper.h"

uint32_t emergencyFlightEndTimeMillis;

void FC_setNeutral(void) {
  configuration_setNormalFlightMode();
}

void FC_periodicTaskAndRPTY(int16_t* RPTY) {
  if (controlMixer_getSignalQuality() <= SIGNAL_BAD) { // the rc-frame signal is not reveived or noisy
    if (controlMixer_didReceiveSignal)
      beepRCAlarm(); // Only make alarm if a control signal was received before the signal loss.

    // There are the possibilities: We are not yet in emergency flight, we are already in emergency flight.
    if (!(MKFlags & MKFLAG_EMERGENCY_FLIGHT)) {
      if (isFlying > 256) {
        MKFlags |= MKFLAG_EMERGENCY_FLIGHT; // Set flag for emergency landing
        configuration_setFailsafeFlightMode();
        // Set the time in whole seconds.
        emergencyFlightEndTimeMillis = millisClock + (staticParams.emergencyFlightDuration  * 1000L);
      }
    } else {
      if (millisClock > emergencyFlightEndTimeMillis) {
        // stop motors but stay in emergency flight.
        MKFlags &= ~(MKFLAG_MOTOR_RUN);
      }
    }

    // In either case, use e. throttle and neutral controls. TODO: If there is supposed to be a navi come-home, this should affect RC control only and not navi.
    RPTY[CONTROL_THROTTLE] = staticParams.emergencyThrottle << 3; // Set emergency throttle
    RPTY[CONTROL_PITCH] = RPTY[CONTROL_ROLL] = RPTY[CONTROL_YAW] = 0;
  } else {
    // Signal is OK.
    if (MKFlags & MKFLAG_EMERGENCY_FLIGHT) {
      MKFlags &= ~MKFLAG_EMERGENCY_FLIGHT; // Clear flag for emergency landing
      // configuration_setNormalFlightParameters();
    }
    // A hack (permenent?): Keep re-setting the parameters, in order to support dynamic params.
    configuration_setNormalFlightMode();
  }

  /*
   * If a Bl-Ctrl is missing, prevent takeoff.
   * Feature unnecessary, removed.
   */
  /*
  if ((MKFlags & MKFLAG_MOTOR_RUN) && missingMotor) {
    // if we are in the lift off condition. Hmmmmmm when is throttleTerm == 0 anyway???
    if (isFlying > 1 && isFlying < 50 && RPTY[CONTROL_THROTTLE] > 0) isFlying = 1; // keep within lift off condition
    RPTY[CONTROL_THROTTLE] = staticParams.minThrottle << 3; // reduce throttle to min to prevent takeoff
  }
  */
}
