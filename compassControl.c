#include <inttypes.h>
#include "controlMixer.h"
#include "attitude.h"
#include "compassControl.h"
#include "definitions.h"
#include <stdlib.h>

// 4 modes of control (like with the simple height controller):
// 0) Off: Normal yaw control, supported by compass (anti drift) if present and enabled
// 1) Heading is captured at takeoff, and held there (plus / minus bending via remote)
// 2) A variable controls the heading (plus / minus bending via remote)
// 3) Navigation controls the heading (plus / minus bending via remote)

// The bending should be linear, or rotation rate controlled.
// The target heading variable is stored here (not in the flight or attitude modules), as here
// is where the regulation takes place. Also, it was found that a target heading was difficult
// to maintain in the flight module and the flight module would sometimes need to write back
// to it. The only possible (but sufficient) output from here is RPTY[CONTROL_YAW].

int32_t navigationTargetHeading;
int32_t magneticTargetHeading;
int32_t bending;

void compass_setTakeoffHeading(int32_t heading) {
  magneticTargetHeading = heading;
}

void CC_periodicTaskAndRPTY(int16_t* RPTY) {
  int16_t currentYaw = RPTY[CONTROL_YAW];

  switch (staticParams.compassMode) {
  case COMPASS_MODE_OFF:
  default:
    bending = 0;
    break;
  case COMPASS_MODE_TAKEOFF:
    // just leave the target heading untouched! It should have been set at takeoff.
    break;
//  case COMPASS_MODE_RCVARIABLE:
//    magneticTargetHeading = (int32_t)dynamicParams.compassControlHeading * (360L * GYRO_DEG_FACTOR_YAW >> 8);
//    break;
  case COMPASS_MODE_NAVIGATION:
    magneticTargetHeading = navigationTargetHeading;
    break;
  }

  if (staticParams.compassMode != COMPASS_MODE_OFF) {
    if (abs(currentYaw) >= staticParams.naviStickThreshold) {
      // Bending: We do not actually need to do anything here, as the stick movement behing the bending will
      // pass right thru and do its stuff in the flight module.
      // All we have to do is to not counteract bending, and to return somewhat gracefully to target heading
      // again (using what parameter as speed?) when pilot stops bending.
      bending += currentYaw;
    } else {
      if (bending > staticParams.compassBendingReturnSpeed) {
        bending -= staticParams.compassBendingReturnSpeed;
      } else if (bending < -staticParams.compassBendingReturnSpeed) {
        bending += staticParams.compassBendingReturnSpeed;
      } else
        bending = 0;
    }

    // We have to output something proportional to the difference between magneticTargetHeading and heading (or a full PID).
    // Bending blends in like: (magneticTargetHeading +- bending - heading)
  }
}
