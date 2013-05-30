#ifndef _COMPASS_H
#define _COMPASSRC_H

#include <inttypes.h>

// 4 modes of control (like with the simple height controller):
// 0) Off: Normal yaw control, supported by compass (anti drift) if present and enabled
// 1) Heading is captured at takeoff, and held there (plus / minus bending via remote)
// 2) A variable controls the heading (plus / minus bending via remote) (not implemented, is it useful at all?)
// 3) Navigation controls the heading (plus / minus bending via remote)

#define COMPASS_MODE_OFF            0
#define COMPASS_MODE_TAKEOFF        1
//#define COMPASS_MODE_RCVARIABLE     2
#define COMPASS_MODE_NAVIGATION     3

// public read write.
extern int32_t navigationTargetHeading;

void compass_setTakeoffHeading(int32_t heading);
void CC_periodicTaskAndRPTY(int16_t* RPTY);

#endif
