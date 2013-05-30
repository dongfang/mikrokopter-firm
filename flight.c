#include <stdlib.h>
#include <avr/io.h>
#include <math.h>
//#include "eeprom.h"
#include "flight.h"
#include "output.h"
//#include "uart0.h"

// Necessary for external control and motor test
//#include "twimaster.h"
#include "attitude.h"
#include "analog.h"
#include "controlMixer.h"
#include "commands.h"
#include "heightControl.h"
#include "definitions.h"
#include "debug.h"

#ifdef USE_MK3MAG
#include "mk3mag.h"
#include "compassControl.h"
#endif

int16_t targetHeading;
int32_t IPart[2];
FlightMode_t flight_flightMode = FM_UNINITALIZED;
int16_t minThrottle, maxThrottle;

/************************************************************************/
/*  Filter for motor value smoothing (necessary???)                     */
/************************************************************************/
/*
 int16_t motorFilter(int16_t newvalue, int16_t oldvalue) {
 switch (staticParams.motorSmoothing) {
 case 0:
 return newvalue;
 case 1:
 return (oldvalue + newvalue) / 2;
 case 2:
 if (newvalue > oldvalue)
 return (1 * (int16_t) oldvalue + newvalue) / 2; //mean of old and new
 else
 return newvalue - (oldvalue - newvalue) * 1; // 2 * new - old
 case 3:
 if (newvalue < oldvalue)
 return (1 * (int16_t) oldvalue + newvalue) / 2; //mean of old and new
 else
 return newvalue - (oldvalue - newvalue) * 1; // 2 * new - old
 default:
 return newvalue;
 }
 }
 */

void resetIParts(void) {
  IPart[X] = IPart[Y] = 0;
}

void flight_setMode(FlightMode_t _flightMode) {
  if (flight_flightMode != _flightMode) {
    resetIParts();
    flight_flightMode = _flightMode;
  }
}

// To be called only when necessary.
void flight_setParameters(void) {
  minThrottle = staticParams.minThrottle << LOG_CONTROL_BYTE_SCALING;
  maxThrottle = staticParams.maxThrottle << LOG_CONTROL_BYTE_SCALING;
}

// A heuristic when the yaw attitude cannot be used for yaw control because the airframe is near-vertical or inverted.
// Fum of abs(pitch) and abs(roll) greater than 75 degrees or about 1.3 radians.
uint8_t isHeadingUnaccountable(void) {
  int16_t x = attitude[X];
  if (x<0) x = -x;
  int16_t y = attitude[Y];
  if (y<0) y = -y;
  int32_t result = (int32_t)x + y;
  return result >= (long)(1.3 * INT16DEG_PI_FACTOR);
}

void flight_setGround(void) {
  resetIParts();
  targetHeading = attitude[Z];
}

void flight_takeOff(void) {
  // This is for GPS module to mark home position.
  // TODO: What a disgrace, change it.
  // MKFlags |= MKFLAG_CALIBRATE;
  HC_setGround();
#ifdef USE_MK3MAG
  attitude_resetHeadingToMagnetic();
  compass_setTakeoffHeading(attitude[YAW]);
#endif
}

/************************************************************************/
/*  Main Flight Control                                                 */
/************************************************************************/
void flight_control(void) {

  // PID controller variables
  float PID;

  // High resolution motor values for smoothing of PID motor outputs
  // static int16_t motorFilters[MAX_MOTORS];

  throttleTerm = controls[CONTROL_THROTTLE]; // in the -1000 to 1000 range there about.

  if (throttleTerm > 200 && (MKFlags & MKFLAG_MOTOR_RUN)) {
    // increment flight-time counter until overflow.
    if (isFlying != 0xFFFF)
      isFlying++;
  }
  /*
   * When standing on the ground, do not apply I controls and zero the yaw stick.
   * Probably to avoid integration effects that will cause the copter to spin
   * or flip when taking off.
   */
  if (isFlying < 256) {
    flight_setGround();
    if (isFlying == 255)
      flight_takeOff();
  }

  // This check removed. Is done on a per-motor basis, after output matrix multiplication.
  if (throttleTerm < minThrottle)
    throttleTerm = minThrottle;
  else if (throttleTerm > maxThrottle)
    throttleTerm = maxThrottle;

  // This is where control affects the target heading. It also (later) directly controls yaw.
  targetHeading -= ((int32_t)controls[CONTROL_YAW] * YAW_STICK_INTEGRATION_SCALER_LSHIFT16) >> 16;
  int16_t headingError;

  if (isHeadingUnaccountable()) {
    // inverted flight. Attitude[Z] is off by PI and we should react in the oppisite direction!
    debugOut.digital[0] |= DEBUG_INVERTED;
    headingError = 0;
  } else {
    debugOut.digital[0] &= ~DEBUG_INVERTED;
    headingError = attitude[Z] - targetHeading;
  }

  // Ahaa. Wait. Here is pretty much same check.
  if (headingError < -YAW_I_LIMIT) {
    headingError = -YAW_I_LIMIT;
    targetHeading = attitude[Z] + YAW_I_LIMIT;
  } else if (headingError > YAW_I_LIMIT) {
    headingError = YAW_I_LIMIT;
    targetHeading = attitude[Z] - YAW_I_LIMIT;
  }
  
  //debugOut.analog[24] = targetHeading;
  //debugOut.analog[25] = attitude[Z];
  //debugOut.analog[26] = headingError;
  //debugOut.analog[27] = positiveDynamic;
  //debugOut.analog[28] = negativeDynamic;

  // Yaw P part.
  PID = ((int32_t)gyro_control[Z] * YAW_RATE_SCALER_LSHIFT16 * dynamicParams.yawGyroP) >> 16;

  if (flight_flightMode != FM_RATE) {
    PID += ((int32_t)headingError * ATT_P_SCALER_LSHIFT16 * dynamicParams.yawGyroI) >> 16;
  }

  yawTerm = PID + controls[CONTROL_YAW];
  // yawTerm = limitYawToThrottle(yawTerm);

  /************************************************************************/
  /* Calculate control feedback from angle (gyro integral)                */
  /* and angular velocity (gyro signal)                                   */
  /************************************************************************/
  for (uint8_t axis = CONTROL_ROLL; axis <= CONTROL_PITCH; axis++) {
    if (flight_flightMode == FM_RETURN_TO_LEVEL) {
      // Control.
      // The P part is attitude error. 
      PID = (((int32_t)attitude[axis] * ATT_P_SCALER_LSHIFT16 * dynamicParams.attGyroP) >> 16) + controls[axis];
      // The I part is attitude error integral.
      IPart[axis] += PID;
      // The D part is rate.
      PID += ((int32_t)gyro_control[axis] * RATE_P_SCALER_LSHIFT16 * dynamicParams.attGyroD) >> 16;
    } else {
      // We want: Normal stick gain, full stick deflection should drive gyros halfway towards saturation.
      // If that is not enough, then fully towards saturation.
      PID = (((int32_t)gyro_control[axis] * RATE_P_SCALER_LSHIFT16 * dynamicParams.rateGyroP) >> 16) + controls[axis];
      IPart[axis] += PID;
      PID -= ((int32_t)gyroD[axis] * dynamicParams.rateGyroD) >> 8;
    }

    // PDPart += (gyroD[axis] * (int16_t) dynamicParams.gyroD) / 16;
    // Right now, let us ignore I.
    // term[axis] = PDPart - controls[axis] + (((int32_t) IPart[axis] * ki) >> 14);
    term[axis] = PID;
    term[axis] += (dynamicParams.levelCorrection[axis] - 128);

    debugOut.analog[12 + axis] = term[axis];
  }

  //debugOut.analog[14] = yawTerm;
  //debugOut.analog[15] = throttleTerm;
  //debugOut.analog[15] = IPart[0] / 1000;

  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Universal Mixer
  // Each (pitch, roll, throttle, yaw) term is in the range [0..255 * CONTROL_SCALING].
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  debugOut.analog[9] = controls[CONTROL_PITCH];
  debugOut.analog[10] = controls[CONTROL_YAW];
  debugOut.analog[11] = controls[CONTROL_THROTTLE];
}
