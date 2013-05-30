// Navigation with a GPS directly attached to the FC's UART1.

#include <inttypes.h>
#include <stdlib.h>
#include <stddef.h>
#include "ubx.h"
#include "configuration.h"
#include "controlMixer.h"
#include "output.h"
#include "isqrt.h"
#include "attitude.h"
#include "attitude.h"

typedef enum {
  NAVI_FLIGHT_MODE_UNDEF,
  NAVI_FLIGHT_MODE_FREE,
  NAVI_FLIGHT_MODE_AID,
  NAVI_FLIGHT_MODE_HOME,
} NaviMode_t;

/*
 * This is not read from internally. It only serves to let a pilot/debugger
 * know the status of the navigation system (thru a data connection).
 */
typedef enum {
  NAVI_STATUS_NO_COMPASS=-1,
  NAVI_STATUS_INVALID_GPS=-2,
  NAVI_STATUS_BAD_GPS_SIGNAL=-3,
  NAVI_STATUS_RTH_POSITION_INVALID=-4,
  NAVI_STATUS_RTH_FALLBACK_ON_HOLD=-5,
  NAVI_STATUS_GPS_TIMEOUT=-6,
  NAVI_STATUS_FREEFLIGHT=0,
  NAVI_STATUS_POSITION_HOLD=1,
  NAVI_STATUS_RTH=2,
  NAVI_STATUS_MANUAL_OVERRIDE=3,
  NAVI_STATUS_HOLD_POSITION_INVALID=4,
} NaviStatus_t;

#define GPS_POSINTEGRAL_LIMIT 32000
#define LOG_NAVI_STICK_GAIN 3
#define GPS_P_LIMIT			100

typedef struct {
  int32_t longitude;
  int32_t latitude;
  int32_t altitude;
  Status_t status;
} GPS_Pos_t;

// GPS coordinates for hold position
GPS_Pos_t holdPosition = { 0, 0, 0, INVALID };
// GPS coordinates for home position
GPS_Pos_t homePosition = { 0, 0, 0, INVALID };
// the current flight mode
NaviMode_t naviMode = NAVI_FLIGHT_MODE_UNDEF;
int16_t naviSticks[2] = {0,0};

int8_t naviStatus;

// ---------------------------------------------------------------------------------
void navi_updateFlightMode(void) {
  static NaviMode_t naviModeOld = NAVI_FLIGHT_MODE_UNDEF;

  if (MKFlags & MKFLAG_EMERGENCY_FLIGHT) {
    naviMode = NAVI_FLIGHT_MODE_FREE;
  } else {
    if (dynamicParams.naviMode < 50)
      naviMode = NAVI_FLIGHT_MODE_FREE;
    else if (dynamicParams.naviMode < 180)
      naviMode = NAVI_FLIGHT_MODE_AID;
    else
      naviMode = NAVI_FLIGHT_MODE_HOME;
  }

  if (naviMode != naviModeOld) {
    beep(100);
    naviModeOld = naviMode;
  }
}

// ---------------------------------------------------------------------------------
// This function defines a good GPS signal condition
uint8_t navi_isGPSSignalOK(void) {
  static uint8_t GPSFix = 0;
  if ((GPSInfo.status != INVALID) && (GPSInfo.satfix == SATFIX_3D)
      && (GPSInfo.flags & FLAG_GPSFIXOK)
      && ((GPSInfo.satnum >= staticParams.GPSMininumSatellites) || GPSFix)) {
    GPSFix = 1;
    return 1;
  } else
    return (0);
}

// ---------------------------------------------------------------------------------
// rescale xy-vector length to  limit
uint8_t navi_limitXY(int32_t *x, int32_t *y, int32_t limit) {
  int32_t len;
  len = isqrt32(*x * *x + *y * *y);
  if (len > limit) {
    // normalize control vector components to the limit
    *x = (*x * limit) / len;
    *y = (*y * limit) / len;
    return 1;
  }
  return 0;
}

// checks nick and roll sticks for manual control
uint8_t navi_isManuallyControlled(int16_t* RPTY) {
  if (abs(RPTY[CONTROL_PITCH]) < staticParams.naviStickThreshold
      && abs(RPTY[CONTROL_ROLL]) < staticParams.naviStickThreshold)
    return 0;
  else
    return 1;
}

// set given position to current gps position
uint8_t navi_writeCurrPositionTo(GPS_Pos_t * pGPSPos) {
  if (pGPSPos == NULL)
    return 0; // bad pointer

  if (navi_isGPSSignalOK()) { // is GPS signal condition is fine
    pGPSPos->longitude = GPSInfo.longitude;
    pGPSPos->latitude = GPSInfo.latitude;
    pGPSPos->altitude = GPSInfo.altitude;
    pGPSPos->status = NEWDATA;
    return 1;
  } else { // bad GPS signal condition
    pGPSPos->status = INVALID;
    return 0;
  }
}

// clear position
uint8_t navi_clearPosition(GPS_Pos_t * pGPSPos) {
  if (pGPSPos == NULL)
    return 0; // bad pointer
  else {
    pGPSPos->longitude = 0;
    pGPSPos->latitude = 0;
    pGPSPos->altitude = 0;
    pGPSPos->status = INVALID;
  }
  return 1;
}

void navi_setNeutral(void) {
  naviSticks[CONTROL_PITCH] = naviSticks[CONTROL_ROLL] = 0;
}

// calculates the GPS control stick values from the deviation to target position
// if the pointer to the target positin is NULL or is the target position invalid
// then the P part of the controller is deactivated.
void navi_PIDController(GPS_Pos_t *pTargetPos) {
  static int32_t PID_Pitch, PID_Roll;
  int32_t coscompass, sincompass;
  int32_t GPSPosDev_North, GPSPosDev_East; // Position deviation in cm
  int32_t P_North = 0, D_North = 0, P_East = 0, D_East = 0, I_North = 0, I_East = 0;
  int32_t PID_North = 0, PID_East = 0;
  static int32_t cos_target_latitude = 1;
  static int32_t GPSPosDevIntegral_North = 0, GPSPosDevIntegral_East = 0;
  static GPS_Pos_t *pLastTargetPos = 0;

  // if GPS data and Compass are ok
  if (navi_isGPSSignalOK() && (magneticHeading >= 0)) {
    if (pTargetPos != NULL) { // if there is a target position
      if (pTargetPos->status != INVALID) { // and the position data are valid
        // if the target data are updated or the target pointer has changed
        if ((pTargetPos->status != PROCESSED) || (pTargetPos != pLastTargetPos)) {
          // reset error integral
          GPSPosDevIntegral_North = 0;
          GPSPosDevIntegral_East = 0;
          // recalculate latitude projection
          cos_target_latitude = cos_360(pTargetPos->latitude / 10000000L);
          // remember last target pointer
          pLastTargetPos = pTargetPos;
          // mark data as processed
          pTargetPos->status = PROCESSED;
        }
        // calculate position deviation from latitude and longitude differences
        GPSPosDev_North = (GPSInfo.latitude - pTargetPos->latitude); // to calculate real cm we would need *111/100 additionally
        GPSPosDev_East = (GPSInfo.longitude - pTargetPos->longitude); // to calculate real cm we would need *111/100 additionally
        // calculate latitude projection
        GPSPosDev_East *= cos_target_latitude;
        GPSPosDev_East >>= LOG_MATH_UNIT_FACTOR;
      } else { // no valid target position available
        // reset error
        GPSPosDev_North = 0;
        GPSPosDev_East = 0;
        // reset error integral
        GPSPosDevIntegral_North = 0;
        GPSPosDevIntegral_East = 0;
      }
    } else { // no target position available
      // reset error
      GPSPosDev_North = 0;
      GPSPosDev_East = 0;
      // reset error integral
      GPSPosDevIntegral_North = 0;
      GPSPosDevIntegral_East = 0;
    }

    //Calculate PID-components of the controller
    // D-Part
    D_North = ((int32_t) staticParams.naviD * GPSInfo.velnorth) >> 9;
    D_East = ((int32_t) staticParams.naviD * GPSInfo.veleast) >> 9;

    // P-Part
    P_North = ((int32_t) staticParams.naviP * GPSPosDev_North) >> 11;
    P_East = ((int32_t) staticParams.naviP * GPSPosDev_East) >> 11;

    // I-Part
    I_North = ((int32_t) staticParams.naviI * GPSPosDevIntegral_North) >> 13;
    I_East = ((int32_t) staticParams.naviI * GPSPosDevIntegral_East) >> 13;

    // combine P & I
    PID_North = P_North + I_North;
    PID_East = P_East + I_East;

    if (!navi_limitXY(&PID_North, &PID_East, GPS_P_LIMIT)) {
      // within limit
      GPSPosDevIntegral_North += GPSPosDev_North >> 4;
      GPSPosDevIntegral_East += GPSPosDev_East >> 4;
      navi_limitXY(&GPSPosDevIntegral_North, &GPSPosDevIntegral_East, GPS_POSINTEGRAL_LIMIT);
    }

    // combine PI- and D-Part
    PID_North += D_North;
    PID_East += D_East;

    // scale combination with gain.
    // dongfang: Lets not do that. P I and D can be scaled instead.
    // PID_North = (PID_North * (int32_t) staticParams.NaviGpsGain) / 100;
    // PID_East = (PID_East * (int32_t) staticParams.NaviGpsGain) / 100;

    // GPS to nick and roll settings
    // A positive nick angle moves head downwards (flying forward).
    // A positive roll angle tilts left side downwards (flying left).
    // If compass heading is 0 the head of the copter is in north direction.
    // A positive nick angle will fly to north and a positive roll angle will fly to west.
    // In case of a positive north deviation/velocity the
    // copter should fly to south (negative nick).
    // In case of a positive east position deviation and a positive east velocity the
    // copter should fly to west (positive roll).
    // The influence of the GPSStickNick and GPSStickRoll variable is contrarily to the stick values
    // in the flight.c. Therefore a positive north deviation/velocity should result in a positive
    // GPSStickNick and a positive east deviation/velocity should result in a negative GPSStickRoll.

    coscompass = -cos_360(heading / GYRO_DEG_FACTOR_YAW);
    sincompass = -sin_360(heading / GYRO_DEG_FACTOR_YAW);

    PID_Pitch = (coscompass * PID_North + sincompass * PID_East) >> (LOG_MATH_UNIT_FACTOR-LOG_NAVI_STICK_GAIN);
    PID_Roll = (sincompass * PID_North - coscompass * PID_East) >> (LOG_MATH_UNIT_FACTOR-LOG_NAVI_STICK_GAIN);

    // limit resulting GPS control vector
    navi_limitXY(&PID_Pitch, &PID_Roll, staticParams.naviStickLimit << LOG_NAVI_STICK_GAIN);

    naviSticks[CONTROL_PITCH] = PID_Pitch;
    naviSticks[CONTROL_ROLL] = PID_Roll;
  } else { // invalid GPS data or bad compass reading
    // reset error integral
    navi_setNeutral();
    GPSPosDevIntegral_North = 0;
    GPSPosDevIntegral_East = 0;
  }
}

void navigation_periodicTaskAndRPTY(int16_t* RPTY) {
  static uint8_t GPS_P_Delay = 0;
  static uint16_t beep_rythm = 0;
  static uint16_t navi_testOscPrescaler = 0;
  static uint8_t  navi_testOscTimer = 0;

  if (!(staticParams.bitConfig & CFG_COMPASS_ENABLED)) {
    navi_setNeutral();
    naviStatus = NAVI_STATUS_NO_COMPASS;
    return;
  }

  navi_updateNaviMode();

  navi_testOscPrescaler++;
  if (navi_testOscPrescaler == 488) {
      navi_testOscPrescaler = 0;
      navi_testOscTimer++;
      if (navi_testOscTimer == staticParams.naviTestOscPeriod) {
          navi_testOscTimer = 0;
          if (staticParams.naviTestOscAmplitude) {
              holdPosition.status = NEWDATA;
              holdPosition.latitude += staticParams.naviTestOscAmplitude * 90L; // there are about 90 units per meter.
          }
      } else if (navi_testOscTimer == staticParams.naviTestOscPeriod/2) {
          if (staticParams.naviTestOscAmplitude) {
              holdPosition.status = NEWDATA;
              holdPosition.latitude -= staticParams.naviTestOscAmplitude * 90L;
          }
      }
  }

  // store home position if start of flight flag is set
  if (MKFlags & MKFLAG_CALIBRATE) {
    MKFlags &= ~(MKFLAG_CALIBRATE);
    if (navi_writeCurrPositionTo(&homePosition)) {
        // shift north to simulate an offset.
        // homePosition.latitude += 10000L;
        beep(500);
      }
    }

  switch (GPSInfo.status) {
  case INVALID: // invalid gps data
    navi_setNeutral();
    naviStatus = NAVI_STATUS_INVALID_GPS;
    if (naviMode != NAVI_FLIGHT_MODE_FREE) {
      beep(1); // beep if signal is neccesary
    }
    break;
  case PROCESSED: // if gps data are already processed do nothing
    // downcount timeout
    if (GPSTimeout)
      GPSTimeout--;
    // if no new data arrived within timeout set current data invalid
    // and therefore disable GPS
    else {
      navi_setNeutral();
      GPSInfo.status = INVALID;
      naviStatus = NAVI_STATUS_GPS_TIMEOUT;
    }
    break;
  case NEWDATA: // new valid data from gps device
    // if the gps data quality is good
    beep_rythm++;
    if (navi_isGPSSignalOK()) {
      switch (naviMode) { // check what's to do
      case NAVI_FLIGHT_MODE_FREE:
        // update hold position to current gps position
        navi_writeCurrPositionTo(&holdPosition); // can get invalid if gps signal is bad
        // disable gps control
        navi_setNeutral();
        naviStatus = NAVI_STATUS_FREEFLIGHT;
        break;

      case NAVI_FLIGHT_MODE_AID:
        if (holdPosition.status != INVALID) {
          if (navi_isManuallyControlled(RPTY)) { // MK controlled by user
            // update hold point to current gps position
            navi_writeCurrPositionTo(&holdPosition);
            // disable gps control
            navi_setNeutral();
            GPS_P_Delay = 0;
            naviStatus = NAVI_STATUS_MANUAL_OVERRIDE;
          } else { // GPS control active
            if (GPS_P_Delay < 7) {
              // delayed activation of P-Part for 8 cycles (8*0.25s = 2s)
              GPS_P_Delay++;
              navi_writeCurrPositionTo(&holdPosition); // update hold point to current gps position
              navi_PIDController(NULL); // activates only the D-Part
              naviStatus = NAVI_STATUS_POSITION_HOLD;
            } else {
              navi_PIDController(&holdPosition); // activates the P&D-Part
              naviStatus = NAVI_STATUS_POSITION_HOLD;
            }
          }
        } else { // invalid Hold Position
        // try to catch a valid hold position from gps data input
          navi_writeCurrPositionTo(&holdPosition);
          navi_setNeutral();
          naviStatus = NAVI_STATUS_HOLD_POSITION_INVALID;
        }
        break;

      case NAVI_FLIGHT_MODE_HOME:
        if (homePosition.status != INVALID) {
          // update hold point to current gps position
          // to avoid a flight back if home comming is deactivated
          navi_writeCurrPositionTo(&holdPosition);
          if (navi_isManuallyControlled(RPTY)) { // MK controlled by user
            navi_setNeutral();
            naviStatus = NAVI_STATUS_MANUAL_OVERRIDE;
          } else {// GPS control active
            navi_PIDController(&homePosition);
          }
          naviStatus = NAVI_STATUS_RTH;
        } else {
          // bad home position
          beep(50); // signal invalid home position
          // try to hold at least the position as a fallback option
          if (holdPosition.status != INVALID) {
            if (navi_isManuallyControlled(RPTY)) {
              // MK controlled by user
              navi_setNeutral();
              naviStatus = NAVI_STATUS_MANUAL_OVERRIDE;
            } else {
              // GPS control active
              navi_PIDController(&holdPosition);
              naviStatus = NAVI_STATUS_RTH_FALLBACK_ON_HOLD;
            }
          } else { // try to catch a valid hold position
            navi_writeCurrPositionTo(&holdPosition);
            naviStatus = NAVI_STATUS_RTH_POSITION_INVALID;
            navi_setNeutral();
          }
        }
        break; // eof TSK_HOME
      default: // unhandled task
        navi_setNeutral();
        break; // eof default
      } // eof switch GPS_Task
    } // eof gps data quality is good
    else { // gps data quality is bad
      // disable gps control
      navi_setNeutral();
      naviStatus = NAVI_STATUS_BAD_GPS_SIGNAL;
      if (naviMode != NAVI_FLIGHT_MODE_FREE) {
        // beep if signal is not sufficient
        if (!(GPSInfo.flags & FLAG_GPSFIXOK) && !(beep_rythm % 5))
          beep(100);
        else if (GPSInfo.satnum < staticParams.GPSMininumSatellites
            && !(beep_rythm % 5))
          beep(10);
      }
    }
    // set current data as processed to avoid further calculations on the same gps data
    GPSInfo.status = PROCESSED;
    break;
  } // eof GPSInfo.status

  RPTY[CONTROL_PITCH] += naviSticks[CONTROL_PITCH];
  RPTY[CONTROL_ROLL] += naviSticks[CONTROL_ROLL];
}
