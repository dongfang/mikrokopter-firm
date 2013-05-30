#include <inttypes.h>
#include "ubx.h"
#include <avr/io.h>
#include "output.h"

// ubx protocol parser state machine
#define	UBXSTATE_IDLE	0
#define	UBXSTATE_SYNC1	1
#define	UBXSTATE_SYNC2	2
#define	UBXSTATE_CLASS	3
#define	UBXSTATE_LEN1	4
#define	UBXSTATE_LEN2	5
#define UBXSTATE_DATA	6
#define	UBXSTATE_CKA	7
#define	UBXSTATE_CKB	8

// ublox protocoll identifier
#define	UBX_CLASS_NAV	0x01

#define	UBX_ID_POSLLH	0x02
#define UBX_ID_SOL		0x06
#define	UBX_ID_VELNED	0x12

#define	UBX_SYNC1_CHAR	0xB5
#define	UBX_SYNC2_CHAR	0x62

typedef struct {
  uint32_t ITOW; // ms GPS Millisecond Time of Week
  int32_t frac; // ns remainder of rounded ms above
  int16_t week; // GPS week
  uint8_t GPSfix; // GPSfix Type, range 0..6
  uint8_t flags; // Navigation Status Flags
  int32_t ECEF_X; // cm ECEF X coordinate
  int32_t ECEF_Y; // cm ECEF Y coordinate
  int32_t ECEF_Z; // cm ECEF Z coordinate
  uint32_t pAcc; // cm 3D Position Accuracy Estimate
  int32_t ECEFVX; // cm/s ECEF X velocity
  int32_t ECEFVY; // cm/s ECEF Y velocity
  int32_t ECEFVZ; // cm/s ECEF Z velocity
  uint32_t sAcc; // cm/s Speed Accuracy Estimate
  uint16_t PDOP; // 0.01 Position DOP
  uint8_t res1; // reserved
  uint8_t numSV; // Number of SVs used in navigation solution
  uint32_t res2; // reserved
  Status_t status;
} UBX_SOL_t;

typedef struct {
  uint32_t ITOW; // ms GPS Millisecond Time of Week
  int32_t lon; // 1e-07 deg Longitude
  int32_t lat; // 1e-07 deg Latitude
  int32_t height; // mm Height above Ellipsoid
  int32_t HMSL; // mm Height above mean sea level
  uint32_t hAcc; // mm Horizontal Accuracy Estimate
  uint32_t vAacc; // mm Vertical Accuracy Estimate
  Status_t status;
} UBX_POSLLH_t;

typedef struct {
  uint32_t ITOW; // ms  GPS Millisecond Time of Week
  int32_t VEL_N; // cm/s  NED north velocity
  int32_t VEL_E; // cm/s  NED east velocity
  int32_t VEL_D; // cm/s  NED down velocity
  uint32_t speed; // cm/s  Speed (3-D)
  uint32_t gSpeed; // cm/s  Ground Speed (2-D)
  int32_t heading; // 1e-05 deg  Heading 2-D
  uint32_t sAcc; // cm/s  Speed Accuracy Estimate
  uint32_t cAcc; // deg  Course / Heading Accuracy Estimate
  Status_t status;
} UBX_VELNED_t;

UBX_SOL_t ubxSol = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, INVALID };
UBX_POSLLH_t ubxPosLlh = { 0, 0, 0, 0, 0, 0, 0, INVALID };
UBX_VELNED_t ubxVelNed = { 0, 0, 0, 0, 0, 0, 0, 0, 0, INVALID };
GPS_INFO_t GPSInfo = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, INVALID };

volatile uint8_t GPSTimeout;
volatile uint16_t GPSDatasetCounter;

void updateGPSInfo(void) {
  if ((ubxSol.status == NEWDATA) && (ubxPosLlh.status == NEWDATA) && (ubxVelNed.status == NEWDATA)) {
    //RED_FLASH;
    // DebugOut.Digital ....blah...
    if (GPSInfo.status != NEWDATA) {
      GPSDatasetCounter++;
      GPSInfo.status = INVALID;
      // NAV SOL
      GPSInfo.flags =   ubxSol.flags;
      GPSInfo.satfix =  ubxSol.GPSfix;
      GPSInfo.satnum =  ubxSol.numSV;

      GPSInfo.position3DAcc =  ubxSol.pAcc;
      GPSInfo.verticalAcc =    ubxPosLlh.vAacc;
      GPSInfo.velocityAcc =    ubxSol.sAcc;

      debugOut.analog[30] = ubxPosLlh.HMSL/1000L;
      debugOut.analog[31] = GPSInfo.verticalAcc/1000L;

      // NAV POSLLH
      GPSInfo.longitude =   ubxPosLlh.lon;
      GPSInfo.latitude =    ubxPosLlh.lat;
      GPSInfo.altitude =    ubxPosLlh.height;

      GPSInfo.veleast =     ubxVelNed.VEL_E;
      GPSInfo.velnorth =    ubxVelNed.VEL_N;
      GPSInfo.veltop =     -ubxVelNed.VEL_D;
      GPSInfo.velground =   ubxVelNed.gSpeed;

      GPSInfo.status = NEWDATA;
    }

    // set state to collect new data
    ubxSol.status = PROCESSED; // never update old data
    ubxPosLlh.status = PROCESSED; // never update old data
    ubxVelNed.status = PROCESSED; // never update old data
  }
}

// this function should be called within the UART RX ISR
void ubx_parser(uint8_t c) {
  static uint8_t ubxstate = UBXSTATE_IDLE;
  static uint8_t cka, ckb;
  static uint16_t msglen;
  static int8_t *ubxP, *ubxEp, *ubxSp; // pointers to data currently transfered

  switch (ubxstate) {
  case UBXSTATE_IDLE: // check 1st sync byte
    if (c == UBX_SYNC1_CHAR)
      ubxstate = UBXSTATE_SYNC1;
    else
      ubxstate = UBXSTATE_IDLE; // out of synchronization
    break;

  case UBXSTATE_SYNC1: // check 2nd sync byte
    if (c == UBX_SYNC2_CHAR)
      ubxstate = UBXSTATE_SYNC2;
    else
      ubxstate = UBXSTATE_IDLE; // out of synchronization
    break;

  case UBXSTATE_SYNC2: // check msg class to be NAV
    if (c == UBX_CLASS_NAV)
      ubxstate = UBXSTATE_CLASS;
    else
      ubxstate = UBXSTATE_IDLE; // unsupported message class
    break;

  case UBXSTATE_CLASS: // check message identifier
    switch (c) {
    case UBX_ID_POSLLH: // geodetic position
      ubxP = (int8_t *)     &ubxPosLlh; // data start pointer
      ubxEp = (int8_t *)   (&ubxPosLlh + 1); // data end pointer
      ubxSp = (int8_t *)    &ubxPosLlh.status; // status pointer
      break;

    case UBX_ID_SOL: // navigation solution
      ubxP = (int8_t *)     &ubxSol; // data start pointer
      ubxEp = (int8_t *)   (&ubxSol + 1); // data end pointer
      ubxSp = (int8_t *)    &ubxSol.status; // status pointer
      break;

    case UBX_ID_VELNED: // velocity vector in tangent plane
      ubxP = (int8_t *)     &ubxVelNed; // data start pointer
      ubxEp = (int8_t *)   (&ubxVelNed + 1); // data end pointer
      ubxSp = (int8_t *)    &ubxVelNed.status; // status pointer
      break;

    default: // unsupported identifier
      ubxstate = UBXSTATE_IDLE;
      break;
    }
    if (ubxstate != UBXSTATE_IDLE) {
      ubxstate = UBXSTATE_LEN1;
      cka = UBX_CLASS_NAV + c;
      ckb = UBX_CLASS_NAV + cka;
    }
    break;

  case UBXSTATE_LEN1: // 1st message length byte
    msglen = c;
    cka += c;
    ckb += cka;
    ubxstate = UBXSTATE_LEN2;
    break;

  case UBXSTATE_LEN2: // 2nd message length byte
    msglen += ((uint16_t) c) << 8;
    cka += c;
    ckb += cka;
    // if the old data are not processed so far then break parsing now
    // to avoid writing new data in ISR during reading by another function
    if (*ubxSp == NEWDATA) {
      updateGPSInfo(); //update GPS info respectively
      ubxstate = UBXSTATE_IDLE;
    } else {// data invalid or already processd
      *ubxSp = INVALID;
      ubxstate = UBXSTATE_DATA;
    }
    break;

  case UBXSTATE_DATA:
    if (ubxP < ubxEp)
      *ubxP++ = c; // copy curent data byte if any space is left
    cka += c;
    ckb += cka;
    if (--msglen == 0)
      ubxstate = UBXSTATE_CKA; // switch to next state if all data was read
    break;

  case UBXSTATE_CKA:
    if (c == cka)
      ubxstate = UBXSTATE_CKB;
    else {
      *ubxSp = INVALID;
      ubxstate = UBXSTATE_IDLE;
    }
    break;

  case UBXSTATE_CKB:
    if (c == ckb) {
      *ubxSp = NEWDATA; // new data are valid
      updateGPSInfo(); //update GPS info respectively
      GPSTimeout = 255;
    } else { // if checksum not match then set data invalid
      *ubxSp = INVALID;
    }
    ubxstate = UBXSTATE_IDLE; // ready to parse new data
    break;

  default: // unknown ubx state
    ubxstate = UBXSTATE_IDLE;
    break;
  }
}
