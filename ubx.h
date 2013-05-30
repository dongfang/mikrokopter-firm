#ifndef _UBX_H
#define _UBX_H

#include <inttypes.h>

typedef enum {
  INVALID, NEWDATA, PROCESSED
} Status_t;

// Satfix types for GPSData.satfix
#define SATFIX_NONE 			 0x00
#define SATFIX_DEADRECKOING 	 0x01
#define SATFIX_2D 				 0x02
#define SATFIX_3D				 0x03
#define SATFIX_GPS_DEADRECKOING  0x04
#define SATFIX_TIMEONLY			 0x05
// Flags for interpretation of the GPSData.flags
#define FLAG_GPSFIXOK			0x01 // (i.e. within DOP & ACC Masks)
#define FLAG_DIFFSOLN			0x02 // (is DGPS used)
#define FLAG_WKNSET				0x04 // (is Week Number valid)
#define FLAG_TOWSET				0x08 //	(is Time of Week valid)
/* enable the UBX protocol at the gps receiver with the following messages enabled
 01-02 NAV - POSLLH
 01-06 Nav - SOL
 01-12 NAV - VELNED */

typedef struct {
  uint8_t flags; // flags
  uint8_t satnum; // number of satelites
  uint8_t satfix; // type of satfix
  int32_t longitude; // in 1e-07 deg
  int32_t latitude; // in 1e-07 deg
  int32_t altitude; // in mm
  uint32_t position3DAcc; // in cm 3d position accuracy
  uint32_t verticalAcc; // in bm vertical position accuracy
  int32_t velnorth; // in cm/s
  int32_t veleast; // in cm/s
  int32_t veltop; // in cm/s
  uint32_t velground; // 2D ground speed in cm/s
  uint32_t velocityAcc; // in cm/s 3d velocity accuracy
  Status_t status; // status of data: invalid | valid
} GPS_INFO_t;

//here you will find the current gps info
extern GPS_INFO_t GPSInfo; // measured position (last gps record)

// this variable should be decremted by the application
extern volatile uint8_t GPSTimeout; // is reset to 255 if a new UBX msg was received

#define USART1_BAUD 57600
// this function should be called within the UART RX ISR
extern void ubx_parser(uint8_t c);

#endif //_UBX_H
