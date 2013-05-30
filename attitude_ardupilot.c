#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>

#include "attitude.h"
#include "configuration.h"
#include <avr/interrupt.h>
#include "timer0.h"
#include "debug.h"

// where our main data flow comes from.
#include "analog.h"
#include "AP_AHRS_DCM.h"
#include "AP_GPS.h"

int16_t attitude[3];

//AP_Compass_HIL compass;
GPS* gps;
AP_AHRS_DCM ahrs(gps);

uint8_t imu_sequence = 0; //incremented on each call to imu_update

/************************************************************************
 * Neutral Readings                                                    
 ************************************************************************/
void attitude_setNeutral(void) {
    analog_setNeutral();
    ahrs.reset();
}

void attitude_update(void) {
    static uint32_t timestampJiffies;
    if (analog_attitudeDataStatus == ATTITUDE_SENSOR_DATA_READY) {
      J3TOGGLE;
      // OOPS: The attitude data might get added to while this is running...
      // debugOut.digital[0] &= ~DEBUG_MAINLOOP_TIMER;
      analog_updateAttitudeData();
      cli();
      float jiffies = jiffiesClock - timestampJiffies;
      timestampJiffies = jiffiesClock;
      sei();
      ahrs.update(attitude, jiffies * T_TIMER0IRQ);
    } else {
      // debugOut.digital[0] |= DEBUG_MAINLOOP_TIMER;
    }
}
