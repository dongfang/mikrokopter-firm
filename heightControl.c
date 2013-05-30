#include <inttypes.h>
#include "attitude.h"
#include "configuration.h"
#include "definitions.h"
#include "debug.h" 

#define INTEGRAL_LIMIT 100000

#define LATCH_TIME 3

// This will translate a height value to a target height in centimeters.
// Currently: One step is 50 cm and zero is 5 meters below the takeoff altitude. With max. 255 steps the max.
// target height is then about 120m.
#define HEIGHT_GAIN 50L
#define HEIGHT_FORMULA(x) ((x) * HEIGHT_GAIN - 500L)

int32_t setHeight;
int32_t targetHeight;

uint16_t hc_testOscPrescaler = 0;
uint8_t hc_testOscTimer = 0;

int32_t maxHeightThisFlight;
int32_t iHeight;

void HC_setGround(void) {
  analog_setGround();
  // This should also happen when height control is enabled in-flight.
  setHeight = targetHeight = analog_getHeight();
  hc_testOscTimer = 0;
  maxHeightThisFlight = 0;
  iHeight = 0;
}

uint8_t HC_isSwitchOn(void) {
  return (dynamicParams.heightSetting >= 255/3);
}

void HC_periodicTask(void) {
  int32_t height = analog_getHeight();
  static uint8_t setHeightLatch = 0;
  
  if (height > maxHeightThisFlight)
    maxHeightThisFlight = height;
  
  // debugOut.analog[25] = dynamicParams.heightSetting;

  if (staticParams.bitConfig & CFG_SIMPLE_HC_HOLD_SWITCH) {
    if (HC_isSwitchOn()) {
      // Switch is ON
      if (setHeightLatch <= LATCH_TIME) {
        if (setHeightLatch == LATCH_TIME) {
          // Freeze the height as target. We want to do this exactly once each time the switch is thrown ON.
          setHeight = height;
          hc_testOscTimer = 0;
          iHeight = 0;
        }
        // Time not yet reached.
        setHeightLatch++;
      }
    } else {
      // Switch is OFF.
      setHeightLatch = 0;
    }
  } else {
    // Switch is not activated; take the "max-height" as the target height.
    setHeight = (uint16_t) (HEIGHT_FORMULA(dynamicParams.heightSetting));
  }
  
  /*
  if (++heightRampingTimer == INTEGRATION_FREQUENCY / 10) {
    heightRampingTimer = 0;
    if (rampedTargetHeight < targetHeight) {
      // climbing
      if (rampedTargetHeight < targetHeight - staticParams.heightSlewRate) {
	rampedTargetHeight += staticParams.heightSlewRate;
      } else {
	rampedTargetHeight = targetHeight;
      }
    } else {
      // descending
      if (rampedTargetHeight > targetHeight + staticParams.heightSlewRate) {
	rampedTargetHeight -= staticParams.heightSlewRate;
      } else {
	rampedTargetHeight = targetHeight;
      }
    }
  }
  */
  //	  uint8_t heightControlTestOscPeriod;
  //	  uint8_t heightControlTestOscAmplitude;

  hc_testOscPrescaler++;
  
  if (hc_testOscPrescaler == 250) {
	  hc_testOscPrescaler = 0;
	  hc_testOscTimer++;
	  if (hc_testOscTimer == staticParams.heightControlTestOscPeriod) {
		  hc_testOscTimer = 0;
		  if (staticParams.heightControlTestOscAmplitude)
			  iHeight = 0;
	  } else if (hc_testOscTimer == staticParams.heightControlTestOscPeriod/2) {
		  if (staticParams.heightControlTestOscAmplitude)
			  iHeight = 0;
	  }
  }

  if (hc_testOscTimer < staticParams.heightControlTestOscPeriod/2)
	  targetHeight = setHeight;
  else
	  targetHeight = setHeight + (uint16_t)staticParams.heightControlTestOscAmplitude * HEIGHT_GAIN;

  //if (staticParams.)
  // height, in meters (so the division factor is: 100)
  // debugOut.analog[24] = (117100 - filteredAirPressure) / 100;
  // Calculated 0 alt number: 108205
  // Experimental 0 alt number: 117100
}

#define LOG_PHEIGHT_SCALE 10
#define LOG_IHEIGHT_SCALE 24
#define LOG_DHEIGHT_SCALE 6

// takes 180-200 usec (with integral term). That is too heavy!!!
// takes 100 usec without integral term.
void HC_periodicTaskAndRPTY(int16_t* RPTY) {
  HC_periodicTask();
  int16_t throttle = RPTY[CONTROL_THROTTLE];
  int32_t height = analog_getHeight();
  int32_t heightError = targetHeight - height;
  int16_t dHeight = analog_getDHeight();
  
  debugOut.analog[22] = height/10L;
  debugOut.analog[23] = dHeight;

  if (heightError > 0) {
    debugOut.digital[0] |= DEBUG_HEIGHT_DIFF;
  } else {
    debugOut.digital[0] &= ~DEBUG_HEIGHT_DIFF;
  }

  if (dHeight > 0) {
    debugOut.digital[1] |= DEBUG_HEIGHT_DIFF;
  } else {
    debugOut.digital[1] &= ~DEBUG_HEIGHT_DIFF;
  }

  int32_t heightErrorForIntegral = heightError;
  int32_t heightErrorForIntegralLimit = staticParams.heightControlMaxIntegralIn << LOG_PHEIGHT_SCALE;

  if (heightErrorForIntegral > heightErrorForIntegralLimit) {
    heightErrorForIntegral = heightErrorForIntegralLimit;
  } else if (heightErrorForIntegral < -heightErrorForIntegralLimit) {
    heightErrorForIntegral =- heightErrorForIntegralLimit;
  }

  // iHeight, at a difference of 5 meters and a freq. of 488 Hz, will grow with 244000 / sec....
  iHeight += heightErrorForIntegral;

#define IHEIGHT_SCALE 24
  // dThrottle is in the range between +/- 1<<(IHEIGHT_SCALE+8)>>(IHEIGHT_SCALE) = +/- 256
  int16_t dThrottleI =  (iHeight * (int32_t)dynamicParams.heightI) >> (IHEIGHT_SCALE);

  if (dThrottleI > staticParams.heightControlMaxIntegralOut) {
    dThrottleI = staticParams.heightControlMaxIntegralOut;
    iHeight = ((int32_t)staticParams.heightControlMaxIntegralOut << IHEIGHT_SCALE) / dynamicParams.heightI;
  } else if (dThrottleI < -staticParams.heightControlMaxIntegralOut) {
    dThrottleI = -staticParams.heightControlMaxIntegralOut;
    iHeight = -((int32_t)staticParams.heightControlMaxIntegralOut << IHEIGHT_SCALE) / dynamicParams.heightI;
  }

  int16_t dThrottleP = (heightError * dynamicParams.heightP) >> LOG_PHEIGHT_SCALE;
  int16_t dThrottleD = (dHeight * dynamicParams.heightD) >> LOG_DHEIGHT_SCALE;

  //debugOut.analog[10] = dThrottleP;
  //debugOut.analog[11] = dThrottleI;
  //debugOut.analog[12] = dThrottleD;
  //debugOut.analog[13] = heightError/10;

  int16_t dThrottle = dThrottleI + dThrottleP - dThrottleD;

  if (dThrottle > staticParams.heightControlMaxThrottleChange)
    dThrottle = staticParams.heightControlMaxThrottleChange;
  else if (dThrottle < -staticParams.heightControlMaxThrottleChange)
    dThrottle = -staticParams.heightControlMaxThrottleChange;

  /*
  debugOut.analog[19] = throttle;
  debugOut.analog[20] = dThrottle;
  debugOut.analog[21] = height;
  debugOut.analog[22] = rampedTargetHeight;
  debugOut.analog[23] = heightError;
  */

  if (staticParams.bitConfig & CFG_SIMPLE_HEIGHT_CONTROL) {
    if (!(staticParams.bitConfig & CFG_SIMPLE_HC_HOLD_SWITCH) || HC_isSwitchOn()) {
      // If switch is not in use --> Just apply height control.
      // If switch is in use     --> only apply height control when switch is also ON.
      throttle += dThrottle;
    }
  }

  /* Experiment: Find hover-throttle */

#define DEFAULT_HOVERTHROTTLE 200
int32_t stronglyFilteredHeightDiff = 0;
// uint16_t hoverThrottle = 0; // DEFAULT_HOVERTHROTTLE;
uint16_t stronglyFilteredThrottle = DEFAULT_HOVERTHROTTLE;
#define HOVERTHROTTLEFILTER 25
  
  stronglyFilteredHeightDiff = (stronglyFilteredHeightDiff
				* (HOVERTHROTTLEFILTER - 1) + dHeight) / HOVERTHROTTLEFILTER;
  stronglyFilteredThrottle = (stronglyFilteredThrottle * (HOVERTHROTTLEFILTER
							  - 1) + throttle) / HOVERTHROTTLEFILTER;
  
  /*
  if (isFlying >= 1000 && stronglyFilteredHeightDiff < 3
      && stronglyFilteredHeightDiff > -3) {
    hoverThrottle = stronglyFilteredThrottle;
    debugOut.digital[0] |= DEBUG_HOVERTHROTTLE;
  } else
    debugOut.digital[0] &= ~DEBUG_HOVERTHROTTLE;

    */
  
  RPTY[CONTROL_THROTTLE] = throttle;
}

/*
 For a variometer thingy:
 When switch is thrown on, freeze throttle (capture it into variable)
 For each iter., add (throttle - frozen throttle) to target height. Maybe don't do ramping.
 Output = frozen throttle + whatever is computed +/-. Integral?
 */
