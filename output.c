#include <inttypes.h>
#include "output.h"
#include "debug.h"
#include "timer0.h"
#include "timer2.h"
#include "twimaster.h"
// For gimbal stab.
#include "attitude.h"
#include "definitions.h"
#include "flight.h"
#include "uart0.h" 
#include "beeper.h"
#include "controlMixer.h"

#define CHECK_MIN_MAX(value, min, max) {if(value < min) value = min; else if(value > max) value = max;}

uint8_t flashCnt[2], flashMask[2];

int16_t throttleTerm;
int32_t yawTerm, term[2];

uint8_t positiveDynamic, negativeDynamic;

float previousManualValues[2];

void output_init(void) {
  // set PC2 & PC3 as output (control of J16 & J17)
  DDRC |= (1 << DDC2) | (1 << DDC3);
  output_setLED(0,0);
  output_setLED(1,0);
  flashCnt[0] = flashCnt[1] = 0;
  flashMask[0] = flashMask[1] = 128;

  for (uint8_t axis=0; axis<2; axis++)
    previousManualValues[axis] = dynamicParams.servoManualControl[axis] * (1<<LOG_CONTROL_BYTE_SCALING);
}

void output_setParameters() {
  if (staticParams.dynamicStability > PID_NORMAL_VALUE) {
    // Normal gain of 1.
    positiveDynamic = 1<<LOG_DYNAMIC_STABILITY_SCALER;
    // Gain between 1 (for staticParams.dynamicStability == PID_NORMAL_VALUE) and 0(for staticParams.dynamicStability == 2*PID_NORMAL_VALUE) 
    negativeDynamic = (1<<(LOG_DYNAMIC_STABILITY_SCALER+1)) - (1<<LOG_DYNAMIC_STABILITY_SCALER) * staticParams.dynamicStability / PID_NORMAL_VALUE;
    if (negativeDynamic < 0)
      negativeDynamic = 0;
  } else {
    negativeDynamic = 1<<LOG_DYNAMIC_STABILITY_SCALER;
    positiveDynamic = (1<<LOG_DYNAMIC_STABILITY_SCALER) * staticParams.dynamicStability / PID_NORMAL_VALUE;
  }

}

void output_setLED(uint8_t num, uint8_t state) {
  if (staticParams.outputFlags & (OUTPUTFLAGS_INVERT_0 << num)) {
    if (state) OUTPUT_LOW(num) else OUTPUT_HIGH(num);
  } else {
    if (state) OUTPUT_HIGH(num) else OUTPUT_LOW(num);
  }
  if (staticParams.outputFlags & OUTPUTFLAGS_USE_ONBOARD_LEDS) {
    if (num) {
      if (state) GRN_ON else GRN_OFF;
    } else {
      if (state) RED_ON else RED_OFF;
    }
  }
}

void flashingLight(uint8_t port, uint8_t timing, uint8_t bitmask, uint8_t manual) {
  if (timing > 250 && manual > 230) {
    // "timing" is set to "manual (a variable)" and the value is very high --> Set to the value in bitmask bit 7.
    output_setLED(port, 1);
  } else if (timing > 250 && manual < 10) {
    // "timing" is set to "manual" (a variable) and the value is very low --> Set to the negated value in bitmask bit 7.
    output_setLED(port, 0);
  } else if (!flashCnt[port]--) {
    // rotating mask over bitmask...
    flashCnt[port] = timing - 1;
    if (flashMask[port] == 1)
      flashMask[port] = 128;
    else
      flashMask[port] >>= 1;
      output_setLED(port, flashMask[port] & bitmask);
  }
}

void output_update(void) {
  if (staticParams.outputFlags & OUTPUTFLAGS_TEST_ON) {
    output_setLED(0, 1);
    output_setLED(1, 1);
  } else if (staticParams.outputFlags & OUTPUTFLAGS_TEST_OFF) {
    output_setLED(0, 0);
    output_setLED(1, 0);
  } else {
    if (staticParams.outputFlags & OUTPUTFLAGS_FLASH_0_AT_BEEP && beepModulation != BEEP_MODULATION_NONE) {
      flashingLight(0, 25, 0x55, 25);
    } else if (staticParams.outputDebugMask) {
      output_setLED(0, debugOut.digital[0] & staticParams.outputDebugMask);
    } else flashingLight(0, staticParams.outputFlash[0].timing, staticParams.outputFlash[0].bitmask, dynamicParams.output0Timing);
    if (staticParams.outputFlags & OUTPUTFLAGS_FLASH_1_AT_BEEP && beepModulation != BEEP_MODULATION_NONE) {
      flashingLight(1, 25, 0x55, 25);
    } else if (staticParams.outputDebugMask) {
      output_setLED(1, debugOut.digital[1] & staticParams.outputDebugMask);
    } else flashingLight(1, staticParams.outputFlash[1].timing, staticParams.outputFlash[1].bitmask, dynamicParams.output1Timing);
  }
}

void beep(uint16_t millis) {
  beepTime = millis;
}

/*
 * Make [numbeeps] beeps.
 */
void beepNumber(uint8_t numbeeps) {
  while(numbeeps--) {
    if(MKFlags & MKFLAG_MOTOR_RUN) return; //auf keinen Fall bei laufenden Motoren!
    beep(100); // 0.1 second
    delay_ms(250); // blocks 250 ms as pause to next beep,
    // this will block the flight control loop,
    // therefore do not use this function if motors are running
  }
}

/*
 * Beep the R/C alarm signal
 */
void beepRCAlarm(void) {
  if(beepModulation == BEEP_MODULATION_NONE) { // If not already beeping an alarm signal (?)
    beepTime = 15000; // 1.5 seconds
    beepModulation = BEEP_MODULATION_RCALARM;
  }
}

/*
 * Beep the I2C bus error signal
 */
void beepI2CAlarm(void) {
  if((beepModulation == BEEP_MODULATION_NONE) && (MKFlags & MKFLAG_MOTOR_RUN)) {
    beepTime = 10000; // 1 second
    beepModulation = BEEP_MODULATION_I2CALARM;
  }
}

/*
 * Beep the battery low alarm signal
 */
void beepBatteryAlarm(void) {
  beepModulation = BEEP_MODULATION_BATTERYALARM;
  if(!beepTime) {
    beepTime = 6000; // 0.6 seconds
  }
}

/*
 * Beep the EEPROM checksum alarm
 */
void beepEEPROMAlarm(void) {
  beepModulation = BEEP_MODULATION_EEPROMALARM;
  if(!beepTime) {
    beepTime = 6000; // 0.6 seconds
  }
}

// Result centered at 0 and scaled to control range steps.
float gimbalStabilizationPart(uint8_t axis) {
  float value = attitude[axis];
  //value *= STABILIZATION_FACTOR;
  value *= ((float)CONTROL_RANGE / 50.0 / (1<<14)); // 1<<14 scales 90 degrees to full range at normal gain setting (50)
  value *= staticParams.servoConfigurations[axis].stabilizationFactor;
  if (staticParams.servoConfigurations[axis].flags & SERVO_STABILIZATION_REVERSE)
    return -value;
  return value;
}

// Constant-speed limitation.
float gimbalManualPart(uint8_t axis) {
  float manualValue = (dynamicParams.servoManualControl[axis] - 128) * (1<<LOG_CONTROL_BYTE_SCALING);
  float diff = manualValue - previousManualValues[axis];
  uint8_t maxSpeed = staticParams.servoManualMaxSpeed;
  if (diff > maxSpeed) diff = maxSpeed;
  else if (diff < -maxSpeed) diff = -maxSpeed;
  manualValue = previousManualValues[axis] + diff;
  previousManualValues[axis] = manualValue;
  return manualValue;
}

// Result centered at 0 and scaled in control range.
float gimbalServoValue(uint8_t axis) {
  float value = gimbalStabilizationPart(axis);
  value += gimbalManualPart(axis);
  //int16_t limit = staticParams.servoConfigurations[axis].minValue * SCALE_FACTOR;
  //if (value < limit) value = limit;
  //limit = staticParams.servoConfigurations[axis].maxValue * SCALE_FACTOR;
  //if (value > limit) value = limit;
  return value;
}

// Result centered at 0 and scaled in control range.
float getAuxValue(uint8_t auxSource) {
  switch(auxSource) {
  case (uint8_t)-1:
    return 0;
  case MIXER_SOURCE_AUX_GIMBAL_ROLL:
    return gimbalServoValue(0);
  case MIXER_SOURCE_AUX_GIMBAL_PITCH:
    return gimbalServoValue(1);
  default: // an R/C variable or channel or what we make of it...
    return controls[auxSource - MIXER_SOURCE_AUX_RCCHANNEL];
  }
}

// value is generally in the 10 bits range.
// mix is 6 bits.
// and dynamics are 6 bits --> 22 bits needed + sign + space to spare.
static inline int32_t mixin(int8_t mix, int16_t value) {
  int32_t x = (int32_t)mix * value;
  if (x > 0) {
    return x * positiveDynamic;
  } else {
    return x * negativeDynamic;
  }
}

void output_applyMulticopterMixer(void) {
  int16_t _outputs[NUM_OUTPUTS];

  for (uint8_t i=0; i<NUM_OUTPUTS; i++) {
    _outputs[i] = 0;
  }

  // Process throttle, roll, pitch, yaw in special way with dynamic stability and with saturation to opposite motor.
  for (uint8_t i=0; i<NUM_OUTPUTS; i++) {
    if (outputMixer[i].outputType == OUTPUT_TYPE_MOTOR) {
      int32_t tmp;
      tmp = ((int32_t)throttleTerm<<6) * outputMixer[i].flightControls[MIXER_SOURCE_THROTTLE];
      tmp += mixin(outputMixer[i].flightControls[MIXER_SOURCE_ROLL], term[CONTROL_ROLL]);
      tmp += mixin(outputMixer[i].flightControls[MIXER_SOURCE_PITCH], term[CONTROL_PITCH]);
      tmp += mixin(outputMixer[i].flightControls[MIXER_SOURCE_YAW], yawTerm);
    
      // Compensate for the factor of 64 multiplied by in matrix mixing and another factor of 64 for the positive/negative dynamic stuff.
      _outputs[i] += tmp >> (LOG_MOTOR_MIXER_UNIT + LOG_DYNAMIC_STABILITY_SCALER);

      // Deduct saturation from opposite motor output.
      int16_t excess = _outputs[i] - (outputMixer[i].maxValue << LOG_CONTROL_BYTE_SCALING);
      if (excess > 0) {
        uint8_t oppositeIndex = outputMixer[i].oppositeMotor;
        if (oppositeIndex != -1)
          _outputs[oppositeIndex] -= excess;
      }
    }
  }

  // I2C part.
  for (uint8_t i=0; i<MAX_I2CCHANNELS; i++) {
    // I2C supports only motors anyway..
    if (outputMixer[i].outputType != OUTPUT_TYPE_MOTOR) continue;
    
    if (outputTestActive) {
      mkblcs[i].throttle = outputTest[i];
    } else if (MKFlags & MKFLAG_MOTOR_RUN) {
      int16_t asByte = _outputs[i] >> LOG_CONTROL_BYTE_SCALING;
      // Apply limits.
      CHECK_MIN_MAX(asByte, outputMixer[i].minValue, outputMixer[i].maxValue);
      if (i<4)
        debugOut.analog[16 + i] = asByte;
      mkblcs[i].throttle = asByte;
    } else {
      mkblcs[i].throttle = 0;
    }
  }
  
  for (uint8_t i=0; i<MAX_PWMCHANNELS; i++) {
    uint8_t sourceIndex = MAX_I2CCHANNELS + i;

    if (outputMixer[sourceIndex].outputType == OUTPUT_TYPE_MOTOR) {
      if (outputTestActive) {
        // When testing, min/max does NOT apply.
        pwmChannels[i] = (int16_t)(outputTest[sourceIndex] * PWM_BYTE_SCALE_FACTOR + PULSELENGTH_1000 + 0.5);
      } else {
        int16_t throttle;
        if (MKFlags & MKFLAG_MOTOR_RUN) {
          throttle = _outputs[sourceIndex];
          int16_t min = outputMixer[sourceIndex].minValue << LOG_CONTROL_BYTE_SCALING;
          int16_t max = outputMixer[sourceIndex].maxValue << LOG_CONTROL_BYTE_SCALING;
          CHECK_MIN_MAX(throttle, min, max);
          throttle = (int16_t)(throttle * PWM_CONTROL_SCALE_FACTOR + PULSELENGTH_1000 + 0.5);
        } else {
          throttle = PULSELENGTH_1000;
        }
        pwmChannels[i] = throttle;
      }
    } else if (outputMixer[sourceIndex].outputType == OUTPUT_TYPE_SERVO) {
      int16_t servoValue;
      if (outputTestActive) {
        servoValue = outputTest[sourceIndex];
        // When testing, min/max DOES apply.
        CHECK_MIN_MAX(servoValue, outputMixer[sourceIndex].minValue, outputMixer[sourceIndex].maxValue);
        servoValue = ((float)servoValue * PWM_BYTE_SCALE_FACTOR + PULSELENGTH_1000 + 0.5);
      } else {
        float fServoValue = getAuxValue(outputMixer[sourceIndex].auxSource);
        int16_t min = (outputMixer[sourceIndex].minValue-128) << LOG_CONTROL_BYTE_SCALING;
        int16_t max = (outputMixer[sourceIndex].maxValue-128) << LOG_CONTROL_BYTE_SCALING;
        CHECK_MIN_MAX(fServoValue, min, max);
        servoValue = (int16_t)(fServoValue * PWM_CONTROL_SCALE_FACTOR + PULSELENGTH_1500 + 0.5);
      }
      pwmChannels[i] = servoValue;
    } else { // undefined channel.
      pwmChannels[i] = PULSELENGTH_1500;
    }
  }
}