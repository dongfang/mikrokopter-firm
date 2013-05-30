#include <util/delay.h>
#include <stddef.h>
#include <string.h>
#include "configuration.h"
#include "sensors.h"
#include "rc.h"
#include "output.h"
#include "flight.h"
#include "debug.h"

int16_t variables[VARIABLE_COUNT];
ParamSet_t staticParams;
ChannelMap_t channelMap;
OutputMixer_t outputMixer;
IMUConfig_t IMUConfig;
volatile DynamicParams_t dynamicParams;

uint8_t CPUType;
uint8_t boardRelease;

VersionInfo_t versionInfo;

// MK flags. TODO: Replace by enum. State machine.
uint16_t isFlying;
volatile uint8_t MKFlags;

const MMXLATION XLATIONS[] = {
{offsetof(ParamSet_t, flightMode), offsetof(DynamicParams_t, flightMode),0,255},

{offsetof(ParamSet_t, levelCorrection[0]), offsetof(DynamicParams_t, levelCorrection[0]),0,255},
{offsetof(ParamSet_t, levelCorrection[1]), offsetof(DynamicParams_t, levelCorrection[1]),0,255},

{offsetof(ParamSet_t, attGyroP), offsetof(DynamicParams_t, attGyroP),0,250},
{offsetof(ParamSet_t, attGyroI), offsetof(DynamicParams_t, attGyroI),0,250},
{offsetof(ParamSet_t, attGyroD), offsetof(DynamicParams_t, attGyroD),0,250},

{offsetof(ParamSet_t, rateGyroP), offsetof(DynamicParams_t, rateGyroP),0,250},
{offsetof(ParamSet_t, rateGyroI), offsetof(DynamicParams_t, rateGyroI),0,250},
{offsetof(ParamSet_t, rateGyroD), offsetof(DynamicParams_t, rateGyroD),0,250},

{offsetof(ParamSet_t, yawGyroP), offsetof(DynamicParams_t, yawGyroP),0,250},
{offsetof(ParamSet_t, yawGyroI), offsetof(DynamicParams_t, yawGyroI),0,250},
{offsetof(ParamSet_t, yawGyroD), offsetof(DynamicParams_t, yawGyroD),0,250},

{offsetof(ParamSet_t, attitudeControl), offsetof(DynamicParams_t, attitudeControl),0,255},
{offsetof(ParamSet_t, externalControl), offsetof(DynamicParams_t, externalControl),0,255},
{offsetof(ParamSet_t, heightP), offsetof(DynamicParams_t, heightP),0,255},
{offsetof(ParamSet_t, heightI), offsetof(DynamicParams_t, heightI),0,255},
{offsetof(ParamSet_t, heightD), offsetof(DynamicParams_t, heightD),0,255},
{offsetof(ParamSet_t, heightSetting), offsetof(DynamicParams_t, heightSetting),0,255},
{offsetof(ParamSet_t, servoConfigurations[0].manualControl), offsetof(DynamicParams_t, servoManualControl[0]),0,255},
{offsetof(ParamSet_t, servoConfigurations[1].manualControl), offsetof(DynamicParams_t, servoManualControl[1]),0,255},
{offsetof(ParamSet_t, outputFlash[0].timing), offsetof(DynamicParams_t, output0Timing),0,255},
{offsetof(ParamSet_t, outputFlash[1].timing), offsetof(DynamicParams_t, output1Timing),0,255},
{offsetof(ParamSet_t, naviMode), offsetof(DynamicParams_t, naviMode),0,255}};

uint8_t configuration_applyVariableToParam(uint8_t src, uint8_t min, uint8_t max) {
  uint8_t result;
  if (src>=(256-VARIABLE_COUNT)) result = variables[src-(256-VARIABLE_COUNT)];
  else result = src;
  if (result < min) result = min;
  else if (result > max) result = max;
  return result;
}

void configuration_fixVariableParams(uint8_t variableNumber) {
  // If variable value is to great, limit it.
  uint8_t value = variables[variableNumber];
  if (value >= 256-VARIABLE_COUNT)
    value = 256-VARIABLE_COUNT - 1;
  for (uint8_t i=0; i<sizeof(DynamicParams_t); i++) {
    uint8_t* pointerToParam =(uint8_t*)(&dynamicParams + i);
    if (*pointerToParam == 256-VARIABLE_COUNT + variableNumber) {
       *pointerToParam = value;
    }
  }
}

void configuration_applyVariablesToParams(void) {
  uint8_t i, src;
  uint8_t* pointerToTgt;

  for(i=0; i<sizeof(XLATIONS)/sizeof(MMXLATION); i++) {
    src = *((uint8_t*)(&staticParams) + XLATIONS[i].sourceIdx);
    pointerToTgt = (uint8_t*)(&dynamicParams) + XLATIONS[i].targetIdx;
    *pointerToTgt = configuration_applyVariableToParam(src, XLATIONS[i].min, XLATIONS[i].max);
  }

  // User parameters are always variable.
  for (i=0; i<sizeof(staticParams.userParams); i++) {
    src = *((uint8_t*)(&staticParams) + offsetof(ParamSet_t, userParams) + i);
    pointerToTgt = (uint8_t*)(&dynamicParams) + offsetof(DynamicParams_t, userParams) + i;
    *pointerToTgt = configuration_applyVariableToParam(src, 0, 255);
  }
}

void setCPUType(void) {   // works only after reset or power on when the registers have default values
#if (MCU_TYPE==atmega644)
	CPUType=ATMEGA644;
#else
  if((UCSR1A == 0x20) && (UCSR1C == 0x06)) CPUType = ATMEGA644P;  // initial Values for 644P after reset
  else CPUType = ATMEGA644;
#endif
}

/*
 * Automatic detection of hardware components is not supported in this development-oriented
 * FC firmware. It would go against the point of it: To enable alternative hardware 
 * configurations with otherwise unsupported components. Instead, one should write
 * custom code + adjust constants for the new hardware, and include the relevant code
 * from the makefile.
 * However - we still do detect the board release. Reason: Otherwise it would be too
 * tedious to have to modify the code for how to turn on and off LEDs when deploying
 * on different HW version....
 */
void setBoardRelease(void) {
  // the board release is coded via the pull up or down the 2 status LED

  PORTB &= ~((1 << PORTB1)|(1 << PORTB0)); // set tristate
  DDRB  &= ~((1 << DDB0)|(1 << DDB0)); // set port direction as input

  _delay_loop_2(1000); // make some delay

  switch( PINB & ((1<<PINB1)|(1<<PINB0))) {
    case 0x00:
      boardRelease = 10; // 1.0
      break;
    case 0x01:
      boardRelease = 11; // 1.1 or 1.2
      break;
    case 0x02:
      boardRelease = 20; // 2.0
      break;
    case 0x03:
      boardRelease = 13; // 1.3
      break;
    default:
      break;
    }
  // set LED ports as output
  DDRB |= (1<<DDB1)|(1<<DDB0);
  RED_OFF;
  GRN_OFF;
}

void configuration_setNormalFlightMode(void) {
  FlightMode_t flight_flightMode;
  if (dynamicParams.flightMode < 255/3)
    flight_flightMode = FM_RETURN_TO_LEVEL;
  else if (dynamicParams.flightMode < 255*2/3)
    flight_flightMode = FM_HEADING_HOLD;
  else 
    flight_flightMode = FM_RATE;
  flight_setMode(flight_flightMode);
  debugOut.analog[20] = flight_flightMode;
}

void configuration_setFailsafeFlightMode(void) {
  flight_setMode(FM_RETURN_TO_LEVEL);
}

// Called after a change in configuration parameters, as a hook for modules to take over changes.
void configuration_paramSetDidChange(void) {
  // This should be OK to do here as long as we don't change parameters during emergency flight. We don't.
  // No longer necessary.
  // configuration_setNormalFlightMode();
  flight_setParameters();
  output_setParameters();
  // Immediately load changes to output, and also signal the paramset change.
  output_init();
}

void setOtherDefaults(void) {
  // Height Control
  staticParams.airpressureFilterConstant = 8;
  staticParams.airpressureWindowLength = 8;
  staticParams.airpressureDWindowLength = 24;

  staticParams.airpressureAccZCorrection = 128+56;
  staticParams.heightP = 10;
  staticParams.heightD = 30;
  staticParams.heightSetting = 251;
  staticParams.heightControlMaxThrottleChange = 10;
  
  // Control
  staticParams.flightMode = 248;
  staticParams.stickP = 8;
  staticParams.stickD = 0;
  staticParams.stickYawP = 8;
  staticParams.stickThrottleD = 4;
  
  staticParams.minThrottle = 8;
  staticParams.maxThrottle = 230;

  staticParams.externalControl = 0;
  staticParams.attitudeControl = 0;
  staticParams.dynamicStability = 80; // 0 means: Regulation only by decreasing throttle. 200: Only by increasing.
  
  staticParams.attGyroP = PID_NORMAL_VALUE;
  staticParams.attGyroI = PID_NORMAL_VALUE;
  staticParams.attGyroIMax = 0;
  staticParams.attGyroD = PID_NORMAL_VALUE * 3/2;

  staticParams.rateGyroP = PID_NORMAL_VALUE;
  staticParams.rateGyroI = PID_NORMAL_VALUE;
  staticParams.rateGyroIMax = 0;
  staticParams.rateGyroD = PID_NORMAL_VALUE;

  staticParams.yawGyroP = PID_NORMAL_VALUE;
  staticParams.yawGyroI = PID_NORMAL_VALUE;
  staticParams.yawGyroD = PID_NORMAL_VALUE;
  
  staticParams.compassMode = 0;
  staticParams.compassYawCorrection = 64;
  staticParams.compassP = PID_NORMAL_VALUE;
  staticParams.levelCorrection[0] = 249; // var1
  staticParams.levelCorrection[1] = 250; // var2

  // Servos
  staticParams.servoCount = 8;
  staticParams.servoManualMaxSpeed = 10;
  for (uint8_t i=0; i<2; i++) {
    staticParams.servoConfigurations[i].manualControl = 128;
    staticParams.servoConfigurations[i].stabilizationFactor = 100;
    staticParams.servoConfigurations[i].flags = 0;
  }
  
  // Battery warning and emergency flight
  staticParams.batteryVoltageWarning = 101;  // 3.7 each for 3S
  staticParams.emergencyThrottle = 35;
  staticParams.emergencyFlightDuration = 30;

  // Outputs
  staticParams.outputFlash[0].bitmask = 1; //0b01011111;
  staticParams.outputFlash[0].timing = 15;
  staticParams.outputFlash[1].bitmask = 3; //0b11110011;
  staticParams.outputFlash[1].timing = 15;

  staticParams.outputDebugMask = DEBUG_INVERTED;
  staticParams.outputFlags   = /* OUTPUTFLAGS_FLASH_0_AT_BEEP | OUTPUTFLAGS_FLASH_1_AT_BEEP | */ OUTPUTFLAGS_USE_ONBOARD_LEDS;

  staticParams.naviMode = 0; // free.

  staticParams.heightControlMaxIntegralIn = 125;
  staticParams.heightControlMaxIntegralOut = 75;
  staticParams.heightControlMaxThrottleChange = 75;
  staticParams.heightControlTestOscPeriod = 0;
  staticParams.heightControlTestOscAmplitude = 0;
}

/***************************************************/
/*    Default Values for parameter set 1           */
/***************************************************/
void paramSet_default(uint8_t setnumber) {
  setOtherDefaults();
 
  for (uint8_t i=0; i<8; i++) {
    staticParams.userParams[i] = i;
  }

  staticParams.bitConfig = 0;//CFG_GYRO_SATURATION_PREVENTION;

  memcpy(staticParams.name, "Default\0", 6);
}

void IMUConfig_default(void) {
  IMUConfig.gyroPIDFilterConstant = 1;
  IMUConfig.gyroDWindowLength = 3;
  // IMUConfig.gyroDFilterConstant = 1;
  IMUConfig.accFilterConstant = 10;
  
  
  gyro_setDefaultParameters();
}

/***************************************************/
/*    Default Values for Mixer Table               */
/***************************************************/
void outputMixer_default(void) { // Quadro
  uint8_t i;
  // clear mixer table (but preset throttle)
  for (i = 0; i < NUM_OUTPUTS; i++) {
    outputMixer[i].flightControls[MIXER_SOURCE_THROTTLE] = i < 4 ? (1<<LOG_MOTOR_MIXER_UNIT) : 0;
    outputMixer[i].flightControls[MIXER_SOURCE_PITCH] = 0;
    outputMixer[i].flightControls[MIXER_SOURCE_ROLL] = 0;
    outputMixer[i].flightControls[MIXER_SOURCE_YAW] = 0;
    outputMixer[i].oppositeMotor = (uint8_t)-1;
    outputMixer[i].auxSource = (uint8_t)-1;
    outputMixer[i].outputType = i < 4 ? OUTPUT_TYPE_MOTOR : OUTPUT_TYPE_UNDEFINED;
    outputMixer[i].minValue = i < 4 ? 1   : 128;
    outputMixer[i].maxValue = i < 4 ? 255 : 128;
  }
  // default = Quadro+
  outputMixer[0].flightControls[MIXER_SOURCE_PITCH] = -(1<<LOG_MOTOR_MIXER_UNIT);
  outputMixer[0].flightControls[MIXER_SOURCE_YAW] = +(1<<LOG_MOTOR_MIXER_UNIT);
  outputMixer[0].oppositeMotor = 1;

  outputMixer[1].flightControls[MIXER_SOURCE_PITCH] = +(1<<LOG_MOTOR_MIXER_UNIT);
  outputMixer[1].flightControls[MIXER_SOURCE_YAW] = +(1<<LOG_MOTOR_MIXER_UNIT);
  outputMixer[1].oppositeMotor = 0;
  
  outputMixer[2].flightControls[MIXER_SOURCE_ROLL] = +(1<<LOG_MOTOR_MIXER_UNIT);
  outputMixer[2].flightControls[MIXER_SOURCE_YAW] = -(1<<LOG_MOTOR_MIXER_UNIT);
  outputMixer[2].oppositeMotor = 3;

  outputMixer[3].flightControls[MIXER_SOURCE_ROLL] = -(1<<LOG_MOTOR_MIXER_UNIT);
  outputMixer[3].flightControls[MIXER_SOURCE_YAW] = -(1<<LOG_MOTOR_MIXER_UNIT);
  outputMixer[3].oppositeMotor = 2;

  outputMixer[8].outputType = OUTPUT_TYPE_SERVO;
  outputMixer[8].auxSource = MIXER_SOURCE_AUX_RCCHANNEL;
  outputMixer[8].minValue = 40;
  outputMixer[8].maxValue = 256-40;

  outputMixer[9].outputType = OUTPUT_TYPE_SERVO;
  outputMixer[9].auxSource = MIXER_SOURCE_AUX_RCCHANNEL+1;
  outputMixer[9].minValue = 10;
  outputMixer[9].maxValue = 256-10;
  
  /*
  // default = Quadro X
  motorMixer.matrix[0][MIX_PITCH] = +(1<<LOG_MOTOR_MIXER_SCALER);
  motorMixer.matrix[0][MIX_ROLL] = +(1<<LOG_MOTOR_MIXER_SCALER);
  motorMixer.matrix[0][MIX_YAW] = +(1<<LOG_MOTOR_MIXER_SCALER);
  motorMixer.matrix[0][MIX_OPPOSITE_MOTOR] = 1;

  motorMixer.matrix[1][MIX_PITCH] = -(1<<LOG_MOTOR_MIXER_SCALER);
  motorMixer.matrix[1][MIX_ROLL] = -(1<<LOG_MOTOR_MIXER_SCALER);
  motorMixer.matrix[1][MIX_YAW] = +(1<<LOG_MOTOR_MIXER_SCALER);
  motorMixer.matrix[1][MIX_OPPOSITE_MOTOR] = 0;

  motorMixer.matrix[2][MIX_PITCH] = +(1<<LOG_MOTOR_MIXER_SCALER);
  motorMixer.matrix[2][MIX_ROLL] = -(1<<LOG_MOTOR_MIXER_SCALER);
  motorMixer.matrix[2][MIX_YAW] = -(1<<LOG_MOTOR_MIXER_SCALER);
  motorMixer.matrix[2][MIX_OPPOSITE_MOTOR] = 3;

  motorMixer.matrix[3][MIX_PITCH] = -(1<<LOG_MOTOR_MIXER_SCALER);
  motorMixer.matrix[3][MIX_ROLL] = +(1<<LOG_MOTOR_MIXER_SCALER);
  motorMixer.matrix[3][MIX_YAW] = -(1<<LOG_MOTOR_MIXER_SCALER);
  motorMixer.matrix[3][MIX_OPPOSITE_MOTOR] = 2;

  memcpy(motorMixer.name, "Quadro X\0", 9);
*/
}

/***************************************************/
/*    Default Values for R/C Channels              */
/***************************************************/
void channelMap_default(void) {
  channelMap.RCPolarity = 0;
  channelMap.channels[CH_PITCH]    = 1;
  channelMap.channels[CH_ROLL]     = 0;
  channelMap.channels[CH_THROTTLE] = 2;
  channelMap.channels[CH_YAW]      = 3;
  channelMap.channels[CH_VARIABLES + 0] = 4;
  channelMap.channels[CH_VARIABLES + 1] = 5;
  channelMap.channels[CH_VARIABLES + 2] = 6;
  channelMap.channels[CH_VARIABLES + 3] = 7;
}
