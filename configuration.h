#ifndef _CONFIGURATION_H
#define _CONFIGURATION_H

#include <inttypes.h>
#include <avr/io.h>

#define MAX_CONTROLCHANNELS 8
#define MAX_I2CCHANNELS 8
#define MAX_PWMCHANNELS 8

// bitmask for VersionInfo_t.HardwareError[0]
#define FC_ERROR0_GYRO_X 		0x01
#define FC_ERROR0_GYRO_Y 		0x02
#define FC_ERROR0_GYRO_Z 		0x04
#define FC_ERROR0_ACCEL_X 		0x08
#define FC_ERROR0_ACCEL_Y  		0x10
#define FC_ERROR0_ACCEL_Z 		0x20
#define FC_ERROR0_PRESSURE		0x40
#define FC_ERROR1_RES0			0x80
// bitmask for VersionInfo_t.HardwareError[1]
#define FC_ERROR1_I2C   	 	0x01
#define FC_ERROR1_BL_MISSING 	0x02
#define FC_ERROR1_SPI_RX	 	0x04
#define FC_ERROR1_PPM	 		0x08
#define FC_ERROR1_MIXER			0x10
#define FC_ERROR1_RES1			0x20
#define FC_ERROR1_RES2			0x40
#define FC_ERROR1_RES3			0x80

#define PID_NORMAL_VALUE 100

typedef struct {
	uint8_t SWMajor;
	uint8_t SWMinor;
	uint8_t SWPatch;
	uint8_t protoMajor;
	uint8_t protoMinor;
	uint8_t hardwareErrors[5];
}__attribute__((packed)) VersionInfo_t;

extern VersionInfo_t versionInfo;

typedef struct {
  uint8_t flightMode;
  
  uint8_t attGyroP;     // Attitude mode Proportional (attitude error to control)
  uint8_t attGyroI;     // Attitude mode Integral (attitude error integral to control. Serves to eliminate permanent attitude errors)
  uint8_t attGyroD;     // Attitude mode rate
  
  uint8_t rateGyroP;    // Rate mode Proportional
  uint8_t rateGyroI;    // Rate mode Integral (serves to eliminate slow rotation errors in rate mode)
  uint8_t rateGyroD;    // Rate mode Differential (GyroD)

  uint8_t yawGyroP;
  uint8_t yawGyroI;
  uint8_t yawGyroD;
  
  // Control
  uint8_t externalControl;
  uint8_t attitudeControl;
  
  // Height control
  uint8_t heightP;
  uint8_t heightI;
  uint8_t heightD;
  uint8_t heightSetting;


  // Output and servo
  uint8_t output0Timing;
  uint8_t output1Timing;

  uint8_t servoManualControl[2];

  // Correction
  uint8_t levelCorrection[2];

  // Simple direct navigation
  uint8_t naviMode;

  /* P */uint8_t userParams[8];
} DynamicParams_t;

extern volatile DynamicParams_t dynamicParams;

typedef struct {
  uint8_t sourceIdx, targetIdx;
  uint8_t min, max;
} MMXLATION;

/*
typedef struct {
  uint8_t sourceIdx, targetIdx;
} XLATION;
*/

typedef struct {
    uint8_t RCPolarity; // 1=positive, 0=negative. Use positive with Futaba receiver, negative with FrSky.
	uint8_t HWTrim;
	uint8_t variableOffset;
    uint8_t channels[MAX_CONTROLCHANNELS];
} ChannelMap_t;

extern ChannelMap_t channelMap;

#define LOG_MOTOR_MIXER_UNIT 6
#define LOG_DYNAMIC_STABILITY_SCALER 6

typedef enum {
  MIXER_SOURCE_ROLL = 0,
  MIXER_SOURCE_PITCH = 1,
  MIXER_SOURCE_THROTTLE = 2,
  MIXER_SOURCE_YAW = 3
} MixerSource_Control;

typedef enum {
  MIXER_SOURCE_AUX_GIMBAL_ROLL = 0,
  MIXER_SOURCE_AUX_GIMBAL_PITCH = 1,
  MIXER_SOURCE_AUX_RCCHANNEL = 2,
  MIXER_SOURCE_AUX_END = 2 + MAX_CONTROLCHANNELS 
} MixerSourceAux;

typedef enum {
  I2C0 = 0,
  I2C1 = 1,
  I2C2 = 2,
  I2C3 = 3,
  I2C4 = 4,
  I2C5 = 5,
  I2C6 = 6,
  I2C7 = 7,
  SERVO0 = MAX_I2CCHANNELS,
  NUM_OUTPUTS = 8 + MAX_PWMCHANNELS
} MixerDestination;

typedef struct {
  uint8_t outputType;
  uint8_t minValue;
  uint8_t maxValue;
  int8_t  flightControls[4];
  uint8_t auxSource;     // for selecting a gimbal axis or an RC channel.
  uint8_t oppositeMotor; // only relevant where used to control a multicopter motor.
}__attribute__((packed)) MixerMatrixRow_t;

typedef MixerMatrixRow_t OutputMixer_t[NUM_OUTPUTS];
extern OutputMixer_t outputMixer;

typedef enum {
  OUTPUT_TYPE_UNDEFINED,
  OUTPUT_TYPE_MOTOR,
  OUTPUT_TYPE_SERVO
} outputTypes;

typedef struct {
  int16_t offsets[3];
} sensorOffset_t;

typedef struct {
  uint8_t manualControl;
  uint8_t stabilizationFactor;
  uint8_t flags;
} Servo_t;

#define SERVO_STABILIZATION_REVERSE 1

typedef struct {
  uint8_t bitmask;
  uint8_t timing;
} output_flash_t;

typedef struct {
  /* Set up so that
   * Nose-up is positive on pitch
   * Roll-right is positive on roll
   * Turn-ccw is positive on yaw
   */
  uint8_t gyroQuadrant;
  /*
   * Set up so that:
   * Acc. Z is +1g in normal attitude
   * Acc. X gets positive in a left roll
   * Acc. Y gets positive in a nose-up attitude.
   */
  uint8_t accQuadrant;
  uint8_t imuReversedFlags;
  uint8_t gyroPIDFilterConstant;
  uint8_t gyroDWindowLength;
  uint8_t accFilterConstant;

} IMUConfig_t;

extern IMUConfig_t IMUConfig;

// values above 250 representing poti1 to poti4
typedef struct {
  // Global bitflags
  uint8_t bitConfig;  // see upper defines for bitcoding

  uint8_t flightMode;
  
  // uint8_t axisCoupling1; // Value: 0-250  Faktor, mit dem Yaw die Achsen Roll und Nick koppelt (NickRollMitkopplung)
  // uint8_t axisCoupling2; // Value: 0-250  Faktor, mit dem Nick und Roll verkoppelt werden
  // uint8_t axisCouplingYawCorrection;// Value: 0-250  Faktor, mit dem Nick und Roll verkoppelt werden

  uint8_t levelCorrection[2];

  // Control
  uint8_t attGyroP;     // Attitude mode Proportional (attitude error to control)
  uint8_t attGyroI;     // Attitude mode Integral (attitude error integral to control. Serves to eliminate permanent attitude errors)
  uint8_t attGyroIMax;  // Attitude mode Integral max. limit
  uint8_t attGyroD;     // Attitude mode rate
  
  uint8_t rateGyroP;    // Rate mode Proportional
  uint8_t rateGyroI;    // Rate mode Integral (serves to eliminate slow rotation errors in rate mode)
  uint8_t rateGyroIMax; // Rate mode Integral max. limit
  uint8_t rateGyroD;    // Rate mode Differential (GyroD)

  uint8_t yawGyroP;
  uint8_t yawGyroI;
  uint8_t yawGyroD;
  
  uint8_t externalControl; // for serial Control
  uint8_t attitudeControl;
  uint8_t dynamicStability;

  uint8_t stickP;
  uint8_t stickD;
  uint8_t stickYawP;
  uint8_t stickThrottleD;

  // Idle throttle. Affects the throttle stick only.
  uint8_t minThrottle;
  // Value of max. throttle stick.
  uint8_t maxThrottle;

  uint8_t compassMode;      // bitflag thing.
  uint8_t compassYawCorrection;
  uint8_t compassBendingReturnSpeed;
  uint8_t compassP;

  uint8_t batteryVoltageWarning;
  uint8_t emergencyThrottle;
  uint8_t emergencyFlightDuration;

  // Height Control
  uint8_t airpressureFilterConstant;
  uint8_t airpressureWindowLength; // 0 means: Use filter.
  uint8_t airpressureDWindowLength; // values 1..5 are legal.
  uint8_t airpressureAccZCorrection;

  uint8_t heightP;
  uint8_t heightI;
  uint8_t heightD;

  uint8_t heightSetting;
  uint8_t heightControlMaxIntegralIn;
  uint8_t heightControlMaxIntegralOut;
  uint8_t heightControlMaxThrottleChange;

  uint8_t heightControlTestOscPeriod;
  uint8_t heightControlTestOscAmplitude;

  // Servos
  uint8_t servoCount;
  uint8_t servoManualMaxSpeed;
  Servo_t servoConfigurations[2]; // [PITCH, ROLL]

  // Outputs
  output_flash_t outputFlash[2];
  uint8_t outputDebugMask;
  uint8_t outputFlags;

 // Shared for both modes of navigation
  uint8_t naviMode;
  uint8_t naviStickThreshold;
  uint8_t naviStickLimit;
  uint8_t GPSMininumSatellites;
  uint8_t naviP;
  uint8_t naviI;
  uint8_t naviD;

  uint8_t naviTestOscPeriod;
  uint8_t naviTestOscAmplitude;

  // User params
  uint8_t userParams[8];

  // Name
  char name[12];
} ParamSet_t;

extern ParamSet_t staticParams;

// MKFlags
#define MKFLAG_MOTOR_RUN  	    (1<<0)
//#define MKFLAG_FLY        	(1<<1)
//#define MKFLAG_CALIBRATE  	    (1<<2)
#define MKFLAG_EMERGENCY_FLIGHT (1<<4)
#define MKFLAG_LOWBAT		    (1<<5)
//#define MKFLAG_RESERVE2		    (1<<6)
//#define MKFLAG_RESERVE3		    (1<<7)

// bit mask for staticParams.bitConfig
#define CFG_SIMPLE_HEIGHT_CONTROL		(1<<0)
#define CFG_SIMPLE_HC_HOLD_SWITCH       (1<<1)
#define CFG_UNUSED      		        (1<<2)
#define CFG_COMPASS_ENABLED             (1<<3)
#define CFG_UNUSED2          	   		(1<<4)
#define CFG_NAVI_ENABLED		        (1<<5)
#define CFG_UNUSED3                     (1<<6)
#define CFG_GYRO_SATURATION_PREVENTION	(1<<7)

#define IMU_REVERSE_GYRO_XY				(1<<0)
#define	IMU_REVERSE_GYRO_Z			 	(1<<1)
#define IMU_REVERSE_ACCEL_XY			(1<<2)
#define IMU_REVERSE_ACCEL_Z				(1<<3)

#define ATMEGA644	0
#define ATMEGA644P	1

// Not really a part of configuration, but LEDs and HW s test are the same.
#define RED_OFF   {if((boardRelease == 10) || (boardRelease == 20)) PORTB &=~(1<<PORTB0); else  PORTB |= (1<<PORTB0);}
#define RED_ON    {if((boardRelease == 10) || (boardRelease == 20)) PORTB |= (1<<PORTB0); else  PORTB &=~(1<<PORTB0);}
#define RED_FLASH PORTB ^= (1<<PORTB0)
#define GRN_OFF   {if(boardRelease  < 12) PORTB &=~(1<<PORTB1); else PORTB |= (1<<PORTB1);}
#define GRN_ON    {if(boardRelease  < 12) PORTB |= (1<<PORTB1); else PORTB &=~(1<<PORTB1);}
#define GRN_FLASH PORTB ^= (1<<PORTB1)

// Mixer table

#define VARIABLE_COUNT  8

extern volatile uint8_t MKFlags;
extern int16_t variables[VARIABLE_COUNT]; // The "Poti"s.
extern uint8_t boardRelease;
extern uint8_t CPUType;

extern volatile uint8_t MKFlags;
extern uint16_t isFlying;

void IMUConfig_default(void);
void channelMap_default(void);
void paramSet_default(uint8_t setnumber);
void outputMixer_default(void);

void configuration_setNormalFlightMode(void);
void configuration_setFailsafeFlightMode(void);
void configuration_applyVariablesToParams(void);

void setCPUType(void);
void setBoardRelease(void);

// Called after a change in configuration parameters, as a hook for modules to take over changes.
void configuration_paramSetDidChange(void);
#endif // _CONFIGURATION_H
