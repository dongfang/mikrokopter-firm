#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "rc.h"
//#include "controlMixer.h"
#include "configuration.h"
#include "commands.h"
#include "definitions.h"

// The channel array is 0-based!
volatile int16_t PPM_in[MAX_CONTROLCHANNELS];
volatile int16_t PPM_diff[MAX_CONTROLCHANNELS];
volatile uint16_t RC_buffer[MAX_CONTROLCHANNELS];
volatile uint8_t inBfrPnt;

volatile uint8_t RCQuality;
uint8_t lastRCCommand;
uint8_t commandTimer;

#define TIME(s) ((int16_t)(((F_CPU/8000)*(float)s + 0.5f)))

/***************************************************************
 *  16bit timer 1 is used to decode the PPM-Signal            
 ***************************************************************/
void RC_Init(void) {
  uint8_t sreg = SREG;

  // disable all interrupts before reconfiguration
  cli();

  // PPM-signal is connected to the Input Capture Pin (PD6) of timer 1
  DDRD &= ~(1<<6);
  PORTD |= (1<<PORTD6);

  // Channel 5,6,7 is decoded to servo signals at pin PD5 (J3), PD4(J4), PD3(J5)
  // set as output
  DDRD |= (1<<DDD5) | (1<<DDD4) | (1<<DDD3);
  // low level
  PORTD &= ~((1<<PORTD5) | (1<<PORTD4) | (1<<PORTD3));

  // PD3 can't be used if 2nd UART is activated
  // because TXD1 is at that port
  if (CPUType != ATMEGA644P) {
    DDRD |= (1<<PORTD3);
    PORTD &= ~(1<<PORTD3);
  }

  // Normal Mode (bits: WGM13=0, WGM12=0, WGM11=0, WGM10=0)
  // Compare output pin A & B is disabled (bits: COM1A1=0, COM1A0=0, COM1B1=0, COM1B0=0)
  // Set clock source to SYSCLK/8 (bit: CS12=0, CS11=1, CS10=1)
  // Enable input capture noise canceler (bit: ICNC1=1)
  // Therefore the counter increments at a clock of 20 MHz/64 = 312.5 kHz or 3.2�s
  // The longest period is 0xFFFF / 312.5 kHz = 0.209712 s.
  TCCR1A &= ~((1<<COM1A1)| (1<<COM1A0) | (1<<COM1B1) | (1<<COM1B0) | (1<<WGM11) | (1<<WGM10));
  TCCR1B &= ~((1<<WGM13) | (1<<WGM12)  | (1<<CS12));
  TCCR1B |= (1<<CS11)    | (1<<ICNC1);
  TCCR1C &= ~((1<<FOC1A) | (1<<FOC1B));

  if (channelMap.RCPolarity) {
    TCCR1B |= (1<<ICES1);
  } else {
    TCCR1B &= ~(1<<ICES1);
  }

  TCCR1C &= ~((1 << FOC1A) | (1 << FOC1B));

  // Timer/Counter1 Interrupt Mask Register
  // Enable Input Capture Interrupt (bit: ICIE1=1)
  // Disable Output Compare A & B Match Interrupts (bit: OCIE1B=0, OICIE1A=0)
  // Enable Overflow Interrupt (bit: TOIE1=0)
  TIMSK1 &= ~((1<<OCIE1B) | (1<<OCIE1A) | (1<<TOIE1));
  TIMSK1 |= (1<<ICIE1);
  RCQuality = 0;
  SREG = sreg;
}

/*
 * This new and much faster interrupt handler should reduce servo jolts.
 */
ISR(TIMER1_CAPT_vect) {
  static uint16_t oldICR1 = 0;
  uint16_t signal = (uint16_t)ICR1 - oldICR1;
  oldICR1 = ICR1;
  //sync gap? (3.5 ms < signal < 25.6 ms)
  if (signal > TIME(3.5)) {
	inBfrPnt = 0;
  } else if (inBfrPnt<MAX_CONTROLCHANNELS) {
	RC_buffer[inBfrPnt++] = signal;
	if (RCQuality <= 200-4) RCQuality+=4; else RCQuality = 200;
  }
}

/********************************************************************/
/*         Every time a positive edge is detected at PD6            */
/********************************************************************/
/*                               t-Frame
    <----------------------------------------------------------------------->
     ____   ______   _____   ________                ______    sync gap      ____
    |    | |      | |     | |        |              |      |                |
    |    | |      | |     | |        |              |      |                |
 ___|    |_|      |_|     |_|        |_.............|      |________________|
    <-----><-------><------><-----------            <------>                <---
 t0       t1      t2       t4                     tn                     t0

 The PPM-Frame length is 22.5 ms.
 Channel high pulse width range is 0.7 ms to 1.7 ms completed by an 0.3 ms low pulse.
 The mininimum time delay of two events coding a channel is ( 0.7 + 0.3) ms = 1 ms.
 The maximum time delay of two events coding a channel is ( 1.7 + 0.3) ms = 2 ms.
 The minimum duration of all channels at minimum value is  8 * 1 ms = 8 ms.
 The maximum duration of all channels at maximum value is  8 * 2 ms = 16 ms.
 The remaining time of (22.5 - 8 ms) ms = 14.5 ms  to (22.5 - 16 ms) ms = 6.5 ms is
 the syncronization gap.
 */

void RC_process(void) {
  if (RCQuality) RCQuality--;
  for (uint8_t channel=0; channel<MAX_CONTROLCHANNELS; channel++) {
	uint16_t signal = RC_buffer[channel];
	if (signal != 0) {
  	  RC_buffer[channel] = 0; // reset to flag value already used.
      if ((signal >= TIME(0.8)) && (signal < TIME(2.2))) {
        signal -= TIME(1.5) /*  + channelMap.HWTrim */;
        PPM_diff[channel] = signal - PPM_in[channel];
        PPM_in[channel] = signal;
      }
    }
  }
}

#define RCChannel(dimension) PPM_in[channelMap.channels[dimension]]
#define RCDiff(dimension) PPM_diff[channelMap.channels[dimension]]
#define COMMAND_THRESHOLD TIME(0.35f)
#define COMMAND_CHANNEL_VERTICAL CH_THROTTLE
#define COMMAND_CHANNEL_HORIZONTAL CH_YAW

// Internal.
uint8_t RC_getStickCommand(void) {
  if (RCChannel(COMMAND_CHANNEL_VERTICAL) > COMMAND_THRESHOLD) {
    // vertical is up
    if (RCChannel(COMMAND_CHANNEL_HORIZONTAL) > COMMAND_THRESHOLD)
      return COMMAND_GYROCAL;
    if (RCChannel(COMMAND_CHANNEL_HORIZONTAL) < -COMMAND_THRESHOLD)
      return COMMAND_ACCCAL;
    return COMMAND_NONE;
  } else if (RCChannel(COMMAND_CHANNEL_VERTICAL) < -COMMAND_THRESHOLD) {
    // vertical is down
    if (RCChannel(COMMAND_CHANNEL_HORIZONTAL) > COMMAND_THRESHOLD)
      return COMMAND_STOP;
    if (RCChannel(COMMAND_CHANNEL_HORIZONTAL) < -COMMAND_THRESHOLD)
      return COMMAND_START;
    return COMMAND_NONE;
  }
  // vertical is around center
  return COMMAND_NONE;
}

/*
 * Get Pitch, Roll, Throttle, Yaw values
 */
void RC_periodicTaskAndRPTY(int16_t* RPTY) {
  int16_t tmp1, tmp2;
  RC_process();
  if (RCQuality) {
    RCQuality--;
    RPTY[CONTROL_ROLL]      = ((RCChannel(CH_ROLL) * staticParams.stickP) >> 3) + RCDiff(CH_ROLL) * staticParams.stickD;
    RPTY[CONTROL_PITCH]     = ((RCChannel(CH_PITCH) * staticParams.stickP) >> 3) + RCDiff(CH_PITCH) * staticParams.stickD;
    int16_t throttle        = RCChannel(CH_THROTTLE) + RCDiff(CH_THROTTLE) * staticParams.stickThrottleD + TIME(0.4);
    // Negative throttle values are taken as zero.
    if (throttle > 0)
      RPTY[CONTROL_THROTTLE]  = throttle;
    else
      RPTY[CONTROL_THROTTLE]  = 0;

    tmp1 = RCChannel(CH_YAW); // - RCDiff(CH_YAW);
    // exponential stick sensitivity in yawing rate
    // 
    tmp2 = ((int32_t)staticParams.stickYawP * (int32_t)tmp1 * abs(tmp1)) >> 14; // expo  y = ax + bx^2
    tmp2 += (staticParams.stickYawP * tmp1) >> 3;
    
    RPTY[CONTROL_YAW] = (/*(RCChannel(CH_YAW) * staticParams.stickYawP) >> 3*/ tmp2);

    uint8_t command = RC_getStickCommand();

    if (lastRCCommand == command) {
      // Keep timer from overrunning.
      if (commandTimer < COMMAND_TIMER)
        commandTimer++;
    } else {
      // There was a change.
      lastRCCommand = command;
      commandTimer = 0;
    }
  } // if RCQuality is no good, we just do nothing.
}

/*
 * Get other channel value
 */
int16_t RC_getVariable(uint8_t varNum) {
  if (varNum < 4)
    // 0th variable is 5th channel (1-based) etc.
    return (RCChannel(varNum + CH_VARIABLES) >> 3) + VARIABLES_OFFSET;
  /*
   * Let's just say:
   * The RC variable i is hardwired to channel i, i>=4
   */
  return (PPM_in[varNum] >> 3) + VARIABLES_OFFSET;
}

uint8_t RC_getSignalQuality(void) {
  if (RCQuality >= 160)
    return SIGNAL_GOOD;
  if (RCQuality >= 140)
    return SIGNAL_OK;
  if (RCQuality >= 120)
    return SIGNAL_BAD;
  return SIGNAL_LOST;
}

/*
 * To should fired only when the right stick is in the center position.
 * This will cause the value of pitch and roll stick to be adjusted
 * to zero (not just to near zero, as per the assumption in rc.c
 * about the rc signal. I had values about 50..70 with a Futaba
 * R617 receiver.) This calibration is not strictly necessary, but
 * for control logic that depends on the exact (non)center position
 * of a stick, it may be useful.
 */
void RC_calibrate(void) {
  // Do nothing.
}

/*
 if (staticParams.GlobalConfig & CFG_HEADING_HOLD) {
 // In HH, it s OK to trim the R/C. The effect should not be conteracted here.
 stickOffsetPitch = stickOffsetRoll = 0;
 } else {
 stickOffsetPitch = RCChannel(CH_PITCH) * staticParams.StickP;
 stickOffsetRoll = RCChannel(CH_ROLL)   * staticParams.StickP;
 }
 }
 */

uint8_t RC_getCommand(void) {
  if (commandTimer == COMMAND_TIMER) {
    // Stick has been held long enough; command committed.
    return lastRCCommand;
  }
  // Not yet sure what the command is.
  return COMMAND_NONE;
}

/*
 * Command arguments on R/C:
 * 2--3--4
 * |     |  +
 * 1  0  5  ^ 0
 * |     |  |  
 * 8--7--6
 *    
 * + <--
 *    0
 * 
 * Not in any of these positions: 0
 */

#define ARGUMENT_THRESHOLD TIME(0.35f)
#define ARGUMENT_CHANNEL_VERTICAL CH_PITCH
#define ARGUMENT_CHANNEL_HORIZONTAL CH_ROLL

uint8_t RC_getArgument(void) {
  if (RCChannel(ARGUMENT_CHANNEL_VERTICAL) > ARGUMENT_THRESHOLD) {
    // vertical is up
    if (RCChannel(ARGUMENT_CHANNEL_HORIZONTAL) > ARGUMENT_THRESHOLD)
      return 2;
    if (RCChannel(ARGUMENT_CHANNEL_HORIZONTAL) < -ARGUMENT_THRESHOLD)
      return 4;
    return 3;
  } else if (RCChannel(ARGUMENT_CHANNEL_VERTICAL) < -ARGUMENT_THRESHOLD) {
    // vertical is down
    if (RCChannel(ARGUMENT_CHANNEL_HORIZONTAL) > ARGUMENT_THRESHOLD)
      return 8;
    if (RCChannel(ARGUMENT_CHANNEL_HORIZONTAL) < -ARGUMENT_THRESHOLD)
      return 6;
    return 7;
  } else {
    // vertical is around center
    if (RCChannel(ARGUMENT_CHANNEL_HORIZONTAL) > ARGUMENT_THRESHOLD)
      return 1;
    if (RCChannel(ARGUMENT_CHANNEL_HORIZONTAL) < -ARGUMENT_THRESHOLD)
      return 5;
    return 0;
  }
}

#ifdef USE_MK3MAG
/*
 * For each time the stick is pulled, returns true.
 */
uint8_t RC_testCompassCalState(void) {
  static uint8_t stickPulled = 1;
  // if pitch is centered or top set stick to zero
  if (RCChannel(CH_PITCH) > -20)
    stickPulled = 0;
  // if pitch is down trigger to next cal state
  if ((RCChannel(CH_PITCH) < -70) && !stickPulled) {
    stickPulled = 1;
    return 1;
  }
  return 0;
}
#endif

/*
 * Abstract controls are not used at the moment.
 t_control rc_control = {
 RC_getPitch, 
 RC_getRoll, 
 RC_getYaw, 
 RC_getThrottle, 
 RC_getSignalQuality, 
 RC_calibrate
 };
 */
