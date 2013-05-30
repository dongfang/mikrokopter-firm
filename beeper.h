#ifndef _BEEPER_H
#define _BEEPER_H

#define BEEP_MODULATION_NONE 0xFFFF
#define BEEP_MODULATION_RCALARM 0x0C00
#define BEEP_MODULATION_I2CALARM 0x0080
#define BEEP_MODULATION_BATTERYALARM 0x0300
#define BEEP_MODULATION_EEPROMALARM 0x0007

extern volatile uint16_t beepModulation;
extern volatile uint16_t beepTime;

void beep(uint16_t millis);
void beepNumber(uint8_t numbeeps);
void beepRCAlarm(void);
void beepI2CAlarm(void);
void beepBatteryAlarm(void);

#endif