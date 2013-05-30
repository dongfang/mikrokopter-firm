#ifndef _EEPROM_H
#define _EEPROM_H

#include <inttypes.h>
#include "configuration.h"

#define EEPROM_ADR_PARAM_BEGIN	0
#define EEPROM_CHECKSUMMED_BLOCK_OVERHEAD 3

#define PID_ACTIVE_SET            	0   // byte
//#define EEPROM_ADR_ACCOFFSET      	1
//#define EEPROM_ADR_GYROOFFSET     	(EEPROM_ADR_ACCOFFSET+sizeof(SensorOffset_t)+EEPROM_CHECKSUMMED_BLOCK_OVERHEAD)
//#define EEPROM_ADR_GYROAMPLIFIER  	(EEPROM_ADR_GYROOFFSET+sizeof(SensorOffset_t)+EEPROM_CHECKSUMMED_BLOCK_OVERHEAD)
//#define EEPROM_ADR_CHANNELMAP	  	    (EEPROM_ADR_GYROAMPLIFIER+sizeof(SensorOffset_t)+EEPROM_CHECKSUMMED_BLOCK_OVERHEAD)
//#define EEPROM_ADR_MIXER_TABLE	  	(EEPROM_ADR_CHANNELMAP+sizeof(ChannelMap_t)+EEPROM_CHECKSUMMED_BLOCK_OVERHEAD)
//#define EEPROM_ADR_PARAMSET_BEGIN 	(EEPROM_ADR_MIXER_TABLE+sizeof(MixerMatrix_t)+EEPROM_CHECKSUMMED_BLOCK_OVERHEAD)

#define EEPROM_ADR_ACCELOFFSET    	16
#define EEPROM_ADR_GYROOFFSET   	32
#define EEPROM_ADR_GYROAMPLIFIER    48
#define EEPROM_ADR_CHANNELMAP	  	64
#define EEPROM_ADR_IMU_CONFIG       100
#define EEPROM_ADR_MIXER_TABLE      128
#define EEPROM_ADR_PARAMSET_BEGIN   384

#define CHANNELMAP_REVISION         0
#define EEPARAM_REVISION	    	5
#define EEMIXER_REVISION	    	3
#define SENSOROFFSET_REVISION       0
#define IMUCONFIG_REVISION          0

void paramSet_readOrDefault(void);
void channelMap_readOrDefault(void);
void outputMixer_readOrDefault(void);
void IMUConfig_readOrDefault(void);
void IMUConfig_writeToEEprom(void);

uint8_t paramSet_readFromEEProm(uint8_t setnumber);
void paramSet_writeToEEProm(uint8_t setnumber);

//uint8_t channelMap_readFromEEProm(void);
void channelMap_writeToEEProm(void);

//uint8_t motorMixer_readFromEEProm(void);
void outputMixer_writeToEEProm(void);

uint8_t gyroAmplifierOffset_readFromEEProm(void);
void gyroAmplifierOffset_writeToEEProm(void);

uint8_t gyroOffset_readFromEEProm(void);
void gyroOffset_writeToEEProm(void);

uint8_t accelOffset_readFromEEProm(void);
void accelOffset_writeToEEProm(void);

uint8_t getParamByte(uint16_t param_id);
void setParamByte(uint16_t param_id, uint8_t value);
//uint16_t getParamWord(uint16_t param_id);
//void setParamWord(uint16_t param_id, uint16_t value);

uint8_t getActiveParamSet(void);
void setActiveParamSet(uint8_t setnumber);

#endif //_EEPROM_H
