#ifndef EEMEM
#define EEMEM __attribute__ ((section (".eeprom")))
#endif

#include "eeprom.h"
//#include "output.h"
#include "configuration.h"
#include "analog.h"
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <stdio.h>

// byte array in eeprom
uint8_t EEPromArray[E2END + 1] EEMEM;

/***************************************************/
/*       Read Parameter from EEPROM as byte        */
/***************************************************/
uint8_t getParamByte(uint16_t param_id) {
  return eeprom_read_byte(&EEPromArray[EEPROM_ADR_PARAM_BEGIN + param_id]);
}

/***************************************************/
/*       Write Parameter to EEPROM as byte         */
/***************************************************/
void setParamByte(uint16_t param_id, uint8_t value) {
  eeprom_write_byte(&EEPromArray[EEPROM_ADR_PARAM_BEGIN + param_id], value);
}

/***************************************************/
/*       Read Parameter from EEPROM as word        */
/***************************************************/
/*
uint16_t getParamWord(uint16_t param_id) {
  return eeprom_read_word((uint16_t *) &EEPromArray[EEPROM_ADR_PARAM_BEGIN
						    + param_id]);
}
*/

/***************************************************/
/*       Write Parameter to EEPROM as word         */
/***************************************************/
/*
void setParamWord(uint16_t param_id, uint16_t value) {
  eeprom_write_word((uint16_t *) &EEPromArray[EEPROM_ADR_PARAM_BEGIN + param_id], value);
}
*/

uint16_t CRC16(uint8_t* data, uint16_t length) {
  uint16_t crc = 0;
  for (uint16_t i=0; i<length; i++) {
    crc  = (uint8_t)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (uint8_t)(crc & 0xff) >> 4;
    crc ^= (crc << 8) << 4;
    crc ^= ((crc & 0xff) << 4) << 1;
  }
  return crc;
}

// offset is where the checksum is stored, offset+1 is the revision number, and offset+2... are the data.
// length is the length of the pure data not including checksum and revision number.
void writeChecksummedBlock(uint8_t revisionNumber, uint8_t* data, uint16_t offset, uint16_t length) {
  uint16_t CRC = CRC16(data, length);
  uint8_t sreg = SREG;
  cli();
  eeprom_write_byte(&EEPromArray[offset], CRC&0xff);
  eeprom_write_byte(&EEPromArray[offset+1], CRC>>8);
  eeprom_write_byte(&EEPromArray[offset+2], revisionNumber);
  eeprom_write_block(data, &EEPromArray[offset+3], length);
  SREG = sreg;
}

// offset is where the checksum is stored, offset+1 is the revision number, and offset+2... are the data.
// length is the length of the pure data not including checksum and revision number.
uint8_t readChecksummedBlock(uint8_t revisionNumber, uint8_t* target, uint16_t offset, uint16_t length) {
  uint16_t CRCRead = eeprom_read_byte(&EEPromArray[offset]) | (eeprom_read_byte(&EEPromArray[offset+1])<<8);
  uint8_t revisionNumberRead = eeprom_read_byte(&EEPromArray[offset+2]);
  eeprom_read_block(target, &EEPromArray[offset+3], length);
  uint16_t CRCCalculated = CRC16(target, length);
  
  uint8_t CRCError = (CRCRead != CRCCalculated);
  uint8_t revisionMismatch = (revisionNumber != revisionNumberRead);
  
  if (CRCError && revisionMismatch) printf("\n\rEEPROM CRC error and revision mismatch; ");
  else if (CRCError) printf("\n\rEEPROM CRC error; ");
  else if (revisionMismatch) printf("\n\rEEPROM revision mismatch; ");
  return (CRCError || revisionMismatch);
}

/***************************************************/
/*       Read Parameter Set from EEPROM            */
/***************************************************/
// setnumber [1..5]
uint8_t paramSet_readFromEEProm(uint8_t setnumber) {
  uint16_t offset = EEPROM_ADR_PARAMSET_BEGIN + (setnumber-1)*(sizeof(ParamSet_t)+EEPROM_CHECKSUMMED_BLOCK_OVERHEAD);
  uint8_t result = readChecksummedBlock(EEPARAM_REVISION, (uint8_t*)&staticParams, offset, sizeof(ParamSet_t));
  configuration_paramSetDidChange();
  return result;
}

/***************************************************/
/*        Write Parameter Set to EEPROM            */
/***************************************************/
void paramSet_writeToEEProm(uint8_t setnumber) {
  uint16_t offset = EEPROM_ADR_PARAMSET_BEGIN + (setnumber-1)*(sizeof(ParamSet_t)+EEPROM_CHECKSUMMED_BLOCK_OVERHEAD);
  writeChecksummedBlock(EEPARAM_REVISION, (uint8_t*)&staticParams, offset, sizeof(ParamSet_t));
}

void paramSet_readOrDefault() {
  uint8_t setnumber = getActiveParamSet();
  // parameter version  check
  if (setnumber<1 ||setnumber>5 || paramSet_readFromEEProm(setnumber)) {
    // if version check faild
    printf("\n\rwriting default parameter sets");
    for (uint8_t i=5; i>0; i--) {
      paramSet_default(i);
      paramSet_writeToEEProm(i);
    }
    // default-Setting is parameter set 1
    setActiveParamSet(1);
    paramSet_readFromEEProm(getActiveParamSet());
    // For some strange reason, the read will have no effect.
    // Lets reset...
    // wdt_enable(WDTO_500MS);
  }
  printf("\n\r\rUsing Parameter Set %d", getActiveParamSet());
}

/***************************************************/
/*          Read IMU Config from EEPROM            */
/***************************************************/
uint8_t IMUConfig_readFromEEprom(void) {
  return readChecksummedBlock(IMUCONFIG_REVISION, (uint8_t*)&IMUConfig, EEPROM_ADR_IMU_CONFIG, sizeof(IMUConfig_t));
}

/***************************************************/
/*          Write IMU Config to EEPROM             */
/***************************************************/
void IMUConfig_writeToEEprom(void) {
  writeChecksummedBlock(IMUCONFIG_REVISION, (uint8_t*)&IMUConfig, EEPROM_ADR_IMU_CONFIG, sizeof(IMUConfig_t));
}

void IMUConfig_readOrDefault(void) {
    if(IMUConfig_readFromEEprom()) {
      printf("\n\rwriting default IMU config");
      IMUConfig_default();
      IMUConfig_writeToEEprom();
    }
}

/***************************************************/
/* MixerTable                                      */
/***************************************************/
void outputMixer_writeToEEProm(void) {
  writeChecksummedBlock(EEMIXER_REVISION, (uint8_t*)&outputMixer, EEPROM_ADR_MIXER_TABLE, sizeof(OutputMixer_t));
}

void outputMixer_readOrDefault(void) {
  // load mixer table
  if (readChecksummedBlock(EEMIXER_REVISION, (uint8_t*)&outputMixer, EEPROM_ADR_MIXER_TABLE, sizeof(OutputMixer_t))) {
    printf("\n\rwriting default motor mixer");
    outputMixer_default(); // Quadro
    outputMixer_writeToEEProm();
  }
}

/***************************************************/
/* ChannelMap                                      */
/***************************************************/
void channelMap_writeToEEProm(void) {
  writeChecksummedBlock(CHANNELMAP_REVISION, (uint8_t*)&channelMap, EEPROM_ADR_CHANNELMAP, sizeof(ChannelMap_t));
}

void channelMap_readOrDefault(void) {
  if (readChecksummedBlock(CHANNELMAP_REVISION, (uint8_t*)&channelMap, EEPROM_ADR_CHANNELMAP, sizeof(ChannelMap_t))) {
    printf("\n\rwriting default channel map");
    channelMap_default();
    channelMap_writeToEEProm();
	wdt_enable(WDTO_500MS);
  }
}

/***************************************************/
/* Sensor offsets                                  */
/***************************************************/
uint8_t gyroAmplifierOffset_readFromEEProm(void) {
  return readChecksummedBlock(SENSOROFFSET_REVISION, (uint8_t*)&gyroAmplifierOffset, EEPROM_ADR_GYROAMPLIFIER, sizeof(sensorOffset_t));
}

void gyroAmplifierOffset_writeToEEProm(void) {
  return writeChecksummedBlock(SENSOROFFSET_REVISION, (uint8_t*)&gyroAmplifierOffset, EEPROM_ADR_GYROAMPLIFIER, sizeof(sensorOffset_t));
}

uint8_t gyroOffset_readFromEEProm(void) {
  return readChecksummedBlock(SENSOROFFSET_REVISION, (uint8_t*)&gyroOffset, EEPROM_ADR_GYROOFFSET, sizeof(sensorOffset_t));
}

void gyroOffset_writeToEEProm(void) {
  writeChecksummedBlock(SENSOROFFSET_REVISION, (uint8_t*)&gyroOffset, EEPROM_ADR_GYROOFFSET, sizeof(sensorOffset_t));
}

uint8_t accelOffset_readFromEEProm(void) {
  return readChecksummedBlock(SENSOROFFSET_REVISION, (uint8_t*)&accelOffset, EEPROM_ADR_ACCELOFFSET, sizeof(sensorOffset_t));
}

void accelOffset_writeToEEProm(void) {
  writeChecksummedBlock(SENSOROFFSET_REVISION, (uint8_t*)&accelOffset, EEPROM_ADR_ACCELOFFSET, sizeof(sensorOffset_t));
}

/***************************************************/
/*       Get active parameter set                  */
/***************************************************/
uint8_t getActiveParamSet(void) {
  uint8_t setnumber;
  setnumber = eeprom_read_byte(&EEPromArray[PID_ACTIVE_SET]);
  if (setnumber > 5) {
    setActiveParamSet(setnumber = 1);
  }
  return setnumber;
}

/***************************************************/
/*       Set active parameter set                  */
/***************************************************/
void setActiveParamSet(uint8_t setnumber) {
  eeprom_write_byte(&EEPromArray[PID_ACTIVE_SET], setnumber);
}

