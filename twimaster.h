#ifndef _I2C_MASTER_H
#define _I2C_MASTER_H

#include <inttypes.h>
#include "configuration.h"

#define TWI_STATE_MOTOR_TX 			0
#define TWI_STATE_MOTOR_RX 			3
#define TWI_STATE_GYRO_OFFSET_TX	7

extern volatile uint8_t twi_state;
extern uint8_t missingMotor;

typedef struct {
  uint8_t throttle;
  uint8_t current;
  uint8_t maxPWM;
	uint8_t present; // 0 if BL was found
	uint8_t error; // I2C error counter
}__attribute__((packed)) MLBLC_t;

extern MLBLC_t mkblcs[MAX_I2CCHANNELS];

extern volatile uint16_t I2CTimeout;

extern void I2C_init(void); // Initialize I2C
extern void I2C_start(uint8_t startState); // Start I2C
extern void I2C_stop(uint8_t startState);  // Stop I2C
extern void I2C_reset(void); // Reset I2C
extern void twi_diagnostics(void);

#endif
