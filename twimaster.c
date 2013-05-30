#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <util/delay.h>
#include <stdio.h>
#include "twimaster.h"
//#include "configuration.h"
//#include "controlMixer.h"
#include "analog.h"

volatile uint8_t twi_state;
volatile uint8_t dac_channel;
volatile uint8_t writeIndex;
volatile uint8_t readIndex;
volatile uint16_t I2CTimeout = 100;
uint8_t missingMotor;
MLBLC_t mkblcs[MAX_I2CCHANNELS];
uint8_t DACChannel;

#define SCL_CLOCK  200000L
#define I2C_TIMEOUT 30000

/**************************************************
 * Initialize I2C (TWI)                         
 **************************************************/
void I2C_init(void) {
	uint8_t i;
	uint8_t sreg = SREG;
	cli();

	// SDA is INPUT
	DDRC &= ~(1 << DDC1);
	// SCL is output
	DDRC |= (1 << DDC0);
	// pull up SDA
	PORTC |= (1 << PORTC0) | (1 << PORTC1);

	// TWI Status Register
	// prescaler 1 (TWPS1 = 0, TWPS0 = 0)
	TWSR &= ~((1 << TWPS1) | (1 << TWPS0));

	// set TWI Bit Rate Register
	TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2;

	twi_state = TWI_STATE_MOTOR_TX;
	writeIndex = 0;
	readIndex = 0;

	for (i = 0; i < MAX_I2CCHANNELS; i++) {
	  mkblcs[i].present = 0;
	  mkblcs[i].maxPWM = 0;
	}

	SREG = sreg;
}

/****************************************
 * Start I2C                          
 ****************************************/
void I2C_start(uint8_t start_state) {
	twi_state = start_state;
	// TWI Control Register
	// clear TWI interrupt flag (TWINT=1)
	// disable TWI Acknowledge Bit (TWEA = 0)
	// enable TWI START Condition Bit (TWSTA = 1), MASTER
	// disable TWI STOP Condition Bit (TWSTO = 0)
	// disable TWI Write Collision Flag (TWWC = 0)
	// enable i2c (TWEN = 1)
	// enable TWI Interrupt (TWIE = 1)
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
}

/****************************************
 * Stop I2C                          
 ****************************************/
void I2C_stop(uint8_t start_state) {
	twi_state = start_state;
	// TWI Control Register
	// clear TWI interrupt flag (TWINT=1)
	// disable TWI Acknowledge Bit (TWEA = 0)
	// diable TWI START Condition Bit (TWSTA = 1), no MASTER
	// enable TWI STOP Condition Bit (TWSTO = 1)
	// disable TWI Write Collision Flag (TWWC = 0)
	// enable i2c (TWEN = 1)
	// disable TWI Interrupt (TWIE = 0)
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

/****************************************
 *    Write to I2C                      
 ****************************************/
void I2C_writeByte(int8_t byte) {
	// move byte to send into TWI Data Register
	TWDR = byte;
	// clear interrupt flag (TWINT = 1)
	// enable i2c bus (TWEN = 1)
	// enable interrupt (TWIE = 1)
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
}

/****************************************
 * Receive byte and send ACK         
 ****************************************/
void I2C_receiveByte(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
}

/****************************************
 * I2C receive last byte and send no ACK 
 ****************************************/
void I2C_receiveLastByte(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
}

/****************************************
 * Reset I2C                         
 ****************************************/
void I2C_reset(void) {
	// stop i2c bus
	I2C_stop(TWI_STATE_MOTOR_TX);
	twi_state = 0;
	writeIndex = TWDR;
	writeIndex = 0;
	readIndex = 0;
	TWCR = (1 << TWINT); // reset to original state incl. interrupt flag reset
	TWAMR = 0;
	TWAR = 0;
	TWDR = 0;
	TWSR = 0;
	TWBR = 0;
	I2C_init();
	I2C_start(TWI_STATE_MOTOR_TX);
}

/****************************************
 * I2C ISR
 ****************************************/
ISR (TWI_vect)
{
	static uint8_t missing_motor = 0;
	switch (twi_state++) { // First i2c_start from SendMotorData()
	// Master Transmit
	case 0: // TWI_STATE_MOTOR_TX
		// skip motor if not used in mixer
		while ((outputMixer[writeIndex].outputType != OUTPUT_TYPE_MOTOR) && (writeIndex < MAX_I2CCHANNELS))
			writeIndex++;
		if (writeIndex >= MAX_I2CCHANNELS) { // writing finished, read now
			writeIndex = 0;
			twi_state = TWI_STATE_MOTOR_RX;
			I2C_writeByte(0x53 + (readIndex * 2)); // select slave adress in rx mode
		} else
			I2C_writeByte(0x52 + (writeIndex * 2)); // select slave adress in tx mode
		break;
	case 1: // Send Data to Slave
		//I2C_writeByte(outputs[writeIndex]>>LOG_CONTROL_OUTPUT_SCALING); // transmit throttle value.
	    I2C_writeByte(mkblcs[writeIndex].throttle); // transmit throttle value.
		break;
	case 2: // repeat case 0+1 for all motors
		if (TWSR == TW_MT_DATA_NACK) { // Data transmitted, NACK received
			if (!missing_motor)
				missing_motor = writeIndex + 1;
			if (++mkblcs[writeIndex].error == 0)
			  mkblcs[writeIndex].error = 255; // increment error counter and handle overflow
		}
		I2C_stop(TWI_STATE_MOTOR_TX);
		I2CTimeout = 10;
		writeIndex++; // next motor
		I2C_start(TWI_STATE_MOTOR_TX); // Repeated start -> switch slave or switch Master Transmit -> Master Receive
		break;
		// Master Receive Data
	case 3:
		if (TWSR != TW_MR_SLA_ACK) { //  SLA+R transmitted, if not ACK received
			// no response from the addressed slave received
		  mkblcs[readIndex].present = 0;
			readIndex++; // next motor
			if (readIndex >= MAX_I2CCHANNELS)
				readIndex = 0; // restart reading of first motor if we have reached the last one
			I2C_stop(TWI_STATE_MOTOR_TX);
		} else {
		  mkblcs[readIndex].present = ('1' - '-') + readIndex;
			I2C_receiveByte(); //Transmit 1st byte
		}
		missingMotor = missing_motor;
		missing_motor = 0;
		break;
	case 4: //Read 1st byte and transmit 2nd Byte
	  mkblcs[readIndex].current = TWDR;
		I2C_receiveLastByte(); // nack
		break;
	case 5:
		//Read 2nd byte
	  mkblcs[readIndex].maxPWM = TWDR;
		readIndex++; // next motor
		if (readIndex >= MAX_I2CCHANNELS)
			readIndex = 0; // restart reading of first motor if we have reached the last one
		I2C_stop(TWI_STATE_MOTOR_TX);
		break;

		// Writing ADC values.
	case 7:
		I2C_writeByte(0x98); // Address the DAC
		break;

	case 8:
		I2C_writeByte(0x10 + (DACChannel << 1)); // Select DAC Channel (0x10 = A, 0x12 = B, 0x14 = C)
		break;

	case 9:
		I2C_writeByte(gyroAmplifierOffset.offsets[DACChannel]);
		break;

	case 10:
		I2C_writeByte(0x80); // 2nd byte for all channels is 0x80
		break;

	case 11:
		I2C_stop(TWI_STATE_MOTOR_TX);
		I2CTimeout = 10;
		// repeat case 7...10 until all DAC Channels are updated
		if (DACChannel < 2) {
			DACChannel++; // jump to next channel
			I2C_start(TWI_STATE_GYRO_OFFSET_TX); // start transmission for next channel
		} else {
			DACChannel = 0; // reset dac channel counter
		}
		break;

	default:
		I2C_stop(TWI_STATE_MOTOR_TX);
		I2CTimeout = 10;
		writeIndex = 0;
		readIndex = 0;
	}
}

extern void twi_diagnostics(void) {
	// Check connected BL-Ctrls
	uint8_t i;

	printf("\n\rFound BL-Ctrl: ");

	for (i=0; i<MAX_I2CCHANNELS; i++) {
		mkblcs[i].throttle = 0;
	}

	I2C_start(TWI_STATE_MOTOR_TX);
	_delay_ms(2);

	readIndex = 0; // read the first I2C-Data

	for (i = 0; i < MAX_I2CCHANNELS; i++) {
		I2C_start(TWI_STATE_MOTOR_TX);
		_delay_ms(2);
		if (mkblcs[i].present)
			printf("%d ",i+1);
	}

	for (i = 0; i < MAX_I2CCHANNELS; i++) {
		if (!mkblcs[i].present && outputMixer[i].outputType == OUTPUT_TYPE_MOTOR)
			printf("\n\r\n\r!! MISSING BL-CTRL: %d !!",i + 1);
		mkblcs[i].error = 0;
	}
}
