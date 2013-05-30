#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <stdarg.h>
#include <string.h>

#include "eeprom.h"
#include "timer0.h"
#include "uart0.h"
#include "rc.h"
#include "externalControl.h"
#include "debug.h"
#include "profiler.h"
#include "beeper.h"

#ifdef USE_DIRECT_GPS
#include "mk3mag.h"
#endif

#define FC_ADDRESS 1
#define NC_ADDRESS 2
#define MK3MAG_ADDRESS 3

#define FALSE	0
#define TRUE	1

DebugOut_t debugOut;

uint8_t requestedAnalogLabel = 255;
uint8_t requestedProfilerLabel = 255;

uint8_t request_verInfo;
uint8_t request_externalControl;
uint8_t request_display;
uint8_t request_display1;
uint8_t request_debugData;
uint8_t request_profilerData;
uint8_t request_PPMChannels;
uint8_t request_outputTest;
uint8_t request_variables;
uint8_t request_OSD;
uint8_t request_DCM_matrix;

/*
#define request_verInfo         (1<<0)
#define request_externalControl (1<<1)
#define request_display         (1<<3)
#define request_display1        (1<<4)
#define request_debugData       (1<<5)
#define request_data3D          (1<<6)
#define request_PPMChannels     (1<<7)
#define request_motorTest       (1<<8)
#define request_variables       (1<<9)
#define request_OSD             (1<<10)
*/

//uint16_t request = 0;

volatile uint8_t txd_buffer[TXD_BUFFER_LEN];
volatile uint8_t rxd_buffer_locked = FALSE;
volatile uint8_t rxd_buffer[RXD_BUFFER_LEN];
volatile uint8_t txd_complete;
volatile uint8_t receivedBytes;
volatile uint8_t *pRxData;
volatile uint8_t rxDataLen;

uint8_t outputTestActive;
uint8_t outputTest[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t confirmFrame;

typedef struct {
	int16_t heading;
}__attribute__((packed)) Heading_t;

IMUData imuData;

uint16_t debugDataTimer;
uint16_t profilerDataTimer;
uint16_t OSDDataTimer;
uint16_t debugDataInterval; // in 1ms
uint16_t profilerDataInterval; // in 1ms
uint16_t imuDataInterval; // in 1ms
uint16_t OSDDataInterval;

#ifdef USE_DIRECT_GPS
int16_t toMk3MagTimer;
#endif

// keep lables in flash to save 512 bytes of sram space
	   //1234567890123456
const char analogLabel0[]  PROGMEM = "AngleRoll";
const char analogLabel1[]  PROGMEM = "AnglePitch";
const char analogLabel2[]  PROGMEM = "AngleYaw";
const char analogLabel3[]  PROGMEM = "GyroX(Roll)";
const char analogLabel4[]  PROGMEM = "GyroY(Pitch)";
const char analogLabel5[]  PROGMEM = "GyroZ(Yaw)";
const char analogLabel6[]  PROGMEM = "AccX(0.01m/s^2)";
const char analogLabel7[]  PROGMEM = "AccY(0.01m/s^2)";
const char analogLabel8[]  PROGMEM = "AccZ(0.01m/s^2)";
const char analogLabel9[]  PROGMEM = "RC pitch";
const char analogLabel10[] PROGMEM = "RC yaw";
const char analogLabel11[] PROGMEM = "RC throttle";
const char analogLabel12[] PROGMEM = "Roll";
const char analogLabel13[] PROGMEM = "Pitch";
const char analogLabel14[] PROGMEM = "rollControl";
const char analogLabel15[] PROGMEM = "pitchControl";
const char analogLabel16[] PROGMEM = "M1";
const char analogLabel17[] PROGMEM = "M2";
const char analogLabel18[] PROGMEM = "M3";
const char analogLabel19[] PROGMEM = "M4";
const char analogLabel20[] PROGMEM = "flightmode";
const char analogLabel21[] PROGMEM = "Att freq";
const char analogLabel22[] PROGMEM = "Height[dm]";
const char analogLabel23[] PROGMEM = "dHeight";
const char analogLabel24[] PROGMEM = "attitudeSumCount";
const char analogLabel25[] PROGMEM = "simpleAirPressure";
const char analogLabel26[] PROGMEM = "OCR0A";
const char analogLabel27[] PROGMEM = "filteredAirPressure";
const char analogLabel28[] PROGMEM = "height";
const char analogLabel29[] PROGMEM = "Gyro Act Cont.";
const char analogLabel30[] PROGMEM = "GPS altitude";
const char analogLabel31[] PROGMEM = "GPS vert accura";

PGM_P ANALOG_LABELS[] PROGMEM = {
    analogLabel0,
    analogLabel1,
    analogLabel2,
    analogLabel3,
    analogLabel4,
    analogLabel5,
    analogLabel6,
    analogLabel7,
    analogLabel8,
    analogLabel9,
    analogLabel10,
    analogLabel11,
    analogLabel12,
    analogLabel13,
    analogLabel14,
    analogLabel15,
    analogLabel16,
    analogLabel17,
    analogLabel18,
    analogLabel19,
    analogLabel20,
    analogLabel21,
    analogLabel22,
    analogLabel23,
    analogLabel24,
    analogLabel25,
    analogLabel26,
    analogLabel27,
    analogLabel28,
    analogLabel29,
    analogLabel30,
    analogLabel31
};

/****************************************************************/
/*              Initialization of the USART0                    */
/****************************************************************/
void usart0_init(void) {
	uint8_t sreg = SREG;
	uint16_t ubrr = (F_CPU / (8 * USART0_BAUD) - 1);

	// disable all interrupts before configuration
	cli();

	// disable RX-Interrupt
	UCSR0B &= ~(1 << RXCIE0);
	// disable TX-Interrupt
	UCSR0B &= ~(1 << TXCIE0);

	// set direction of RXD0 and TXD0 pins
	// set RXD0 (PD0) as an input pin
	PORTD |= (1 << PORTD0);
	DDRD &= ~(1 << DDD0);
	// set TXD0 (PD1) as an output pin
	PORTD |= (1 << PORTD1);
	DDRD |= (1 << DDD1);

	// USART0 Baud Rate Register
	// set clock divider
	UBRR0H = (uint8_t) (ubrr >> 8);
	UBRR0L = (uint8_t) ubrr;

	// USART0 Control and Status Register A, B, C

	// enable double speed operation in
	UCSR0A |= (1 << U2X0);
	// enable receiver and transmitter in
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);
	// set asynchronous mode
	UCSR0C &= ~(1 << UMSEL01);
	UCSR0C &= ~(1 << UMSEL00);
	// no parity
	UCSR0C &= ~(1 << UPM01);
	UCSR0C &= ~(1 << UPM00);
	// 1 stop bit
	UCSR0C &= ~(1 << USBS0);
	// 8-bit
	UCSR0B &= ~(1 << UCSZ02);
	UCSR0C |= (1 << UCSZ01);
	UCSR0C |= (1 << UCSZ00);

	// flush receive buffer
	while (UCSR0A & (1 << RXC0))
		UDR0;

	// enable interrupts at the end
	// enable RX-Interrupt
	UCSR0B |= (1 << RXCIE0);
	// enable TX-Interrupt
	UCSR0B |= (1 << TXCIE0);

	// initialize the debug timer
	debugDataTimer = setDelay(debugDataInterval);
	profilerDataTimer = setDelay(profilerDataInterval);

	// unlock rxd_buffer
	rxd_buffer_locked = FALSE;
	pRxData = 0;
	rxDataLen = 0;

	// no bytes to send
	txd_complete = TRUE;

#ifdef USE_DIRECT_GPS
	toMk3MagTimer = setDelay(220);
#endif

	versionInfo.SWMajor = VERSION_MAJOR;
	versionInfo.SWMinor = VERSION_MINOR;
	versionInfo.SWPatch = VERSION_PATCH;
	versionInfo.protoMajor = VERSION_SERIAL_MAJOR;
	versionInfo.protoMinor = VERSION_SERIAL_MINOR;

	// restore global interrupt flags
	SREG = sreg;
}

/****************************************************************/
/* USART0 transmitter ISR                                       */
/****************************************************************/
ISR(USART0_TX_vect) {
	static uint16_t ptr_txd_buffer = 0;
	uint8_t tmp_tx;
	if (!txd_complete) { // transmission not completed
		ptr_txd_buffer++; // die [0] wurde schon gesendet
		tmp_tx = txd_buffer[ptr_txd_buffer];
		// if terminating character or end of txd buffer was reached
		if ((tmp_tx == '\r') || (ptr_txd_buffer == TXD_BUFFER_LEN)) {
			ptr_txd_buffer = 0; // reset txd pointer
			txd_complete = 1; // stop transmission
		}
		UDR0 = tmp_tx; // send current byte will trigger this ISR again
	}
	// transmission completed
	else
		ptr_txd_buffer = 0;
}

/****************************************************************/
/* USART0 receiver               ISR                            */
/****************************************************************/
ISR(USART0_RX_vect) {
	static uint16_t checksum;
	static uint8_t ptr_rxd_buffer = 0;
	uint8_t checksum1, checksum2;
	uint8_t c;

	c = UDR0; // catch the received byte

	if (rxd_buffer_locked)
		return; // if rxd buffer is locked immediately return

	// the rxd buffer is unlocked
	if ((ptr_rxd_buffer == 0) && (c == '#')) { // if rxd buffer is empty and synchronization character is received
		rxd_buffer[ptr_rxd_buffer++] = c; // copy 1st byte to buffer
		checksum = c; // init checksum
	}
	else if (ptr_rxd_buffer < RXD_BUFFER_LEN) { // collect incoming bytes
		if (c != '\r') { // no termination character
			rxd_buffer[ptr_rxd_buffer++] = c; // copy byte to rxd buffer
			checksum += c; // update checksum
		} else { // termination character was received
			// the last 2 bytes are no subject for checksum calculation
			// they are the checksum itself
			checksum -= rxd_buffer[ptr_rxd_buffer - 2];
			checksum -= rxd_buffer[ptr_rxd_buffer - 1];
			// calculate checksum from transmitted data
			checksum %= 4096;
			checksum1 = '=' + checksum / 64;
			checksum2 = '=' + checksum % 64;
			// compare checksum to transmitted checksum bytes
			if ((checksum1 == rxd_buffer[ptr_rxd_buffer - 2]) && (checksum2
					== rxd_buffer[ptr_rxd_buffer - 1])) {
				// checksum valid
				rxd_buffer[ptr_rxd_buffer] = '\r'; // set termination character
				receivedBytes = ptr_rxd_buffer + 1;// store number of received bytes
				rxd_buffer_locked = TRUE; // lock the rxd buffer
				// if 2nd byte is an 'R' enable watchdog that will result in an reset
				if (rxd_buffer[2] == 'R') {
					wdt_enable(WDTO_250MS);
				} // Reset-Commando
			} else { // checksum invalid
				rxd_buffer_locked = FALSE; // unlock rxd buffer
			}
			ptr_rxd_buffer = 0; // reset rxd buffer pointer
		}
	} else { // rxd buffer overrun
		ptr_rxd_buffer = 0; // reset rxd buffer
		rxd_buffer_locked = FALSE; // unlock rxd buffer
	}
}

// --------------------------------------------------------------------------
void addChecksum(uint16_t datalen) {
	uint16_t tmpchecksum = 0, i;
	for (i = 0; i < datalen; i++) {
		tmpchecksum += txd_buffer[i];
	}
	tmpchecksum %= 4096;
	txd_buffer[i++] = '=' + (tmpchecksum >> 6);
	txd_buffer[i++] = '=' + (tmpchecksum & 0x3F);
	txd_buffer[i++] = '\r';
	txd_complete = FALSE;
	UDR0 = txd_buffer[0]; // initiates the transmittion (continued in the TXD ISR)
}

// --------------------------------------------------------------------------
// application example:
// sendData('A', FC_ADDRESS, 2, (uint8_t *)&request_DebugLabel, sizeof(request_DebugLabel), label, 16);
/*
 void sendData(uint8_t cmd, uint8_t addr, uint8_t numofbuffers, ...) { // uint8_t *pdata, uint8_t len, ...
 va_list ap;
 uint16_t txd_bufferIndex = 0;
 uint8_t *currentBuffer;
 uint8_t currentBufferIndex;
 uint16_t lengthOfCurrentBuffer;
 uint8_t shift = 0;

 txd_buffer[txd_bufferIndex++] = '#';			// Start character
 txd_buffer[txd_bufferIndex++] = 'a' + addr;	        // Address (a=0; b=1,...)
 txd_buffer[txd_bufferIndex++] = cmd;			// Command

 va_start(ap, numofbuffers);

 while(numofbuffers) {
 currentBuffer = va_arg(ap, uint8_t*);
 lengthOfCurrentBuffer = va_arg(ap, int);
 currentBufferIndex = 0;
 // Encode data: 3 bytes of data are encoded into 4 bytes,
 // where the 2 most significant bits are both 0.
 while(currentBufferIndex != lengthOfCurrentBuffer) {
 if (!shift) txd_buffer[txd_bufferIndex] = 0;
 txd_buffer[txd_bufferIndex]  |= currentBuffer[currentBufferIndex] >> (shift + 2);
 txd_buffer[++txd_bufferIndex] = (currentBuffer[currentBufferIndex] << (4 - shift)) & 0b00111111;
 shift += 2;
 if (shift == 6) { shift=0; txd_bufferIndex++; }
 currentBufferIndex++;
 }
 }
 // If the number of data bytes was not divisible by 3, stuff
 //  with 0 pseudodata  until length is again divisible by 3.
 if (shift == 2) {
 // We need to stuff with zero bytes at the end.
 txd_buffer[txd_bufferIndex]  &= 0b00110000;
 txd_buffer[++txd_bufferIndex] = 0;
 shift = 4;
 }
 if (shift == 4) {
 // We need to stuff with zero bytes at the end.
 txd_buffer[txd_bufferIndex++] &= 0b00111100;
 txd_buffer[txd_bufferIndex]    = 0;
 }
 va_end(ap);
 Addchecksum(pt); // add checksum after data block and initates the transmission
 }
 */

void sendData(uint8_t cmd, uint8_t addr, uint8_t numofbuffers, ...) { // uint8_t *pdata, uint8_t len, ...
	va_list ap;
	uint16_t pt = 0;
	uint8_t a, b, c;
	uint8_t ptr = 0;

	uint8_t *pdata = 0;
	int len = 0;

	txd_buffer[pt++] = '#'; // Start character
	txd_buffer[pt++] = 'a' + addr; // Address (a=0; b=1,...)
	txd_buffer[pt++] = cmd; // Command

	va_start(ap, numofbuffers);

	if (numofbuffers) {
		pdata = va_arg(ap, uint8_t*);
		len = va_arg(ap, int);
		ptr = 0;
		numofbuffers--;
	}

	while (len) {
		if (len) {
			a = pdata[ptr++];
			len--;
			if ((!len) && numofbuffers) {
				pdata = va_arg(ap, uint8_t*);
				len = va_arg(ap, int);
				ptr = 0;
				numofbuffers--;
			}
		} else
			a = 0;
		if (len) {
			b = pdata[ptr++];
			len--;
			if ((!len) && numofbuffers) {
				pdata = va_arg(ap, uint8_t*);
				len = va_arg(ap, int);
				ptr = 0;
				numofbuffers--;
			}
		} else
			b = 0;
		if (len) {
			c = pdata[ptr++];
			len--;
			if ((!len) && numofbuffers) {
				pdata = va_arg(ap, uint8_t*);
				len = va_arg(ap, int);
				ptr = 0;
				numofbuffers--;
			}
		} else
			c = 0;
		txd_buffer[pt++] = '=' + (a >> 2);
		txd_buffer[pt++] = '=' + (((a & 0x03) << 4) | ((b & 0xf0) >> 4));
		txd_buffer[pt++] = '=' + (((b & 0x0f) << 2) | ((c & 0xc0) >> 6));
		txd_buffer[pt++] = '=' + (c & 0x3f);
	}
	va_end(ap);
	addChecksum(pt); // add checksum after data block and initates the transmission
}

// --------------------------------------------------------------------------
void decode64(void) {
	uint8_t a, b, c, d;
	uint8_t x, y, z;
	uint8_t ptrIn = 3;
	uint8_t ptrOut = 3;
	uint8_t len = receivedBytes - 6;

	while (len) {
		a = rxd_buffer[ptrIn++] - '=';
		b = rxd_buffer[ptrIn++] - '=';
		c = rxd_buffer[ptrIn++] - '=';
		d = rxd_buffer[ptrIn++] - '=';
		//if(ptrIn > ReceivedBytes - 3) break;

		x = (a << 2) | (b >> 4);
		y = ((b & 0x0f) << 4) | (c >> 2);
		z = ((c & 0x03) << 6) | d;

		if (len--)
			rxd_buffer[ptrOut++] = x;
		else
			break;
		if (len--)
			rxd_buffer[ptrOut++] = y;
		else
			break;
		if (len--)
			rxd_buffer[ptrOut++] = z;
		else
			break;
	}
	pRxData = &rxd_buffer[3];
	rxDataLen = ptrOut - 3;
}

// --------------------------------------------------------------------------
void usart0_processRxData(void) {
	// We control the outputTestActive var from here: Count it down.
	if (outputTestActive)
	  outputTestActive--;
	// if data in the rxd buffer are not locked immediately return
	if (!rxd_buffer_locked)
		return;

	uint8_t tempchar[3];
	decode64(); // decode data block in rxd_buffer

	switch (rxd_buffer[1] - 'a') {

	case FC_ADDRESS:
		switch (rxd_buffer[2]) {
#ifdef USE_DIRECT_GPS
		case 'K':// compass value
		  // What is the point of this - the compass will overwrite this soon?
		magneticHeading = ((Heading_t *)pRxData)->heading;
		// compassOffCourse = ((540 + compassHeading - compassCourse) % 360) - 180;
		break;
#endif
		case 't': // motor test
			memcpy(&outputTest[0], (uint8_t*) pRxData, /*sizeof(outputTest)*/ 12); // 12 is an mktool limitation.
			outputTestActive = 255;
			// Huh??
			externalControlActive = 255;
			break;

		case 'n':// Read motor mixer
            tempchar[0] = EEMIXER_REVISION;
            tempchar[1] = sizeof(OutputMixer_t);
            while (!txd_complete)
                ; // wait for previous frame to be sent
			sendData('N', FC_ADDRESS, 2, &tempchar, 2, (uint8_t*)&outputMixer, sizeof(OutputMixer_t));
			break;

		case 'm':// "Set Mixer Table
			if (pRxData[0] == EEMIXER_REVISION && (pRxData[1] == sizeof(OutputMixer_t))) {
				memcpy(&outputMixer, (uint8_t*)&pRxData[2], sizeof(OutputMixer_t));
				outputMixer_writeToEEProm();
				while (!txd_complete)
					; // wait for previous frame to be sent
				tempchar[0] = 1;
			} else {
				tempchar[0] = 0;
			}
			sendData('M', FC_ADDRESS, 1, &tempchar, 1);
			break;

		case 'p': // get PPM channels
			request_PPMChannels = TRUE;
			break;

        case 'i':// Read IMU configuration
            tempchar[0] = IMUCONFIG_REVISION;
            tempchar[1] = sizeof(IMUConfig);
            while (!txd_complete)
                ; // wait for previous frame to be sent
            sendData('I', FC_ADDRESS, 2, &tempchar, 2, (uint8_t *) &IMUConfig, sizeof(IMUConfig));
            break;

        case 'j':// Save IMU configuration
          if (!(MKFlags & MKFLAG_MOTOR_RUN)) // save settings only if motors are off
          {
              if ((pRxData[0] == IMUCONFIG_REVISION) && (pRxData[1] == sizeof(IMUConfig))) {
                  memcpy(&IMUConfig, (uint8_t*) &pRxData[2], sizeof(IMUConfig));
                  IMUConfig_writeToEEprom();
                  tempchar[0] = 1; //indicate ok data
              } else {
                  tempchar[0] = 0; //indicate bad data
              }
              while (!txd_complete)
                  ; // wait for previous frame to be sent
              sendData('J', FC_ADDRESS, 1, &tempchar, 1);
          }
          break;

		case 'q':// request settings
			if (pRxData[0] == 0xFF) {
				pRxData[0] = getParamByte(PID_ACTIVE_SET);
			}
			// limit settings range
			if (pRxData[0] < 1)
				pRxData[0] = 1; // limit to 1
			else if (pRxData[0] > 5)
				pRxData[0] = 5; // limit to 5
			// load requested parameter set

			paramSet_readFromEEProm(pRxData[0]);

			tempchar[0] = pRxData[0];
			tempchar[1] = EEPARAM_REVISION;
			tempchar[2] = sizeof(staticParams);
			while (!txd_complete)
				; // wait for previous frame to be sent
			sendData('Q', FC_ADDRESS, 2, &tempchar, 3, (uint8_t *) &staticParams, sizeof(staticParams));
			break;

		case 's': // save settings
			if (!(MKFlags & MKFLAG_MOTOR_RUN)) // save settings only if motors are off
			{
				if ((1 <= pRxData[0]) && (pRxData[0] <= 5) && (pRxData[1] == EEPARAM_REVISION) && (pRxData[2] == sizeof(staticParams))) // check for setting to be in range and version of settings
				{
					memcpy(&staticParams, (uint8_t*) &pRxData[3], sizeof(staticParams));
					paramSet_writeToEEProm(pRxData[0]);
					setActiveParamSet(pRxData[0]);
					configuration_paramSetDidChange();
					tempchar[0] = getActiveParamSet();
					beepNumber(tempchar[0]);
				} else {
					tempchar[0] = sizeof(staticParams); //indicate bad data
				}
				while (!txd_complete)
					; // wait for previous frame to be sent
				sendData('S', FC_ADDRESS, 1, &tempchar, 1);
			}
			break;

		default:
			//unsupported command received
			break;
		} // case FC_ADDRESS:

	default: // any Slave Address
		switch (rxd_buffer[2]) {
		case 'a':// request for labels of the analog debug outputs
			requestedAnalogLabel = pRxData[0];
			if (requestedAnalogLabel > 31)
				requestedAnalogLabel = 31;
			break;

		case 'b': // submit extern control
			memcpy(&externalControl, (uint8_t*) pRxData, sizeof(ExternalControl_t));
			confirmFrame = externalControl.frame;
			externalControlActive = 255;
			break;

        case 'd': // request for the debug data
            debugDataInterval = (uint16_t) pRxData[0] * 10;
            if (debugDataInterval > 0)
                request_debugData = TRUE;
            break;

        case 'e': // Requeset for the DCM matrix
            request_DCM_matrix = TRUE;
            break;

        case 'f':
            requestedProfilerLabel = pRxData[0];
            if (requestedProfilerLabel > 15)
                requestedProfilerLabel = 15;
            break;

        case 'u':
            profilerDataInterval = (uint16_t) pRxData[0] * 10;
            if (profilerDataInterval > 0)
                request_profilerData = TRUE;
            break;

		case 'o':// request for OSD data (FC style)
		  OSDDataInterval = (uint16_t) pRxData[0] * 10;
		  if (OSDDataInterval > 0)
		    request_OSD = TRUE;
		  break;
		  
		case 'v': // request for version and board release
			request_verInfo = TRUE;
			break;

		case 'x':
			request_variables = TRUE;
			break;

		case 'g':// get external control data
			request_externalControl = TRUE;
			break;

		default:
			//unsupported command received
			break;
		}
		break; // default:
	}
	// unlock the rxd buffer after processing
	pRxData = 0;
	rxDataLen = 0;
	rxd_buffer_locked = FALSE;
}

/************************************************************************/
/* Routine f�r die Serielle Ausgabe                                     */
/************************************************************************/
int uart_putchar(char c, FILE* fims) {
	if (c == '\n')
		uart_putchar('\r', fims);
	// wait until previous character was send
	loop_until_bit_is_set(UCSR0A, UDRE0);
	// send character
	UDR0 = c;
	return (0);
}

//---------------------------------------------------------------------------------------------
void usart0_transmitTxData(void) {
	if (!txd_complete)
		return;

	if (request_verInfo && txd_complete) {
		sendData('V', FC_ADDRESS, 1, (uint8_t *) &versionInfo, sizeof(versionInfo));
		request_verInfo = FALSE;
	}

	if (request_display && txd_complete) {
		request_display = FALSE;
	}

	if (request_display1 && txd_complete) {
		request_display1 = FALSE;
	}

	if (requestedAnalogLabel != 0xFF && txd_complete) {
		char label[17]; // local sram buffer
		memset(label, ' ', sizeof(label));
		strcpy_P(label, (PGM_P)pgm_read_word(&(ANALOG_LABELS[requestedAnalogLabel]))); // read label from flash to sram buffer
		sendData('A', FC_ADDRESS, 2, (uint8_t *) &requestedAnalogLabel, sizeof(requestedAnalogLabel), label, 16);
		requestedAnalogLabel = 0xFF;
	}

    if (requestedProfilerLabel != 0xFF && txd_complete) {
        char label[17]; // local sram buffer
        memset(label, ' ', sizeof(label));
        strcpy_P(label, (PGM_P)pgm_read_word(&(PROFILER_LABELS[requestedProfilerLabel]))); // read label from flash to sram buffer
        sendData('F', FC_ADDRESS, 2, (uint8_t *) &requestedProfilerLabel, sizeof(requestedProfilerLabel), label, 16);
        requestedProfilerLabel = 0xFF;
    }

	if (confirmFrame && txd_complete) { // Datensatz ohne checksum best�tigen
		sendData('B', FC_ADDRESS, 1, (uint8_t*) &confirmFrame, sizeof(confirmFrame));
		confirmFrame = 0;
	}

	if (((debugDataInterval && checkDelay(debugDataTimer)) || request_debugData) && txd_complete) {
		sendData('D', FC_ADDRESS, 1, (uint8_t *)&debugOut, sizeof(debugOut));
		debugDataTimer = setDelay(debugDataInterval);
		request_debugData = FALSE;
	}

    if (((profilerDataInterval && checkDelay(profilerDataTimer)) || request_profilerData) && txd_complete) {
        sendData('U', FC_ADDRESS, 2, (uint8_t *)&totalProfilerHits, sizeof(totalProfilerHits), (uint8_t *)&activitiesTimerHits, sizeof(activitiesTimerHits));
        profilerDataTimer = setDelay(profilerDataInterval);
        request_profilerData = FALSE;
    }

	if (request_DCM_matrix && txd_complete) {
		/*
		sendData('E', FC_ADDRESS, 1,
				(uint8_t *) &dcmGyro, sizeof(dcmGyro));
		*/
		request_DCM_matrix = FALSE;
	}

	if (request_externalControl && txd_complete) {
		sendData('G', FC_ADDRESS, 1, (uint8_t *) &externalControl,
				sizeof(externalControl));
		request_externalControl = FALSE;
	}

#ifdef USE_DIRECT_GPS
	if((checkDelay(toMk3MagTimer)) && txd_complete) {
		toMk3Mag.attitude[0] = (int16_t)(attitude[PITCH] / (GYRO_DEG_FACTOR_PITCHROLL/10)); // approx. 0.1 deg
		toMk3Mag.attitude[1] = (int16_t)(attitude[ROLL] / (GYRO_DEG_FACTOR_PITCHROLL/10)); // approx. 0.1 deg
		toMk3Mag.userParam[0] = dynamicParams.userParams[0];
		toMk3Mag.userParam[1] = dynamicParams.userParams[1];
		toMk3Mag.calState = compassCalState;
		sendData('w', MK3MAG_ADDRESS, 1,(uint8_t *) &toMk3Mag,sizeof(toMk3Mag));
		// the last state is 5 and should be send only once to avoid multiple flash writing
		if(compassCalState > 4) compassCalState = 0;
		toMk3MagTimer = setDelay(99);
	}
#endif

	if (request_outputTest && txd_complete) {
		sendData('T', FC_ADDRESS, 0);
		request_outputTest = FALSE;
	}

	if (request_PPMChannels && txd_complete) {
		uint8_t length = MAX_CONTROLCHANNELS;
		sendData('P', FC_ADDRESS, 2, &length, 1, (uint8_t*)&PPM_in, sizeof(PPM_in));
		request_PPMChannels = FALSE;
	}

	if (request_variables && txd_complete) {
		sendData('X', FC_ADDRESS, 1, (uint8_t *) &variables, sizeof(variables));
		request_variables = FALSE;
	}

#ifdef USE_DIRECT_GPS
	if (((OSD_interval && checkDelay(OSD_timer)) || request_OSD) && txd_complete) {
	  int32_t height = analog_getHeight();
	  data3D.anglePitch = (int16_t) (attitude[PITCH] / (GYRO_DEG_FACTOR_PITCHROLL/10)); // convert to multiple of 0.1 deg
	  data3D.angleRoll = (int16_t) (attitude[ROLL] / (GYRO_DEG_FACTOR_PITCHROLL/10)); // convert to multiple of 0.1 deg
	  data3D.heading = (int16_t) (heading / (GYRO_DEG_FACTOR_YAW/10)); // convert to multiple of 0.1 deg
  	  sendData('O', FC_ADDRESS, 4, (uint8_t*)&data3D, sizeof(data3D), (uint8_t*)&GPSInfo, sizeof(GPSInfo), (uint8_t*)&height, sizeof(height), (uint8_t*)UBat, sizeof(UBat));
	  OSD_timer = setDelay(OSD_interval);
	  request_OSD = FALSE;
	}
#endif
}
