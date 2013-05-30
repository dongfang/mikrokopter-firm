#ifndef _UART0_H
#define _UART0_H

#define TXD_BUFFER_LEN  140
#define RXD_BUFFER_LEN  140

#include <inttypes.h>
#include <stdio.h>
#include "ubx.h"

//Baud rate of the USART
#define USART0_BAUD 57600

extern void usart0_init(void);
extern void usart0_transmitTxData(void);
extern void usart0_processRxData(void);
//extern int16_t uart_putchar(int8_t c);

extern int uart_putchar(char c, FILE* fims);

// extern uint8_t remotePollDisplayLine;

extern uint8_t outputTestActive;
extern uint8_t outputTest[16];

typedef struct {
  float pitchRate;  // in radians
  float rollRate;   // in radians
  float yawRate;    // in radians

  float pitch; // in radians
  float roll;  // in radians
  float heading;    // in radians

  float xAcc;
  float yAcc;
  float zAcc;
}__attribute__((packed)) IMUData;

typedef struct {
  IMUData imuData;
  GPS_INFO_t GPSInfo;
  int32_t airpressureHeight;
  int16_t batteryVoltage;
}__attribute__((packed)) OSDData_t;

#endif //_UART0_H
