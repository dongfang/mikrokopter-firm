#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart1.h"
#include "ubx.h"

#define USART1_BAUD 57600

/****************************************************************/
/*              Initialization of the USART1                    */
/****************************************************************/
void usart1_init(void) {
	// USART1 Control and Status Register A, B, C and baud rate register
	uint8_t sreg = SREG;
	uint16_t ubrr = (uint16_t) (F_CPU / (8 * USART1_BAUD) - 1);

	// disable all interrupts before reconfiguration
	cli();

	// disable RX-Interrupt, disable TX-Interrupt, disable DRE-Interrupt
	UCSR1B &= ~ ((1 << RXCIE1) | (1 << TXCIE1) | (1 << UDRIE1));

	// set direction of RXD1 and TXD1 pins
	// set RXD1 (PD2) as an input pin,  set TXD1 (PD3) as an output pin
	PORTD |= (1 << PORTD2) | (1 << PORTD3);
	DDRD &= ~(1 << DDD2);
	DDRD |= (1 << DDD3);

	// USART0 Baud Rate Register
	// set clock divider
	UBRR1H = (uint8_t) (ubrr >> 8);
	UBRR1L = (uint8_t) ubrr;

	// enable double speed operation
	UCSR1A |= (1 << U2X1);
	// enable receiver and transmitter
	UCSR1B = (1 << TXEN1) | (1 << RXEN1);
	// set asynchronous mode,  no parity, 1 stop bit
	UCSR1C &= ~((1 << UMSEL11) | (1 << UMSEL10) | (1 << UPM11) | (1 << UPM10) | (1 << USBS1));

	// 8-bit
	UCSR1B &= ~(1 << UCSZ12);
	UCSR1C |= (1 << UCSZ11) | (1 << UCSZ10);

	// flush receive buffer explicit
	while (UCSR1A & (1 << RXC1))
		UDR1;

	// enable interrupts at the end
	// enable RX-Interrupt
	UCSR1B |= (1 << RXCIE1);
	// enable TX-Interrupt
	//UCSR1B |= (1 << TXCIE1);
	// enable DRE interrupt
	//UCSR1B |= (1 << UDRIE1);

	// restore global interrupt flags
	SREG = sreg;
}

/****************************************************************/
/*               USART1 data register empty ISR                 */
/****************************************************************/
/*ISR(USART1_UDRE_vect) {
 }
 */

/****************************************************************/
/*               USART1 transmitter ISR                         */
/****************************************************************/
/*ISR(USART1_TX_vect) {
 }
 */
/****************************************************************/
/*               USART1 receiver ISR                            */
/****************************************************************/
ISR(USART1_RX_vect) {
    ubx_parser(UDR1);  // get data byte and put it into the ubx protocol parser
}
