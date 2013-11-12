#include <avr/io.h>
#include "usart.h"

void USART_init(void)
{
	/* Disable the power reduction for the USART */
	PRR &= ~(1 << PRUSART0);

	/* Set the USART mode to asynchronous */
	UCSR0C &= ~(1 << UMSEL01);
	UCSR0C &= ~(1 << UMSEL00);

	/* Disable the parity generation and check */
	UCSR0C &= ~(1 << UPM01);
	UCSR0C &= ~(1 << UPM00);

	/* Set number of stop bits to 1 */
	UCSR0C &= ~(1 << USBS0);

	/* Set the character size to 8 bit */
	UCSR0B &= ~(1 << UCSZ02);
	UCSR0C |= (1 << UCSZ01);
	UCSR0C |= (1 << UCSZ00);

	/* Set the baud rate to ~4800 */
	UBRR0 = 12;

	/* Enable the receiver */
	UCSR0B |= (1 << RXEN0);

	/* Enable the transmitter */
	UCSR0B |= (1 << TXEN0);
}

void USART_send_bytes(const uint8_t * data, uint8_t length)
{
	uint8_t i;
	/* Loop through all bytes */
	for (i = 0; i < length; i++) {
		/* Wait until the transmit buffer is empty */
		while (!(UCSR0A & (1 << UDRE0)));

		/* Send one byte */
		UDR0 = data[i];
	}
}

void USART_receive_bytes(uint8_t * data, uint8_t length)
{
	uint8_t i;
	for (i = 0; i < length; i++) {
		/* Wait for data to be received */
		while (!(UCSR0A & (1 << RXC0)));

		/* Receive one byte */
		data[i] = UDR0;
	}
}
