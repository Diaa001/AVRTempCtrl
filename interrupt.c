#include "interrupt.h"

uint8_t _sreg_save;

/* USART0 RX complete interrupt handler */
ISR(USART0_RX_vect) {
	uint8_t data = UDR0;

	/* Wait until the transmit buffer is empty */
	while (!(UCSR0A & (1 << UDRE0)));

	UDR0 = data + 1;
}
