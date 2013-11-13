#include <string.h>
#include "interrupt.h"
#include "usart.h"
#include "adc.h"

uint8_t _sreg_save;

/* USART0 RX complete interrupt handler */
ISR(USART0_RX_vect) {
	/* Make sure the buffer does not overflow */
	if (rx_buffer_pointer >= RX_BUFFER_LENGTH) {
		/* Reset at the beginning of the buffer after clearing the buffer */
		memset((void *) rx_buffer[1 - rx_buffer_sel], '\0', RX_BUFFER_LENGTH);
		rx_buffer_pointer = 0;
	}

	/* Read the next character into the buffer */
	rx_buffer[1 - rx_buffer_sel][rx_buffer_pointer] = UDR0;

	/* Check whether the current character marks the end of the message */
	if (rx_buffer[1 - rx_buffer_sel][rx_buffer_pointer] != '\n') {
		/* Increment the buffer pointer */
		rx_buffer_pointer++;
	} else {
		/* Set RX complete flag that is handled in the main routine */
		rx_complete = 1;

		/* Switch buffers */
		rx_buffer_sel = 1 - rx_buffer_sel;

		/* Reset the buffer pointer */
		rx_buffer_pointer = 0;
	}
}

ISR(TIMER0_COMPB_vect)
{
	/* Increment the index of the ADC queue and set the start flag */
	_ADC_queue_index = ((_ADC_queue_index & 0x3f) + 1) | ADC_FLAG_QUEUE_START;
}

ISR(ADC_vect)
{
	/* Store the ADC result */
	_ADC_results[_ADC_queue_index & 0x3f] = ADC;

	/* Set the conversion finished flag */
	_ADC_queue_index = (_ADC_queue_index & 0x3f) | ADC_FLAG_QUEUE_FINISHED;
}
