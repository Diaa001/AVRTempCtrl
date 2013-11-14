#include <string.h>
#include "interrupt.h"
#include "usart.h"
#include "timer.h"
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

	char byte = UDR0;

	/* Check whether the current character marks the end of the message */
	if (byte != '\n') {
		/* Read the next character into the buffer */
		rx_buffer[1 - rx_buffer_sel][rx_buffer_pointer] = byte;

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
	_task = ((_task & 0x3f) < NUMBER_OF_TASKS - 1) ? (_task & 0x3f) + 1 : 0;

	/* Activate the start flag. The routinge processing the task is responsible to turn off the flag. */
	_task |= TASK_START;
}

ISR(ADC_vect)
{
	/* Store the ADC result */
	_ADC_result = ADC;

	/* Set the conversion complete flag */
	_ADC_result |= (1 << ADC_CONVERSION_COMPLETE_BIT);
}
