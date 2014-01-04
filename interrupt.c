#include <string.h>
#include "interrupt.h"
#include "usart.h"
#include "timer.h"
#include "adc.h"
#include "encoder.h"
#include "temperature.h"

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

ISR(PCINT1_vect)
{
	/* Get the new encoder state */
	uint8_t new_state = PINB & ((1 << ENCODER_INPUT_A) | (1 << ENCODER_INPUT_B));

	/* Compare with the previous state and set apply the increment.
	   A clockwise rotation goes through the states 1..2..3..4 and then repeats
	   whereas a counter clockwise rotation goes through 1..4..3..2 repeating.
	   If somehow the rotation causes more than one increment it is ignored. */
	if (_encoder_state == ENCODER_STATE_1) {
		/* Next state is 2, previous is 4 */
		if (new_state == ENCODER_STATE_2)
			_encoder_increment += 1;
		else if (new_state == ENCODER_STATE_4)
			_encoder_increment -= 1;
	} else if (_encoder_state == ENCODER_STATE_2) {
		/* Next state is 3, previous is 1 */
		if (new_state == ENCODER_STATE_3)
			_encoder_increment += 1;
		else if (new_state == ENCODER_STATE_1)
			_encoder_increment -= 1;
	} else if (_encoder_state == ENCODER_STATE_3) {
		/* Next state is 4, previous is 2 */
		if (new_state == ENCODER_STATE_4)
			_encoder_increment += 1;
		else if (new_state == ENCODER_STATE_2)
			_encoder_increment -= 1;
	} else if (_encoder_state == ENCODER_STATE_4) {
		/* Next state is 1, previous is 3 */
		if (new_state == ENCODER_STATE_1)
			_encoder_increment += 1;
		else if (new_state == ENCODER_STATE_3)
			_encoder_increment -= 1;
	}

	/* Set the new encoder state */
	_encoder_state = new_state;
}

ISR(PCINT2_vect)
{
	uint8_t pin = ADS1248_READY_0_PIN;
	if (pin & ADS1248_READY_0) {
		ADS1248_READY_0_PCMSK &= ~(1 << ADS1248_READY_0_PCINT);
		_temperature_ADS1248_ready[0] |= TEMPERATURE_ADS1248_READY_FLAG;
	} if (pin & ADS1248_READY_1) {
		ADS1248_READY_1_PCMSK &= ~(1 << ADS1248_READY_1_PCINT);
		_temperature_ADS1248_ready[1] |= TEMPERATURE_ADS1248_READY_FLAG;
	}
}
