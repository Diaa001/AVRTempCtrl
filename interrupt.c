/**
	\file
	\ingroup Interrupts
	\brief Definitions of functions and variables related to interrupts

	\addtogroup Interrupts
	\{
 */

#include <string.h>
#include "interrupt.h"
#include "usart.h"
#include "timer.h"
#include "adc.h"
#include "encoder.h"
#include "temperature.h"
#include "buttons.h"

uint8_t _sreg_save;

/**
	\brief USART0 RX complete interrupt handler

	This interrupt handler is called whenever a new byte has been received
	by the USART interface.
	It reads the incoming byte and appends it to a the string \ref rx_buffer
	until a newline character is encountered.
	If that happens the \ref rx_complete flag is set.
	It stores a maximum of \ref RX_BUFFER_LENGTH characters.
	If the buffer is full, it is cleared and the incoming character is written
	into the emptied buffer.

	See also \ref USART "here".
 */
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

/**
	\brief Timer 0 overflow interrupt handler (scheduler function)

	This interrupt handler is called always when the timer 0 counter overflows.
	It schedules the tasks by incrementing the \ref _task variable and setting
	the \ref TASK_START flag.
	The main() routine then handles the processing of the task and in the end
	clears the \ref TASK_START flag.
 */
ISR(TIMER0_COMPB_vect)
{
	/* Switch to the next task */
	_task = ((_task & 0x3f) < NUMBER_OF_TASKS - 1) ? (_task & 0x3f) + 1 : 0;

	/* Activate the start flag. The routine processing the task is responsible to turn off the flag. */
	_task |= TASK_START;
}

/**
	\brief ADC conversion complete interrupt handler

	This interrupt handler is called always when a conversion with the internal ADC
	completes.
	It stores the ADC value in variable \ref _ADC_result and sets a conversion complete flag.
	The main() routine should then store this value elsewhere and clear the conversion
	complete flag.
 */
ISR(ADC_vect)
{
	/* Store the ADC result */
	_ADC_result = ADC;

	/* Set the conversion complete flag */
	_ADC_result |= (1 << ADC_CONVERSION_COMPLETE_BIT);
}

/**
	\brief Pin change interrupt handler 1

	This interrupt handler is called whenever a pin of port B changes its
	value unless it's masked.
	The rotary encoder is connected to port B.
	The function uses the \ref _encoder_state variable which stores the last state
	to determine the encoder increment.
	It then stores the new state in \ref _encoder_state.
	If a state is skipped for some reason (e.g. the encoder was rotated to fast)
	the increment is considered to be zero because it cannot be decided wheter
	the increment was positive or negative.
	This should usually not happen.
 */
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

/**
	\brief Pin change interrupt handler 2

	This interrupt handler is called whenever a pin of port C changes its
	value unless it's masked.
	The (conversion) READY signal from the two ADS1248 ADCs is connected
	to this port as well as the three buttons.
	The routine checks which state has changed and stores the new state
	while setting a flag to indicate the change.
	The main() routine should then take actions to accomodate the state
	change.
	If it does not acknowledge a button press state change and the next
	change already occurs then an entire button action is lost.
 */
ISR(PCINT2_vect)
{
	uint8_t pin = PINC;

	/* Check whether the ready signal from the first ADS1248 has gone low */
	if (!(pin & (1 << ADS1248_READY_0))) {
		/* Disable the interrupt for this pin */
		ADS1248_READY_0_PCMSK &= ~(1 << ADS1248_READY_0_PCINT);

		/* Set the ready flag */
		_temperature_ADS1248_ready[0] |= TEMPERATURE_ADS1248_READY_FLAG;
	}

	/* Check whether the ready signal from the second ADS1248 has gone low */
	if (!(pin & (1 << ADS1248_READY_1))) {
		/* Disable the interrupt for this pin */
		ADS1248_READY_1_PCMSK &= ~(1 << ADS1248_READY_1_PCINT);

		/* Set the ready flag */
		_temperature_ADS1248_ready[1] |= TEMPERATURE_ADS1248_READY_FLAG;
	}

	if ((pin & BUTTON_0) && !(_button_state[0] & BUTTON_STATE_FLAG)) {
		_button_state[0] = BUTTON_STATE_FLAG | BUTTON_CHANGED_FLAG;
	} else if (!(pin & BUTTON_0) && (_button_state[0] & BUTTON_STATE_FLAG)) {
		_button_state[0] = BUTTON_CHANGED_FLAG;
	}

	if ((pin & BUTTON_1) && !(_button_state[1] & BUTTON_STATE_FLAG)) {
		_button_state[1] = BUTTON_STATE_FLAG | BUTTON_CHANGED_FLAG;
	} else if (!(pin & BUTTON_1) && (_button_state[1] & BUTTON_STATE_FLAG)) {
		_button_state[1] = BUTTON_CHANGED_FLAG;
	}

	if ((pin & BUTTON_2) && !(_button_state[2] & BUTTON_STATE_FLAG)) {
		_button_state[2] = BUTTON_STATE_FLAG | BUTTON_CHANGED_FLAG;
	} else if (!(pin & BUTTON_2) && (_button_state[2] & BUTTON_STATE_FLAG)) {
		_button_state[2] = BUTTON_CHANGED_FLAG;
	}
}

/** \} */
