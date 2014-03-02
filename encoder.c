/**
	\file
	\ingroup Encoder
	\brief Definition of functions and variables related to the rotary encoder user interface
 */

#include "encoder.h"

volatile uint8_t _encoder_state;
volatile int8_t _encoder_increment;

extern void encoder_init(void)
{
	/* Set the I/O ports where the encoder is connected as inputs */
	DDRB &= ~((1 << ENCODER_INPUT_A) | (1 << ENCODER_INPUT_B));

	/* Disable the pullup resistors */
	PORTB &= ~((1 << ENCODER_INPUT_A) | (1 << ENCODER_INPUT_B));

	/* Save the initial encoder state */
	_encoder_state = PINB & ((1 << ENCODER_INPUT_A) | (1 << ENCODER_INPUT_B));

	/* Enable the pin change interrupt for PCINT pins 8-15 */
	PCICR |= (1 << PCIE1);

	/* Select the I/O pins that generate interrupts */
	PCMSK1 |= ((1 << PCINT8) | (1 << PCINT9));
}
