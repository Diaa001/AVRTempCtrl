#include "buttons.h"

uint8_t _button_state [BUTTONS_NUM];

void buttons_init(void)
{
	/* Set the data directions of the button pins */
	BUTTON_0_DDR_REG |= (1 << BUTTON_0);
	BUTTON_1_DDR_REG |= (1 << BUTTON_1);
	BUTTON_2_DDR_REG |= (1 << BUTTON_2);

	/* Turn off the pullup resistor */
	BUTTON_0_PORT_REG &= ~(1 << BUTTON_0);
	BUTTON_1_PORT_REG &= ~(1 << BUTTON_1);
	BUTTON_2_PORT_REG &= ~(1 << BUTTON_2);

	/* Save the current button state */
	_button_state[0] = (BUTTON_0_PIN_REG & BUTTON_0) ? 1 : 0;
	_button_state[1] = (BUTTON_1_PIN_REG & BUTTON_1) ? 1 : 0;
	_button_state[2] = (BUTTON_2_PIN_REG & BUTTON_2) ? 1 : 0;

	/* Enable the pin change interrupt for PCINT pins 16-23 */
	PCICR |= (1 << BUTTON_PCINT_ENABLE_FLAG);

	/* Select the I/O pins that generate interrupts */
	BUTTON_PCINT_MASK_REG |= ((1 << BUTTON_0_PCINT) | (1 << BUTTON_1_PCINT) | (1 << BUTTON_2_PCINT));
}
