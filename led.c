/**
	\file
	\ingroup LED
	\brief Definition of functions related to LED displays
 */

#include "led.h"

void LED_init(void)
{
	/* Set the LED ports as outputs */
	LED_DDR |= (1 << LED_0);
	LED_DDR |= (1 << LED_1);

	/* Turn off LEDs initially */
	LED_PORT &= ~(1 << LED_0);
	LED_PORT &= ~(1 << LED_1);
}

/**
	Controls the microcontroller pins that are connected to the LED displays.
	\param led The LED that should be set (one of \ref LED_ACTIVE or \ref LED_ERROR).
	\param state The state that the LED should take on (one of \ref LED_ON or \ref LED_OFF).
 */
void LED_set(uint8_t led, uint8_t state)
{
	if (state)
		LED_PORT |= (1 << led);
	else
		LED_PORT &= ~(1 << led);
}
