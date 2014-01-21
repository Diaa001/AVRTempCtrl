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

void LED_set(uint8_t led, uint8_t state)
{
	if (state)
		LED_PORT |= (1 << led);
	else
		LED_PORT &= ~(1 << led);
}
