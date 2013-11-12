#include <avr/io.h>

#include "adc.h"

void ADC_init(void)
{
	/* Use AVCC (5 V) as voltage reference */
	ADMUX |= (1 << REFS0);
	ADMUX &= ~(1 << REFS1);

	/* Set ADC input to differential mode with inputs channel 0 (+)
	   and channel 1 (-) and gain 10*/
	ADMUX &= ~(0x1f);
	ADMUX |= (1 << MUX3);
	ADMUX |= (1 << MUX0);

	/* Left align the 10 bit ADC value in the 16 bit register */
	ADMUX |= (1 << ADLAR);

	/* Enable the ADC */
	ADCSRA |= (1 << ADEN);

	/* Set ADC clock prescaler to 1/128 */
	ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
}
