#include <avr/io.h>

#include "adc.h"

volatile uint16_t _ADC_result;

void ADC_init(void)
{
	/* Use AVCC (5 V) as voltage reference */
	ADMUX |= (1 << REFS0);
	ADMUX &= ~(1 << REFS1);

	/* Set ADC input to channel 2 */
	ADC_select_channel_2();

	/* Left align the 10 bit ADC value in the 16 bit register */
	ADMUX |= (1 << ADLAR);

	/* Enable the ADC */
	ADCSRA |= (1 << ADEN);

	/* Set ADC clock prescaler to 1/128 */
	ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);

	/* Enable the conversion complete interrupt */
	ADCSRA |= (1 << ADIE);
}
