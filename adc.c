/**
	\file
	\ingroup ADC
	\brief Definition of functions, variables, and constants related to the internal ADC
 */

#include <avr/io.h>

#include "adc.h"

volatile uint16_t _ADC_result;

/**
	Initializes the internal ADC.
	It is important that the ADC channel selection is such that it corresponds to the first scheduled ADC measurement task.
	This is because the ADC channel is selected always after retrieving the result and not before the measurement.
	That allows the ADC to settle before measurements.
 */
void ADC_init(void)
{
	/* Use AVCC (5 V) as voltage reference */
	ADMUX |= (1 << REFS0);
	ADMUX &= ~(1 << REFS1);

	/* Set ADC input to channel 7 */
	ADC_select_channel_7();

	/* Left align the 10 bit ADC value in the 16 bit register */
	ADMUX |= (1 << ADLAR);

	/* Enable the ADC */
	ADCSRA |= (1 << ADEN);

	/* Set ADC clock prescaler to 1/128 */
	ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);

	/* Enable the conversion complete interrupt */
	ADCSRA |= (1 << ADIE);
}
