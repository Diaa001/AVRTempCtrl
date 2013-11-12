#include <stdint.h>
#include <avr/io.h>

#define F_CPU 1000000UL

#include <util/delay.h>

#include "usart.h"
#include "timer.h"

int main (void) {
	USART_init();
	timer_8bit_cnt0_init();
	timer_16bit_cnt1_init();

	/* ### Set up the ADC */

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

	/* The temperature setpoint (in ADC units) */
	int16_t setpoint = 0;
	int16_t kp = 1;

	/* Main loop */
	while(1) {
		/* Read from the ADC */
		ADCSRA |= (1 << ADSC);
		while (ADCSRA & (1 << ADSC));
		int16_t adc_val = (int16_t) ADC;
		adc_val /= 64;
		USART_send_bytes(&adc_val, 2);

		/* Calculate the temperature difference */
		int16_t temp_diff = (adc_val - setpoint);

		/* Set the new pulse width */
		if (temp_diff > 0)
			OCR1B = (kp * (uint16_t)temp_diff) * 60;
		else
			OCR1B = 0;
		_delay_ms(1000);
	}

	/* The program will never reach this point. */
	return 0;
}
