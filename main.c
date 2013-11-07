#include <avr/io.h>

#define F_CPU 1000000UL

#include <util/delay.h>

int main (void) {
	/* ### Set up the 8 bit timer 0 ### */

	/* Initialize the timer value to 0 */
	TCNT0 = 0;

	/* Set waveform generation mode to fast PWM with OCRx update at BOTTOM */
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	TCCR0B |= (1 << WGM02);

	/* Set OC0A to normal port operation */
	TCCR0A &= ~(1 << COM0A0);
	TCCR0A &= ~(1 << COM0A1);

	/* Set OC0B at BOTTOM and clear at OCR0B match */
	TCCR0A &= ~(1 << COM0B0);
	TCCR0A |= (1 << COM0B1);

	/* Select clock prescaler 1/1024 */
	TCCR0B |= (1 << CS02) | (1 << CS00);
	TCCR0B &= ~(1 << CS01);

	/* Set the counter frequency (f = F_CPU / 1024 / OCR0A) */
	OCR0A = 191;

	/* Set the duty cycle (0: 0%, OCR0A: 100%) */
	OCR0B = 64;

	/* Set the OC0B pin as output */
	DDRB |= (1 << PB4);

	/* ### Set up the 16 bit timer 1 ### */

	/* Initialize the timer value to 0 */
	TCNT1 = 0;

	/* Set waveform generation mode to fast PWM with OCRx update at BOTTOM */
	TCCR1A |= (1 << WGM11) | (1 << WGM10);
	TCCR1B |= (1 << WGM13) | (1 << WGM12);

	/* Set OC1A to normal port operation */
	TCCR1A &= ~(1 << COM1A1);
	TCCR1A &= ~(1 << COM1A0);

	/* Set OC1B at BOTTOM and clear at OCR1B match */
	TCCR1A &= ~(1 << COM1B0);
	TCCR1A |= (1 << COM1B1);

	/* Set the clock prescaler 1/64 */
	TCCR1B |= (1 << CS11) | (1 << CS10);
	TCCR1B &= ~(1 << CS12);

	/* Set the counter frequency (f = F_CPU / 64 / OCR1A) */
	OCR1A = 15625;

	/* Set the duty cycle (0: 0%, OCR1A: 100%) */
	OCR1B = 156;

	/* Set the OC1B pin as output */
	DDRD |= (1 << PD4);

	/* ### Set up the ADC */

	/* Set internal 1.1 V reference */
	ADMUX |= (1 << REFS1);
	ADMUX &= ~(1 << REFS0);

	/* Set ADC input to channel 0 */
	ADMUX &= ~(0x1f);

	/* Enable the ADC */
	ADCSRA |= (1 << ADEN);

	/* Set ADC clock prescaler to 1/128 */
	ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);

	/* Main loop */
	while(1) {
		ADCSRA |= (1 << ADSC);
		while (ADCSRA & (1 << ADSC));
		OCR1B = ADC * 15;
		_delay_ms(1000);
	}

	/* The program will never reach this point. */
	return 0;
}
