#include <avr/io.h>

#include "timer.h"

volatile uint8_t _task = NUMBER_OF_TASKS - 1;

void timer_8bit_cnt0_init(void)
{
	/* Initialize the timer value to 0 */
	TCNT0 = 0;

	/* Set waveform generation mode to fast PWM with OCRx update at BOTTOM */
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	TCCR0B |= (1 << WGM02);

	/* Set OC0A to normal port operation */
	TCCR0A &= ~(1 << COM0A0);
	TCCR0A &= ~(1 << COM0A1);

	/* Select clock prescaler 1/1024 */
	TCCR0B |= (1 << CS02) | (1 << CS00);
	TCCR0B &= ~(1 << CS01);

	/* Set the counter frequency (f = F_CPU / 1024 / (1 + OCR0A)) */
	OCR0A = 255;

	/* Set the duty cycle (0: 0%, OCR0A: 100%) */
	OCR0B = 64;

	/* Set the OC0B pin as output */
	DDRB |= (1 << PB4);

	/* Enable the Compare Match B Interrupt */
	TIMSK0 |= (1 << OCIE0B);
}

void timer_16bit_cnt1_init(void)
{
	/* Initialize the timer value such that it starts at zero always after the PID task */
	TCNT1 = -(TASK_PID * 2048 + 1024);

	/* Set waveform generation mode to phase and frequency correct PWM */
	TCCR1B |= (1 << WGM13);
	TCCR1B &= ~(1 << WGM12);
	TCCR1A &= ~(1 << WGM11);
	TCCR1A |= (1 << WGM10);

	/* Set OC1A to normal port operation */
	TCCR1A &= ~(1 << COM1A1);
	TCCR1A &= ~(1 << COM1A0);

	/* Set OC1B at BOTTOM and clear at OCR1B match */
	TCCR1A &= ~(1 << COM1B0);
	TCCR1A |= (1 << COM1B1);

	/* Set the clock prescaler 1/64 */
	TCCR1B |= (1 << CS11) | (1 << CS10);
	TCCR1B &= ~(1 << CS12);

	/* Set the counter frequency (f = F_CPU / (2 * 64 * OCR1A)) equal to NUMBER_OF_TASKS
	   times the timer0 frequency.
	   1024: timer0 clock perscaler
	   255: timer0 TOP value
	   64: timer1 clock prescaler */
	//OCR1A = NUMBER_OF_TASKS * 1024 * (1 + 255) / (2 * 64);
	OCR1A = 32768;

	/* Set the duty cycle (0: 0%, OCR1A: 100%) */
	OCR1B = 0;

	/* Set the OC1B pin as output */
	DDRD |= (1 << PD4);
}
