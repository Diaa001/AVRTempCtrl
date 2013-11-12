#include <stdint.h>
#include <avr/io.h>

#define F_CPU 4000000UL

#include <util/delay.h>

#include "usart.h"
#include "timer.h"
#include "adc.h"
#include "clock.h"

int main (void) {
	/* Set clock to 4 MHz by setting the clock division (prescale) to 2 */
	clock_set_prescale_2();

	USART_init();
	timer_8bit_cnt0_init();
	timer_16bit_cnt1_init();
	ADC_init();

	/* The temperature setpoint (in ADC units) */
	int16_t setpoint = 0;
	int16_t kp = 1;

	/* Main loop */
	while(1) {
		/* Read from the ADC */
		ADC_start_conversion();
		ADC_wait_for_conversion();
		int16_t adc_val = (int16_t) ADC_get_result();
		adc_val /= 64;
		USART_send_bytes((uint8_t *) &adc_val, 2);

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
