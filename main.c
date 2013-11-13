#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>

#define F_CPU 4000000UL

#include <util/delay.h>

#include "usart.h"
#include "timer.h"
#include "adc.h"
#include "clock.h"
#include "humidity.h"
#include "interrupt.h"

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

	interrupts_on();

	/* Main loop */
	while(1) {
		if (rx_complete) {
			if (strcmp((const char *) rx_buffer[rx_buffer_sel], "Temperature\n") == 0) {
				/* Measure the temperature */
				ADC_select_channel_diff_1_0_10x();
				_delay_ms(10);
				/* Read from the ADC */
				ADC_start_conversion();
				ADC_wait_for_conversion();
				int16_t adc_val = (int16_t) ADC_get_result();
				adc_val /= 64;
				sprintf((char *) tx_buffer, "Temperature: %i ADC\n", adc_val);
				USART_send_bytes((uint8_t *) tx_buffer, strlen((const char *) tx_buffer));

				/* Calculate the temperature difference */
				int16_t temp_diff = (adc_val - setpoint);

				/* Set the new pulse width */
				if (temp_diff > 0)
					OCR1B = (kp * (uint16_t)temp_diff) * 60;
				else
					OCR1B = 0;

			} else if (strcmp((const char *) rx_buffer[rx_buffer_sel], "Humidity\n") == 0) {
				/* Measure the humidity */
				ADC_select_channel_2();
				_delay_ms(10);
				/* Read from the ADC */
				ADC_start_conversion();
				ADC_wait_for_conversion();
				uint16_t adc_val2 = ADC_get_result();
				adc_val2 >>= 6;
				sprintf((char *) tx_buffer, "Humidity: %i %%\n", honeywell_convert_ADC_to_RH(adc_val2));
				USART_send_bytes((uint8_t *) tx_buffer, strlen((const char *) tx_buffer));
			} else {
				USART_send_bytes((uint8_t *) "Error\n", 6);
			}
			memset((void *) rx_buffer[rx_buffer_sel], '\0', RX_BUFFER_LENGTH);
			rx_complete = 0;
		} /* rx_complete */
	} /* Main loop */

	/* The program will never reach this point. */
	return 0;
}
