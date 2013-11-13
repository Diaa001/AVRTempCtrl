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

	interrupts_on();

	/* Main loop */
	while(1) {
		/* Process tasks */
		if (_ADC_queue_index & ADC_FLAG_QUEUE_START) {
			/* The index overflows in the timer interrupt handler */
			if ((_ADC_queue_index & 0x3f) >= ADC_NUM_INPUTS)
				_ADC_queue_index = 0;

			/* Start a conversion */
			ADC_start_conversion();

			/* Disable the ADC_FLAG_QUEUE_START flag */
			_ADC_queue_index &= 0x3f;
		} else if (_ADC_queue_index & ADC_FLAG_QUEUE_FINISHED) {
			/* Select the channel for the next conversion. After changing the channel there is time
			   for the ADC inputs to settle. */
			if ((_ADC_queue_index & 0x3f) == ADC_HUM_0) {
				ADC_select_channel_diff_1_0_10x();
			} else if ((_ADC_queue_index & 0x3f) == ADC_TEMP_0) {
				ADC_select_channel_2();
			}

			/* Disable the ADC_FLAG_QUEUE_FINISHED flag */
			_ADC_queue_index &= 0x3f;
		} else if (rx_complete) {
			if (strcmp((const char *) rx_buffer[rx_buffer_sel], "GET:TEMPERATURE\n") == 0) {
				interrupts_suspend();
				int16_t adc_val = (int16_t) _ADC_results[ADC_TEMP_0];
				interrupts_resume();
				adc_val /= 64;
				sprintf((char *) tx_buffer, "Temperature: %i ADC\n", adc_val);
				USART_send_bytes((uint8_t *) tx_buffer, strlen((const char *) tx_buffer));
			} else if (strcmp((const char *) rx_buffer[rx_buffer_sel], "GET:HUMIDITY\n") == 0) {
				interrupts_suspend();
				uint16_t adc_val2 = _ADC_results[ADC_HUM_0];
				interrupts_resume();
				adc_val2 >>= 6;
				sprintf((char *) tx_buffer, "Humidity: %i %%\n", honeywell_convert_ADC_to_RH(adc_val2));
				USART_send_bytes((uint8_t *) tx_buffer, strlen((const char *) tx_buffer));
			} else {
				USART_send_bytes((uint8_t *) "Error\n", 6);
			}
			memset((void *) rx_buffer[rx_buffer_sel], '\0', RX_BUFFER_LENGTH);
			rx_complete = 0;
		}
	} /* Main loop */

	/* The program will never reach this point. */
	return 0;
}
