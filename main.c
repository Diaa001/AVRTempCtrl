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
#include "temperature.h"
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
		/* Actions sorted by priority */
		if (_task & 0xc0) {
			/* Process scheduled tasks */
			uint8_t task = _task & 0x3f;
			if (task == TASK_ADC_TEMP_0 || task == TASK_ADC_HUM_0) {
				/* Start a conversion. ADC input was already selected after last conversion completed. */
				ADC_start_conversion();
			}
			/* Disable the task flags */
			_task = task;
		} else if (_ADC_result & (1 << ADC_CONVERSION_COMPLETE_BIT)) {
			/* Process completed ADC conversion */

			/* Disable the conversion complete flag */
			_ADC_result &= ~(1 << ADC_CONVERSION_COMPLETE_BIT);

			uint8_t task = _task & 0x3f;
			if (task == TASK_ADC_TEMP_0) {
				/* Store the ADC value */
				temperature_ADC[0] = _ADC_result;

				/* Next measurement is humidity sensor 0 */
				ADC_select_channel_2();
			} else if (task == TASK_ADC_HUM_0) {
				/* Store the ADC value */
				humidity_ADC[0] = _ADC_result;

				/* Next measurement is temperature sensor 0 */
				ADC_select_channel_diff_1_0_10x();
			}
		} else if (rx_complete) {
			if (strcmp((const char *) rx_buffer[rx_buffer_sel], "GET:TEMPERATURE") == 0) {
				interrupts_suspend();
				int16_t adc_val = (int16_t) temperature_ADC[0];
				interrupts_resume();
				adc_val /= 64;
				int16_t temperature = temperature_ADC_Pt1000_to_temp(adc_val);
				sprintf((char *) tx_buffer, "Temperature: %i/100 C\n", temperature);
				USART_send_bytes((uint8_t *) tx_buffer, strlen((const char *) tx_buffer));
			} else if (strcmp((const char *) rx_buffer[rx_buffer_sel], "GET:HUMIDITY") == 0) {
				interrupts_suspend();
				uint16_t adc_val2 = humidity_ADC[0];
				interrupts_resume();
				adc_val2 >>= 6;
				sprintf((char *) tx_buffer, "Humidity: %i %%\n", honeywell_convert_ADC_to_RH(adc_val2));
				USART_send_bytes((uint8_t *) tx_buffer, strlen((const char *) tx_buffer));
			} else {
				/* Shorten the command such that the error message does not overflow */
				rx_buffer[rx_buffer_sel][RX_BUFFER_LENGTH - strlen("Invalid command: \n") - 1] = '\0';

				/* Create the error message */
				sprintf((char *) tx_buffer, "Invalid command: %s\n", (const char *) rx_buffer[rx_buffer_sel]);

				/* Transmit the error message */
				USART_send_bytes((uint8_t *) tx_buffer, strlen((const char *) tx_buffer));
			}
			memset((void *) rx_buffer[rx_buffer_sel], '\0', RX_BUFFER_LENGTH);
			rx_complete = 0;
		}
	} /* Main loop */

	/* The program will never reach this point. */
	return 0;
}
