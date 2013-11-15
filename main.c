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
#include "pid.h"

#define PID_CTRL_COOLING	0
#define PID_CTRL_HEATING	1
#define PID_CTRL_OFF		2
pidData_t PID_controller_settings[2];	///< Structures holding the PID controller parameters, the integral, and such
uint8_t PID_controller_state;		///< Controller mode: PID_CTRL_OFF, PID_CTRL_COOLING, or PID_CTRL_HEATING
int16_t PID_controller_setpoint;	///< Setpoint of the PID controller in ADC units

int main (void) {
	/* Set clock to 4 MHz by setting the clock division (prescale) to 2 */
	clock_set_prescale_2();

	USART_init();
	timer_8bit_cnt0_init();
	timer_16bit_cnt1_init();
	ADC_init();

	/* Initialize the PID controller data structures (one for heating and one for cooling) */
	pid_Init(128, 0, 0, &PID_controller_settings[PID_CTRL_COOLING]);
	pid_Init(128, 0, 0, &PID_controller_settings[PID_CTRL_HEATING]);

	DDRD |= (1 << PD6);

	interrupts_on();

	/* Main loop */
	while(1) {
		/* Actions sorted by priority */
		if (_task & 0xc0) {
			PORTD |= (1 << PD6);
			/* Process scheduled tasks */
			uint8_t task = _task & 0x3f;
			if (task == TASK_ADC_TEMP_0 || task == TASK_ADC_HUM_0) {
				/* Start a conversion. ADC input was already selected after last conversion completed. */
				ADC_start_conversion();
			} else if (task == TASK_PID) {
				uint8_t active_PID = PID_controller_state;

				/* Determine the active PID controller. If no PID is active select one to continue updating the values. */
				if (active_PID == PID_CTRL_OFF)
					active_PID = PID_CTRL_COOLING;

				/* Determine the controller output (or if it it is off, at least update the values) */
				int16_t controller_output;
				controller_output = pid_Controller(PID_controller_setpoint, temperature_ADC[0] >> 6, &PID_controller_settings[active_PID]);

				/* Set the controller output sign (heating/cooling) or set it to zero when it's off */
				if (PID_controller_state == PID_CTRL_OFF)
					controller_output = 0;
				else if (PID_controller_state == PID_CTRL_COOLING)
					/* Cooling requires reversed mode (output larger than zero when temperature error smaller than zero) */
					controller_output = -controller_output;

				/* Limit the controller output to positive values */
				if (controller_output < 0)
					controller_output = 0;

				/* Calculate the duty cycle for the PWM. Use a large integer because the multiplication exceeds the 16 bit range. */
				uint32_t tmp = (((uint32_t) controller_output) * OCR1A + MAX_INT) >> 15;

				/* Set the pulse width modulation (PWM) output */
				OCR1B = (uint16_t) tmp;
			}
			/* Disable the task flags */
			_task = task;
		} else if (_ADC_result & (1 << ADC_CONVERSION_COMPLETE_BIT)) {
			PORTD |= (1 << PD6);
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
		PORTD &= ~(1 << PD6);
	} /* Main loop */

	/* The program will never reach this point. */
	return 0;
}
