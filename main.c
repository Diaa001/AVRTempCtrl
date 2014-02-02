#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>

#include <util/delay.h>

#include "usart.h"
#include "timer.h"
#include "adc.h"
#include "clock.h"
#include "temperature.h"
#include "humidity.h"
#include "interrupt.h"
#include "pid.h"
#include "eeprom.h"
#include "encoder.h"
#include "spi.h"
#include "display.h"
#include "led.h"
#include "buttons.h"

#define SUBSTR(A, B)		((A) + strlen(B))
#define EQ_CMD(A, B)		(strncmp((A), (B), strlen(B)) == 0)
#define EQ_SUBCMD(A, C, B)	(strncmp(SUBSTR((A), (C)), (B), strlen(B)) == 0)

#define PID_CTRL_COOLING	0
#define PID_CTRL_HEATING	1
#define PID_CTRL_OFF		2
pidData_t PID_controller_settings[2];		///< Structures holding the PID controller parameters, the integral, and such
uint8_t PID_controller_state = PID_CTRL_OFF;	///< Controller mode: PID_CTRL_OFF, PID_CTRL_COOLING, or PID_CTRL_HEATING
int16_t PID_controller_setpoint_T;		///< Setpoint of the PID controller in degrees Celsius (times 100)
int16_t PID_controller_setpoint_ADC;		///< Setpoint of the PID controller in ADC units

int main (void) {
	/* Set clock to 4 MHz by setting the clock division (prescale) to 2 */
	clock_set_prescale_2();

	USART_init();
	timer_8bit_cnt0_init();
	timer_16bit_cnt1_init();
	LED_init();
	buttons_init();
	ADC_init();
	SPI_init();

	/* Delay to allow slave devices to start up */
	_delay_ms(10);

	/* Initialize the ADCs for Pt1000 temperature measurements */
	temperature_ADS1248_init();

	/* Initialize the 7 segment display controller */
	display_init();

	/* Initialize the PID controller data structures (one for heating and one for cooling) */
	/* Get the initial values from the EEPROM */
	int16_t kp, ki, kd;
	kp = eeprom_read_word(EE_CTRL_KP0);
	ki = eeprom_read_word(EE_CTRL_KI0);
	kd = eeprom_read_word(EE_CTRL_KD0);
	pid_Init(kp, ki, kd, &PID_controller_settings[PID_CTRL_COOLING]);
	kp = eeprom_read_word(EE_CTRL_KP1);
	ki = eeprom_read_word(EE_CTRL_KI1);
	kd = eeprom_read_word(EE_CTRL_KD1);
	pid_Init(kp, ki, kd, &PID_controller_settings[PID_CTRL_HEATING]);

	/* Set the setpoint from the EEPROM memory */
	PID_controller_setpoint_T = eeprom_read_word(EE_CTRL_SETPOINT);
	PID_controller_setpoint_ADC = temperature_to_ADC_Pt1000(PID_controller_setpoint_T);

	/* Enable the rotary encoder */
	encoder_init();

	/* Debug output to visualize the time each (scheduled) task requires */
	DDRA |= (1 << PA7);

	interrupts_on();

	int8_t ADS1248_channel;

	/* Main loop */
	while(1) {
		/* Actions sorted by priority */
		if (_task & 0xc0) {
			PORTB |= (1 << PA7);
			/* Process scheduled tasks */
			uint8_t task = _task & 0x3f;
			if (task == TASK_ADC_TEMP_0 || task == TASK_ADC_HUM_0) {
				/* Start a conversion. ADC input was already selected after last conversion completed. */
				ADC_start_conversion();
			} else if (task == TASK_ADC_TEMP_1) {
				temperature_ADS1248_start_conversion(0);
			} else if (task == TASK_PID) {
				uint8_t active_PID = PID_controller_state;

				/* Determine the active PID controller. If no PID is active select one to continue updating the values. */
				if (active_PID == PID_CTRL_OFF)
					active_PID = PID_CTRL_COOLING;

				/* Determine the controller output (or if it it is off, at least update the values) */
				int16_t controller_output;
				controller_output = pid_Controller(PID_controller_setpoint_ADC, temperature_ADC[0] >> 6, &PID_controller_settings[active_PID]);

				/* Set the controller output sign (heating/cooling) or set it to zero when it's off */
				if (PID_controller_state == PID_CTRL_OFF)
					controller_output = 0;
				else if (PID_controller_state == PID_CTRL_COOLING)
					/* Cooling requires reversed mode (output larger than zero when temperature error smaller than zero) */
					controller_output = -controller_output;

				/* Calculate the duty cycle for the PWM. Negative values are not allowed and maximum controller_output is 32767.
				   Maximum allowable PWM value is 32768. */
				uint16_t pwm = 0;
				if (controller_output < 0)
					pwm = 0;
				else if (controller_output > 0)
					pwm = ((uint16_t) controller_output) + 1;

				/* Set the pulse width modulation (PWM) output */
				OCR1B = pwm;
			} else if (task == TASK_DISPLAY) {
				interrupts_suspend();
				int16_t adc_val = (int16_t) temperature_ADC[0];
				interrupts_resume();
				adc_val /= 64;
				int16_t temperature = temperature_ADC_Pt1000_to_temp(adc_val);

				interrupts_suspend();
				uint16_t adc_val2 = humidity_ADC[0];
				interrupts_resume();
				adc_val2 >>= 6;
				uint8_t humidity = honeywell_convert_ADC_to_RH(adc_val2);

				display_temperature(temperature);
				display_humidity(humidity);
			}
			/* Disable the task flags */
			_task = task;
		} else if ((ADS1248_channel = temperature_ADS1248_ready()) >= 0) {
			if (ADS1248_channel == 0)
				temperature_ADC[1] = temperature_ADS1248_read_result(ADS1248_channel);
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
		} else if (_encoder_increment) {
			/* _encoder_increment could change at this point through an interrupt */

			/* Save the rotary encoder increment quickly while suspending interrupts */
			interrupts_suspend();
			int8_t increment = _encoder_increment;
			_encoder_increment = 0;
			interrupts_resume();

			/* Test for the increment again because it could have been reset between the if
			   condition and the interrupt suspension. */
			if (increment) {
				/* Set the new PID controller setpoint using the increment */
				PID_controller_setpoint_T += increment * 25;
				PID_controller_setpoint_ADC = temperature_to_ADC_Pt1000(PID_controller_setpoint_T);
			}
		} else if (_button_state[BUTTON_CTRL] == (BUTTON_STATE_FLAG | BUTTON_CHANGED_FLAG)) {
			/* Turn off the changed flag of the button state */
			_button_state[BUTTON_CTRL] &= ~(BUTTON_CHANGED_FLAG);

			/* Toggle the PID controller state */
			if (PID_controller_state == PID_CTRL_OFF) {
				PID_controller_state = PID_CTRL_COOLING;
				LED_set(LED_ACTIVE, LED_ON);
			} else {
				PID_controller_state = PID_CTRL_OFF;
				LED_set(LED_ACTIVE, LED_OFF);
			}
		} else if (rx_complete) {
			char * cmd = (char *) rx_buffer[rx_buffer_sel];
			if (EQ_CMD(cmd, ":SET")) {
				if (EQ_SUBCMD(cmd, ":SET", ":SETPOINT")) {
					int16_t temperature;
					if (temperature_string_to_temp(SUBSTR(cmd, ":SET:SETPOINT "), &temperature)) {
						PID_controller_setpoint_T = temperature;
						PID_controller_setpoint_ADC = temperature_to_ADC_Pt1000(PID_controller_setpoint_T);
						PID_controller_settings[0].sumError = 0;
						PID_controller_settings[1].sumError = 0;
					} else {
						goto CMD_ERROR;
					}
				} else if (EQ_SUBCMD(cmd, ":SET", ":KP0")) {
					int16_t factor = atoi(SUBSTR(cmd, ":SET:KP0 "));
					PID_controller_settings[0].P_Factor = factor;
					PID_controller_settings[0].maxError = MAX_INT / (factor + 1);
				} else if (EQ_SUBCMD(cmd, ":SET", ":KP1")) {
					int16_t factor = atoi(SUBSTR(cmd, ":SET:KP1 "));
					PID_controller_settings[1].P_Factor = factor;
					PID_controller_settings[1].maxError = MAX_INT / (factor + 1);
				} else if (EQ_SUBCMD(cmd, ":SET", ":KI0")) {
					int16_t factor = atoi(SUBSTR(cmd, ":SET:KI0 "));
					PID_controller_settings[0].I_Factor = factor;
					PID_controller_settings[0].maxSumError = MAX_I_TERM / (factor + 1);
				} else if (EQ_SUBCMD(cmd, ":SET", ":KI1")) {
					int16_t factor = atoi(SUBSTR(cmd, ":SET:KI1 "));
					PID_controller_settings[1].I_Factor = factor;
					PID_controller_settings[1].maxSumError = MAX_I_TERM / (factor + 1);
				} else if (EQ_SUBCMD(cmd, ":SET", ":KD0")) {
					int16_t factor = atoi(SUBSTR(cmd, ":SET:KD0 "));
					PID_controller_settings[0].D_Factor = factor;
				} else if (EQ_SUBCMD(cmd, ":SET", ":KD1")) {
					int16_t factor = atoi(SUBSTR(cmd, ":SET:KD1 "));
					PID_controller_settings[1].D_Factor = factor;
				} else if (EQ_SUBCMD(cmd, ":SET", ":STATE")) {
					if (EQ_SUBCMD(cmd, ":SET:STATE ", "OFF")) {
						PID_controller_state = PID_CTRL_OFF;
						LED_set(LED_ACTIVE, LED_OFF);
					} else if (EQ_SUBCMD(cmd, ":SET:STATE ", "COOL")) {
						PID_controller_state = PID_CTRL_COOLING;
						LED_set(LED_ACTIVE, LED_ON);
					} else if (EQ_SUBCMD(cmd, ":SET:STATE ", "HEAT")) {
						PID_controller_state = PID_CTRL_HEATING;
						LED_set(LED_ACTIVE, LED_ON);
					} else {
						goto CMD_ERROR;
					}
					PID_controller_settings[0].sumError = 0;
					PID_controller_settings[1].sumError = 0;
				} else {
					goto CMD_ERROR;
				}
				USART_send_bytes((const uint8_t *) "OK\n", 3);
			} else if (EQ_CMD(cmd, ":GET")) {
				if (EQ_SUBCMD(cmd, ":GET", ":SETPOINT")) {
					/* Convert the temperature to a string */
					temperature_to_string(PID_controller_setpoint_T, tx_buffer);

					/* Add a newline after the number */
					uint8_t len = strlen(tx_buffer);
					tx_buffer[len + 0] = '\n';
					tx_buffer[len + 1] = '\0';
				} else if (EQ_SUBCMD(cmd, ":GET", ":KP0")) {
					sprintf((char * ) tx_buffer, "%i\n", PID_controller_settings[0].P_Factor);
				} else if (EQ_SUBCMD(cmd, ":GET", ":KP1")) {
					sprintf((char * ) tx_buffer, "%i\n", PID_controller_settings[1].P_Factor);
				} else if (EQ_SUBCMD(cmd, ":GET", ":KI0")) {
					sprintf((char * ) tx_buffer, "%i\n", PID_controller_settings[0].I_Factor);
				} else if (EQ_SUBCMD(cmd, ":GET", ":KI1")) {
					sprintf((char * ) tx_buffer, "%i\n", PID_controller_settings[1].I_Factor);
				} else if (EQ_SUBCMD(cmd, ":GET", ":KD0")) {
					sprintf((char * ) tx_buffer, "%i\n", PID_controller_settings[0].D_Factor);
				} else if (EQ_SUBCMD(cmd, ":GET", ":KD1")) {
					sprintf((char * ) tx_buffer, "%i\n", PID_controller_settings[1].D_Factor);
				} else if (EQ_SUBCMD(cmd, ":GET", ":INTEGRAL0")) {
					sprintf((char * ) tx_buffer, "%li\n", PID_controller_settings[0].sumError);
				} else if (EQ_SUBCMD(cmd, ":GET", ":INTEGRAL1")) {
					sprintf((char * ) tx_buffer, "%li\n", PID_controller_settings[1].sumError);
				} else if (EQ_SUBCMD(cmd, ":GET", ":PWM")) {
					sprintf((char * ) tx_buffer, "%i/%i\n", OCR1B, OCR1A);
				} else if (EQ_SUBCMD(cmd, ":GET", ":TEMPERATURE0")) {
					interrupts_suspend();
					int16_t adc_val = (int16_t) temperature_ADC[0];
					interrupts_resume();
					adc_val /= 64;
					int16_t temperature = temperature_ADC_Pt1000_to_temp(adc_val);

					/* Convert the temperature to a string */
					temperature_to_string(temperature, tx_buffer);

					/* Add a newline after the number */
					uint8_t len = strlen(tx_buffer);
					tx_buffer[len + 0] = '\n';
					tx_buffer[len + 1] = '\0';
				} else if (EQ_SUBCMD(cmd, ":GET", ":TEMPERATURE1")) {
					interrupts_suspend();
					int16_t adc_val = (int16_t) temperature_ADC[1];
					interrupts_resume();
					int16_t temperature = temperature_ADS1248_to_temp(adc_val);

					/* Convert the temperature to a string */
					temperature_to_string(temperature, tx_buffer);

					/* Add a newline after the number */
					uint8_t len = strlen(tx_buffer);
					tx_buffer[len + 0] = '\n';
					tx_buffer[len + 1] = '\0';
				} else if (EQ_SUBCMD(cmd, ":GET", ":TEMPADC0")) {
					interrupts_suspend();
					int16_t adc_val = (int16_t) temperature_ADC[0];
					interrupts_resume();
					sprintf((char *) tx_buffer, "%hi\n", adc_val);
				} else if (EQ_SUBCMD(cmd, ":GET", ":TEMPADC1")) {
					interrupts_suspend();
					int16_t adc_val = (int16_t) temperature_ADC[1];
					interrupts_resume();
					sprintf((char *) tx_buffer, "%hi\n", adc_val);
				} else if (EQ_SUBCMD(cmd, ":GET", ":HUMIDITY0")) {
					interrupts_suspend();
					uint16_t adc_val2 = humidity_ADC[0];
					interrupts_resume();
					adc_val2 >>= 6;
					sprintf((char *) tx_buffer, "%i %%\n", honeywell_convert_ADC_to_RH(adc_val2));
				} else if (EQ_SUBCMD(cmd, ":GET", ":STATE")) {
					if (PID_controller_state == PID_CTRL_OFF)
						strcpy((char *) tx_buffer, "OFF\n");
					else if (PID_controller_state == PID_CTRL_COOLING)
						strcpy((char *) tx_buffer, "COOL\n");
					else if (PID_controller_state == PID_CTRL_HEATING)
						strcpy((char *) tx_buffer, "HEAT\n");
				} else {
					goto CMD_ERROR;
				}
				USART_send_bytes((uint8_t *) tx_buffer, strlen((const char *) tx_buffer));
				/* Erase the transmit buffer */
				memset((void *) tx_buffer, '\0', TX_BUFFER_LENGTH);
			} else if (EQ_CMD(cmd, ":RESET")) {
				if (EQ_SUBCMD(cmd, ":RESET", ":INTEGRAL0")) {
					PID_controller_settings[0].sumError = 0;
				} else if (EQ_SUBCMD(cmd, ":RESET", ":INTEGRAL1")) {
					PID_controller_settings[1].sumError = 0;
				} else {
					goto CMD_ERROR;
				}
				USART_send_bytes((const uint8_t *) "OK\n", 3);
			} else if (EQ_CMD(cmd, ":SAVE")) {
				if (EQ_SUBCMD(cmd, ":SAVE", ":ALL")) {
					eeprom_write_word(EE_CTRL_SETPOINT, PID_controller_setpoint_T);
					eeprom_write_word(EE_CTRL_KP0, PID_controller_settings[0].P_Factor);
					eeprom_write_word(EE_CTRL_KI0, PID_controller_settings[0].I_Factor);
					eeprom_write_word(EE_CTRL_KD0, PID_controller_settings[0].D_Factor);
					eeprom_write_word(EE_CTRL_KP1, PID_controller_settings[1].P_Factor);
					eeprom_write_word(EE_CTRL_KI1, PID_controller_settings[1].I_Factor);
					eeprom_write_word(EE_CTRL_KD1, PID_controller_settings[1].D_Factor);
				} else if (EQ_SUBCMD(cmd, ":SAVE", ":SETPOINT")) {
					eeprom_write_word(EE_CTRL_SETPOINT, PID_controller_setpoint_T);
				} else if (EQ_SUBCMD(cmd, ":SAVE", ":KP0")) {
					eeprom_write_word(EE_CTRL_KP0, PID_controller_settings[0].P_Factor);
				} else if (EQ_SUBCMD(cmd, ":SAVE", ":KI0")) {
					eeprom_write_word(EE_CTRL_KI0, PID_controller_settings[0].I_Factor);
				} else if (EQ_SUBCMD(cmd, ":SAVE", ":KD0")) {
					eeprom_write_word(EE_CTRL_KD0, PID_controller_settings[0].D_Factor);
				} else if (EQ_SUBCMD(cmd, ":SAVE", ":KP1")) {
					eeprom_write_word(EE_CTRL_KP1, PID_controller_settings[1].P_Factor);
				} else if (EQ_SUBCMD(cmd, ":SAVE", ":KI1")) {
					eeprom_write_word(EE_CTRL_KI1, PID_controller_settings[1].I_Factor);
				} else if (EQ_SUBCMD(cmd, ":SAVE", ":KD1")) {
					eeprom_write_word(EE_CTRL_KD1, PID_controller_settings[1].D_Factor);
				} else {
					goto CMD_ERROR;
				}
				USART_send_bytes((const uint8_t *) "OK\n", 3);
			} else if (EQ_CMD(cmd, ":RECALL")) {
				if (EQ_SUBCMD(cmd, ":RECALL", ":ALL")) {
					PID_controller_setpoint_T = eeprom_read_word(EE_CTRL_SETPOINT);
					PID_controller_setpoint_ADC = temperature_to_ADC_Pt1000(PID_controller_setpoint_T);
					PID_controller_settings[0].P_Factor = eeprom_read_word(EE_CTRL_KP0);
					PID_controller_settings[0].I_Factor = eeprom_read_word(EE_CTRL_KI0);
					PID_controller_settings[0].D_Factor = eeprom_read_word(EE_CTRL_KD0);
					PID_controller_settings[1].P_Factor = eeprom_read_word(EE_CTRL_KP1);
					PID_controller_settings[1].I_Factor = eeprom_read_word(EE_CTRL_KI1);
					PID_controller_settings[1].D_Factor = eeprom_read_word(EE_CTRL_KD1);
				} else if (EQ_SUBCMD(cmd, ":RECALL", ":SETPOINT")) {
					PID_controller_setpoint_T = eeprom_read_word(EE_CTRL_SETPOINT);
					PID_controller_setpoint_ADC = temperature_to_ADC_Pt1000(PID_controller_setpoint_T);
				} else if (EQ_SUBCMD(cmd, ":RECALL", ":KP0")) {
					PID_controller_settings[0].P_Factor = eeprom_read_word(EE_CTRL_KP0);
				} else if (EQ_SUBCMD(cmd, ":RECALL", ":KI0")) {
					PID_controller_settings[0].I_Factor = eeprom_read_word(EE_CTRL_KI0);
				} else if (EQ_SUBCMD(cmd, ":RECALL", ":KD0")) {
					PID_controller_settings[0].D_Factor = eeprom_read_word(EE_CTRL_KD0);
				} else if (EQ_SUBCMD(cmd, ":RECALL", ":KP1")) {
					PID_controller_settings[1].P_Factor = eeprom_read_word(EE_CTRL_KP1);
				} else if (EQ_SUBCMD(cmd, ":RECALL", ":KI1")) {
					PID_controller_settings[1].I_Factor = eeprom_read_word(EE_CTRL_KI1);
				} else if (EQ_SUBCMD(cmd, ":RECALL", ":KD1")) {
					PID_controller_settings[1].D_Factor = eeprom_read_word(EE_CTRL_KD1);
				} else {
					goto CMD_ERROR;
				}
				USART_send_bytes((const uint8_t *) "OK\n", 3);
			} else {
				CMD_ERROR:
				/* Shorten the cmd such that the error message does not overflow */
				cmd[RX_BUFFER_LENGTH - strlen("Invalid command: \n") - 1] = '\0';

				/* Create the error message */
				sprintf((char *) tx_buffer, "Invalid command: %s\n", cmd);

				/* Transmit the error message */
				USART_send_bytes((uint8_t *) tx_buffer, strlen((const char *) tx_buffer));

				/* Erase the transmit buffer */
				memset((void *) tx_buffer, '\0', TX_BUFFER_LENGTH);
			}
			memset((void *) rx_buffer[rx_buffer_sel], '\0', RX_BUFFER_LENGTH);
			rx_complete = 0;
		}
		PORTB &= ~(1 << PA7);
	} /* Main loop */

	/* The program will never reach this point. */
	return 0;
}
