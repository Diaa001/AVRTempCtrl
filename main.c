#include <stdlib.h>
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
#include "eeprom.h"

#define SUBSTR(A, B)		((A) + strlen(B))
#define EQ_CMD(A, B)		(strncmp((A), (B), strlen(B)) == 0)
#define EQ_SUBCMD(A, C, B)	(strncmp(SUBSTR((A), (C)), (B), strlen(B)) == 0)

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
	PID_controller_setpoint = eeprom_read_word(EE_CTRL_SETPOINT);

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
			char * cmd = (char *) rx_buffer[rx_buffer_sel];
			if (EQ_CMD(cmd, ":SET")) {
				if (EQ_SUBCMD(cmd, ":SET", ":SETPOINT")) {
					int16_t setpoint = atoi(SUBSTR(cmd, ":SET:SETPOINT "));
					setpoint = temperature_to_ADC_Pt1000((int8_t)setpoint);
					PID_controller_settings[0].sumError = 0;
					PID_controller_settings[1].sumError = 0;
					PID_controller_setpoint = setpoint;
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
					} else if (EQ_SUBCMD(cmd, ":SET:STATE ", "COOL")) {
						PID_controller_state = PID_CTRL_COOLING;
					} else if (EQ_SUBCMD(cmd, ":SET:STATE ", "HEAT")) {
						PID_controller_state = PID_CTRL_HEATING;
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
					int16_t temperature = temperature_ADC_Pt1000_to_temp(PID_controller_setpoint);
					sprintf((char * ) tx_buffer, "%i/100\n", temperature);
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
					sprintf((char *) tx_buffer, "Temperature: %i/100 C\n", temperature);
				} else if (EQ_SUBCMD(cmd, ":GET", ":HUMIDITY0")) {
					interrupts_suspend();
					uint16_t adc_val2 = humidity_ADC[0];
					interrupts_resume();
					adc_val2 >>= 6;
					sprintf((char *) tx_buffer, "Humidity: %i %%\n", honeywell_convert_ADC_to_RH(adc_val2));
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
					eeprom_write_word(EE_CTRL_SETPOINT, PID_controller_setpoint);
					eeprom_write_word(EE_CTRL_KP0, PID_controller_settings[0].P_Factor);
					eeprom_write_word(EE_CTRL_KI0, PID_controller_settings[0].I_Factor);
					eeprom_write_word(EE_CTRL_KD0, PID_controller_settings[0].D_Factor);
					eeprom_write_word(EE_CTRL_KP1, PID_controller_settings[1].P_Factor);
					eeprom_write_word(EE_CTRL_KI1, PID_controller_settings[1].I_Factor);
					eeprom_write_word(EE_CTRL_KD1, PID_controller_settings[1].D_Factor);
				} else if (EQ_SUBCMD(cmd, ":SAVE", ":SETPOINT")) {
					eeprom_write_word(EE_CTRL_SETPOINT, PID_controller_setpoint);
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
				}
			} else if (EQ_CMD(cmd, ":RECALL")) {
				if (EQ_SUBCMD(cmd, ":RECALL", ":ALL")) {
					PID_controller_setpoint = eeprom_read_word(EE_CTRL_SETPOINT);
					PID_controller_settings[0].P_Factor = eeprom_read_word(EE_CTRL_KP0);
					PID_controller_settings[0].I_Factor = eeprom_read_word(EE_CTRL_KI0);
					PID_controller_settings[0].D_Factor = eeprom_read_word(EE_CTRL_KD0);
					PID_controller_settings[1].P_Factor = eeprom_read_word(EE_CTRL_KP1);
					PID_controller_settings[1].I_Factor = eeprom_read_word(EE_CTRL_KI1);
					PID_controller_settings[1].D_Factor = eeprom_read_word(EE_CTRL_KD1);
				} else if (EQ_SUBCMD(cmd, ":RECALL", ":SETPOINT")) {
					PID_controller_setpoint = eeprom_read_word(EE_CTRL_SETPOINT);
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
				}
			} else {
				CMD_ERROR:
				/* Shorten the cmd such that the error message does not overflow */
				cmd[RX_BUFFER_LENGTH - strlen("Invalid command: \n") - 1] = '\0';

				/* Create the error message */
				sprintf((char *) tx_buffer, "Invalid command: %s\n", cmd);

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
