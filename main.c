/**
	\file
	\ingroup Main
	\brief Definition of the main routine with periphery initialization and main loop of execution

	\addtogroup Main Main routine
	\brief The main routine initializes and processes all tasks

	The main routine initializes the microcontroller and all its peripherals (internal and
	external).
	When this is completed it enters an infinite loop where it awaits assignment of tasks through
	interrupts.

	The interrupt handlers all set a flag that indicate an action to be taken.
	In every iteration of the main loop each of these flags is checked and if
	one of the flags is set some code block is executed.
	Then the next loop iteration occurs.
	The flag checking is organised in a very large <tt>if-elseif</tt> statement.
	The order of the \c if and \c elseif statements is based on the priority of the
	individual tasks.

	The code in the main() routine often uses helper functions defined in files other than main.c.
	This helps keeping the length of the routine from being over excessive.

	\warning The execution inside the main() function and the functions called therein can be
	interrupted at any time by an interrupt handler.
	After the interrupt handler finishes the execution within the main() function resumes.

	\warning Care must be taken when accesing variables that can be changed in interrupt handlers.
	See \ref Interrupts for details.
*/

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

#define SUBSTR(A, B)		((A) + strlen(B))					///< Returns the pointer within A offset by the length of the string B
#define EQ_CMD(A, B)		(strncmp((A), (B), strlen(B)) == 0)			///< Tests whether string A matches B for the length of string B
#define EQ_SUBCMD(A, C, B)	(strncmp(SUBSTR((A), (C)), (B), strlen(B)) == 0)	///< Tests whether string B matches A, offset by the length of string C

#define PID_CTRL_COOLING	0		///< PID controller state that represents cooling
#define PID_CTRL_HEATING	1		///< PID controller state that represents heating
#define PID_CTRL_OFF		2		///< PID controller state that represents control off
pidData_t PID_controller_settings[2];		///< Structures holding the PID controller parameters, the integral, and such
uint8_t PID_controller_state = PID_CTRL_OFF;	///< Controller mode: PID_CTRL_OFF, PID_CTRL_COOLING, or PID_CTRL_HEATING
int16_t PID_controller_setpoint_T;		///< Setpoint of the PID controller in degrees Celsius (times 100)
int16_t PID_controller_setpoint_ADC;		///< Setpoint of the PID controller in ADC units

uint8_t alarm_state;				///< Alarm state. Zero if no alarm, non-zero when an alarm occurs. An alarm must be acknowledged by the user.

/**
	Checks whether an alarm situation exists by reading sensors and interlock states
	\return 1 if an alarm situation exists, 0 otherwise
 */
uint8_t check_alarm(void)
{
	/* Read sensor values */
	uint16_t adc_val = humidity_ADC[0];
	adc_val >>= 6;
	uint8_t humidity = honeywell_convert_ADC_to_RH(adc_val);

	uint8_t water_interlock_open = (PIND & (1 << PD2));

	/* Check values against limits */
	return humidity > 60 || water_interlock_open;
}

/**
	\brief Main program routine

	Program entry function that initializes all peripherals and then enters an infinite loop
	where it handles all of the scheduled and unscheduled tasks.
	See \ref Main "here" for details.

	\return This function should never return.
	\ingroup Main
*/
int main (void) {
	/* Set clock to 4 MHz by setting the clock division (prescale) to 2 */
	clock_set_prescale_2();

	/* Turn on power to the humidity sensor */
	DDRB |= (1 << PB2);
	PORTB &= ~(1 << PB2);
	DDRA |= (1 << PA6);
	PORTA |= (1 << PA6);

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
	PID_controller_setpoint_ADC = temperature_to_ADS1248(PID_controller_setpoint_T);

	/* Enable the rotary encoder */
	encoder_init();

	/* Set the water interlock pin as input with pullup resistor */
	DDRD &= ~(1 << PD2);
	PORTD |= (1 << PD2);

	/* Debug output to visualize the time each (scheduled) task requires */
	DDRA |= (1 << PA0);

	interrupts_on();

	int8_t ADS1248_channel;

	/* Main loop */
	while(1) {
		/* Actions sorted by priority */
		if (_task & TASK_START) {
			PORTA |= (1 << PA0);
			/* Process scheduled tasks */
			uint8_t task = _task & 0x3f;
			if (task == TASK_ADC_TEMP_0) {
				temperature_ADS1248_start_conversion(3);
			} else if (task == TASK_ADC_HUM_0) {
				/* Start a conversion. ADC input was already selected after last conversion completed. */
				ADC_start_conversion();
			} else if (task == TASK_PID) {
				uint8_t active_PID = PID_controller_state;

				/* Determine the active PID controller. If no PID is active select one to continue updating the values. */
				if (active_PID == PID_CTRL_OFF)
					active_PID = PID_CTRL_COOLING;

				/* Determine the controller output (or if it it is off, at least update the values) */
				int16_t controller_output;
				controller_output = pid_Controller(PID_controller_setpoint_ADC, temperature_ADC[0], &PID_controller_settings[active_PID]);

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

				/* The alarm automatically turns off the PID controller, however ensure zero PWM output here anyway */
				if (!alarm_state)
					/* Set the pulse width modulation (PWM) output */
					OCR1B = pwm;
				else
					OCR1B = 0;
			} else if (task == TASK_DISPLAY) {
				interrupts_suspend();
				int16_t adc_val = (int16_t) temperature_ADC[0];
				interrupts_resume();
				int16_t temperature = temperature_ADS1248_to_temp(adc_val);

				interrupts_suspend();
				uint16_t adc_val2 = humidity_ADC[0];
				interrupts_resume();
				adc_val2 >>= 6;
				uint8_t humidity = honeywell_convert_ADC_to_RH(adc_val2);

				display_temperature(temperature);
				display_humidity(humidity);
			} else if (task == TASK_ALARM) {
				/* Only execute alarm checking when the controller is on */
				if (PID_controller_state != PID_CTRL_OFF && check_alarm()) {
					/* Turn the alarm on and stop the PID controller */
					alarm_state = 1;
					PID_controller_state = PID_CTRL_OFF;

					/* Turn off the PWM signal, i.e. turn of cooling / heating power */
					OCR1B = 0;

					/* Indicate the alarm with the ERROR LED */
					LED_set(LED_ERROR, LED_ON);
				}
			}
			/* Disable the task flags */
			_task = task;
		} else if ((ADS1248_channel = temperature_ADS1248_ready()) >= 0) {
			if (ADS1248_channel == 3)
				temperature_ADC[0] = temperature_ADS1248_read_result(ADS1248_channel);
		} else if (_ADC_result & (1 << ADC_CONVERSION_COMPLETE_BIT)) {
			/* Disable the conversion complete flag */
			interrupts_suspend();
			_ADC_result &= ~(1 << ADC_CONVERSION_COMPLETE_BIT);
			uint16_t result = _ADC_result;
			interrupts_resume();

			/* Process completed ADC conversion */
			uint8_t task = _task & 0x3f;
			if (task == TASK_ADC_HUM_0) {
				/* Store the ADC value */
				humidity_ADC[0] = result;

				/* Next measurement is humidity sensor 0 (again) */
				// ADC_select_channel_7();
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
				PID_controller_setpoint_ADC = temperature_to_ADS1248(PID_controller_setpoint_T);
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
		} else if (_rx_complete) {
			char * cmd = (char *) _rx_buffer[_rx_buffer_sel];
			if (EQ_CMD(cmd, ":SET")) {
				uint8_t alarm_error = 0;
				if (EQ_SUBCMD(cmd, ":SET", ":SETPOINT")) {
					int16_t temperature;
					if (temperature_string_to_temp(SUBSTR(cmd, ":SET:SETPOINT "), &temperature)) {
						PID_controller_setpoint_T = temperature;
						PID_controller_setpoint_ADC = temperature_to_ADS1248(PID_controller_setpoint_T);
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
				} else if (EQ_SUBCMD(cmd, ":SET", ":STATE:CTRL")) {
					if (EQ_SUBCMD(cmd, ":SET:STATE:CTRL ", "OFF")) {
						PID_controller_state = PID_CTRL_OFF;
						LED_set(LED_ACTIVE, LED_OFF);
					} else if (EQ_SUBCMD(cmd, ":SET:STATE:CTRL ", "COOL")) {
						/* Do not allow cooling in alarm state */
						if (alarm_state || check_alarm()) {
							alarm_state = 1;
							alarm_error = 1;
						}

						/* Turn on cooling */
						PID_controller_state = PID_CTRL_COOLING;
						LED_set(LED_ACTIVE, LED_ON);
					} else if (EQ_SUBCMD(cmd, ":SET:STATE:CTRL ", "HEAT")) {
						/* Do not allow heating in alarm state */
						if (alarm_state || check_alarm()) {
							alarm_state = 1;
							alarm_error = 1;
						}

						/* Turn on heating */
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
				if (!alarm_error)
					USART_send_bytes((const uint8_t *) "OK\n", 3);
				else
					USART_send_bytes((const uint8_t *) "ERROR: ALARM STATE\n", 19);
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
					sprintf((char * ) tx_buffer, "%hu/%hu\n", OCR1B, OCR1A);
				} else if (EQ_SUBCMD(cmd, ":GET", ":TEMPERATURE0")) {
					interrupts_suspend();
					int16_t adc_val = (int16_t) temperature_ADC[0];
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
				} else if (EQ_SUBCMD(cmd, ":GET", ":HUMIDITY0")) {
					interrupts_suspend();
					uint16_t adc_val2 = humidity_ADC[0];
					interrupts_resume();
					adc_val2 >>= 6;
					sprintf((char *) tx_buffer, "%i %%\n", honeywell_convert_ADC_to_RH(adc_val2));
				} else if (EQ_SUBCMD(cmd, ":GET", ":STATE")) {
					if (EQ_SUBCMD(cmd, ":GET:STATE", ":CTRL")) {
						/* Check alarm state */
						if (alarm_state)
							strcpy((char *) tx_buffer, "ALARM\n");
						else if (PID_controller_state == PID_CTRL_OFF)
							strcpy((char *) tx_buffer, "OFF\n");
						else if (PID_controller_state == PID_CTRL_COOLING)
							strcpy((char *) tx_buffer, "COOL\n");
						else if (PID_controller_state == PID_CTRL_HEATING)
							strcpy((char *) tx_buffer, "HEAT\n");
					} else if (EQ_SUBCMD(cmd, ":GET:STATE", ":INTERLOCK")) {
							/* Get the state of the water interlock */
							if (check_alarm()) {
								strcpy((char *) tx_buffer, "UNSAFE\n");
							} else {
								strcpy((char *) tx_buffer, "SAFE\n");
							}
					} else if (EQ_SUBCMD(cmd, ":GET:STATE", ":ALARM")) {
							if (alarm_state) {
								strcpy((char *) tx_buffer, "ON\n");
							} else {
								strcpy((char *) tx_buffer, "OFF\n");
							}
					}
				} else if (EQ_SUBCMD(cmd, ":GET", ":STEADY")) {
					if (alarm_state)
						strcpy((char *) tx_buffer, "ALARM\n");
					else
						strcpy((char *) tx_buffer, "NO\n");
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
				} else if (EQ_SUBCMD(cmd, ":RESET", ":ALARM")) {
					alarm_state = 0;
					LED_set(LED_ERROR, LED_OFF);
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
					PID_controller_setpoint_ADC = temperature_to_ADS1248(PID_controller_setpoint_T);
					PID_controller_settings[0].P_Factor = eeprom_read_word(EE_CTRL_KP0);
					PID_controller_settings[0].I_Factor = eeprom_read_word(EE_CTRL_KI0);
					PID_controller_settings[0].D_Factor = eeprom_read_word(EE_CTRL_KD0);
					PID_controller_settings[1].P_Factor = eeprom_read_word(EE_CTRL_KP1);
					PID_controller_settings[1].I_Factor = eeprom_read_word(EE_CTRL_KI1);
					PID_controller_settings[1].D_Factor = eeprom_read_word(EE_CTRL_KD1);
				} else if (EQ_SUBCMD(cmd, ":RECALL", ":SETPOINT")) {
					PID_controller_setpoint_T = eeprom_read_word(EE_CTRL_SETPOINT);
					PID_controller_setpoint_ADC = temperature_to_ADS1248(PID_controller_setpoint_T);
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
			memset((void *) _rx_buffer[_rx_buffer_sel], '\0', RX_BUFFER_LENGTH);
			_rx_complete = 0;
		}
		PORTA &= ~(1 << PA0);
	} /* Main loop */

	/* The program will never reach this point. */
	return 0;
}
