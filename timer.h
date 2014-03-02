/**
	\file
	\ingroup Timer
	\brief Declarations of functions, variables, and constants related to the timers of the microcontroller

	\addtogroup Timer
	\brief Timers for scheduling and temperature controller output

	Timers are part of the microcontroller and are used to make real-time measurements.
	They keep on counting no matter regardless of the CPU being busy.
	There is a total of 3 such counters:
	- Timer 0 is a 8 bit timer
	- Timer 1 is a 16 bit timer
	- Timer 2 is a 8 bit timer

	Each of the timers has two output compare registers that check whether the timer
	value equals their set values.
	If that happens two things can be configured to happen:
	- an interrupt
	- toggling of one microcontroller pin

	Additionally there can be an interrupt when the counter overflows.

	There are different ways of how and when to toggle the pins associated with
	the output compare registers.
	Multiple ways exist to implement pulse width modulation (PWM).

	Timer 0 is used as interrupt source for the scheduler (see \ref Interrupts).

	Timer 1 is used as pulse width modulation output for the temperature controller.

	\{
 */

#ifndef __TIMER_H__
#define __TIMER_H__

/* *** Timer constants ************************************************************************************************************************************* */

#define NUMBER_OF_TASKS	16	///< The total number of scheduled tasks
#define TASK_ADC_TEMP_0	0	///< Task that measures the temperature with temperature sensor 0
#define TASK_ADC_HUM_0	1	///< Task that measures the humidity with humidity sensor 0
#define TASK_PID	2	///< Task that performs the PID calculation and PWM
#define TASK_DISPLAY	3	///< Task that updates the 7 segment display
#define TASK_ALARM	4	///< Task that checks for the alarm state
#define TASK_IDLE_10	5	///< Dummy task that does not process anything
#define TASK_IDLE_9	6	///< Dummy task that does not process anything
#define TASK_IDLE_8	7	///< Dummy task that does not process anything
#define TASK_IDLE_7	8	///< Dummy task that does not process anything
#define TASK_IDLE_6	9	///< Dummy task that does not process anything
#define TASK_IDLE_5	10	///< Dummy task that does not process anything
#define TASK_IDLE_4	11	///< Dummy task that does not process anything
#define TASK_IDLE_3	12	///< Dummy task that does not process anything
#define TASK_IDLE_2	13	///< Dummy task that does not process anything
#define TASK_IDLE_1	14	///< Dummy task that does not process anything
#define TASK_IDLE_0	15	///< Dummy task that does not process anything

#define TASK_START	0x80	///< Flag that indicates the start of a task
#define TASK_FINISHED	0x40	///< Flag that indicates that a task is finished

/* *** Timer variables ************************************************************************************************************************************* */

extern volatile uint8_t _task;	///< Indicates the current task as well as flags

/* *** Timer functions ************************************************************************************************************************************* */

void timer_8bit_cnt0_init(void);	///< Initializes 8 bit timer 0
void timer_16bit_cnt1_init(void);	///< Initializes 16 bit timer 1

/** \} */

#endif /* __TIMER_H__ */
