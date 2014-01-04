#ifndef __TIMER_H__
#define __TIMER_H__

#define NUMBER_OF_TASKS	16
#define TASK_ADC_TEMP_0	0	///< Task that measures the temperature with temperature sensor 0
#define TASK_ADC_TEMP_1	1	///< Task that measures the temperature with temperature sensor 1
#define TASK_ADC_HUM_0	2	///< Task that measures the humidity with humidity sensor 0
#define TASK_PID	3	///< Task that performs the PID calculation and PWM
#define TASK_DISPLAY	4	///< Task that updates the 7 segment display
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

#define TASK_START	0x80
#define TASK_FINISHED	0x40
extern volatile uint8_t _task;	///< Selects the current task measurement plus flags

void timer_8bit_cnt0_init(void);
void timer_16bit_cnt1_init(void);

#endif /* __TIMER_H__ */
