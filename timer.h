#ifndef __TIMER_H__
#define __TIMER_H__

#define NUMBER_OF_TASKS	3
#define TASK_ADC_TEMP_0	0
#define TASK_ADC_HUM_0	1
#define TASK_PID	2

#define TASK_START	0x80
#define TASK_FINISHED	0x40
extern volatile uint8_t _task;	///< Selects the current task measurement plus flags

void timer_8bit_cnt0_init(void);
void timer_16bit_cnt1_init(void);

#endif /* __TIMER_H__ */
