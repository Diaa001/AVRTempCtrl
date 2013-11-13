#ifndef __ADC_H__
#define __ADC_H__

#define ADC_NUM_INPUTS	2
#define ADC_TEMP_0	0
#define ADC_HUM_0	1

#define ADC_FLAG_QUEUE_START	0x80
#define ADC_FLAG_QUEUE_FINISHED	0x40
extern volatile uint8_t _ADC_queue_index;		///< Selects the current ADC measurement plus flags
extern volatile uint16_t _ADC_results [ADC_NUM_INPUTS];	///< Result of the ADC measurement corresponding to the flags in _adc_queue

#define ADC_start_conversion() {ADCSRA |= (1 << ADSC);}
#define ADC_wait_for_conversion() {while (ADCSRA & (1 << ADSC));}
#define ADC_get_result() ADC

#define ADC_select_channel_2() {\
	ADMUX &= ~(0x1f);\
	ADMUX |= (1 << MUX1);\
}

#define ADC_select_channel_diff_1_0_10x() {\
	ADMUX &= ~(0x1f);\
	ADMUX |= (1 << MUX3);\
	ADMUX |= (1 << MUX0);\
}

void ADC_init(void);

#endif /* __ADC_H__ */
