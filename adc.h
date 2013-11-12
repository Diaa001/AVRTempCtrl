#ifndef __ADC_H__
#define __ADC_H__

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
