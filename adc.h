#ifndef __ADC_H__
#define __ADC_H__

#define ADC_CONVERSION_COMPLETE_BIT	0	///< The complete bit is on the very right because the ADC value is left aligned
extern volatile uint16_t _ADC_result;		///< Result of the latest ADC measurement plus flag for conversion complete

#define ADC_start_conversion() {ADCSRA |= (1 << ADSC);}
#define ADC_wait_for_conversion() {while (ADCSRA & (1 << ADSC));}
#define ADC_get_result() ADC

#define ADC_select_channel_2() {\
	ADMUX &= ~(0x1f);\
	ADMUX |= (1 << MUX1);\
}

#define ADC_select_channel_7() {\
	ADMUX &= ~(0x1f);\
	ADMUX |= (1 << MUX0) | (1 << MUX1) | (1 << MUX2);\
}

#define ADC_select_channel_diff_1_0_10x() {\
	ADMUX &= ~(0x1f);\
	ADMUX |= (1 << MUX3);\
	ADMUX |= (1 << MUX0);\
}

void ADC_init(void);

#endif /* __ADC_H__ */
