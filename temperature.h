#ifndef __TEMPERATURE_H__
#define __TEMPERATURE_H__

#include <stdint.h>

#define TEMPERATURE_NUMBER_OF_ADC	1
extern int16_t temperature_ADC [TEMPERATURE_NUMBER_OF_ADC];
extern int16_t temperature_to_ADC_Pt1000_lookup [];

#define temperature_to_ADC_Pt1000_lookup_index(T)	(((T) + 8192) >> 10)
#define temperature_ADC_Pt1000_to_temp_lookup_index(A)	(((A) + 512) >> 6)

int16_t temperature_to_ADC_Pt1000(int16_t temperature);
int16_t temperature_ADC_Pt1000_to_temp(int16_t A);

#endif /* __TEMPERATURE_H__ */
