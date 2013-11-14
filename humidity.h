#ifndef __HUMIDITY_H__
#define __HUMIDITY_H__

#include <stdint.h>

#define HUMIDITY_NUMBER_OF_ADC	1
extern int16_t humidity_ADC [HUMIDITY_NUMBER_OF_ADC];

uint8_t honeywell_convert_ADC_to_RH(uint16_t adc);

#endif /* __HUMIDITY_H__ */
