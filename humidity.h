#ifndef __HUMIDITY_H__
#define __HUMIDITY_H__

#include <stdint.h>

uint8_t honeywell_convert_ADC_to_RH(uint16_t adc);

#endif /* __HUMIDITY_H__ */
