#include "humidity.h"

uint8_t honeywell_convert_ADC_to_RH(uint16_t adc)
{
	/* Prevent integer overflow */
	uint16_t tmp;
	tmp = adc * 5;
	if (tmp < 802)
		return 0;

	/* Simplified calculation with integers */
	/* Original equation: Vout = (Vsupply) * (0.0062 * RH + 0.16) */
	/* FIXME: Add temperature compensation: True RH = RH / (1.0546 * 0.00216 * T), T in degrees Celsius */
	return (uint8_t)((adc * 5 - 802) >> 5);
}
