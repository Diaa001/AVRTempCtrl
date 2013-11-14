#include <stdint.h>
#include "temperature.h"

int16_t temperature_ADC [TEMPERATURE_NUMBER_OF_ADC];

/* ADC values of Pt1000 measurements for temperatures -80, -72, -64, -56, -48,
   -40, -32, -24, -16, -8, +0, +8, +16, +24, +32, +40, +48, +56, +64, +72, +80 */
int16_t temperature_to_ADC_Pt1000_lookup [] = {
	-19126, -17341, -15624, -13970, -12377, -10839, -9355, -7921, -6535, -5195, -3897,
	-2641, -1424, -244, +900, +2011, +3088, +4134, +5151, +6139, +7099
};

int16_t temperature_to_ADC_Pt1000(int8_t temperature)
{
	int8_t a;
	int16_t A, B;

	/* Get the index of the temperature in the table equal or just smaller
	   to 'temperature' */
	int8_t index = temperature_to_ADC_Pt1000_lookup_index(temperature);

	/* Temperature of the table entry */
	a = (index << 3) - 80;

	/* ADC value at temperature a */
	A = temperature_to_ADC_Pt1000_lookup[index];

	/* ADC value at temperature a + 8 (next table entry) */
	B = temperature_to_ADC_Pt1000_lookup[index + 1];

	/* Interpolate (+4: for rounding) */
	return A + (((B - A) * (temperature - a) + 4) >> 3);
}
