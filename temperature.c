#include <stdint.h>
#include "temperature.h"

int16_t temperature_ADC [TEMPERATURE_NUMBER_OF_ADC];

/* ADC values of Pt1000 measurements for temperatures -80, -72, -64, -56, -48,
   -40, -32, -24, -16, -8, +0, +8, +16, +24, +32, +40, +48, +56, +64, +72, +80 */
int16_t temperature_to_ADC_Pt1000_lookup [] = {
	-598, -542, -488, -437, -387, -339, -292, -248, -204, -162, -122, -83,
	-44, -8, +28, +63, +97, +129, +161, +192, +222
};

/* Temperature values (times 100) of Pt1000 measurements for ADC values -512, -448,
   -384, -320, -256, -192, -128, -64, +0, +64, +128, +192, +256, +320, +384, +448, +512 */
int16_t temperature_ADC_Pt1000_to_temp_lookup [] = {
	-6763, -5783, -4756, -3681, -2554, -1369, -124, +1186, +2568,
	+4027, +5570, +7204, +8938, +10781, +12744, +14839, +17080
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

int16_t temperature_ADC_Pt1000_to_temp(int16_t ADC)
{
	int16_t a;
	int16_t A, B;

	/* Get the index of the ADC value in the table equal or just smaller to 'ADC' */
	int8_t index = temperature_ADC_Pt1000_to_temp_lookup_index(ADC);

	/* ADC value of the table entry */
	a = (index << 6) - 512;

	/* Temperature (x100) at ADC value a */
	A = temperature_ADC_Pt1000_to_temp_lookup[index];

	/* Temperature (x100) at ADC value a + 64 (next table entry) */
	B = temperature_ADC_Pt1000_to_temp_lookup[index + 1];

	/* Interpolate (+32: for rounding). Use 32 bit integers because intermediate values are large. */
	int32_t tmp = A + (((((int32_t) B) - A) * (ADC - a) + 32) >> 6);
	return (uint16_t) tmp;
}
