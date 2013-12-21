#include <stdint.h>
#include "temperature.h"

int16_t temperature_ADC [TEMPERATURE_NUMBER_OF_ADC];

/* ADC values of Pt1000 measurements for temperatures -8192, -7168, -6144, -5120,
   -4096, -3072, -2048, -1024, +0, +1024, +2048, +3072, +4096, +5120, +6144,
   +7168, +8192 (100 x degrees Celsius) */
int16_t temperature_to_ADC_Pt1000_lookup [] = {
	-611, -539, -471, -406, -344, -285, -228, -174,
	-122, -72, -24, +22, +67, +110, +151, +191, +229
};

/* Temperature values (times 100) of Pt1000 measurements for ADC values -512, -448,
   -384, -320, -256, -192, -128, -64, +0, +64, +128, +192, +256, +320, +384, +448, +512 */
int16_t temperature_ADC_Pt1000_to_temp_lookup [] = {
	-6763, -5783, -4756, -3681, -2554, -1369, -124, +1186, +2568,
	+4027, +5570, +7204, +8938, +10781, +12744, +14839, +17080
};

int16_t temperature_to_ADC_Pt1000(int16_t temperature)
{
	int16_t a;
	int16_t A, B;

	/* Get the index of the temperature in the table equal or just smaller
	   to 'temperature' */
	int8_t index = temperature_to_ADC_Pt1000_lookup_index(temperature);

	/* Temperature of the table entry */
	a = (index << 10) - 8192;

	/* ADC value at temperature a */
	A = temperature_to_ADC_Pt1000_lookup[index];

	/* ADC value at temperature a + 1024 (next table entry) */
	B = temperature_to_ADC_Pt1000_lookup[index + 1];

	/* Interpolate (+512: for rounding) */
	return A + (((B - A) * ((int32_t)(temperature - a)) + 512) >> 10);
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

uint8_t temperature_string_to_temp(const char * string, int16_t * temperature)
{
	/* Skip whitespace */
	while (string[0] != '\0' && (string[0] == ' ' || string[0] == '\t'))
		string++;

	/* End of string reached? */
	if (string[0] == '\0')
		return 0;

	/* Extract the sign */
	uint8_t sign = 0;
	if (string[0] == '-') {
		sign = 1;
		string++;
	} else if (string[0] == '+') {
		sign = 0;
		string++;
	} else if (string[0] != '.' && (string[0] < '0' || string[0] > '9')) {
		return 0;
	}

	*temperature = 0;
	while (string[0] >= '0' && string[0] <= '9') {
		*temperature *= 10;
		*temperature += string[0] - '0';
		string++;
	}

	/* Multiply the temperature by 100 to avoid fractional numbers */
	*temperature *= 100;

	/* No decimal point? */
	if (string[0] == '\0') {
		if (sign)
			*temperature = -(*temperature);
		return 1;
	} else if (string[0] == '.') {
		string++;
		/* Accept 2 decimal places */
		if (string[0] >= '0' && string[0] <= '9') {
			*temperature += 10 * (string[0] - '0');
			string++;
		}
		if (string[0] >= '0' && string[0] <= '9') {
			*temperature += string[0] - '0';
		}
		if (sign)
			*temperature = -(*temperature);
		return 1;
	}

	return 1;
}

void temperature_to_string(int16_t temperature, char * string)
{
	/* Determine the sign of the temperature */
	if (temperature < 0) {
		temperature = -temperature;
		string[0] = '-';
		string++;
	}

	/* Determine the number of characters required to print the temperature */
	uint8_t characters = 4;
	if (temperature >= 10000)
		characters = 6;
	else if (temperature >= 1000)
		characters = 5;

	/* Spell out the number */
	string[characters] = '\0';
	int8_t i;
	for (i = characters - 1; i >= 0; i--) {
		if (i == characters - 3) {
			string[i] = '.';
			continue;
		}
		int division = temperature / 10;
		string[i] = temperature - 10 * division + '0';
		temperature = division;
	}
}
