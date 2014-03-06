/**
	\file
	\ingroup Humidity
	\brief Definition of functions and variables related to humidity sensors
 */

#include "humidity.h"
#include "lookup.h"

/* *** Humidity variables ********************************************************************************************************************************** */

int16_t humidity_ADC [HUMIDITY_NUMBER_OF_ADC];

/**
	\brief Lookup table used for dewpoint calculation

	Lookup table for H1(RH^2) (humidity value in %, squared) with logarithmic step size.
	First step of the dewpoint calculation.
	See \ref humidity_dewpoint() for details.
 */
static int16_t dewpoint_H1_lookup [] = {
	-2357,	// H1(1)
	-2179,	// H1(2)
	-2002,	// H1(4)
	-1824,	// H1(8)
	-1647,	// H1(16)
	-1470,	// H1(32)
	-1292,	// H1(64)
	-1115,	// H1(128)
	-937,	// H1(256)
	-760,	// H1(512)
	-582,	// H1(1024)
	-405,	// H1(2048)
	-227,	// H1(4096)
	-50,	// H1(8192)
	126	// H1(16384)
};

/**
	\brief Lookup table used for dewpoint calculation

	Lookup table for H2(T100) (temperature in Celsius, times 100).
	Second step of the dewpoint calculation.
	See \ref humidity_dewpoint() for details.
 */
static int16_t dewpoint_H2_lookup [] = {
	-9219,	// H2(-12288)
	-7787,	// H2(-11264)
	-6564,	// H2(-10240)
	-5507,	// H2(-9216)
	-4584,	// H2(-8192)
	-3771,	// H2(-7168)
	-3050,	// H2(-6144)
	-2406,	// H2(-5120)
	-1827,	// H2(-4096)
	-1304,	// H2(-3072)
	-829,	// H2(-2048)
	-396,	// H2(-1024)
	0,	// H2(0)
	365,	// H2(1024)
	701,	// H2(2048)
	1012,	// H2(3072)
	1301,	// H2(4096)
	1569,	// H2(5120)
	1820,	// H2(6144)
	2054,	// H2(7168)
	2274,	// H2(8192)
	2480,	// H2(9216)
	2674,	// H2(10240)
	2856,	// H2(11264)
	3029	// H2(12288)
};

/**
	\brief Lookup table used for dewpoint calculation

	Lookup table for Dp(H) (H = H1 + H2).
	Third step of the dewpoint calculation.
	See \ref humidity_dewpoint() for details.
 */
static int16_t dewpoint_Dp_lookup [] = {
	-14490,	// Dp(-13312)
	-14018,	// Dp(-12288)
	-13499,	// Dp(-11264)
	-12924,	// Dp(-10240)
	-12285,	// Dp(-9216)
	-11569,	// Dp(-8192)
	-10763,	// Dp(-7168)
	-9849,	// Dp(-6144)
	-8801,	// Dp(-5120)
	-7591,	// Dp(-4096)
	-6175,	// Dp(-3072)
	-4497,	// Dp(-2048)
	-2477,	// Dp(-1024)
	0,	// Dp(0)
	3113,	// Dp(1024)
	7140,	// Dp(2048)
	12554,	// Dp(3072)
	20218	// Dp(4096)
};

/* *** Humidity functions ********************************************************************************************************************************** */

/**
	Conversion of the ADC value into units of relative humidity in percent for the Honeywell HIH-4000-001 humidity sensor.
	The formula used is from the datasheet.
	There is some non-linearity in the analog output but because of an accuracy of 5 % this is not relevant.
	\param adc The ADC value to be converted into percent relative humidity.
	\return The converted value in percent relative humidity.

	\note Temperature compensation as specified in the datasheet should be used but is currently not implemented.
 */
uint8_t humidity_ADC_to_RH_honeywell(uint16_t adc)
{
	/* Prevent integer overflow */
	uint16_t tmp;
	tmp = adc * 5;
	if (tmp < 802)
		return 0;

	/* Simplified calculation with integers */
	/* Original equation: Vout = (Vsupply) * (0.0062 * RH + 0.16) */
	return (uint8_t)((adc * 5 - 802) >> 5);
}

/**
	Calculates the dewpoint from temperature and humidity.
	The formula used is from a Sensirion application note:
	"SHTxx Application Note Dew-point Calculation".
	It states that

		H = (log10(RH) - 2) / 0.4343 + (17.62 * T) / (243.12 + T);
		Dp = 243.12 * H / (17.62 - H);

	where temperature \c T is given in degrees Celsius and \c RH is the relative humidity in percent.
	Then \c Dp is the dewpoint in degrees Celsius.

	This formula is too complex to calculate on a microcontroller because of the logarithm and the divisions.
	For this reason lookup tables are used.
	Multiple tables are required because the problem is two dimensional and storing the values in a 2d array
	requires too much memory.
	The formula is broken down to \c H1, \c H2, and \c Dp with

		H = H1 + H2 = 512 * (log10(RH^2) / 2 - 2) / 0.4343 + 512 * (17.62 * T) / (243.12 + T);
		Dp = 100 * 243.12 * H / (17.62 * 512 - H);

	with a lookup table for each of them.
	The factor 512 is used to give the results of the formula a larger range such that the values can be expressed as integers.
	For \c H1 a lookup table with logarithmic steps is used.
	The factor 100 is used to give the dewpoint as an integer with two decimal places included.
	The relative humidity is squared to get lower errors when interpolating.

	The result of the function is accurate to approximately 1 degree Celsius in the input range [-30, 30] degrees Celsius
	and [1, 100] % relative humidity.

	\param temperature Input temperature in degrees Celsius times 100.
	\param humidity Input humidity in percent (relative humidity).
	\return Dewpoint in degrees Celsius times 100.
 */
int16_t humidity_dewpoint(int16_t temperature, uint8_t humidity)
{
	int16_t H1, H2;

	/* The lookup table for H1 requires the relative humidity squared. */
	uint16_t humidity_squared = ((uint16_t) humidity) * ((uint16_t) humidity);

	/* Calculate H1, minimum input value is 1 * 1 = 1, maximum 100 * 100 = 10000 */
	H1 = lookup_logstep(humidity_squared, dewpoint_H1_lookup, 1, 10000);

	/* Calculate H2, step size 1024, 10 = log2(1024) */
	H2 = lookup(temperature, dewpoint_H2_lookup, -12288, 12288, 10);

	/* Use H = H1 + H2 to calculate the dewpoint, step size 1024, 10 = log2(1024) */
	return lookup(H1 + H2, dewpoint_Dp_lookup, -13312, 4096, 10);
}
