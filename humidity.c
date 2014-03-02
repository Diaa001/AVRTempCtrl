/**
	\file
	\ingroup Humidity
	\brief Definition of functions and variables related to humidity sensors
 */

#include "humidity.h"

int16_t humidity_ADC [HUMIDITY_NUMBER_OF_ADC];

/**
	Conversion of the ADC value into units of relative humidity in percent for the Honeywell HIH-4000-001 humidity sensor.
	The formula used is from the datasheet.
	There is some non-linearity in the analog output but because of an accuracy of 5 % this is not relevant.
	\param adc The ADC value to be converted into percent relative humidity.
	\return The converted value in percent relative humidity.

	\note Temperature compensation as specified in the datasheet should be used but is currently not implemented.
 */
uint8_t honeywell_convert_ADC_to_RH(uint16_t adc)
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
