/**
	\file
	\ingroup Humidity
	\brief Declaration of functions, variables, and constants related to humidity sensors

	\addtogroup Humidity
	\brief Humidity sensors and measurements

	There are a number of humidity sensors connected to the system.
	The measurements are made in a scheduled task and the values stored in the variable \ref humidity_ADC.
	These values are used for the interlock system and for user readout.
	For this reason it is necessary to convert the ADC values int physical units.

	The Honeywell HIH-4000-001 humidity sensor is powered with 5 V and has an analog output
	that is connected to internal ADC of the microcontroller.
	The output voltage is between 0.75 V and 3.75 V in a nearly linear relation to humidity.
	This requires the ADC reference voltage to be higher than 3.75 V and thus 5 V is used.
	This of course limits the ADC resolution because only codes from approximately 150 to 770
	are used out of the 10 bit range (0 - 1023).
	This is not so bad because the accuracy of (almost all) humidity sensors is worse than 1 %
	and therefore 1 % resolution is enough.

	\{
 */

#ifndef __HUMIDITY_H__
#define __HUMIDITY_H__

#include <stdint.h>

/* *** Humidity constants ********************************************************************************************************************************** */

#define HUMIDITY_NUMBER_OF_ADC	1			///< Number or humidity sensors connected to the system

/* *** Humidity variables ********************************************************************************************************************************** */

extern int16_t humidity_ADC [HUMIDITY_NUMBER_OF_ADC];	///< Last measurement value of each humidity sensor

/* *** Humidity functions ********************************************************************************************************************************** */

uint8_t honeywell_convert_ADC_to_RH(uint16_t adc);	///< Conversion of ADC values to relative humidity for honeywell humidity sensors

/** \} */

#endif /* __HUMIDITY_H__ */
