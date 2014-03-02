/**
	\file
	\ingroup Display
	\brief Declarations of functions and constants related to the 7-segment display

	\addtogroup Display 7-segment display
	\brief Display of temperature, humidity, set point etc.

	The 7-segement display is used to display the current temperature and humidity
	values.
	It can also show the current set point and changes being made to it.
	The display consists of a total of 8 digits grouped into 5 and 3 digits.
	Five digits are used for temperature display in the form -XX.XX (each digit
	has its own decimal point so it actually has 8 segments) and 3 digits for
	the relative humidity as an integer from 0 to 100.

	The set point is displayed on the temperature display with the humidity
	display showing the characters 'SP'.
	Of course temperature, humidity, and setpoint cannot be shown at the same time.
	The user interface takes care about what is displayed at what time.

	The display is controlled by the MAX7221 7-segment display controller.
	It communicates with the microcontroller through SPI.
	It allows the transmission of each digit independently in two encoding modes.
	In one mode the digit can simply be the integer intended for display
	or in the other mode the integer is used as a bit field to determine which
	segment of a digit is turned on.

	\{

 */

#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <stdint.h>

/* *** Display functions *********************************************************************************************************************************** */

void display_init(void);			///< Initializes the MAX7221 7-segment display controller
void display_temperature(int16_t temperature);	///< Displays the given temperature
void display_humidity(uint8_t humidity);	///< Displays the given humidity

/** \} */

#endif /* __DISPLAY_H__ */
