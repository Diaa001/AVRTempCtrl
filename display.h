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

/* *** Display constants *********************************************************************************************************************************** */

#define DISPLAY_REG_NOP			0x0	///< Display command that executes no operation
#define DISPLAY_REG_DIGIT_0		0x1	///< Display command that sets digit 0
#define DISPLAY_REG_DIGIT_1		0x2	///< Display command that sets digit 1
#define DISPLAY_REG_DIGIT_2		0x3	///< Display command that sets digit 2
#define DISPLAY_REG_DIGIT_3		0x4	///< Display command that sets digit 3
#define DISPLAY_REG_DIGIT_4		0x5	///< Display command that sets digit 4
#define DISPLAY_REG_DIGIT_5		0x6	///< Display command that sets digit 5
#define DISPLAY_REG_DIGIT_6		0x7	///< Display command that sets digit 6
#define DISPLAY_REG_DIGIT_7		0x8	///< Display command that sets digit 7
#define DISPLAY_REG_DECODE_MODE		0x9	///< Display command that sets decoding mode for each digit
#define DISPLAY_REG_INTENSITY		0xA	///< Display command to set the display intensity
#define DISPLAY_REG_SCAN_LIMIT		0xB	///< Display command to set the number of digits in the display
#define DISPLAY_REG_SHUTDOWN		0xC	///< Display command to shutdown the display
#define DISPLAY_REG_DISPLAY_TEST	0xF	///< Display command to initiate a display test

#define DISPLAY_CODE_B_MINUS		0xA	///< Display code for a minus sign
#define DISPLAY_CODE_B_CHAR_E		0xB	///< Display code for character E
#define DISPLAY_CODE_B_CHAR_H		0xC	///< Display code for character H
#define DISPLAY_CODE_B_CHAR_L		0xD	///< Display code for character L
#define DISPLAY_CODE_B_CHAR_P		0xE	///< Display code for character P
#define DISPLAY_CODE_B_BLANK		0xF	///< Display code for blank digit

#define DISPLAY_CODE_DECIMAL_POINT	0x80	///< Display flag for a decimal point

/* *** Display functions *********************************************************************************************************************************** */

void display_init(void);			///< Initializes the MAX7221 7-segment display controller
void display_temperature(int16_t temperature);	///< Displays the given temperature
void display_humidity(uint8_t humidity);	///< Displays the given humidity

/** \} */

#endif /* __DISPLAY_H__ */
