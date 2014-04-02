/**
	\file
	\ingroup Display
	\brief Definitions of functions related to the 7-segment display
 */

#include "display.h"
#include "spi.h"

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

void display_init(void)
{
	SPI_set_sample_rising_edge();

	/* Disable shutdown mode */
	SPI_select(SPI_CS_MAX7221);
	SPI_send(DISPLAY_REG_SHUTDOWN);
	SPI_send(0x1);
	SPI_deselect(SPI_CS_MAX7221);

	/* Disable display test mode */
	SPI_select(SPI_CS_MAX7221);
	SPI_send(DISPLAY_REG_DISPLAY_TEST);
	SPI_send(0x0);
	SPI_deselect(SPI_CS_MAX7221);

	/* Set the decoding mode to Code-B font for all digits */
	SPI_select(SPI_CS_MAX7221);
	SPI_send(DISPLAY_REG_DECODE_MODE);
	SPI_send(0xFF);
	SPI_deselect(SPI_CS_MAX7221);

	/* Set the display intensity */
	SPI_select(SPI_CS_MAX7221);
	SPI_send(DISPLAY_REG_INTENSITY);
	SPI_send(0xF);
	SPI_deselect(SPI_CS_MAX7221);

	/* Set the scan limit to include all 8 digits */
	SPI_select(SPI_CS_MAX7221);
	SPI_send(DISPLAY_REG_SCAN_LIMIT);
	SPI_send(0x07);
	SPI_deselect(SPI_CS_MAX7221);
}

/**
	Transforms the given temperature that is represented as degrees Celsius
	times 100 into decimal digits in preparation for display.
	Then it transfers the information to the 7-segment display controller.
	The display is updated as soon as the chip select signal is deactivated.
	\param temperature Integer that represents temperature in Celsius multiplied by the factor 100.
 */
void display_temperature(int16_t temperature)
{
	SPI_set_sample_rising_edge();

	/* Values of each of the 5 digits */
	uint8_t digit [5];

	/* Test for negativity */
	if (temperature < 0) {
		temperature = -temperature;
		digit[0] = DISPLAY_CODE_B_MINUS;
	} else {
		digit[0] = DISPLAY_CODE_B_BLANK;
	}

	/* Determine the value of each digit */
	int16_t tmp;
	tmp = temperature / 10;
	digit[4] = temperature - 10 * tmp;

	temperature = tmp;
	tmp = temperature / 10;
	digit[3] = temperature - 10 * tmp;

	temperature = tmp;
	tmp = temperature / 10;
	digit[2] = temperature - 10 * tmp;
	digit[2] |= DISPLAY_CODE_DECIMAL_POINT;

	temperature = tmp;
	tmp = temperature / 10;
	digit[1] = temperature - 10 * tmp;

	/* Eliminate leading zeros */
	if (digit[1] == 0) {
		/* The 0th digit may contain a minus sign */
		digit[1] = digit[0];
		digit[0] = DISPLAY_CODE_B_BLANK;
	}

	/* Send the SPI commands for digit 0 */
	SPI_select(SPI_CS_MAX7221);
	SPI_send(DISPLAY_REG_DIGIT_0);
	SPI_send(digit[0]);
	SPI_deselect(SPI_CS_MAX7221);

	/* Send the SPI commands for digit 1 */
	SPI_select(SPI_CS_MAX7221);
	SPI_send(DISPLAY_REG_DIGIT_1);
	SPI_send(digit[1]);
	SPI_deselect(SPI_CS_MAX7221);

	/* Send the SPI commands for digit 2 */
	SPI_select(SPI_CS_MAX7221);
	SPI_send(DISPLAY_REG_DIGIT_2);
	SPI_send(digit[2]);
	SPI_deselect(SPI_CS_MAX7221);

	/* Send the SPI commands for digit 3 */
	SPI_select(SPI_CS_MAX7221);
	SPI_send(DISPLAY_REG_DIGIT_3);
	SPI_send(digit[3]);
	SPI_deselect(SPI_CS_MAX7221);

	/* Send the SPI commands for digit 4 */
	SPI_select(SPI_CS_MAX7221);
	SPI_send(DISPLAY_REG_DIGIT_4);
	SPI_send(digit[4]);
	SPI_deselect(SPI_CS_MAX7221);
}

/**
	Transforms the humidity value given in percent relative humidity into
	decimal digits and sends the information to the 7-segment display controller.
	The display is updated when the chip select signal is deactivated.
	\param humidity Integer that represents relative humidity in percent.
 */
void display_humidity(uint8_t humidity)
{
	SPI_set_sample_rising_edge();

	/* Values of each of the 3 digits */
	uint8_t digit [3];

	/* Determine the value of each digit */
	int8_t tmp;
	tmp = humidity / 10;
	digit[2] = humidity - 10 * tmp;

	humidity = tmp;
	tmp = humidity / 10;
	digit[1] = humidity - 10 * tmp;

	humidity = tmp;
	tmp = humidity / 10;
	digit[0] = humidity - 10 * tmp;

	/* Eliminate leading zeros */
	if (digit[0] == 0) {
		digit[0] = DISPLAY_CODE_B_BLANK;
		if (digit[1] == 0) {
			digit[1] = DISPLAY_CODE_B_BLANK;
		}
	}

	/* Send the SPI commands */
	SPI_select(SPI_CS_MAX7221);
	SPI_send(DISPLAY_REG_DIGIT_5);
	SPI_send(digit[0]);
	SPI_deselect(SPI_CS_MAX7221);

	SPI_select(SPI_CS_MAX7221);
	SPI_send(DISPLAY_REG_DIGIT_6);
	SPI_send(digit[1]);
	SPI_deselect(SPI_CS_MAX7221);

	SPI_select(SPI_CS_MAX7221);
	SPI_send(DISPLAY_REG_DIGIT_7);
	SPI_send(digit[2]);
	SPI_deselect(SPI_CS_MAX7221);
}
