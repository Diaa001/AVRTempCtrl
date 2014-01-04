#include "display.h"
#include "spi.h"

void display_init(void)
{
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

void display_temperature(int16_t temperature)
{
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

void display_humidity(int8_t humidity)
{
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
