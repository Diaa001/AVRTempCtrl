#include <stdint.h>
#include <util/delay.h>
#include "temperature.h"
#include "spi.h"
#include "interrupt.h"

int16_t temperature_ADC [TEMPERATURE_NUMBER_OF_SENSORS];
uint8_t _temperature_ADS1248_ready [TEMPERATURE_NUMBER_OF_ADS1248];

/**
	ADC values of ADS1248 measurements for temperatures -12288, -11264, -10240, -9216,
	-8192, -7168, -6144, -5120, -4096, -3072, -2048, -1024, 0, 1024, 2048, 3072, 4096,
	5120, 6144, 7168, 8192, 9216, 10240, 11264, 12288 (100 x degrees Celsius)
*/
int16_t temperature_to_ADS1248_lookup [] = {
	-32046, -29332, -26626, -23927, -21237, -18555, -15880, -13214, -10555, -7905, -5262,
	-2627, 0, 2618, 5229, 7832, 10427, 13014, 15594, 18165, 20728, 23283, 25831, 28370, 30902
};

/**
	Temperature values (Celsius times 100) of ADS1248 measurements for ADC values -32768,
	-30720, -28672, -26624, -24576, -22528, -20480, -18432, -16384, -14336, -12288, -10240,
	-8192, -6144, -4096, -2048, 0, 2048, 4096, 6144, 8192, 10240, 12288, 14336, 16384,
	18432, 20480, 22528, 24576, 26624, 28672, 30720, 32768
 */
int16_t temperature_ADS1248_to_temp_lookup [] = {
	-12560, -11788, -11014, -10239, -9462, -8683, -7903, -7121, -6337, -5551, -4763, -3974,
	-3183, -2390, -1595, -798, 0, 800, 1602, 2407, 3213, 4021, 4831, 5644, 6458, 7274, 8092,
	8912, 9735, 10559, 11385, 12214, 13044
};

int16_t temperature_to_ADS1248(int16_t temperature)
{
	int16_t a;
	int16_t A, B;

	if (temperature < ADS1248_LOOKUP_TMIN)
		temperature = ADS1248_LOOKUP_TMIN;
	if (temperature > ADS1248_LOOKUP_TMAX)
		temperature = ADS1248_LOOKUP_TMAX;

	/* Get the index of the temperature in the table equal or just smaller
	   to 'temperature' */
	int8_t index = temperature_to_ADS1248_lookup_index(temperature);

	/* Temperature of the table entry */
	a = (index << ADS1248_LOOKUP_TSTEP_LOG) + ADS1248_LOOKUP_TMIN;

	/* ADC value at temperature a */
	A = temperature_to_ADS1248_lookup[index];

	/* ADC value at temperature a + 1024 (next table entry) */
	B = temperature_to_ADS1248_lookup[index + 1];

	/* Interpolate (+512: for rounding) */
	return A + (((B - A) * ((int32_t)(temperature - a)) + ADS1248_LOOKUP_TSTEP / 2) >> ADS1248_LOOKUP_TSTEP_LOG);
}

int16_t temperature_ADS1248_to_temp(int16_t ADC_val)
{
	int16_t a;
	int16_t A, B;

	/* Get the index of the ADC value in the table equal or just smaller to 'ADC' */
	int8_t index = temperature_ADS1248_to_temp_lookup_index(ADC_val);

	/* ADC value of the table entry */
	a = (index << ADS1248_LOOKUP_ADCSTEP_LOG) + ADS1248_LOOKUP_ADCMIN;

	/* Temperature (x100) at ADC value a */
	A = temperature_ADS1248_to_temp_lookup[index];

	/* Temperature (x100) at ADC value a + 64 (next table entry) */
	B = temperature_ADS1248_to_temp_lookup[index + 1];

	/* Interpolate (+32: for rounding). Use 32 bit integers because intermediate values are large. */
	int32_t tmp = A + (((((int32_t) B) - A) * (ADC_val - a) + ADS1248_LOOKUP_ADCSTEP / 2) >> ADS1248_LOOKUP_ADCSTEP_LOG);
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

void temperature_ADS1248_init(void)
{
	/* Initialize both ADS1248 ADCs */
	uint8_t id;
	for (id = 0; id < 2; id++) {
		/* Set the start and ready pins as output / input */
		if (id == 0) {
			ADS1248_START_0_DDR |= (1 << ADS1248_START_0);
			ADS1248_READY_0_DDR &= ~(1 << ADS1248_READY_0);

			/* Enable the pin change interrupt for the READY signal */
			PCICR |= (1 << ADS1248_READY_0_PCIE);
		} else if (id == 1) {
			ADS1248_START_1_DDR |= (1 << ADS1248_START_1);
			ADS1248_READY_1_DDR &= ~(1 << ADS1248_READY_1);

			/* Enable the pin change interrupt for the READY signal */
			PCICR |= (1 << ADS1248_READY_1_PCIE);
		}

		SPI_set_sample_falling_edge();

		/* Select the chip to receive SPI commands */
		if (id == 0)
			SPI_select(SPI_CS_ADS1248_0);
		else if (id == 1)
			SPI_select(SPI_CS_ADS1248_1);

		/* Start programming registers, start with MUX0 */
		SPI_send(ADS1248_CMD_WREG | ADS1248_REG_MUX0);

		/* Number of registers to program after MUX0 */
		SPI_send(3);

		/* Program register MUX0: BCS = off, inputs = AIN0, AIN1 */
		SPI_send((0x0 << 3) | (0x1 << 0));

		/* Program register VBIAS: all bias off */
		SPI_send(0);

		/* Program register MUX1: internal reference always on, REF0 as reference, normal measurement */
		SPI_send((0x1 << 5));

		/* Program register SYS0: Set the gain of the PGA (programmable gain amplifier) to 16x, 40 samples per second */
		SPI_send((0x4 << 4) | 0x3);

		/* Program registers, continue with IDAC0 */
		SPI_send(ADS1248_CMD_WREG | ADS1248_REG_IDAC0);

		/* NUMBER of registers to program after IDAC0 */
		SPI_send(1);

		/* Program register IDAC0: DRDY mode off, 500 uA excitation current */
		SPI_send(0x4);

		/* Program register IDAC1: Select AIN0 and AIN1 as excitation current outputs */
		SPI_send((0x0 << 4) | (0x1 << 0));

		/* Deselect the chip */
		if (id == 0)
			SPI_deselect(SPI_CS_ADS1248_0);
		else if (id == 1)
			SPI_deselect(SPI_CS_ADS1248_1);
	}
}

static void temperature_ADS1248_select_sensor(uint8_t channel)
{
	if (channel < 4)
		SPI_select(SPI_CS_ADS1248_0);
	else
		SPI_select(SPI_CS_ADS1248_1);

	/* Start programming registers, start with MUX0 */
	SPI_send(ADS1248_CMD_WREG | ADS1248_REG_MUX0);

	/* Number of registers to program after MUX0 */
	SPI_send(0);

	/* Program register MUX0: BCS = off, inputs = AIN[2 * channel], AIN[2 * channel + 1] */
	SPI_send(((channel & 0x3) << 4) | (((channel & 0x3) << 1) + 1));

	/* Program registers, continue with IDAC1 */
	SPI_send(ADS1248_CMD_WREG | ADS1248_REG_IDAC1);

	/* NUMBER of registers to program after IDAC1 */
	SPI_send(0);

	/* Program register IDAC1: Select AIN[2 * channel] and AIN[2 * channel + 1] as excitation current outputs */
	SPI_send(((channel & 0x3) << 5) | (((channel & 0x3) << 1) + 1));

	if (channel < 4)
		SPI_deselect(SPI_CS_ADS1248_0);
	else
		SPI_deselect(SPI_CS_ADS1248_1);
}

void temperature_ADS1248_start_conversion(uint8_t channel)
{
	/* Select the sensor to measure before starting the conversion */
	temperature_ADS1248_select_sensor(channel);

	/* Wait for the ADC to settle */
	_delay_ms(30);

	if (channel < 4) {
		/* Pulse the START signal */
		ADS1248_START_0_PORT |= (1 << ADS1248_START_0);
		_delay_us(1);
		ADS1248_START_0_PORT &= ~(1 << ADS1248_START_0);

		/* Enable the pin change interrupt on the READY signal */
		ADS1248_READY_0_PCMSK |= (1 << ADS1248_READY_0_PCINT);
		_temperature_ADS1248_ready[0] = channel;
	} else {
		/* Pulse the START signal */
		ADS1248_START_1_PORT |= (1 << ADS1248_START_1);
		_delay_us(1);
		ADS1248_START_1_PORT &= ~(1 << ADS1248_START_1);

		/* Enable the pin change interrupt on the READY signal */
		ADS1248_READY_1_PCMSK |= (1 << ADS1248_READY_1_PCINT);
		_temperature_ADS1248_ready[1] = channel;
	}
}

int8_t temperature_ADS1248_ready(void) {
	/* If no channel is ready, return -1; */
	uint8_t channel = -1;

	interrupts_suspend();

	/* Determine the ADS1248 channel that is ready */
	if (_temperature_ADS1248_ready[0] & TEMPERATURE_ADS1248_READY_FLAG) {
		_temperature_ADS1248_ready[0] &= ~TEMPERATURE_ADS1248_READY_FLAG;
		channel = _temperature_ADS1248_ready[0];
	} else if (_temperature_ADS1248_ready[1] & TEMPERATURE_ADS1248_READY_FLAG) {
		_temperature_ADS1248_ready[1] &= ~TEMPERATURE_ADS1248_READY_FLAG;
		channel = _temperature_ADS1248_ready[1];
	}

	interrupts_resume();

	return channel;
}

int16_t temperature_ADS1248_read_result(uint8_t channel)
{
	SPI_set_sample_falling_edge();

	/* Select the chip to receive SPI commands */
	if (channel < 4)
		SPI_select(SPI_CS_ADS1248_0);
	else
		SPI_select(SPI_CS_ADS1248_1);

	/* Read the two significant bytes */
	uint16_t result;
	SPI_send(ADS1248_CMD_RDATA);
	result = SPI_send_receive(ADS1248_CMD_NOP);
	result = SPI_send_receive(ADS1248_CMD_NOP) | (result << 8);

	/* Discard the least significant byte */
	SPI_send_receive(ADS1248_CMD_NOP);

	/* Deselect the chip */
	if (channel < 4)
		SPI_deselect(SPI_CS_ADS1248_0);
	else
		SPI_deselect(SPI_CS_ADS1248_1);

	return (int16_t) result;
}
