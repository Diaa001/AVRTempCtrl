/**
	\file
	\ingroup Temperature
	\brief Definition of functions and variables related to temperature sensors
 */
#include <stdint.h>
#include <util/delay.h>
#include "temperature.h"
#include "spi.h"
#include "interrupt.h"

int16_t temperature_ADC [TEMPERATURE_NUMBER_OF_SENSORS];
uint8_t _temperature_ADS1248_ready [TEMPERATURE_NUMBER_OF_ADS1248];

/**
	\brief Lookup table for conversion from temperature in Celsius to ADC value

	ADC values of ADS1248 measurements for temperatures -12288, -11264, -10240, -9216,
	-8192, -7168, -6144, -5120, -4096, -3072, -2048, -1024, 0, 1024, 2048, 3072, 4096,
	5120, 6144, 7168, 8192, 9216, 10240, 11264, 12288 (100 x degrees Celsius)
*/
int16_t temperature_to_ADS1248_lookup [] = {
	-32046, -29332, -26626, -23927, -21237, -18555, -15880, -13214, -10555, -7905, -5262,
	-2627, 0, 2618, 5229, 7832, 10427, 13014, 15594, 18165, 20728, 23283, 25831, 28370, 30902
};

/**
	\brief Lookup table for conversion from ADC value to temperature in Celsius

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

/**
	Converts an integer that is 100 times the Celsius value of a temperature into ADC units of the ADS1248 ADC
	by using a lookup table. The reason for using a lookup table is the fact that the conversion is non-linear
	and computing resources on the micro controller are limited. Because the micro controller has no instruction
	for division the step size in the table is a power of 2 which simplifies the division to a bit shift operation.
	\param temperature Signed integer value of degrees Celsius (times 100 to avoid using floating point numbers)
	\return ADC value that corresponds to the given temperature
 */
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

/**
	Converts an integer that represents the ADC value read from a ADS1248 ADC into 100 times the Celsius value
	of a temperature measured with the Pt1000 sensor by using a lookup table. The reason for using a lookup table
	is the fact that the conversion is non-linear and computing resources on the micro controller are limited.
	Because the micro controller has no instruction	for division the step size in the table is a power of 2 which
	reduces the division to a bit shift operation.
	\param ADC_val Signed integer ADC value of ADS1248 ADC (leading 16 bit)
	\return Temperature in Celsius that corresponds to the given ADC value (times 100 to avoid using floating point numbers)
 */
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

/**
	Reads a string and converts a value therein into a temperature. The string can have either sign, plus or minus, or
	neither. Leading spaces or tabs are ignored. Two decimal places are considered and the string may start with a decimal
	point. The resulting number is multiplied by 100 and stored in the second argument.
	\param string String containing a decimal number to be converted
	\param temperature Integer value to hold the converted decimal number, times 100
	\return Positive, when successful, zero otherwise
 */
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

/**
	Converts a temperature given in Celsius (times 100 to avoid the use of fractional/floating point values) into
	a string with a minus sign (where appropriate) and two decimal places.
	\param temperature Temperature in Celsius (times 100)
	\param string Pointer to a string that will hold the result
 */
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

/**
	Sequentially initializes all ADS1248 ADC chips using SPI communication. Micro controller
	inputs and outputs as well as interrupts are set up for the start and conversion ready
	signals. The ADC is set up to in terms of differential inputs, use of reference voltage,
	PGA (programmable gain amplifier) gain, conversion rate, and excitation currents.
 */
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

		/* Program register SYS0: Set the gain of the PGA (programmable gain amplifier) to 8x, 40 samples per second */
		SPI_send((0x3 << 4) | 0x3);

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

/**
	\brief Selects the ADC channel of the appropriate ADS1248 chip

	Internal function used to reprogram the ADS1248 ADCs corresponding to the
	given channel to use a different pair of inputs for the differential
	measurement and the excitation current output.
	\param channel Channel / sensor number to be selected
	\warning A delay of several micro seconds is required after calling this function
	before starting a conversion to allow the analog multiplexer to settle. The amount
	of delay depends on the conversion rate that the ADC is programmed to.
	SPS	| Delay
	------- | -------
	5	| > 200 ms
	10	| > 100 ms
	20	| > 50 ms
	40	| > 25 ms
	80	| > 12.5 ms
	160	| > 6.5 ms
	320	| > 3.5 ms
	640	| > 1.7 ms
	1000	| > 1.2 ms
	2000	| > 0.6 ms
 */
static void temperature_ADS1248_select_sensor(uint8_t channel)
{
	SPI_set_sample_falling_edge();

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

/**
	Reprograms the appropritate ADS1248 ADC to use the appropriate inputs corresponding to the
	given channel / sensor number and - after a delay to allow the device to settle - starts a
	conversion by pulsing the start signal high.
	\param channel The channel number for which to start a conversion
	\warning When changing the conversion rate of the ADC a delay has to be changed in this function!
	SPS	| Delay
	------- | -------
	5	| > 200 ms
	10	| > 100 ms
	20	| > 50 ms
	40	| > 25 ms
	80	| > 12.5 ms
	160	| > 6.5 ms
	320	| > 3.5 ms
	640	| > 1.7 ms
	1000	| > 1.2 ms
	2000	| > 0.6 ms
 */
void temperature_ADS1248_start_conversion(uint8_t channel)
{
	/* Select the sensor to measure before starting the conversion */
	temperature_ADS1248_select_sensor(channel);

	/* Wait for the ADC to settle. This delay depends on the conversion rate of the ADC!

		SPS	Delay
		5	> 200 ms
		10	> 100 ms
		20	> 50 ms
		40	> 25 ms
		80	> 12.5 ms
		160	> 6.5 ms
		320	> 3.5 ms
		640	> 1.7 ms
		1000	> 1.2 ms
		2000	> 0.6 ms
	*/
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

/**
	Checks whether an ADC conversion is ready for readout
	\return The channel number which is ready for readout, or -1 if no channel is ready
 */
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

/**
	Reads the ADC result from the ADS1248 chip corresponding to the channel / sensor
	number using SPI communication. The ADS1248 ADC has 24 bit resolution but only
	the most significant 16 bits are used because only approximately 19-20 bits are
	noise free by specification and in the applicable temperature range the resolution
	is more than enough for 2 decimal places with 16 bit.
	\param channel The channel number for which to read the ADC result
	\return The ADC value read from the ADS1248 chip corresponding to the given
	channel / sensor number
 */
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
