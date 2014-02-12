#include <avr/io.h>
#include "spi.h"

void SPI_init(void)
{
	/* Disable the SPI power reduction mode */
	PRR0 &= ~(1 << PRSPI);

	/* Set MOSI and SCK as outputs */
	DDRB |= (1 << PB5) | (1 << PB7);

	/* Set MISO as input */
	DDRB &= ~(1 << PB6);

	/* Set the chip select pins as outputs */
	DDRD |= (1 << PD5) | (1 << PD6) | (1 << PD7);

	/* Set the bit order to MSB first */
	SPCR &= ~(1 << DORD);

	/* Set the clock polarity (normally low) */
	SPCR &= ~(1 << CPOL);

	/* Set the clock phase to setup at the leading edge and sample at the trailing edge */
	SPCR |= (1 << CPHA);

	/* Set the SPI clock frequency to f/16 */
	SPCR |= (1 << SPR0);
	SPCR &= ~(1 << SPR1);

	/* Select master mode */
	SPCR |= (1 << MSTR);

	/* Enable SPI */
	SPCR |= (1 << SPE);

	/* Deselect the slaves */
	SPI_deselect(SPI_CS_ADS1248_0);
	SPI_deselect(SPI_CS_ADS1248_1);
	SPI_deselect(SPI_CS_MAX7221);
}

void SPI_select(uint8_t cs)
{
	SPI_CS_PORT &= ~(1 << cs);
}

void SPI_deselect(uint8_t cs)
{
	SPI_CS_PORT |= (1 << cs);
}

uint8_t SPI_send_receive(uint8_t data)
{
	/* Send the data byte */
	SPDR = data;

	/* Wait for the transmission to finish */
	while (!(SPSR & (1 << SPIF)));

	uint8_t data_received = SPDR;
	return data_received;
}

void SPI_set_sample_rising_edge(void)
{
	SPCR &= ~(1 << CPHA);
}

void SPI_set_sample_falling_edge(void)
{
	SPCR |= (1 << CPHA);
}
