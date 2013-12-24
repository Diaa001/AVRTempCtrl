#ifndef __SPI_H__
#define __SPI_H__

#include <avr/io.h>

#define SPI_CS_PORT		PORTD
#define SPI_CS_ADS1248_0	PD5
#define SPI_CS_ADS1248_1	PD6

#define SPI_send(a)	SPI_send_receive(a)

void SPI_init(void);
void SPI_select(uint8_t cs);
void SPI_deselect(uint8_t cs);
uint8_t SPI_send_receive(uint8_t data);

#endif /* __SPI_H__ */
