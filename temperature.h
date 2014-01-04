#ifndef __TEMPERATURE_H__
#define __TEMPERATURE_H__

#include <stdint.h>

#define ADS1248_REG_MUX0	0x00
#define ADS1248_REG_VBIAS	0x01
#define ADS1248_REG_MUX1	0x02
#define ADS1248_REG_SYS0	0x03
#define ADS1248_REG_OFC0	0x04
#define ADS1248_REG_OFC1	0x05
#define ADS1248_REG_OFC2	0x06
#define ADS1248_REG_FSC0	0x07
#define ADS1248_REG_FSC1	0x08
#define ADS1248_REG_FSC2	0x09
#define ADS1248_REG_IDAC0	0x0a
#define ADS1248_REG_IDAC1	0x0b
#define ADS1248_REG_GPIOCFG	0x0c
#define ADS1248_REG_GPIODIR	0x0d
#define ADS1248_REG_GPIODAT	0x0e

#define ADS1248_CMD_WAKEUP	0x00
#define ADS1248_CMD_SLEEP	0x02
#define ADS1248_CMD_SYNC	0x04
#define ADS1248_CMD_RESET	0x06
#define ADS1248_CMD_NOP		0xff
#define ADS1248_CMD_RDATA	0x12
#define ADS1248_CMD_RDATAC	0x14
#define ADS1248_CMD_SDATAC	0x16
#define ADS1248_CMD_RREG	0x20
#define ADS1248_CMD_WREG	0x40
#define ADS1248_CMD_SYSOCAL	0x60
#define ADS1248_CMD_SYSGCAL	0x61
#define ADS1248_CMD_SELFOCAL	0x62

/* Start and ready ports and pins */
#define ADS1248_START_0_PORT	PORTB
#define ADS1248_START_0_DDR	DDRB
#define ADS1248_START_0		PB4
#define ADS1248_START_1_PORT	PORTC
#define ADS1248_START_1_DDR	DDRC
#define ADS1248_START_1		PC1
#define ADS1248_READY_0_PIN	PORTC
#define ADS1248_READY_0_DDR	DDRC
#define ADS1248_READY_0_PCINT	PCINT16
#define ADS1248_READY_0_PCIE	PCIE2
#define ADS1248_READY_0_PCMSK	PCMSK2
#define ADS1248_READY_0		PC0
#define ADS1248_READY_1_PIN	PORTC
#define ADS1248_READY_1_DDR	DDRC
#define ADS1248_READY_1_PCINT	PCINT18
#define ADS1248_READY_1_PCIE	PCIE2
#define ADS1248_READY_1_PCMSK	PCMSK2
#define ADS1248_READY_1		PC2

#define TEMPERATURE_ADS1248_READY_FLAG	0x80

#define TEMPERATURE_NUMBER_OF_ADC	2
extern int16_t temperature_ADC [TEMPERATURE_NUMBER_OF_ADC];
#define TEMPERATURE_NUMBER_OF_ADS1248	2
extern uint8_t _temperature_ADS1248_ready [TEMPERATURE_NUMBER_OF_ADS1248];

#define temperature_to_ADC_Pt1000_lookup_index(T)	(((T) + 8192) >> 10)
#define temperature_ADC_Pt1000_to_temp_lookup_index(A)	(((A) + 512) >> 6)

int16_t temperature_to_ADC_Pt1000(int16_t temperature);
int16_t temperature_ADC_Pt1000_to_temp(int16_t A);

uint8_t temperature_string_to_temp(const char * string, int16_t * temperature);
void temperature_to_string(int16_t temperature, char * string);

void temperature_ADS1248_init(void);
void temperature_ADS1248_start_conversion(uint8_t channel);
int8_t temperature_ADS1248_ready(void);
int16_t temperature_ADS1248_read_result(uint8_t channel);

#endif /* __TEMPERATURE_H__ */
