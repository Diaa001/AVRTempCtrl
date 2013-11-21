#ifndef __EEPROM_H__
#define __EEPROM_H__

#include <stdint.h>
#include <avr/eeprom.h>

/* Define the addresses of values in the EEPROM memory */
#define EE_CTRL_SETPOINT	((uint16_t *) 0x2)
#define EE_CTRL_KP0		((uint16_t *) 0x4)
#define EE_CTRL_KI0		((uint16_t *) 0x6)
#define EE_CTRL_KD0		((uint16_t *) 0x8)
#define EE_CTRL_KP1		((uint16_t *) 0xa)
#define EE_CTRL_KI1		((uint16_t *) 0xc)
#define EE_CTRL_KD1		((uint16_t *) 0xe)

#endif /* __EEPROM_H__ */
