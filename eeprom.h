#ifndef __EEPROM_H__
#define __EEPROM_H__

#include <stdint.h>
#include <avr/eeprom.h>

/* Define the addresses of values in the EEPROM memory */
#define EE_CTRL_SETPOINT	((uint16_t *) 0x1)
#define EE_CTRL_KP0		((uint16_t *) 0x2)
#define EE_CTRL_KI0		((uint16_t *) 0x3)
#define EE_CTRL_KD0		((uint16_t *) 0x4)
#define EE_CTRL_KP1		((uint16_t *) 0x5)
#define EE_CTRL_KI1		((uint16_t *) 0x6)
#define EE_CTRL_KD1		((uint16_t *) 0x7)

#endif /* __EEPROM_H__ */
