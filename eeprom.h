/**
	\file
	\ingroup EERPROM
	\brief Declaration of constants related to the internal EEPROM

	\addtogroup EEPROM
	\brief Memory that retains data during power loss

	The EEPROM (Electrically Erasable Programmable Read-Only Memory, located within the microcontroller) is used to store data
	that should not be lost when power is removed.
	This data includes
	- controller setpoint
	- controller parameters (Kp, Kd, Ki) for cooling
	- controller parameters (Kp, Kd, Ki) for heating

	At each power reset all of the values are used to initialize their corresponding RAM memory locations.
	When changing parameters with the SET instruction the EEPROM is unaffected and the values are lost when
	the next SET instruction is used, a RECALL instruction is used or when power is lost.
	If the SAVE instruction is used the corresponding value is saved to the EEPROM overwriting the old value.
	From this time on, this value will be used when powering up or when calling the RECALL instruction.
	The RECALL instruction just reads the EEPROM memory location and updates the RAM memory with the value read.

	The data in the EEPROM is saved directly, word after word or byte after byte.
	The microcontroller should have the EESAVE fuse bit programmed such that the data in the EEPROM is not lost
	when the microcontroller is reprogrammed.

	\{
 */

#ifndef __EEPROM_H__
#define __EEPROM_H__

#include <stdint.h>
#include <avr/eeprom.h>

/* *** EEPROM Constants ************************************************************************************************************************************ */

/* Define the addresses of values in the EEPROM memory */
#define EE_CTRL_SETPOINT	((uint16_t *) 0x2)	///< Memory location for the 16 bit setpoint
#define EE_CTRL_KP0		((uint16_t *) 0x4)	///< Memory location for the 16 bit KP0 controller parameter (cooling)
#define EE_CTRL_KI0		((uint16_t *) 0x6)	///< Memory location for the 16 bit KI0 controller parameter (cooling)
#define EE_CTRL_KD0		((uint16_t *) 0x8)	///< Memory location for the 16 bit KD0 controller parameter (cooling)
#define EE_CTRL_KP1		((uint16_t *) 0xa)	///< Memory location for the 16 bit KP1 controller parameter (heating)
#define EE_CTRL_KI1		((uint16_t *) 0xc)	///< Memory location for the 16 bit KI1 controller parameter (heating)
#define EE_CTRL_KD1		((uint16_t *) 0xe)	///< Memory location for the 16 bit KD1 controller parameter (heating)

/** \} */

#endif /* __EEPROM_H__ */
