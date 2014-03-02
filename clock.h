/**
	\file
	\brief Delcaration of marcos related to the internal clock
 */

#ifndef __CLOCK_H__
#define __CLOCK_H__

#include "avr/io.h"

/* *** Clock macros **************************************************************************************************************************************** */

/**
	\brief Sets the internal clock prescaler to 2

	The internal RC oscillator runs at 8 MHz and is divided by 2 if this macro is called.
	By default the prescaler is 8, meaning 1 MHz operation.
	This however depends on the fuse bits of the microcontroller (more details \ref DevelDoc_Programming "here").
	When changing the clock frequencies all timings (scheduler, ADC speeds, etc.) have to be adjusted.
 */
#define clock_set_prescale_2() {\
	CLKPR = (1 << CLKPCE);\
	CLKPR = (1 << CLKPS0);\
}

#endif /* __CLOCK_H__ */
