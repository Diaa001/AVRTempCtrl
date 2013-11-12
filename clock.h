#ifndef __CLOCK_H__
#define __CLOCK_H__

#include "avr/io.h"

#define clock_set_prescale_2() {\
	CLKPR = (1 << CLKPCE);\
	CLKPR = (1 << CLKPS0);\
}

#endif /* __CLOCK_H__ */
