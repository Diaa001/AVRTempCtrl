/**
	\file
	\ingroup LED
	\brief Declaration of functions and constants related to LED displays

	\addtogroup LED LED displays
	\brief LED displays show the state of the temperature controller

	LEDs are used to indicate the state of the temperature controller to the user.
	There is a total of 5 LEDs, 3 of which are controlled in hardware, not by this
	firmware.
	The other two LEDs are used to indicate the active state of the temperature
	controller as well as to display the alarm / error state.

	The LEDs are controlled simply by setting a I/O pin output of the microcontroller.
	The 5 V output is applied to the LED through a 150 Ohm resistor.

	\{
 */

#ifndef __LED_H__
#define __LED_H__

#include <avr/io.h>

/* *** LED constants *************************************************************************************************************************************** */

#define	LED_0		PC3	///< Pin to which LED 0 is connected
#define	LED_1		PC4	///< Pin to which LED 1 is connected
#define	LED_PORT	PORTC	///< Microcontroller port where LEDs are connected
#define	LED_DDR		DDRC	///< Data direction register for the port where LEDs are connected

#define	LED_ACTIVE	LED_0	///< Assignment for the LED used to indicate temperature controller state
#define	LED_ERROR	LED_1	///< Assignment for the LED used to indicate alarm / error state
#define	LED_OFF		0	///< I/O pin output required to turn a LED off
#define	LED_ON		1	///< I/O pin output required to turn a LED on

/* *** LED functions *************************************************************************************************************************************** */

extern void LED_init(void);				///< Initializes the LED pins
extern void LED_set(uint8_t led, uint8_t state);	///< Sets the given LED to the given state

/** \} */

#endif /* __LED_H__ */
