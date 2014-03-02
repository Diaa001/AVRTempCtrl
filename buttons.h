/**
	\file
	\ingroup Buttons
	\brief Declaration of functions, variables, and constants related to buttons

	\addtogroup Buttons
	\brief Buttons to manipulate the state of the temperature controller

	Buttons are a physical interface for users to manipulate the temperature controller.
	There are 3 buttons available.
	They control:
	- Controller state (control on and off, alarm off)
	- Setpoint control
	- ?

	Buttons are connecte to input pins of the microcontroller through a hardware debouncer.
	This means that the state that is read is always definite.
	Interrupts are used to detect the change of state of each button (see \ref Interrupts for
	details).
	The main program can check whether the state of a button has changed by looking at the flag
	that is set in the interrupt handler.

	\{

 */

#ifndef __BUTTONS_H__
#define __BUTTONS_H__

#include <avr/io.h>

/* *** Button constants ************************************************************************************************************************************ */

#define BUTTON_0			PC5	///< Pin to which button 0 is connected
#define BUTTON_1			PC6	///< Pin to which button 1 is connected
#define BUTTON_2			PC7	///< Pin to which button 2 is connected
#define BUTTON_0_DDR_REG		DDRC	///< Data direction register for the port to which button 0 is connected
#define BUTTON_1_DDR_REG		DDRC	///< Data direction register for the port to which button 1 is connected
#define BUTTON_2_DDR_REG		DDRC	///< Data direction register for the port to which button 2 is connected
#define BUTTON_0_PORT_REG		PORTC	///< Port register for the port to which button 0 is connected
#define BUTTON_1_PORT_REG		PORTC	///< Port register for the port to which button 1 is connected
#define BUTTON_2_PORT_REG		PORTC	///< Port register for the port to which button 2 is connected
#define BUTTON_0_PIN_REG		PINC	///< Pin register for the port to which button 0 is connected
#define BUTTON_1_PIN_REG		PINC	///< Pin register for the port to which button 1 is connected
#define BUTTON_2_PIN_REG		PINC	///< Pin register for the port to which button 2 is connected
#define BUTTON_0_PCINT			PCINT21	///< Pin change interrupt number of the pin tat button 0 is connected to
#define BUTTON_1_PCINT			PCINT22	///< Pin change interrupt number of the pin tat button 1 is connected to
#define BUTTON_2_PCINT			PCINT23	///< Pin change interrupt number of the pin tat button 2 is connected to
#define BUTTON_PCINT_ENABLE_FLAG	PCIE2	///< Pin change interrupt enable flat for the port the buttons are connected to
#define BUTTON_PCINT_MASK_REG		PCMSK2	///< Pin change interrupt mask register for the port the buttons are connected to

#define BUTTONS_NUM			3	///< Total number of buttons
#define BUTTON_STATE_FLAG		0x01	///< Flag used to indicate the button state in \ref _button_state
#define BUTTON_CHANGED_FLAG		0x80	///< Flag used to indigate change of state of a button in \ref _button_state

#define BUTTON_CTRL			0	///< Assignement for the button that controls the state of the temperature controller

/* *** Button variables ************************************************************************************************************************************ */

extern uint8_t _button_state [BUTTONS_NUM];	///< Variable that holds the state and the changed flag of all buttons

/* *** Button functions ************************************************************************************************************************************ */

void buttons_init(void);			///< Initializes the port pins where the buttons are connected

/** \} */

#endif /* __BUTTONS_H__ */
