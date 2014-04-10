/**
	\file
	\ingroup Encoder
	\brief Declaration of functions, variables, and constants related to the rotary encoder user interface

	\addtogroup Encoder Rotary encoder
	\brief Rotary encoder to manipulate the state of the temperature controller

	A rotary encoder is a device used in mechanical user interfaces.
	It can be rotated indefinitely in both directions.
	The implementation here is suited for optical, incremental rotary encoders with two channels.
	The channels output a digital signal of alternating ones and zeros when the encoder is turned.
	The two channels have a phase difference which allows to determine in which direction the encoder was rotated.
	In clockwise rotation the signal changes like this:
	Position	| p0	| p1	| p2	| p3	| p4	| p5	| p6	| p7	| p8	| p9	| ...
	--------------- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | -----
	Output A	| 0	| 0	| 1	| 1	| 0	| 0	| 1	| 1	| 0	| 0	| ...
	Output B	| 0	| 1	| 1	| 0	| 0	| 1	| 1	| 0	| 0	| 1	| ...

	The pin change interrupt is used to detect rotation of the encoder.
	The interrupt handler then decides whether the increment is positive or negative.
	It uses the previous state which is stored in the \ref _encoder_state variable and saves the new state after the increment is decided.
	The \ref _encoder_increment variable is incremented or decremented according to the decision.

	If \ref _encoder_increment is non-zero a task in the main() function handles the increment and updates whatever the user interface intends.
	That task then sets the \ref _encoder_increment variable to zero.

	\{
 */

#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdint.h>
#include <avr/io.h>

/* *** Encoder constants *********************************************************************************************************************************** */

#define ENCODER_INPUT_A		PB0							///< Microcontroller pin which encoder channel A is connected to
#define ENCODER_INPUT_B		PB1							///< Microcontroller pin which encoder channel B is connected to

#define ENCODER_STATE_1		((0 << ENCODER_INPUT_A) | (0 << ENCODER_INPUT_B))	///< Constant that identifies encoder state 1
#define ENCODER_STATE_2		((1 << ENCODER_INPUT_A) | (0 << ENCODER_INPUT_B))	///< Constant that identifies encoder state 2
#define ENCODER_STATE_3		((1 << ENCODER_INPUT_A) | (1 << ENCODER_INPUT_B))	///< Constant that identifies encoder state 3
#define ENCODER_STATE_4		((0 << ENCODER_INPUT_A) | (1 << ENCODER_INPUT_B))	///< Constant that identifies encoder state 4

#define ENCODER_SETPOINT_INCREMENT	25						///< The increment in degrees celsius (times 100) caused by the encoder

/* *** Encoder variables *********************************************************************************************************************************** */

extern volatile uint8_t _encoder_state;							///< Last state of the encoder
extern volatile int8_t _encoder_increment;						///< Increment of the encoder

/* *** Encoder functions *********************************************************************************************************************************** */

extern void encoder_init(void);								///< Initializes the input pins used for the encoder

/** \} */

#endif /* __ENCODER_H__ */
