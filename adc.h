/**
	\file
	\ingroup ADC
	\brief Declaration of functions, variables, and constants related to the internal ADC

	\addtogroup ADC Internal ADC
	\brief Analog to digital converter, internal to the microcontroller

	The internal ADC (analog to digital converter) is used for all analog measurements
	except Pt1000 temperature sensor measurements.
	The ADC has 8 channels, some of which can be used in differential mode.
	Resolution is 10 bit and the reference is set to be the analog power voltage.

	Functions are provided to select the ADC channel and to handle conversions and their result.

	The starting of conversions is handled by a scheduled task.
	Upon completion of the conversion the ADC is set to cause an interrupt.
	This interrupt is handled by an interrupt handler that reads the result and sets the
	variable _ADC_result.
	Additionally it sets the ADC_CONVERSION_COMPLETE_BIT in the same variable.
	The flag is at bit position 0 because the 10 bit ADC result is left aligned in the 16 bit integer.

	\{
 */

#ifndef __ADC_H__
#define __ADC_H__

/* *** ADC variables *************************************************************************************************************************************** */

#define ADC_CONVERSION_COMPLETE_BIT	0				///< Flag that signals completion of an ADC conversion, set by the interrupt handler
extern volatile uint16_t _ADC_result;					///< Result of the latest ADC measurement plus flag for conversion complete

/* *** ADC macro functions ********************************************************************************************************************************* */

#define ADC_start_conversion() {ADCSRA |= (1 << ADSC);}			///< Starts an ADC conversion that will cause an interrupt when finished
#define ADC_wait_for_conversion() {while (ADCSRA & (1 << ADSC));}	///< Blocks until the ADC conversion is finished
#define ADC_get_result() ADC						///< Returns the latest ADC conversion result

#define ADC_select_channel_2() {\
	ADMUX &= ~(0x1f);\
	ADMUX |= (1 << MUX1);\
}									///< Programs the ADC to use channel 2 for measurements

#define ADC_select_channel_7() {\
	ADMUX &= ~(0x1f);\
	ADMUX |= (1 << MUX0) | (1 << MUX1) | (1 << MUX2);\
}									///< Programs the ADC to use channel 7 for measurements

#define ADC_select_channel_diff_1_0_10x() {\
	ADMUX &= ~(0x1f);\
	ADMUX |= (1 << MUX3);\
	ADMUX |= (1 << MUX0);\
}									///< Programs the ADC to use channel 0 and 1 in a differential measurement with gain 10

/* *** ADC functions *************************************************************************************************************************************** */

void ADC_init(void);							///< Initializes the ADC

/** \} */

#endif /* __ADC_H__ */
