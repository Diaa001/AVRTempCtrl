#ifndef __BUTTONS_H__
#define __BUTTONS_H__

#include <avr/io.h>

#define BUTTON_0			PC5
#define BUTTON_1			PC6
#define BUTTON_2			PC7
#define BUTTON_0_DDR_REG		DDRC
#define BUTTON_1_DDR_REG		DDRC
#define BUTTON_2_DDR_REG		DDRC
#define BUTTON_0_PORT_REG		PORTC
#define BUTTON_1_PORT_REG		PORTC
#define BUTTON_2_PORT_REG		PORTC
#define BUTTON_0_PIN_REG		PINC
#define BUTTON_1_PIN_REG		PINC
#define BUTTON_2_PIN_REG		PINC
#define BUTTON_0_PCINT			PCINT21
#define BUTTON_1_PCINT			PCINT22
#define BUTTON_2_PCINT			PCINT23
#define BUTTON_PCINT_ENABLE_FLAG	PCIE2
#define BUTTON_PCINT_MASK_REG		PCMSK2

#define BUTTONS_NUM			3
#define BUTTON_STATE_FLAG		0x01
#define BUTTON_CHANGED_FLAG		0x80

extern uint8_t _button_state [BUTTONS_NUM];

void buttons_init(void);

#endif /* __BUTTONS_H__ */
