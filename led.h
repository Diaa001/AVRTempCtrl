#ifndef __LED_H__
#define __LED_H__

#include <avr/io.h>

#define	LED_0		PC3
#define	LED_1		PC4
#define	LED_PORT	PORTC
#define	LED_DDR		DDRC

#define	LED_ACTIVE	LED_0
#define	LED_ERROR	LED_1
#define	LED_OFF		0
#define	LED_ON		1

extern void LED_init(void);
extern void LED_set(uint8_t led, uint8_t state);

#endif /* __LED_H__ */
