#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdint.h>
#include <avr/io.h>

#define ENCODER_INPUT_A		PB0
#define ENCODER_INPUT_B		PB1

#define ENCODER_STATE_1		((0 << ENCODER_INPUT_A) | (0 << ENCODER_INPUT_B))
#define ENCODER_STATE_2		((1 << ENCODER_INPUT_A) | (0 << ENCODER_INPUT_B))
#define ENCODER_STATE_3		((1 << ENCODER_INPUT_A) | (1 << ENCODER_INPUT_B))
#define ENCODER_STATE_4		((0 << ENCODER_INPUT_A) | (1 << ENCODER_INPUT_B))

extern volatile uint8_t _encoder_state;
extern volatile int8_t _encoder_increment;

extern void encoder_init(void);

#endif /* __ENCODER_H__ */
