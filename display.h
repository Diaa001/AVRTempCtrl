#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <stdint.h>

#define DISPLAY_REG_NOP			0x0
#define DISPLAY_REG_DIGIT_0		0x1
#define DISPLAY_REG_DIGIT_1		0x2
#define DISPLAY_REG_DIGIT_2		0x3
#define DISPLAY_REG_DIGIT_3		0x4
#define DISPLAY_REG_DIGIT_4		0x5
#define DISPLAY_REG_DIGIT_5		0x6
#define DISPLAY_REG_DIGIT_6		0x7
#define DISPLAY_REG_DIGIT_7		0x8
#define DISPLAY_REG_DECODE_MODE		0x9
#define DISPLAY_REG_INTENSITY		0xA
#define DISPLAY_REG_SCAN_LIMIT		0xB
#define DISPLAY_REG_SHUTDOWN		0xC
#define DISPLAY_REG_DISPLAY_TEST	0xF

#define DISPLAY_CODE_B_MINUS		0xA
#define DISPLAY_CODE_B_CHAR_E		0xB
#define DISPLAY_CODE_B_CHAR_H		0xC
#define DISPLAY_CODE_B_CHAR_L		0xD
#define DISPLAY_CODE_B_CHAR_P		0xE
#define DISPLAY_CODE_B_BLANK		0xF

#define DISPLAY_CODE_DECIMAL_POINT	0x80

void display_init(void);
void display_temperature(int16_t temperature);
void display_humidity(int8_t humidity);

#endif /* __DISPLAY_H__ */
