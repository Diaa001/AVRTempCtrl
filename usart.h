#ifndef __USART_H__
#define __USART_H__

#include <stdint.h>

void USART_init(void);
void USART_send_bytes(const uint8_t * data, uint8_t length);
void USART_receive_bytes(uint8_t * data, uint8_t length);

#endif
