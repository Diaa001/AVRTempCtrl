#ifndef __USART_H__
#define __USART_H__

#include <stdint.h>

#define RX_BUFFER_LENGTH 64
#define TX_BUFFER_LENGTH 64

extern volatile char rx_buffer [2][RX_BUFFER_LENGTH];
extern volatile uint8_t rx_buffer_pointer;
extern volatile uint8_t rx_buffer_sel;
extern volatile uint8_t rx_complete;
extern char tx_buffer [TX_BUFFER_LENGTH];

void USART_init(void);
void USART_send_bytes(const uint8_t * data, uint8_t length);
void USART_receive_bytes(uint8_t * data, uint8_t length);

#endif
