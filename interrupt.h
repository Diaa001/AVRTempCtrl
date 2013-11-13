#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

#include <stdint.h>
#include <avr/interrupt.h>

extern uint8_t _sreg_save;

#define interrupts_suspend()	{_sreg_save = SREG; cli();}
#define interrupts_resume()	{SREG = _sreg_save;}
#define interrupts_on()		sei()
#define interrupts_off()	cli()

#endif /* __INTERRUPT_H__ */
