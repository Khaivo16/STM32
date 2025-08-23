#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 

//SysTick STM32F10x
//delay_us,delay_ms

////////////////////////////////////////////////////////////////////////////////// 
	 
void delay_init(void);
void delayms(uint32_t time);
void delayus(uint32_t time);
uint32_t millis(void);
u32 micros(void);

#endif





























