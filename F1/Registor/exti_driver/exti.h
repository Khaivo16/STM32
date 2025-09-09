#ifndef __EXTI_H_
#define __EXTI_H_

#include "gpio.h"
	
typedef enum {
	RISING,
	FALLING,
	CHANGE,
} EXTI_Trigger;

#define portA  0
#define portB  1
#define portC  2
#define portD  3
#define portE  4
	
void EXTIx_Init(GPIO_TypeDef * GPIOx, uint8_t Pin, EXTI_Trigger Trigger);
void EXTI0_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

#endif














