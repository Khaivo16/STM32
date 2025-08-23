#ifndef __EXTI_H
#define __EXIT_H	 
#include "sys.h"  	
//////////////////////////////////////////////////////////////////////////////////	 
								  
////////////////////////////////////////////////////////////////////////////////// 	 
 /* Includes
 */

/* Default priority for NVIC */
#ifndef EXTI_PRIORITY
#define EXTI_PRIORITY	0x0A
#endif
/**
 * Interrupt trigger enumeration
 * 
 * Parameters:
 * 	- EXTI_Trigger_Rising:
 * 		Trigger interrupt on rising edge on line
 * 	- EXTI_Trigger_Falling:
 * 		Trigger interrupt on falling edge on line
 * 	- EXTI_Trigger_Rising_Falling:
 * 		Trigger interrupt on any edge on line
 */
typedef enum {
	Trigger_Rising = 0x08,
	Trigger_Falling = 0x0C,
	Trigger_Rising_Falling = 0x10
} EXTI_Trigger_t;

void EXTIx_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, EXTI_Trigger_t trigger);
void EXTIX_Init(void);	//		 					    
#endif

























