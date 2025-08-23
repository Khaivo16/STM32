#include "stm32f10x.h"
#include "LED.h"
#include "delay.h"
#include "usart.h"
#include "stdlib.h"
#include "EXTI.h"
#include "ADC.h"

uint16_t count=0;

int main(void)
 {	
	  
	 delay_init();
	 
	 
	 LED_init();
	 ADCx_init(ADC1,ADC_Channel_7);

  while(1)
	{
	count=ADC_avr(ADC1,ADC_Channel_7,100);
	delay_ms(100);
		
			
		}

	}
//* Pinout
// *	CHANNEL			ADC1	ADC2	ADC3
// *	0							PA0		PA0		PA0
// *	1							PA1		PA1		PA1
// *	2							PA2		PA2		PA2
// *	3							PA3		PA3		PA3
// *	4							PA4		PA4		PF6
// *	5							PA5		PA5		PF7
// *	6							PA6		PA6		PF8
// *	7							PA7		PA7		PF9
// *	8							PB0		PB0		PF10
// *	9							PB1		PB1		PF3
// *	10						PC0		PC0		PC0
// *	11						PC1		PC1		PC1
// *	12						PC2		PC2		PC2
// *	13						PC3		PC3		PC3
// *	14						PC4		PC4		PF4
// *	15						PC5		PC5		PF5
// *
// * |----------------------------------------------------------------------
// */

