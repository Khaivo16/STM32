#include "stm32f10x.h"
#include "LED.h"
#include "delay.h"
#include "stdlib.h"
#include "EXTI.h"
#include "uart.h"
#include "pwm.h"
#include "string.h"
#include <math.h>

#define LED1 PBout(12)





int main(void)
{	
	delay_init();
	LED_init();
	PWMx_init(TIM4,CH1,PP1,100,72);
	PWMx_init(TIM4,CH2,PP1,100,72);
	PWMx_init(TIM4,CH3,PP1,100,72);
	uartx_config(USART1,pins_PA9_PA10,9600);
	char c;
	char str[10];
	int pwm=0;
	int val1,val2,val3=0;
	
  while(1)
	{
			if (get_str(USART1,str,10)) {
				val1 = atof(strtok(str, ","));
				val2 = atof(strtok(NULL, ","));
				val3 = atoi(strtok(NULL, "\n"));
				TIM4->CCR1=val1;
				TIM4->CCR2=val2;
				TIM4->CCR3=val3;
				}
			
		
  } 
}

