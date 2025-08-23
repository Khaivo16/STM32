#include "stm32f10x.h"
#include "LED.h"
#include "delay.h"
#include "stdlib.h"
#include "EXTI.h"
#include "uart.h"
#include "pwm.h"
#include "string.h"
#include <math.h>
#include "timer.h"
#include "INPUT_CAPTURE.h" 




int main(void)
{	
	delay_init();
	LED_init();
	uartx_config(USART1,pins_PA9_PA10,9600);
//	GPIO_SETUP(GPIOB,GPIO_Pin_0,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
//	GPIO_SETUP(GPIOA,GPIO_Pin_6,GPIO_Mode_IN_FLOATING,GPIO_Speed_50MHz);
//	GPIO_SETUP(GPIOA,GPIO_Pin_7,GPIO_Mode_IN_FLOATING,GPIO_Speed_50MHz);
	

	HCSR04_Init();
	

//	count=millis();
  while(1)
		
	{		
			send_num(USART1,HCSR04_GetDistance());
			delayms(100);
  } 
}

//void TIM2_IRQHandler(void)
//{
//  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
//  {
//		//LED1=!LED1;
//		count++;
//		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
//	}
//}

