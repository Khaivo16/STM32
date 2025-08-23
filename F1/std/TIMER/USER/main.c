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

#define LED1 PBout(4)
#define echo PAin(1)
uint32_t count=0;



int main(void)
{	
	delay_init();
	LED_init();

	GPIO_SETUP(GPIOB,GPIO_Pin_4,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
	GPIO_SETUP(GPIOA,GPIO_Pin_1,GPIO_Mode_IN_FLOATING,GPIO_Speed_50MHz);
	Timerbase_init(TIM2,16000,1000);
//	count=millis();
  while(1)
	{
		
		
  } 
}

void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
		LED1=!LED1;
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}
}

