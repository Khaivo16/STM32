#include "stm32f10x.h"
#include "LED.h"
#include "delay.h"
#include "usart.h"
#include "stdlib.h"
#include "EXTI.h"

#define LED1 PBout(12) 
#define ngat PBin(9)
#define ngat1 PAin(5)


void EXTI9_5_IRQHandler(void)
{
  //if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	delay_ms(10);
	if (ngat==1)
  {
    /* Toggle LED1 */
		LED1=!LED1;
    /* Clear the  EXTI line 0 pending bit */
   
  }
	if (ngat1==1)
  {
    /* Toggle LED1 */
		LED1=!LED1;
    /* Clear the  EXTI line 0 pending bit */
   
  }
	 EXTI_ClearITPendingBit(EXTI_Line9|EXTI_Line5);
}


int main(void)
 {	
	  
	 delay_init();
	 
	 
	 LED_init();
	 GPIO_SETUP(GPIOB,GPIO_Pin_12,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
	 LED1=1;
	 EXTIx_init(GPIOB,GPIO_Pin_9,EXTI_Trigger_Rising);
	 EXTIx_init(GPIOA,GPIO_Pin_5,EXTI_Trigger_Rising);

  while(1)
	{
			
		}

	}


