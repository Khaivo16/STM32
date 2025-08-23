#include "EXTI.h"
void EXTIx_init(GPIO_TypeDef* GPIOx,u16 Pinx,EXTITrigger_TypeDef trig){
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	uint8_t GPIO_PortSource,GPIO_PinSource;
	uint32_t EXTI_Line;
	uint8_t NVIC_IRQChannel;    
	
	if (GPIOx==GPIOA){GPIO_PortSource=GPIO_PortSourceGPIOA;}
	else if (GPIOx==GPIOB){GPIO_PortSource=GPIO_PortSourceGPIOB;}
	else if (GPIOx==GPIOC){GPIO_PortSource=GPIO_PortSourceGPIOC;}
	else if (GPIOx==GPIOD){GPIO_PortSource=GPIO_PortSourceGPIOD;}
	else if (GPIOx==GPIOE){GPIO_PortSource=GPIO_PortSourceGPIOE;}		
//----------------------------------------------------------------------------		
	if (Pinx==GPIO_Pin_0){GPIO_PinSource=GPIO_PinSource0;EXTI_Line=EXTI_Line0;NVIC_IRQChannel=EXTI0_IRQn;}
	else if (Pinx==GPIO_Pin_1){GPIO_PinSource=GPIO_PinSource1;EXTI_Line=EXTI_Line1;NVIC_IRQChannel=EXTI1_IRQn;}
	else if (Pinx==GPIO_Pin_2){GPIO_PinSource=GPIO_PinSource2;EXTI_Line=EXTI_Line2;NVIC_IRQChannel=EXTI2_IRQn;}
	else if (Pinx==GPIO_Pin_3){GPIO_PinSource=GPIO_PinSource3;EXTI_Line=EXTI_Line3;NVIC_IRQChannel=EXTI3_IRQn;}
	else if (Pinx==GPIO_Pin_4){GPIO_PinSource=GPIO_PinSource4;EXTI_Line=EXTI_Line4;NVIC_IRQChannel=EXTI4_IRQn;}
	else if (Pinx==GPIO_Pin_5){GPIO_PinSource=GPIO_PinSource5;EXTI_Line=EXTI_Line5;NVIC_IRQChannel=EXTI9_5_IRQn;}
	else if (Pinx==GPIO_Pin_6){GPIO_PinSource=GPIO_PinSource6;EXTI_Line=EXTI_Line6;NVIC_IRQChannel=EXTI9_5_IRQn;}
	else if (Pinx==GPIO_Pin_7){GPIO_PinSource=GPIO_PinSource7;EXTI_Line=EXTI_Line7;NVIC_IRQChannel=EXTI9_5_IRQn;}
	else if (Pinx==GPIO_Pin_8){GPIO_PinSource=GPIO_PinSource8;EXTI_Line=EXTI_Line8;NVIC_IRQChannel=EXTI9_5_IRQn ;}
	else if (Pinx==GPIO_Pin_9){GPIO_PinSource=GPIO_PinSource9;EXTI_Line=EXTI_Line9;NVIC_IRQChannel=EXTI9_5_IRQn ;}
	else if (Pinx==GPIO_Pin_10){GPIO_PinSource=GPIO_PinSource10;EXTI_Line=EXTI_Line10;NVIC_IRQChannel=EXTI15_10_IRQn;}
	else if (Pinx==GPIO_Pin_11){GPIO_PinSource=GPIO_PinSource11;EXTI_Line=EXTI_Line11;NVIC_IRQChannel=EXTI15_10_IRQn;}
	else if (Pinx==GPIO_Pin_12){GPIO_PinSource=GPIO_PinSource12;EXTI_Line=EXTI_Line12;NVIC_IRQChannel=EXTI15_10_IRQn;}
	else if (Pinx==GPIO_Pin_13){GPIO_PinSource=GPIO_PinSource13;EXTI_Line=EXTI_Line13;NVIC_IRQChannel=EXTI15_10_IRQn;}
	else if (Pinx==GPIO_Pin_14){GPIO_PinSource=GPIO_PinSource14;EXTI_Line=EXTI_Line14;NVIC_IRQChannel=EXTI15_10_IRQn;}
	else if (Pinx==GPIO_Pin_15){GPIO_PinSource=GPIO_PinSource15;EXTI_Line=EXTI_Line15;NVIC_IRQChannel=EXTI15_10_IRQn;}
//--------------------------------------------------------------------------------------------------------------------------	
	if (trig==EXTI_Trigger_Rising){GPIO_SETUP(GPIOx,Pinx,GPIO_Mode_IPD,GPIO_Speed_50MHz);}
	else if (trig==EXTI_Trigger_Falling) {GPIO_SETUP(GPIOx,Pinx,GPIO_Mode_IPU,GPIO_Speed_50MHz);}
	else if (trig==EXTI_Trigger_Rising_Falling) {GPIO_SETUP(GPIOx,Pinx,GPIO_Mode_IN_FLOATING,GPIO_Speed_50MHz);}
			

  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI0 Line to PA.00 pin */
  GPIO_EXTILineConfig(GPIO_PortSource,GPIO_PinSource);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = trig;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = NVIC_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = GPIO_PinSource;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	
}