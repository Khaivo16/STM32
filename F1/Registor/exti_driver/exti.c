/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
	
#include "exti.h"




void EXTIx_Init(GPIO_TypeDef * GPIOx, uint8_t Pin, EXTI_Trigger Trigger)
{
	IRQn_Type IRQn;
	uint8_t port = 0;
	RCC->APB2ENR |= 1<<0;
	
	if(GPIOx==GPIOA) port = portA;
	if(GPIOx==GPIOB) port = portB;
	if(GPIOx==GPIOC) port = portC;
	if(GPIOx==GPIOD) port = portD;
	if(GPIOx==GPIOE) port = portE;
	
	if(Pin==0)	{IRQn=EXTI0_IRQn;}
	if(Pin==1)	{IRQn=EXTI1_IRQn;}
	if(Pin==2)	{IRQn=EXTI2_IRQn;}
	if(Pin==3)	{IRQn=EXTI3_IRQn;}
	if(Pin==4)	{IRQn=EXTI4_IRQn;}
	if(Pin==5)	{IRQn=EXTI9_5_IRQn;}
	if(Pin==6)	{IRQn=EXTI9_5_IRQn;}
	if(Pin==7)	{IRQn=EXTI9_5_IRQn;}
	if(Pin==8)	{IRQn=EXTI9_5_IRQn;}
	if(Pin==9)	{IRQn=EXTI9_5_IRQn;}
	if(Pin==10)	{IRQn=EXTI9_5_IRQn;}
	if(Pin==11)	{IRQn=EXTI15_10_IRQn;}
	if(Pin==12)	{IRQn=EXTI15_10_IRQn;}
	if(Pin==13)	{IRQn=EXTI15_10_IRQn;}
	if(Pin==14)	{IRQn=EXTI15_10_IRQn;}
	if(Pin==15)	{IRQn=EXTI15_10_IRQn;}
	if(Pin==16)	{IRQn=EXTI15_10_IRQn;}
	
	if(Trigger==RISING)				{GPIOx_Init(GPIOx,Pin,INPUT_PUPD,PD,0);	EXTI->RTSR |= 1<<Pin;}
	else if(Trigger==FALLING)	{GPIOx_Init(GPIOx,Pin,INPUT_PUPD,PU,0);EXTI->FTSR |= 1<<Pin;}
	else if(Trigger==CHANGE)	{GPIOx_Init(GPIOx,Pin,INPUT_FLOATING,NOPULL,0);EXTI->RTSR |= 1<<Pin; EXTI->FTSR |= 1<<Pin;}

	AFIO->EXTICR[Pin/4] &= ~(0xf<<((Pin%4)*4));AFIO->EXTICR[Pin/4] |= (port<<((Pin%4)*4));
	
	EXTI->IMR|= 1<<Pin;		//	Mo ngat cho line
	
	NVICx_Init(IRQn, 0x0a, Pin);
}

uint8_t __IO tang = 0;

void EXTI0_IRQHandler(void)
{
	if(EXTI->PR & (1<<0))	//// Kiem tra co
	{
		// xu lý ngat
	
		tang++;
	}
	EXTI->PR |= 1<<0;
}

void EXTI9_5_IRQHandler(void)
{
	if(EXTI->PR & (1<<5))	//// Kiem tra co
	{
		// xu lý ngat
		GPIOC->ODR ^= 1<<13;
	}
	EXTI->PR |= 1<<5;
}


void EXTI15_10_IRQHandler(void)
{
	if(EXTI->PR & (1<<10))	//// Kiem tra co
	{
		// xu lý ngat
		GPIOC->ODR ^= 1<<13;
	}
	EXTI->PR |= 1<<10;
}













