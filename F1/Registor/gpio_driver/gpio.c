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
	
#include "gpio.h"


void NVICx_Init(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
{ 
  uint32_t prioritygroup = 0x00U;
  
  
  prioritygroup = NVIC_GetPriorityGrouping();
  
  NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));
	/* Enable interrupt */
	NVIC_EnableIRQ(IRQn);
}

void GPIOx_Init(GPIO_TypeDef *GPIOx, uint8_t Pin, uint8_t Mode, uint8_t Pull, uint8_t Speed)
{
    RCC->APB2ENR |= (1 << 0);

    // Enable clock for GPIOx
    if (GPIOx == GPIOA)			 RCC->APB2ENR |= (1 << 2);
    else if (GPIOx == GPIOB) RCC->APB2ENR |= (1 << 3);
    else if (GPIOx == GPIOC) RCC->APB2ENR |= (1 << 4);
    else if (GPIOx == GPIOD) RCC->APB2ENR |= (1 << 5);

		// cau hinh Mode + Speed
    if (Pin < 8) 	{GPIOx->CRL &= ~(0xF << (Pin * 4));	GPIOx->CRL |= ((Mode<<2) + Speed)<<((Pin)*4);}
		else 					{GPIOx->CRH &= ~(0xF << ((Pin-8) * 4));	GPIOx->CRH |= ((Mode<<2) + Speed)<<((Pin-8)*4);}
		
		// cau hinh Pull
    if (Pull == PU) 			GPIOx->ODR |= (1 << Pin);
    else if (Pull == PD)	GPIOx->ODR &= ~(1 << Pin);
}

void GPIOx_WritePin(GPIO_TypeDef *GPIOx, uint8_t Pin, uint8_t PinState)
{
	if(PinState == SET) GPIOx->ODR |= SET<<Pin;
	else GPIOx->ODR &= ~(SET)<<Pin;
}

uint8_t GPIOx_ReadPin(GPIO_TypeDef *GPIOx, uint8_t Pin)
{
	if(GPIOx->IDR & (1<<Pin)) return SET;
	else return RESET;
}

void GPIOx_TogglePin(GPIO_TypeDef *GPIOx, uint8_t Pin, uint16_t TimeMs)
{
	GPIOx->ODR ^= 1<<Pin;
	DelayMs(TimeMs);
}

