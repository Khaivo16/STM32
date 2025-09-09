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


#include "adc.h"

void ADCx_Init(ADC_TypeDef  *ADCx, uint8_t Channel)
{
	RCC->APB2ENR |= 1<<0;  //AFIO
	
	if(ADCx == ADC1)	RCC->APB2ENR |= 1<<9;	//ADC1
	if(ADCx == ADC2)	RCC->APB2ENR |= 1<<10;	//ADC2
	
	if(Channel == 0) GPIOx_Init(GPIOA,0,INPUT_ANALOG,NOPULL,MODE_INPUT);
	if(Channel == 1) GPIOx_Init(GPIOA,1,INPUT_ANALOG,NOPULL,MODE_INPUT);
	if(Channel == 2) GPIOx_Init(GPIOA,2,INPUT_ANALOG,NOPULL,MODE_INPUT);
	if(Channel == 3) GPIOx_Init(GPIOA,3,INPUT_ANALOG,NOPULL,MODE_INPUT);
	if(Channel == 4) GPIOx_Init(GPIOA,4,INPUT_ANALOG,NOPULL,MODE_INPUT);
	if(Channel == 5) GPIOx_Init(GPIOA,5,INPUT_ANALOG,NOPULL,MODE_INPUT);
	if(Channel == 6) GPIOx_Init(GPIOA,6,INPUT_ANALOG,NOPULL,MODE_INPUT);
	if(Channel == 7) GPIOx_Init(GPIOA,7,INPUT_ANALOG,NOPULL,MODE_INPUT);
	if(Channel == 8) GPIOx_Init(GPIOB,0,INPUT_ANALOG,NOPULL,MODE_INPUT);
	if(Channel == 9) GPIOx_Init(GPIOB,1,INPUT_ANALOG,NOPULL,MODE_INPUT);
	if(Channel == 10) GPIOx_Init(GPIOC,0,INPUT_ANALOG,NOPULL,MODE_INPUT);
	if(Channel == 11) GPIOx_Init(GPIOC,1,INPUT_ANALOG,NOPULL,MODE_INPUT);
	if(Channel == 12) GPIOx_Init(GPIOC,2,INPUT_ANALOG,NOPULL,MODE_INPUT);
	if(Channel == 13) GPIOx_Init(GPIOC,3,INPUT_ANALOG,NOPULL,MODE_INPUT);
	if(Channel == 14) GPIOx_Init(GPIOC,4,INPUT_ANALOG,NOPULL,MODE_INPUT);
	if(Channel == 15) GPIOx_Init(GPIOC,5,INPUT_ANALOG,NOPULL,MODE_INPUT);
	
	ADCx->CR1 |= 0<<8; //DIS SCAN
	
	ADCx->CR2 |= (1<<1) | (1<<0);	// Continuous conversion Mode + ADON
	
	ADCx->CR2 |= 0<<11;	//Right Aligment
	
	ADCx->CR2 &= ~(7<<17);	ADCx->CR2 |= (7<<17);	// SWSTART
	
	ADCx->SQR1 &= ~(0xf<<20);	ADCx->SQR1 |= 0<<20;	// 1 conversion
	
	//SamplingTime
	if(Channel < 10) {ADCx->SMPR2 &= ~(7<<(Channel*3));	ADCx->SMPR2 |= (7<<17);	}// 239.5 cycles //SamplingTime
	else {ADCx->SMPR2 &= ~(7<<(Channel*3));	ADCx->SMPR2 |= (7<<((Channel-10)*3)); }
	
	ADCx->SQR3 = Channel;	// RANK1
	
	ADCx->CR2 |= (1<<3);	// RESET CALI
	while(ADCx->CR2 & (1<<3));
	
	ADCx->CR2 |= (1<<2);	// CALI
	while(ADCx->CR2 & (1<<2));
	
	ADCx->CR2 |= (1<<0);	// ADC  EN = ADON
}

uint16_t Timeout = 0;
//#define Ti 0xffff
uint16_t ADCx_Read(ADC_TypeDef  *ADCx, uint8_t Channel)
{
	//SamplingTime
	if(Channel < 10) {ADCx->SMPR2 &= ~(7<<(Channel*3));	ADCx->SMPR2 |= (7<<17);	}// 239.5 cycles 
	else {ADCx->SMPR2 &= ~(7<<(Channel*3));	ADCx->SMPR2 |= (7<<((Channel-10)*3)); }
	
	ADCx->SQR3 = Channel;	// RANK1
	
	ADCx->CR2 |= (1<<22) | (1<<0);	// ADC  EN = ADON
	// End od conversion
	
	Timeout = 100;
	while((ADCx->SR & (1<<1)) == 0 && Timeout) {--Timeout;}
	
	return ADCx->DR;
}






