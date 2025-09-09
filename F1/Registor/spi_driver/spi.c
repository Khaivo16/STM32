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
	
#include "spi.h"



void SPIx_Init(SPI_TypeDef *SPIx, uint32_t Mode, uint8_t Baud, uint8_t Datasize)
{
	// AFIO
	RCC->APB2ENR |= 1<<0;
	
	if(SPIx == SPI1) {
		/**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
		RCC->APB2ENR |= 1<<12;
		GPIOx_Init(GPIOA,5,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		GPIOx_Init(GPIOA,6,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		GPIOx_Init(GPIOA,7,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
	}
	else if(SPIx == SPI2) {
		/**SPI1 GPIO Configuration
    PB13     ------> SPI1_SCK
    PB14    ------> SPI1_MISO
    PB15    ------> SPI1_MOSI
    */
		RCC->APB1ENR |= 1<<14;
		GPIOx_Init(GPIOB,13,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		GPIOx_Init(GPIOB,14,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		GPIOx_Init(GPIOB,15,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
	}
	
	SPIx->CR1 |= Mode;													// MASTER/SLAVE
	
	SPIx->CR1 |= 0<<10;													// 2LINE
	
	SPIx->CR1 |= 0<<0;													// SPI_PHASE_1EDGE
	
	SPIx->CR1 |= 1<<1;													// SPI_POLARITY_HIGH
	
	SPIx->CR1 &= ~(7<<3);	SPIx->CR1 |= Baud<<3;	// BAUD
	
	SPIx->CR1 |= Datasize<<11;									// Datasize
	
	SPIx->CR1 |= 1<<9;													// SPI_NSS_SOFT
	
	SPIx->CR1 |= 0<<7;													// MSB
	
	SPIx->CR1 |= 1<<6;													// EN SPI
}

void SPI_Send(SPI_TypeDef *SPIx, uint8_t Data)
{
	uint16_t t = 0;
	while((SPIx->SR & (1<<1)) == 0){			// 0: Tx buffer not empty
		t++;
		if(t>=0xffff) break;
	}
	
	// 1: Tx buffer empty
	SPIx->DR = Data;
}

uint8_t SPI_Rec(SPI_TypeDef *SPIx)
{
	uint16_t t = 0;
	while((SPIx->SR & (1<<0)) == 0){			// 0: Rx buffer empty
		t++;
		if(t>=0xffff) return 0;
	}
	
	// 1: Rx buffer not empty
	return SPIx->DR;
}

uint8_t SPI_Send_Rec(SPI_TypeDef *SPIx, uint8_t Data)
{
	uint16_t t = 0;
	
	while((SPIx->SR & (1<<1)) == 0){
		t++;
		if(t>=0xffff) break;
	}
	SPIx->DR = Data;
	
	
	t = 0;
	while((SPIx->SR & (1<<0)) == 0){
		t++;
		if(t>=0xffff) return 0;
	}
	
	return SPIx->DR;
}





