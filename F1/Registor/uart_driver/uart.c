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

#include "uart.h"



int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
 
	USARTx_PutC(USART1, ch);
	
  return ch;
}


void USARTx_Init(USART_TypeDef *USARTx, USART_Pin Pins, uint32_t  Speed)
{
	IRQn_Type IRQn;
	uint8_t u = 0;
	uint32_t PCLKx = 0;
	float USARTDIV = 0.00;
	uint16_t DIV_Mantissa = 0, DIV_Fraction = 0;
	
	RCC->APB2ENR |= 1<<0;
	
	if(USARTx == USART1)	{u =1; IRQn = USART1_IRQn;RCC->APB2ENR |= 1<<14;PCLKx = (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos]);}
	if(USARTx == USART2)	{u =2; IRQn = USART2_IRQn;RCC->APB1ENR |= 1<<17;PCLKx = (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);}
	if(USARTx == USART3)	{u =3; IRQn = USART3_IRQn;RCC->APB1ENR |= 1<<18;PCLKx = (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);}
	
	if(Pins == PA9PA10){
		GPIOx_Init(GPIOA, 9,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		GPIOx_Init(GPIOA, 10,INPUT_FLOATING,NOPULL,MODE_INPUT);
	}
	if(Pins == PB6PB7){
		AFIO->MAPR |= 1<<2;
		GPIOx_Init(GPIOB, 6,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		GPIOx_Init(GPIOB, 7,INPUT_FLOATING,NOPULL,MODE_INPUT);
	}
	if(Pins == PA2PA3){
		GPIOx_Init(GPIOA, 2,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		GPIOx_Init(GPIOA, 3,INPUT_FLOATING,NOPULL,MODE_INPUT);
	}
	if(Pins == PB10PB11){
		GPIOx_Init(GPIOB, 10,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		GPIOx_Init(GPIOB, 11,INPUT_FLOATING,NOPULL,MODE_INPUT);
	}
	
	USARTDIV=(float)PCLKx/(Speed*16.0); 
	
	DIV_Mantissa = (uint16_t)USARTDIV;
	
	DIV_Fraction = (uint16_t) (USARTDIV - DIV_Mantissa)*16;
	
	USARTx->BRR = (DIV_Mantissa<<4) + DIV_Fraction;
	
	USARTx->CR1 |= (1<<2)|(1<<3); // mode tx-rx
	//USARTx->CR1 |= 0<<12;	//	1 Start bit, 8 Data bits, n Stop bit
	//USARTx->CR2 &= ~(3<<12); USARTx->CR2 |=0<<12;	//1 Stop bit
	
	USARTx->CR1 |= (1<<5);	// RX-IT: ngat nhan
	
	NVICx_Init(IRQn, 0, u);
	
	USARTx->CR1 |= 1<<13;	// enable
}

void USARTx_PutC(USART_TypeDef *USARTx, char Data)
{
//	UART_HandleTypeDef *huart = Get_huart(USARTx);
//	HAL_UART_Transmit(huart, (uint8_t *)&Data, 1, 100);
	while(!(USARTx->SR & (1<<7)));			// USART_SR_TXE = 1<< 7;
	USARTx->DR = Data;
}

void USARTx_PutS(USART_TypeDef *USARTx, char *Str)
{
	while(*Str) USARTx_PutC(USARTx, *Str++);
}

char Buffer_USART1[200];
char Buffer_USART2[200];
char Buffer_USART3[200];

USART_ST USART1_ST = {Buffer_USART1, Size_USART1, 0, 0, 0};
USART_ST USART2_ST = {Buffer_USART2, Size_USART2, 0, 0, 0};
USART_ST USART3_ST = {Buffer_USART3, Size_USART3, 0, 0, 0};

// thêm 1 char
void USARTtoBUFF(USART_ST * u, char c)
{
	if(u->In < u->Size)
	{
		u->Buffer[u->In] = c;
		u->In++;
		u->Num++;
		if(u->In==u->Size) u->In=0;
	}
}

// tra ve so ky ty da lay
uint16_t USARTx_isEMPTY(USART_TypeDef *USARTx)
{
	USART_ST *u;
	if(USARTx == USART1){u =&USART1_ST;}
	if(USARTx == USART2){u =&USART2_ST;}
	if(USARTx == USART3){u =&USART3_ST;}
	return u->Num;
}

// lay 1 ki tu
char USARTx_GetC(USART_TypeDef *USARTx)
{
	USART_ST * u;
	char c = 0;
	
	if(USARTx == USART1){u =&USART1_ST;}
	if(USARTx == USART2){u =&USART2_ST;}
	if(USARTx == USART3){u =&USART3_ST;}
	
	if(u->Num > 0)
	{
		c = u->Buffer[u->Out];
		u->Out++;
		u->Num--;
		if(u->Out==u->Size) u->Out=0;
	}
	return c;
}

uint16_t USARTx_GetS(USART_TypeDef *USARTx, char *Buffer, uint16_t Size)
{
	char c = 0;
	uint16_t i = 0;
	
	if(USARTx_isEMPTY(USARTx) == 0) return 0;
	
	while(i<Size)
	{
		c = USARTx_GetC(USARTx);
		if(c)
		{
			Buffer[i] = c;
			if(Buffer[i] == '\n') {i++; break;}			// ket thuc 1 chuoi nhan
			else i++;
		}
	}
	Buffer[i] = '\0';														// ky tu chuoi luon = \0
	return i;																		// i != 0 có nghia là da lay duco du lieu
}


void USART1_IRQHandler(void)
{
	if ((USART1->SR & USART_SR_RXNE) != RESET){
		USARTtoBUFF(&USART1_ST, USART1->DR);
//		__HAL_UART_GET_FLAG(huart, USART_SR_RXNE);
	}
}
void USART2_IRQHandler(void)
{
	if ((USART2->SR & USART_SR_RXNE) != RESET){
		USARTtoBUFF(&USART2_ST, USART2->DR);
//		__HAL_UART_GET_FLAG(huart, USART_SR_RXNE);
	}

}
void USART3_IRQHandler(void)
{
	if ((USART3->SR & USART_SR_RXNE) != RESET){
		USARTtoBUFF(&USART3_ST, USART3->DR);
//		__HAL_UART_GET_FLAG(huart, USART_SR_RXNE);
	}
}









