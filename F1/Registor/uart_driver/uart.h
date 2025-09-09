#ifndef __UART_H_
#define __UART_H_

#include "gpio.h"
#include "stdio.h"
#include "string.h"

typedef enum
{
  PA9PA10,	//USART1
  PB6PB7,		//USART1
	PA2PA3,		//USART2
	PB10PB11	//USART3
} USART_Pin;

typedef struct
{
	char *Buffer;
  uint16_t Size;
	uint16_t In;
	uint16_t Out;
	uint16_t Num;
} USART_ST;

#define Size_USART1 200
#define Size_USART2 200
#define Size_USART3 200



void USARTx_Init(USART_TypeDef *USARTx, USART_Pin Pins, uint32_t  Speed);

void USARTx_PutC(USART_TypeDef *USARTx, char Data);

void USARTx_PutS(USART_TypeDef *USARTx, char *Str);


void USARTtoBUFF(USART_ST * u, char c);

uint16_t USARTx_isEMPTY(USART_TypeDef *USARTx);

char USARTx_GetC(USART_TypeDef *USARTx);
uint16_t USARTx_GetS(USART_TypeDef *USARTx, char *Buffer, uint16_t Size);


void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);



#endif








