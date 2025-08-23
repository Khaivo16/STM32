#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	
#ifdef STM32F10X_MD
#define USE_USART1
#define USE_USART2
#define USE_USART3
#endif
#ifdef STM32F10X_HD
#define USE_USART1
#define USE_USART2
#define USE_USART3
#define USE_UART4
#define USE_UART5
#endif

#ifdef STM32F40X
#define USE_USART1
#define USE_USART2
#define USE_USART3
#define USE_UART4
#define USE_UART5
#define USE_USART6
#define USE_UART7
#define USE_UART8
#endif

typedef enum
{ 		Pins_PA9PA10,	//USART1
			Pins_PB6PB7,	////USART1
			Pins_PA2PA3,	//USART2
			Pins_PD5PD6,	//USART2
			Pins_PB10PB11,	//USART3
			Pins_PC10PC11,	//USART3//UART4
			Pins_PD8PD9,	//USART3
			Pins_PA0PA1,	//UART4
			Pins_PC12PD2,	//UART5
			Pins_PC6PC7,	//USART6
			Pins_PG14PG9,	//USART6
			Pins_PB8PE7,	//UART7
			Pins_PF7PF6,	//UART7
			Pins_PE1PE0		//UART8
}USART_Pins;

#define USART_REC_LEN  			200  	
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; 
extern u16 USART_RX_STA;  

void USARTx_Init(USART_TypeDef * USARTx, USART_Pins pins, u32 bound);
void USART_Putc(USART_TypeDef* USARTx, volatile char c);
void USART_Puts(USART_TypeDef* USARTx, char* str);
u8 USART_Getc(USART_TypeDef * USARTx );
u8 USART_Gets(USART_TypeDef * USARTx , char *buffer, u16 len);

void uart_init(u32 bound);
#endif


