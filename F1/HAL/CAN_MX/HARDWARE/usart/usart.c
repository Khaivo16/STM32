/**	
 * |----------------------------------------------------------------------
			Pins_PA9PA10,	//USART1
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
 * |----------------------------------------------------------------------
 */
#include "usart.h"


typedef struct {
	int8_t *Buffer;
	uint16_t Size;
	uint16_t Num;
	uint16_t In;
	uint16_t Out;
} BUFFER_t;

/* Set alternate function mappings */
////#if defined(STM32F4xx) || defined(STM32F7xx)

//#if defined(USART1)
//#define GPIO_AF_USART1     GPIO_AF7_USART1
//#define IRQ_USART1         USART1_IRQn
//#endif
//#if defined(USART2)
//#define GPIO_AF_USART2     GPIO_AF7_USART2
//#define IRQ_USART2         USART2_IRQn
//#endif
//#if defined(USART3)
//#define GPIO_AF_USART3     GPIO_AF7_USART3
//#define IRQ_USART3         USART3_IRQn
//#endif
//#if defined(UART4)
//#define GPIO_AF_UART4      GPIO_AF8_UART4
//#define IRQ_UART4          UART4_IRQn
//#endif
//#if defined(UART5)
//#define IRQ_UART5          UART5_IRQn
//#endif
//#if defined(USART6)
//#define GPIO_AF_USART6     GPIO_AF8_USART6
//#define IRQ_USART6         USART6_IRQn
//#endif
//#if defined(UART7)
//#define GPIO_AF_UART7      GPIO_AF8_UART7
//#define IRQ_UART7          UART7_IRQn
//#endif
//#if defined(UART8)
//#define GPIO_AF_UART8      GPIO_AF8_UART8
//#define IRQ_UART8          UART8_IRQn
//#endif



//#if defined(USART1)
//#define IRQ_USART1         USART1_IRQn
//#endif
//#if defined(USART2)
//#define IRQ_USART2         USART2_IRQn
//#endif
//#if defined(USART8)
//#define IRQ_USART3         USART3_8_IRQn
//#define IRQ_USART4         USART3_8_IRQn
//#define IRQ_USART5         USART3_8_IRQn
//#define IRQ_USART6         USART3_8_IRQn
//#define IRQ_USART7         USART3_8_IRQn
//#define IRQ_USART8         USART3_8_IRQn
//#elif defined(USART6)
//#define IRQ_USART3         USART3_6_IRQn
//#define IRQ_USART4         USART3_6_IRQn
//#define IRQ_USART5         USART3_6_IRQn
//#define IRQ_USART6         USART3_6_IRQn
//#elif defined(USART4)
//#define IRQ_USART3         USART3_4_IRQn
//#define IRQ_USART4         USART3_4_IRQn
//#endif

//#endif

/* Set variables for buffers */
#ifdef USE_USART1
int8_t USART1_Buffer[USART1_BUFFER_SIZE];
#endif
#ifdef USE_USART2
int8_t USART2_Buffer[USART2_BUFFER_SIZE];
#endif
#ifdef USE_USART3
int8_t USART3_Buffer[USART3_BUFFER_SIZE];
#endif
#ifdef USE_UART4
int8_t UART4_Buffer[UART4_BUFFER_SIZE];
#endif
#ifdef USE_UART5
int8_t UART5_Buffer[UART5_BUFFER_SIZE];
#endif
#ifdef USE_USART6
int8_t USART6_Buffer[USART6_BUFFER_SIZE];
#endif
#ifdef USE_UART7
int8_t UART7_Buffer[UART7_BUFFER_SIZE];
#endif
#ifdef USE_UART8
int8_t UART8_Buffer[UART8_BUFFER_SIZE];
#endif

/* STM32F0xx added */
//#ifdef USE_USART4
//uint8_t USART4_Buffer[USART4_BUFFER_SIZE];
//#endif
//#ifdef USE_USART5
//uint8_t USART5_Buffer[USART5_BUFFER_SIZE];
//#endif
//#ifdef USE_USART7
//uint8_t USART7_Buffer[USART7_BUFFER_SIZE];
//#endif
//#ifdef USE_USART8
//uint8_t USART8_Buffer[USART8_BUFFER_SIZE];
//#endif

#ifdef USE_USART1
BUFFER_t USART1x = {USART1_Buffer, USART1_BUFFER_SIZE, 0, 0, 0};
#endif
#ifdef USE_USART2
BUFFER_t USART2x = {USART2_Buffer, USART2_BUFFER_SIZE, 0, 0, 0};
#endif
#ifdef USE_USART3
BUFFER_t USART3x = {USART3_Buffer, USART3_BUFFER_SIZE, 0, 0, 0};
#endif
#ifdef USE_UART4
BUFFER_t UART4x = {UART4_Buffer, UART4_BUFFER_SIZE, 0, 0, 0};
#endif
#ifdef USE_UART5
BUFFER_t UART5x = {UART5_Buffer, UART5_BUFFER_SIZE, 0, 0, 0};
#endif
#ifdef USE_USART6
BUFFER_t USART6x = {USART6_Buffer, USART6_BUFFER_SIZE, 0, 0, 0};
#endif
#ifdef USE_UART7
BUFFER_t UART7x = {UART7_Buffer, UART7_BUFFER_SIZE, 0, 0, 0};
#endif
#ifdef USE_UART8
BUFFER_t UART8x = {UART8_Buffer, UART8_BUFFER_SIZE, 0, 0, 0};
#endif

/* Private functions */
void USART1_InitPins(USART_PinsPack_t pinspack);
void USART2_InitPins(USART_PinsPack_t pinspack);
void USART3_InitPins(USART_PinsPack_t pinspack);
void UART4_InitPins(USART_PinsPack_t pinspack);
void UART5_InitPins(USART_PinsPack_t pinspack);
void USART6_InitPins(USART_PinsPack_t pinspack);
void UART7_InitPins(USART_PinsPack_t pinspack);
void UART8_InitPins(USART_PinsPack_t pinspack);
static void USART_INT_InsertToBuffer(BUFFER_t* u, uint8_t c);
static void USART_INT_ClearAllFlags(USART_TypeDef* USARTx, IRQn_Type irq);
static BUFFER_t* USART_INT_GetUSARTBuffer(USART_TypeDef* USARTx);
static uint8_t USART_INT_GetSubPriority(USART_TypeDef* USARTx);
uint8_t USART_BufferFull(USART_TypeDef* USARTx);

/* Private initializator function */
static void USART_INT_Init(
	USART_TypeDef* USARTx,
	USART_PinsPack_t pinspack,
	uint32_t baudrate,
	USART_HardwareFlowControl_t FlowControl,
	uint32_t Mode,
	uint32_t Parity,
	uint32_t StopBits,
	uint32_t WordLength
);

// Th¨ºm m? sau ?ay ?? h? tr? ch?c n?ng printf, m¨¤ kh?ng c?n ph?i ch?n MicroLIB s? d?ng  
//	u8 t;
//printf("t:%d\r\n",t);
/* We need to implement own __FILE struct */
/* FILE struct is used from __FILE */
struct __FILE {
    int dummy;
};
 
/* You need this if you want use printf */
/* Struct FILE is implemented in stdio.h */
FILE __stdout;

/* Printf handler */
int fputc(int ch, FILE* fil) {
	/* Send over USART */
	USART_Putc(USART1, ch);
	
	/* Return character */
	return ch;
}

void USART_Init(USART_TypeDef* USARTx, USART_PinsPack_t pinspack, uint32_t baudrate) {
#ifdef USE_USART1
	if (USARTx == USART1) {
		USART_INT_Init(USART1, pinspack, baudrate, USART1_HARDWARE_FLOW_CONTROL, USART1_MODE, USART1_PARITY, USART1_STOP_BITS, USART1_WORD_LENGTH);
	}
#endif
#ifdef USE_USART2
	if (USARTx == USART2) {
		USART_INT_Init(USART2, pinspack, baudrate, USART2_HARDWARE_FLOW_CONTROL, USART2_MODE, USART2_PARITY, USART2_STOP_BITS, USART2_WORD_LENGTH);
	}
#endif
#ifdef USE_USART3
	if (USARTx == USART3) {
		USART_INT_Init(USART3, pinspack, baudrate, USART3_HARDWARE_FLOW_CONTROL, USART3_MODE, USART3_PARITY, USART3_STOP_BITS, USART3_WORD_LENGTH);
	}
#endif
#ifdef USE_UART4
	if (USARTx == UART4) {
		USART_INT_Init(UART4, pinspack, baudrate, UART4_HARDWARE_FLOW_CONTROL, UART4_MODE, UART4_PARITY, UART4_STOP_BITS, UART4_WORD_LENGTH);
	}
#endif
#ifdef USE_UART5
	if (USARTx == UART5) {
		USART_INT_Init(UART5, pinspack, baudrate, UART5_HARDWARE_FLOW_CONTROL, UART5_MODE, UART5_PARITY, UART5_STOP_BITS, UART5_WORD_LENGTH);
	}
#endif
#ifdef USE_USART6
//	if (USARTx == USART6) {
//		USART_INT_Init(USART6, pinspack, baudrate, USART6_HARDWARE_FLOW_CONTROL, USART6_MODE, USART6_PARITY, USART6_STOP_BITS, USART6_WORD_LENGTH);
//	}
#endif
#ifdef USE_UART7
	if (USARTx == UART7) {
		USART_INT_Init(UART7, pinspack, baudrate, UART7_HARDWARE_FLOW_CONTROL, UART7_MODE, UART7_PARITY, UART7_STOP_BITS, UART7_WORD_LENGTH);
	}
#endif
#ifdef USE_UART8
	if (USARTx == UART8) {
		USART_INT_Init(UART8, pinspack, baudrate, UART8_HARDWARE_FLOW_CONTROL, UART8_MODE, UART8_PARITY, UART8_STOP_BITS, UART8_WORD_LENGTH);
	}
#endif
	
/* STM32F0xx related */
//#ifdef USART4
//	if (USARTx == USART4) {
//		USART_INT_Init(USART4, pinspack, baudrate, USART4_HARDWARE_FLOW_CONTROL, USART4_MODE, USART4_PARITY, USART4_STOP_BITS, USART4_WORD_LENGTH);
//	}
//#endif	
//#ifdef USART5
//	if (USARTx == USART5) {
//		USART_INT_Init(USART5, pinspack, baudrate, USART5_HARDWARE_FLOW_CONTROL, USART5_MODE, USART5_PARITY, USART5_STOP_BITS, USART5_WORD_LENGTH);
//	}
//#endif	
//#ifdef USART7
//	if (USARTx == USART7) {
//		USART_INT_Init(USART7, pinspack, baudrate, USART7_HARDWARE_FLOW_CONTROL, USART7_MODE, USART7_PARITY, USART7_STOP_BITS, USART7_WORD_LENGTH);
//	}
//#endif	
//#ifdef USART8
//	if (USARTx == USART8) {
//		USART_INT_Init(USART8, pinspack, baudrate, USART8_HARDWARE_FLOW_CONTROL, USART8_MODE, USART8_PARITY, USART8_STOP_BITS, USART8_WORD_LENGTH);
//	}
//#endif
}



uint8_t USART_Getc(USART_TypeDef* USARTx) {
	int8_t c = 0;
	BUFFER_t* u = USART_INT_GetUSARTBuffer(USARTx);
	
	/* Check if we have any data in buffer */
	if (u->Num > 0) {
		if (u->Out == u->Size) {
			u->Out = 0;
		}
		c = *(u->Buffer + u->Out);
		u->Out++;
		u->Num--;
	}
	return c;
}

uint16_t USART_Gets(USART_TypeDef* USARTx, char* buffer, uint16_t bufsize) {
	uint16_t i = 0;
	
	/* Check for any data on USART */
	if (USART_BufferEmpty(USARTx) || (!USART_FindCharacter(USARTx, '\n') && !USART_BufferFull(USARTx))) {
		return 0;
	}
	
	/* If available buffer size is more than 0 characters */
	while (i < (bufsize - 1)) {
		/* We have available data */
		buffer[i] = (char) USART_Getc(USARTx);
		/* Check for end of string */
		if (buffer[i] == '\n') {
			i++;
			/* Done */
			break;
		} else {
			i++;
		}
	}
	
	/* Add zero to the end of string */
	buffer[i] = 0;               

	return (i);
}

void USART_Puts(USART_TypeDef* USARTx, char* str) {
	/* Go through entire string */
	while (*str) {
		/* Wait to be ready, buffer empty */
		USART_WAIT(USARTx);
		/* Send data */
		USART_WRITE_DATA(USARTx, (uint16_t)(*str++));
		/* Wait to be ready, buffer empty */
		USART_WAIT(USARTx);
	}
}

void USART_Send(USART_TypeDef* USARTx, uint8_t* DataArray, uint16_t count) {
	/* Go through entire data array */
	while (count--) {
		/* Wait to be ready, buffer empty */
		USART_WAIT(USARTx);
		/* Send data */
		USART_WRITE_DATA(USARTx, (uint16_t)(*DataArray++));
		/* Wait to be ready, buffer empty */
		USART_WAIT(USARTx);
	}
}

int16_t USART_FindCharacter(USART_TypeDef* USARTx, uint8_t c) {
	uint16_t num, out;
	BUFFER_t* u = USART_INT_GetUSARTBuffer(USARTx);
	
	/* Temp variables */
	num = u->Num;
	out = u->Out;
	
	while (num > 0) {
		/* Check overflow */
		if (out == u->Size) {
			out = 0;
		}
		if (u->Buffer[out] == c) {
			/* Character found */
			return 1;
		}
		out++;
		num--;
	}
	
	/* Character is not in buffer */
	return 0;
}

//int16_t USART_FindString(USART_TypeDef* USARTx, char* str) {
//	return BUFFER_Find(USART_INT_GetUSARTBuffer(USARTx), (uint8_t *)str, strlen(str));
//}

uint8_t USART_BufferEmpty(USART_TypeDef* USARTx) {
	BUFFER_t* u = USART_INT_GetUSARTBuffer(USARTx);
	return (u->Num == 0);
}

uint8_t USART_BufferFull(USART_TypeDef* USARTx) {
	BUFFER_t* u = USART_INT_GetUSARTBuffer(USARTx);
	return (u->Num == u->Size);
}

//uint16_t USART_BufferCount(USART_TypeDef* USARTx) {
//	return BUFFER_GetFull(USART_INT_GetUSARTBuffer(USARTx));
//}

void USART_ClearBuffer(USART_TypeDef* USARTx) {
	BUFFER_t* u = USART_INT_GetUSARTBuffer(USARTx);
	
	u->Num = 0;
	u->In = 0;
	u->Out = 0;
}

//void USART_SetCustomStringEndCharacter(USART_TypeDef* USARTx, uint8_t Character) {
//	BUFFER_SetStringDelimiter(USART_INT_GetUSARTBuffer(USARTx), Character);
//}

/************************************/
/*              CALLBACKS           */
/************************************/
__weak void USART_InitCustomPinsCallback(USART_TypeDef* USARTx, uint16_t AlternateFunction) { 
	/* NOTE: This function Should not be modified, when the callback is needed,
           the USART_InitCustomPinsCallback could be implemented in the user file
	*/
}

__weak void USART1_ReceiveHandler(uint8_t c) { }
__weak void USART2_ReceiveHandler(uint8_t c) { }
__weak void USART3_ReceiveHandler(uint8_t c) { }
__weak void UART4_ReceiveHandler(uint8_t c) { }
__weak void UART5_ReceiveHandler(uint8_t c) { }
__weak void USART6_ReceiveHandler(uint8_t c) { }
__weak void UART7_ReceiveHandler(uint8_t c) { }
__weak void UART8_ReceiveHandler(uint8_t c) { }

/* Private functions */
static void USART_INT_InsertToBuffer(BUFFER_t* u, uint8_t c) {
	/* Still available space in buffer */
	if (u->Num < u->Size) {
		/* Check overflow */
		if (u->In == u->Size) {
			u->In = 0;
		}
		/* Add to buffer */
		u->Buffer[u->In] = c;
		u->In++;
		u->Num++;
	}
}

static BUFFER_t* USART_INT_GetUSARTBuffer(USART_TypeDef* USARTx) {
	BUFFER_t* u;
	
#ifdef USE_USART1
	if (USARTx == USART1) {
		u = &USART1x;
	}
#endif
#ifdef USE_USART2
	if (USARTx == USART2) {
		u = &USART2x;
	}
#endif
#ifdef USE_USART3
	if (USARTx == USART3) {
		u = &USART3x;
	}
#endif
#ifdef USE_UART4
	if (USARTx == UART4) {
		u = &UART4x;
	}
#endif
#ifdef USE_UART5
	if (USARTx == UART5) {
		u = &UART5x;
	}
#endif
#ifdef USE_USART6
//	if (USARTx == USART6) {
//		u = &USART6x;
//	}
#endif
#ifdef USE_UART7
	if (USARTx == UART7) {
		u = &UART7x;
	}
#endif
#ifdef USE_UART8
	if (USARTx == UART8) {
		u = &UART8x;
	}
#endif

/* STM32F0xx related */
//#ifdef USE_USART4
//	if (USARTx == USART4) {
//		u = &USART4x;
//	}
//#endif
//#ifdef USE_USART5
//	if (USARTx == USART5) {
//		u = &USART5x;
//	}
//#endif
//#ifdef USE_USART7
//	if (USARTx == USART7) {
//		u = &USART7x;
//	}
//#endif
//#ifdef USE_USART8
//	if (USARTx == USART8) {
//		u = &USART8x;
//	}
//#endif

	return u;
}

static uint8_t USART_INT_GetSubPriority(USART_TypeDef* USARTx) {
	uint8_t u;
	
#ifdef USE_USART1
	if (USARTx == USART1) {
		u = 0;
	}
#endif
#ifdef USE_USART2
	if (USARTx == USART2) {
		u = 1;
	}
#endif
#ifdef USE_USART3
	if (USARTx == USART3) {
		u = 2;
	}
#endif
#ifdef USE_UART4
	if (USARTx == UART4) {
		u = 4;
	}
#endif
#ifdef USE_UART5
	if (USARTx == UART5) {
		u = 5;
	}
#endif
#ifdef USE_USART6
//	if (USARTx == USART6) {
//		u = 6;
//	}
#endif
#ifdef USE_UART7
	if (USARTx == UART7) {
		u = 7;
	}
#endif
#ifdef USE_UART8
	if (USARTx == UART8) {
		u = 8;
	}
#endif
	

/* STM32F0xx related */
//#ifdef USE_USART4
//	if (USARTx == USART4) {
//		u = 4;
//	}
//#endif
//#ifdef USE_USART5
//	if (USARTx == USART5) {
//		u = 5;
//	}
//#endif
//#ifdef USE_USART7
//	if (USARTx == USART7) {
//		u = 7;
//	}
//#endif
//#ifdef USE_USART8
//	if (USARTx == USART8) {
//		u = 8;
//	}
//#endif
	
	return u;
}

/* PIN initializations */
#ifdef USE_USART1
void USART1_InitPins(USART_PinsPack_t pinspack) {	
	/* Init pins */
#if defined(GPIOA)
	if (pinspack == Pins_PA9PA10) {
		//GPIO_InitAlternate(GPIOA, GPIO_PIN_9 | GPIO_PIN_10, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_USART1);
		pinMode(PA9,AF_PP);
		pinMode(PA10,INPUT);
	}
#endif
#if defined(GPIOB)
	if (pinspack == Pins_PB6PB7) {
		//GPIO_InitAlternate(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_USART1);
		pinMode(PB6,AF_PP);
		pinMode(PB7,INPUT);
	}
#endif
#if defined(GPIOA) && defined(GPIOB)
	if (pinspack == Pins_PA9PB7) {
		//GPIO_InitAlternate(GPIOA, GPIO_PIN_9, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_USART1);
		//GPIO_InitAlternate(GPIOB, GPIO_PIN_7, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_USART1);
		pinMode(PA9,AF_PP);
		pinMode(PB7,INPUT);
	}
#endif
	if (pinspack == USART_PinsPack_Custom) {
		/* Init custom pins, callback used */
		//USART_InitCustomPinsCallback(USART1, GPIO_AF_USART1);
	}
}
#endif

#ifdef USE_USART2
void USART2_InitPins(USART_PinsPack_t pinspack) {
	/* Init pins */
#if defined(GPIOA)
	if (pinspack == Pins_PA2PA3) {
		//GPIO_InitAlternate(GPIOA, GPIO_PIN_2 | GPIO_PIN_3, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_USART2);
		pinMode(PA2,AF_PP);
		pinMode(PA3,INPUT);
	}
#endif
#if defined(GPIOD)
	if (pinspack == Pins_PD5PD6) {
		//GPIO_InitAlternate(GPIOD, GPIO_PIN_5 | GPIO_PIN_6, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_USART2);
		pinMode(PD5,AF_PP);
		pinMode(PD6,INPUT);
	}
#endif
	if (pinspack == USART_PinsPack_Custom) {
		/* Init custom pins, callback used */
		//USART_InitCustomPinsCallback(USART2, GPIO_AF_USART2);
	}
}
#endif

#ifdef USE_USART3
void USART3_InitPins(USART_PinsPack_t pinspack) {
	/* Init pins */
#if defined(GPIOB)
	if (pinspack == Pins_PB10PB11) {
		//GPIO_InitAlternate(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_USART3);
		pinMode(PB10,AF_PP);
		pinMode(PB11,INPUT);
	}
#endif
#if defined(GPIOC)
	if (pinspack == Pins_PC10PC11) {
		//GPIO_InitAlternate(GPIOC, GPIO_PIN_10 | GPIO_PIN_11, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_USART3);
		pinMode(PC10,AF_PP);
		pinMode(PC11,INPUT);
	}
#endif
#if defined(GPIOD)
	if (pinspack == Pins_PD8PD9) {
		//GPIO_InitAlternate(GPIOD, GPIO_PIN_8 | GPIO_PIN_9, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_USART3);
		pinMode(PD8,AF_PP);
		pinMode(PD9,INPUT);
	}
#endif
	if (pinspack == USART_PinsPack_Custom) {
		/* Init custom pins, callback used */
		//USART_InitCustomPinsCallback(USART3, GPIO_AF_USART3);
	}
}
#endif

#ifdef USE_UART4
void UART4_InitPins(USART_PinsPack_t pinspack) {
	/* Init pins */
#if defined(GPIOA)
	if (pinspack == Pins_PA0PA1) {
		//GPIO_InitAlternate(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_UART4);
		pinMode(PA0,AF_PP);
		pinMode(PA1,INPUT);
	}
#endif
#if defined(GPIOC)
	if (pinspack == Pins_PC10PC11) {
		//GPIO_InitAlternate(GPIOC, GPIO_PIN_10 | GPIO_PIN_11, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_UART4);
		pinMode(PC10,AF_PP);
		pinMode(PC11,INPUT);
	}
#endif
	if (pinspack == USART_PinsPack_Custom) {
		/* Init custom pins, callback used */
		//USART_InitCustomPinsCallback(UART4, GPIO_AF_UART4);
	}
}
#endif

#ifdef USE_UART5
void UART5_InitPins(USART_PinsPack_t pinspack) {
	/* Init pins */
#if defined(GPIOC) && defined(GPIOD)
	if (pinspack == Pins_PC12PD2) {
		//GPIO_InitAlternate(GPIOC, GPIO_PIN_12, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_UART5);
		//GPIO_InitAlternate(GPIOD, GPIO_PIN_2, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_UART5);
		pinMode(PC12,AF_PP);
		pinMode(PD2,INPUT);
	}
#endif
	if (pinspack == USART_PinsPack_Custom) {
		/* Init custom pins, callback used */
		//USART_InitCustomPinsCallback(UART5, GPIO_AF_UART5);
	}
}
#endif

#ifdef USE_USART6
void USART6_InitPins(USART_PinsPack_t pinspack) {
	/* Init pins */
#if defined(GPIOC)
	if (pinspack == Pins_PC6PC7) {
		//GPIO_InitAlternate(GPIOC, GPIO_PIN_6 | GPIO_PIN_7, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_USART6);
		pinMode(PC6,AF_PP);
		pinMode(PC7,INPUT);
	}
#endif
#if defined(GPIOG)
//	if (pinspack == Pins_PG14PG9) {
//		//GPIO_InitAlternate(GPIOG, GPIO_PIN_14 | GPIO_PIN_9, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_USART6);
//		pinMode(PG14,AF_PP);
//		pinMode(PG9,INPUT);
//	}
#endif
	if (pinspack == USART_PinsPack_Custom) {
		/* Init custom pins, callback used */
		//USART_InitCustomPinsCallback(USART6, GPIO_AF_USART6);
	}
}
#endif

#ifdef USE_UART7
void UART7_InitPins(USART_PinsPack_t pinspack) {
	/* Init pins */
#if defined(GPIOE)
	if (pinspack == Pins_PE8PE7) {
		//GPIO_InitAlternate(GPIOE, GPIO_PIN_8 | GPIO_PIN_7, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_UART7);
		pinMode(PE8,AF_PP);
		pinMode(PE7,INPUT);
	}
#endif
#if defined(GPIOF)
	if (pinspack == Pins_PF7PF6) {
		//GPIO_InitAlternate(GPIOF, GPIO_PIN_7 | GPIO_PIN_6, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_UART7);
		pinMode(PF7,AF_PP);
		pinMode(PF6,INPUT);
	}
#endif
	if (pinspack == USART_PinsPack_Custom) {
		/* Init custom pins, callback used */
		//USART_InitCustomPinsCallback(UART7, GPIO_AF_UART7);
	}
}
#endif

#ifdef USE_UART8
void UART8_InitPins(USART_PinsPack_t pinspack) {
	/* Init pins */
#if defined(GPIOE)
	if (pinspack == Pins_PE1PE0) {
		//GPIO_InitAlternate(GPIOE, GPIO_PIN_1 | GPIO_PIN_0, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_High, GPIO_AF_UART8);
		pinMode(PE1,AF_PP);
		pinMode(PE0,INPUT);
	}
#endif
	if (pinspack == USART_PinsPack_Custom) {
		/* Init custom pins, callback used */
		//USART_InitCustomPinsCallback(UART8, GPIO_AF_UART8);
	}
}
#endif

//#ifdef USART_DMA 
//extern UART_HandleTypeDef uartdma; 
//#endif

/* Interrupt handlers */
#ifdef USE_USART1
void USART1_IRQHandler(void) {
	
	#ifdef USART_DMA 
	{
		HAL_UART_IRQHandler(&uartdma);  

		if (__HAL_UART_GET_FLAG (&uartdma, UART_FLAG_IDLE))
		{
			__HAL_UART_CLEAR_IDLEFLAG (&uartdma);
			HAL_UART_RxCpltCallback (&uartdma);  
		}	
	}	
	#else
		{
			/* Check if interrupt was because data is received */
			if (USART1->USART_STATUS_REG & USART_ISR_RXNE) {
			#ifdef USART1_USE_CUSTOM_IRQ
					/* Call user function */
					USART1_ReceiveHandler(USART1->DR);
			#else
					/* Put received data into internal buffer */
					USART_INT_InsertToBuffer(&USART1x, USART1->DR);
			#endif
			}
			/* Clear all USART flags */
			USART_INT_ClearAllFlags(USART1, USART1_IRQn);
		}
	#endif	
	
}
#endif

#ifdef USE_USART2
void USART2_IRQHandler(void) {
	#ifdef USART_DMA 
		HAL_UART_IRQHandler(&uartdma);  

  if (__HAL_UART_GET_FLAG (&uartdma, UART_FLAG_IDLE))
  {
	  __HAL_UART_CLEAR_IDLEFLAG (&uartdma);
	  HAL_UART_RxCpltCallback (&uartdma);  
	}	
	
	#else
			/* Check if interrupt was because data is received */
			if (USART2->USART_STATUS_REG & USART_ISR_RXNE) {
			#ifdef USART2_USE_CUSTOM_IRQ
					/* Call user function */
					USART2_ReceiveHandler(USART12->DR);
			#else 
					/* Put received data into internal buffer */
					USART_INT_InsertToBuffer(&USART2x, USART2->DR);
			#endif
			}
			
			/* Clear all USART flags */
			USART_INT_ClearAllFlags(USART2, USART2_IRQn);
	#endif	
}
#endif

#ifdef USE_USART3
void USART3_IRQHandler(void) {
	#ifdef USART_DMA 
		HAL_UART_IRQHandler(&uartdma);  

  if (__HAL_UART_GET_FLAG (&uartdma, UART_FLAG_IDLE))
  {
	  __HAL_UART_CLEAR_IDLEFLAG (&uartdma);
	  HAL_UART_RxCpltCallback (&uartdma);  
	}	
	
	#else
			/* Check if interrupt was because data is received */
			if (USART3->USART_STATUS_REG & USART_ISR_RXNE) {
			#ifdef USART3_USE_CUSTOM_IRQ
					/* Call user function */
					USART3_ReceiveHandler(USART3->DR);
			#else 
					/* Put received data into internal buffer */
					USART_INT_InsertToBuffer(&USART3x, USART3->DR);
			#endif
			}
			
			/* Clear all USART flags */
			USART_INT_ClearAllFlags(USART3, USART3_IRQn);
	#endif
}
#endif

#ifdef USE_UART4
void UART4_IRQHandler(void) {
	#ifdef USART_DMA 
		HAL_UART_IRQHandler(&uartdma);  

  if (__HAL_UART_GET_FLAG (&uartdma, UART_FLAG_IDLE))
  {
	  __HAL_UART_CLEAR_IDLEFLAG (&uartdma);
	  HAL_UART_RxCpltCallback (&uartdma);  
	}	
	
	#else
	
	/* Check if interrupt was because data is received */
			if (UART4->USART_STATUS_REG & USART_ISR_RXNE) {
		#ifdef UART4_USE_CUSTOM_IRQ
				/* Call user function */
				UART4_ReceiveHandler(UART4->DR);
		#else
				/* Put received data into internal buffer */
				USART_INT_InsertToBuffer(&UART4x, UART4->DR);
		#endif
			}
			
			/* Clear all USART flags */
			USART_INT_ClearAllFlags(UART4, UART4_IRQn);
	 #endif
}
#endif

#ifdef USE_UART5
void UART5_IRQHandler(void) {
	#ifdef USART_DMA 
		HAL_UART_IRQHandler(&uartdma);  

  if (__HAL_UART_GET_FLAG (&uartdma, UART_FLAG_IDLE))
  {
	  __HAL_UART_CLEAR_IDLEFLAG (&uartdma);
	  HAL_UART_RxCpltCallback (&uartdma);  
	}	
	
	#else
	
	/* Check if interrupt was because data is received */
			if (UART5->USART_STATUS_REG & USART_ISR_RXNE) {
		#ifdef UART5_USE_CUSTOM_IRQ
				/* Call user function */
				UART5_ReceiveHandler(UART5->DR);
		#else
				/* Put received data into internal buffer */
				USART_INT_InsertToBuffer(&UART5x, UART5->DR);
		#endif
			}
			
			/* Clear all USART flags */
			USART_INT_ClearAllFlags(UART5, UART5_IRQn);
	 #endif
}
#endif

#ifdef USE_USART6
void USART6_IRQHandler(void) {
	#ifdef USART_DMA 
		HAL_UART_IRQHandler(&uartdma);  

  if (__HAL_UART_GET_FLAG (&uartdma, UART_FLAG_IDLE))
  {
	  __HAL_UART_CLEAR_IDLEFLAG (&uartdma);
	  HAL_UART_RxCpltCallback (&uartdma);  
	}	
	
	#else	
	
//	/* Check if interrupt was because data is received */
//			if (USART6->USART_STATUS_REG & USART_ISR_RXNE) {
//		#ifdef USART6_USE_CUSTOM_IRQ
//				/* Call user function */
//				USART6_ReceiveHandler(USART6->DR);
//		#else
//				/* Put received data into internal buffer */
//				USART_INT_InsertToBuffer(&USART6x, USART6->DR);
//		#endif
//			}
//			
//			/* Clear all USART flags */
//			USART_INT_ClearAllFlags(USART6, USART6_IRQn);
	 #endif
}
#endif

#ifdef USE_UART7
void UART7_IRQHandler(void) {
	#ifdef USART_DMA 
		HAL_UART_IRQHandler(&uartdma);  

  if (__HAL_UART_GET_FLAG (&uartdma, UART_FLAG_IDLE))
  {
	  __HAL_UART_CLEAR_IDLEFLAG (&uartdma);
	  HAL_UART_RxCpltCallback (&uartdma);  
	}	
	
	#else		
	
	/* Check if interrupt was because data is received */
			if (UART7->USART_STATUS_REG & USART_ISR_RXNE) {
		#ifdef UART7_USE_CUSTOM_IRQ
				/* Call user function */
				UART7_ReceiveHandler(USART7->DR);
		#else
				/* Put received data into internal buffer */
				USART_INT_InsertToBuffer(&UART7x, USART7->DR);
		#endif
			}
			
			/* Clear all USART flags */
			USART_INT_ClearAllFlags(UART7, UART7_IRQn);
	 #endif
}
#endif

#ifdef USE_UART8
void UART8_IRQHandler(void) {
	#ifdef USART_DMA 
		HAL_UART_IRQHandler(&uartdma);  

  if (__HAL_UART_GET_FLAG (&uartdma, UART_FLAG_IDLE))
  {
	  __HAL_UART_CLEAR_IDLEFLAG (&uartdma);
	  HAL_UART_RxCpltCallback (&uartdma);  
	}	
	
	#else

	/* Check if interrupt was because data is received */
			if (UART8->USART_STATUS_REG & USART_ISR_RXNE) {
		#ifdef UART8_USE_CUSTOM_IRQ
				/* Call user function */
				UART8_ReceiveHandler(USART8->DR);
		#else
				/* Put received data into internal buffer */
				USART_INT_InsertToBuffer(&UART8x, USART8->DR);
		#endif
			}
			
			/* Clear all USART flags */
			USART_INT_ClearAllFlags(UART8, UART8_IRQn);
	 #endif
}
#endif

//#if defined(STM32F0xx)
//#ifdef USE_USART8
//void USART3_8_IRQHandler(void) {
//	/* Check if interrupt was because data is received */
//	if (USART3->USART_STATUS_REG & USART_ISR_RXNE) {
//#ifdef USART3_USE_CUSTOM_IRQ
//		/* Call user function */
//		UART8_ReceiveHandler(USART3->DR);
//#else
//		/* Put received data into internal buffer */
//		USART_INT_InsertToBuffer(&USART3, USART3->DR);
//#endif
//	}

//	/* Check if interrupt was because data is received */
//	if (USART4->USART_STATUS_REG & USART_ISR_RXNE) {
//#ifdef USART4_USE_CUSTOM_IRQ
//		/* Call user function */
//		UART8_ReceiveHandler(USART4->DR);
//#else
//		/* Put received data into internal buffer */
//		USART_INT_InsertToBuffer(&USART4, USART1->DR);
//#endif
//	}

//	/* Check if interrupt was because data is received */
//	if (USART5->USART_STATUS_REG & USART_ISR_RXNE) {
//#ifdef USART5_USE_CUSTOM_IRQ
//		/* Call user function */
//		UART8_ReceiveHandler(USART5->DR);
//#else
//		/* Put received data into internal buffer */
//		USART_INT_InsertToBuffer(&USART5, USART5->DR);
//#endif
//	}

//	/* Check if interrupt was because data is received */
//	if (USART6->USART_STATUS_REG & USART_ISR_RXNE) {
//#ifdef USART6_USE_CUSTOM_IRQ
//		/* Call user function */
//		UART8_ReceiveHandler(USART6->DR);
//#else
//		/* Put received data into internal buffer */
//		USART_INT_InsertToBuffer(&USART6, USART6->DR);
//#endif
//	}

//	/* Check if interrupt was because data is received */
//	if (USART7->USART_STATUS_REG & USART_ISR_RXNE) {
//#ifdef USART7_USE_CUSTOM_IRQ
//		/* Call user function */
//		UART8_ReceiveHandler(USART7->DR);
//#else
//		/* Put received data into internal buffer */
//		USART_INT_InsertToBuffer(&USART7, USART7->DR);
//#endif
//	}

//	/* Check if interrupt was because data is received */
//	if (USART8->USART_STATUS_REG & USART_ISR_RXNE) {
//#ifdef USART8_USE_CUSTOM_IRQ
//		/* Call user function */
//		UART8_ReceiveHandler(USART8->DR);
//#else
//		/* Put received data into internal buffer */
//		USART_INT_InsertToBuffer(&USART8, USART8->DR);
//#endif
//	}
//	
//	/* Clear all USART flags */
//	USART_INT_ClearAllFlags(USART3, IRQ_USART3);
//	USART_INT_ClearAllFlags(USART4, IRQ_USART4);
//	USART_INT_ClearAllFlags(USART5, IRQ_USART5);
//	USART_INT_ClearAllFlags(USART6, IRQ_USART6);
//	USART_INT_ClearAllFlags(USART7, IRQ_USART7);
//	USART_INT_ClearAllFlags(USART8, IRQ_USART8);
//}
//#elif defined(USART6)
//void USART3_6_IRQHandler(void) {
//	/* Check if interrupt was because data is received */
//	if (USART3->USART_STATUS_REG & USART_ISR_RXNE) {
//#ifdef USART3_USE_CUSTOM_IRQ
//		/* Call user function */
//		UART8_ReceiveHandler(USART_READ_DATA(USART3));
//#else
//		/* Put received data into internal buffer */
//		USART_INT_InsertToBuffer(&USART3, USART_READ_DATA(USART3));
//#endif
//	}

//	/* Check if interrupt was because data is received */
//	if (USART4->USART_STATUS_REG & USART_ISR_RXNE) {
//#ifdef USART4_USE_CUSTOM_IRQ
//		/* Call user function */
//		UART8_ReceiveHandler(USART_READ_DATA(USART4));
//#else
//		/* Put received data into internal buffer */
//		USART_INT_InsertToBuffer(&USART4, USART_READ_DATA(USART4));
//#endif
//	}

//	/* Check if interrupt was because data is received */
//	if (USART5->USART_STATUS_REG & USART_ISR_RXNE) {
//#ifdef USART5_USE_CUSTOM_IRQ
//		/* Call user function */
//		UART8_ReceiveHandler(USART_READ_DATA(USART5));
//#else
//		/* Put received data into internal buffer */
//		USART_INT_InsertToBuffer(&USART5, USART_READ_DATA(USART5));
//#endif
//	}

//	/* Check if interrupt was because data is received */
//	if (USART6->USART_STATUS_REG & USART_ISR_RXNE) {
//#ifdef USART6_USE_CUSTOM_IRQ
//		/* Call user function */
//		UART8_ReceiveHandler(USART_READ_DATA(USART6));
//#else
//		/* Put received data into internal buffer */
//		USART_INT_InsertToBuffer(&USART6, USART_READ_DATA(USART6));
//#endif
//	}
//	
//	/* Clear all USART flags */
//	USART_INT_ClearAllFlags(USART3);
//	USART_INT_ClearAllFlags(USART4);
//	USART_INT_ClearAllFlags(USART5);
//	USART_INT_ClearAllFlags(USART6);
//}
//#elif defined(USART4)
//void USART3_6_IRQHandler(void) {
//	/* Check if interrupt was because data is received */
//	if (USART3->USART_STATUS_REG & USART_ISR_RXNE) {
//#ifdef USART3_USE_CUSTOM_IRQ
//		/* Call user function */
//		UART8_ReceiveHandler(USART_READ_DATA(USART3));
//#else
//		/* Put received data into internal buffer */
//		USART_INT_InsertToBuffer(&USART3, USART_READ_DATA(USART3));
//#endif
//	}

//	/* Check if interrupt was because data is received */
//	if (USART4->USART_STATUS_REG & USART_ISR_RXNE) {
//#ifdef USART4_USE_CUSTOM_IRQ
//		/* Call user function */
//		UART8_ReceiveHandler(USART_READ_DATA(USART4));
//#else
//		/* Put received data into internal buffer */
//		USART_INT_InsertToBuffer(&USART4, USART_READ_DATA(USART4));
//#endif
//	}
//	
//	/* Clear all USART flags */
//	USART_INT_ClearAllFlags(USART3);
//	USART_INT_ClearAllFlags(USART4);
//}
//#endif
//#endif

static void USART_INT_Init(
	USART_TypeDef* USARTx,
	USART_PinsPack_t pinspack,
	uint32_t baudrate,
	USART_HardwareFlowControl_t FlowControl,
	uint32_t Mode,
	uint32_t Parity,
	uint32_t StopBits,
	uint32_t WordLength
) {
	UART_HandleTypeDef UARTHandle;
	IRQn_Type irq;
	
	/*
	 * Initialize USARTx pins
	 * Set channel for USARTx NVIC
	 */
#ifdef USE_USART1
	if (USARTx == USART1) {
		/* Enable USART clock */
		__HAL_RCC_USART1_CLK_ENABLE();
		__HAL_RCC_USART1_FORCE_RESET();
		__HAL_RCC_USART1_RELEASE_RESET();
		
		/* Init pins */
		USART1_InitPins(pinspack);
		
		/* Set IRQ channel */
		irq = USART1_IRQn;
	}
#endif
#ifdef USE_USART2
	if (USARTx == USART2) {
		/* Enable USART clock */
		__HAL_RCC_USART2_CLK_ENABLE();
		__HAL_RCC_USART2_FORCE_RESET();
		__HAL_RCC_USART2_RELEASE_RESET();
		
		/* Init pins */
		USART2_InitPins(pinspack);
		
		/* Set IRQ channel */
		irq = USART2_IRQn;
	}
#endif
#ifdef USE_USART3
	if (USARTx == USART3) {
		/* Enable USART clock */
		__HAL_RCC_USART3_CLK_ENABLE();
		__HAL_RCC_USART3_FORCE_RESET();
		__HAL_RCC_USART3_RELEASE_RESET();
		
		/* Init pins */
		USART3_InitPins(pinspack);
		
		/* Set IRQ channel */
		irq = USART3_IRQn;
	}
#endif
#ifdef USE_UART4
	if (USARTx == UART4) {
		/* Enable UART clock */
		__HAL_RCC_UART4_CLK_ENABLE();
		__HAL_RCC_UART4_FORCE_RESET();
		__HAL_RCC_UART4_RELEASE_RESET();
		
		/* Init pins */
		UART4_InitPins(pinspack);
		
		/* Set IRQ channel */
		irq = UART4_IRQn;
	}
#endif
#ifdef USE_UART5
	if (USARTx == UART5) {
		/* Enable UART clock */
		__HAL_RCC_UART5_CLK_ENABLE();
		__HAL_RCC_UART5_FORCE_RESET();
		__HAL_RCC_UART5_RELEASE_RESET();

		/* Init pins */
		UART5_InitPins(pinspack);
		
		/* Set IRQ channel */
		irq = UART5_IRQn;
	}
#endif
#ifdef USE_USART6
//	if (USARTx == USART6) {
//		/* Enable UART clock */
//		__HAL_RCC_USART6_CLK_ENABLE();
//		__HAL_RCC_USART6_FORCE_RESET();
//		__HAL_RCC_USART6_RELEASE_RESET();
//		
//		/* Init pins */
//		USART6_InitPins(pinspack);
//		
//		/* Set IRQ channel */
//		irq = USART6_IRQn;
//	}
#endif
#ifdef USE_UART7
	if (USARTx == UART7) {
		/* Enable UART clock */
		__HAL_RCC_UART7_CLK_ENABLE();
		__HAL_RCC_UART7_FORCE_RESET();
		__HAL_RCC_UART7_RELEASE_RESET();
		
		/* Init pins */
		UART7_InitPins(pinspack);
		
		/* Set IRQ channel */
		irq = UART7_IRQn;
	}
#endif
#ifdef USE_UART8
	if (USARTx == UART8) {
		/* Enable UART clock */
		__HAL_RCC_UART8_CLK_ENABLE();
		__HAL_RCC_UART8_FORCE_RESET();
		__HAL_RCC_UART8_RELEASE_RESET();

		/* Init pins */
		UART8_InitPins(pinspack);
		
		/* Set IRQ channel */
		irq = UART8_IRQn;
	}
#endif
	
/* STM32F0xx related */
//#ifdef USE_USART4
//	if (USARTx == USART4) {
//		/* Enable UART clock */
//		__HAL_RCC_USART4_CLK_ENABLE();
//		__HAL_RCC_USART4_FORCE_RESET();
//		__HAL_RCC_USART4_RELEASE_RESET();
//		
//		/* Init pins */
//		USART4_InitPins(pinspack);
//		
//		/* Set IRQ channel */
//		irq = IRQ_USART4;
//	}
//#endif
//#ifdef USE_USART5
//	if (USARTx == USART5) {
//		/* Enable UART clock */
//		__HAL_RCC_USART5_CLK_ENABLE();
//		__HAL_RCC_USART5_FORCE_RESET();
//		__HAL_RCC_USART5_RELEASE_RESET();
//		
//		/* Init pins */
//		USART5_InitPins(pinspack);
//		
//		/* Set IRQ channel */
//		irq = IRQ_USART5;
//	}
//#endif
//#ifdef USE_USART7
//	if (USARTx == USART7) {
//		/* Enable UART clock */
//		__HAL_RCC_USART7_CLK_ENABLE();
//		__HAL_RCC_USART7_FORCE_RESET();
//		__HAL_RCC_USART7_RELEASE_RESET();
//		
//		/* Init pins */
//		USART7_InitPins(pinspack);
//		
//		/* Set IRQ channel */
//		irq = IRQ_USART7;
//	}
//#endif
//#ifdef USE_USART8
//	if (USARTx == USART8) {
//		/* Enable UART clock */
//		__HAL_RCC_USART8_CLK_ENABLE();
//		__HAL_RCC_USART8_FORCE_RESET();
//		__HAL_RCC_USART8_RELEASE_RESET();
//		
//		/* Init pins */
//		USART8_InitPins(pinspack);
//		
//		/* Set IRQ channel */
//		irq = IRQ_USART8;
//	}
//#endif
	
	/* Fill default settings */
	UARTHandle.Instance = USARTx;
	UARTHandle.Init.BaudRate = baudrate;
	UARTHandle.Init.HwFlowCtl = FlowControl;
	UARTHandle.Init.Mode = Mode;
	UARTHandle.Init.Parity = Parity;
	UARTHandle.Init.StopBits = StopBits;
	UARTHandle.Init.WordLength = WordLength;
	UARTHandle.Init.OverSampling = UART_OVERSAMPLING_16;
//#if defined(STM32F0xx)
//	UARTHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//#endif
//#if defined(STM32F7xx)
//    UARTHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//	UARTHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//#endif


	/* Disable IRQ */
	HAL_NVIC_DisableIRQ(irq);

	/* Set priority */
	HAL_NVIC_SetPriority(irq, USART_NVIC_PRIORITY, USART_INT_GetSubPriority(USARTx));
	
	/* Enable interrupt */
	HAL_NVIC_EnableIRQ(irq);
	
	/* Clear interrupt */
	HAL_NVIC_ClearPendingIRQ(irq);
	
	/* Init USART */
	HAL_UART_Init(&UARTHandle);
	
	/* Enable RX interrupt */
	USARTx->CR1 |= USART_CR1_RXNEIE;
}

static UART_HandleTypeDef UART_Handle;
static void USART_INT_ClearAllFlags(USART_TypeDef* USARTx, IRQn_Type irq) {
	UART_Handle.Instance = USARTx;
	
#ifdef __HAL_UART_CLEAR_PEFLAG
	__HAL_UART_CLEAR_PEFLAG(&UART_Handle);
#endif
#ifdef __HAL_UART_CLEAR_FEFLAG
	__HAL_UART_CLEAR_FEFLAG(&UART_Handle);
#endif
#ifdef __HAL_UART_CLEAR_NEFLAG
	__HAL_UART_CLEAR_NEFLAG(&UART_Handle);
#endif
#ifdef __HAL_UART_CLEAR_OREFLAG
	__HAL_UART_CLEAR_OREFLAG(&UART_Handle);
#endif
#ifdef __HAL_UART_CLEAR_IDLEFLAG
	__HAL_UART_CLEAR_IDLEFLAG(&UART_Handle);
#endif
	
	/* Clear IRQ bit */
	HAL_NVIC_ClearPendingIRQ(irq);
}

/////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////
extern DMA_HandleTypeDef hdma_usart1_rx;
//extern DMA_HandleTypeDef hdma_usart1_tx;
//extern DMA_HandleTypeDef hdma_usart2_rx;
//extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef uartdma;
//extern UART_HandleTypeDef huart2;
//void DMA1_Channel4_IRQHandler(void)
//{
  
//  HAL_DMA_IRQHandler(&hdma_usart1_tx);
 
//}
void DMA1_Channel1_IRQHandler(void)
{
 
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
 
}

//void USART1_IRQHandler(void)
//{
//  

//  HAL_UART_IRQHandler(&uartdma);
//  

//  if (__HAL_UART_GET_FLAG (&uartdma, UART_FLAG_IDLE))
//  {
//	  __HAL_UART_CLEAR_IDLEFLAG (&uartdma);
//	  HAL_UART_RxCpltCallback (&uartdma);  
//	}


//}

UART_HandleTypeDef uartdma;
//UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
//DMA_HandleTypeDef hdma_usart1_tx;
//DMA_HandleTypeDef hdma_usart2_rx;
//DMA_HandleTypeDef hdma_usart2_tx;

/* USART1 init function */

void DMA_USARTx_Init(USART_TypeDef* USARTx)
{
		  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
		goi ham khoi tao USART_Init(USART1, Pins_PA9PA10, 115200);
    */
//    pinMode(A9,AF_PP);
//		pinMode(A10,INPUT);
	#ifdef USART1
	if(USARTx==USART1){
	
	__HAL_RCC_USART1_CLK_ENABLE();
	/* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
	}
	#endif
	
	#ifdef USART2
	if(USARTx==USART2){
	
	__HAL_RCC_USART2_CLK_ENABLE();
	/* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
	}
	#endif
	
	#ifdef USART3
	if(USARTx==USART3){
	
	__HAL_RCC_USART3_CLK_ENABLE();
	/* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
	}
	#endif
	
	#ifdef UART4
	if(USARTx==UART4){
	
	__HAL_RCC_UART4_CLK_ENABLE();
	/* USART1 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
	}
	#endif
	
	#ifdef UART5
	if(USARTx==UART5){
	
	__HAL_RCC_UART5_CLK_ENABLE();
	/* USART1 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
	}
	#endif
	
	#ifdef USART6
	if(USARTx==USART6){
	
	__HAL_RCC_USART6_CLK_ENABLE();
	/* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
	}
	#endif
	
	#ifdef UART7
	if(USARTx==UART7){
	
	__HAL_RCC_UART7_CLK_ENABLE();
	/* USART1 interrupt Init */
    HAL_NVIC_SetPriority(UART7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART7_IRQn);
	}
	#endif
	
	#ifdef UART8
	if(USARTx==UART8){
	
	__HAL_RCC_UART8_CLK_ENABLE();
	/* USART1 interrupt Init */
    HAL_NVIC_SetPriority(UART8_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART8_IRQn);
	}
	#endif
	
  uartdma.Instance = USARTx;
  uartdma.Init.BaudRate = 115200;
  uartdma.Init.WordLength = UART_WORDLENGTH_8B;
  uartdma.Init.StopBits = UART_STOPBITS_1;
  uartdma.Init.Parity = UART_PARITY_NONE;
  uartdma.Init.Mode = UART_MODE_TX_RX;
  uartdma.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  uartdma.Init.OverSampling = UART_OVERSAMPLING_16;
  //uartdma.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  //uartdma.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
		HAL_UART_Init(&uartdma);// khai bao roi USART_Init(USART1, Pins_PA9PA10, 115200);
//	
	/* USART1 DMA Init */
    /* USART1_RX Init */
		__HAL_RCC_DMA1_CLK_ENABLE();
		  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
			HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
			
    hdma_usart1_rx.Instance = DMA1_Channel1;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_usart1_rx);

    __HAL_LINKDMA(&uartdma,hdmarx,hdma_usart1_rx);

  /* DMA1_Channel4_IRQn interrupt configuration */
//  	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
//  	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
//    /* USART1_TX Init */
//    hdma_usart1_tx.Instance = DMA1_Channel4;
//    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
//    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
//    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
//    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
//    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
//    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
//    {
//      _Error_Handler(__FILE__, __LINE__);
//    }

//    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    
__HAL_UART_ENABLE_IT(&uartdma, UART_IT_IDLE);//kiemtra co ngat
}

//void MX_DMA_Init(void) 
//{
  /* DMA controller clock enable */ 

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
//  /* DMA1_Channel6_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
//  /* DMA1_Channel7_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

//}


////////////////////////////////////////////////////////////////////////////////////////
