/**

   ----------------------------------------------------------------------

   ----------------------------------------------------------------------

 */
#ifndef USART_H
#define USART_H 121

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup STM32Fxxx_HAL_Libraries
 * @{
 */

/**
 * @defgroup USART
 * @brief    
 * @{
	
///////////////////////////////////////////////////////////////////	
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
///////////////////////////////////////////////////////////////////////////////			
 
             |PINSPACK 1     |PINSPACK 2     |PINSPACK 3	
U(S)ARTX     |TX     RX      |TX     RX      |TX     RX

//STM32F4xx and STM32F7xx
USART1       |PA9    PA10    |PB6    PB7     |PA9    PB7
USART2       |PA2    PA3     |PD5    PD6     |-      -
USART3       |PB10   PB11    |PC10   PC11    |PD8    PD9
UART4        |PA0    PA1     |PC10   PC11    |-      -
UART5        |PC12   PD2     |-      -       |-      -
USART6       |PC6    PC7     |PG14   PG9     |-      -
UART7        |PE8    PE7     |PF7    PF6     |-      -
UART8        |PE1    PE0     |-      -       |-      -

//Additions for STM32F0xx
USART4       |PA0    PA1     |PC10   PC11    |PE8    PE9
USART5       |PB3    PB4     |PC12   PD2     |PE10   PE11
USART7       |PC0    PC1     |PC6    PC7     |PF2    PF3
USART8       |PC2    PC3     |PC8    PC9     |PD13   PD14
\endverbatim
 * Open \ref defines.h file, copy define you want to change and fill settings
\code
//Change X with possible U(S)ARTs: USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8, for STM32F0xx additions: USART4, USART5, USART7, USART8
//Set flow control
#define X_HARDWARE_FLOW_CONTROL    USART_HardwareFlowControl_None
//Set mode
#define X_MODE                     USART_MODE_TX_RX
//Set parity
#define X_PARITY                   USART_PARITY_NONE
//Set stopbits
#define X_STOP_BITS                USART_STOPBITS_1
//Set USART datasize
#define X_WORD_LENGTH              UART_WORDLENGTH_8B
\endcode

\endverbatim
 */
#include "stm32f1xx_hal.h"
//#include "attributes.h"
#include "sys.h"
//#include "stm32_gpio.h"

/**
 * @defgroup USART_Typedefs
 * @brief    USART Typedefs
 * @{
 */
 
#if defined (STM32F103xG)
#define USE_USART1
#define USE_USART2
#define USE_USART3
#define USE_UART4
#define USE_UART5
#define USE_USART6
#endif
#if defined (STM32F103xB)
#define USE_USART1
#define USE_USART2
#define USE_USART3
#endif


////////////////////////////////////
//#define USART_DMA 
extern UART_HandleTypeDef uartdma;
//extern UART_HandleTypeDef huart2;
////////////////////////////////////

void DMA_USARTx_Init(USART_TypeDef* USARTx);

///////////////////////////////////////////////////


/**
 * When you initialize USARTx, you have to select which pins pack you will use
 *
 * 
 */
typedef enum {
	Pins_PA9PA10,	//USART1
	Pins_PB6PB7,	////USART1
	Pins_PA9PB7,	//1
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
	Pins_PE1PE0,		//UART8
	USART_PinsPack_Custom
} USART_PinsPack_t;
/**
 * @brief  USART Hardware flow control selection
 * @note   Corresponsing pins must be initialized in case you don't use "None" options
 */
typedef enum {
	USART_HardwareFlowControl_None = UART_HWCONTROL_NONE,      /*!< No flow control */
	USART_HardwareFlowControl_RTS = UART_HWCONTROL_RTS,        /*!< RTS flow control */
	USART_HardwareFlowControl_CTS = UART_HWCONTROL_CTS,        /*!< CTS flow control */
	USART_HardwareFlowControl_RTS_CTS = UART_HWCONTROL_RTS_CTS /*!< RTS and CTS flow control */
} USART_HardwareFlowControl_t;

/**
 * @}
 */

/**
 * @defgroup USART_Macros
 * @brief    USART default values for defines
 * @{
 *
 * All values can be overwritten in your project's defines.h file.
 * 
 * Do this only in case you know what are you doing.
 */

/* Default buffer size for each USART */
#ifndef USART_BUFFER_SIZE
#define USART_BUFFER_SIZE 				32
#endif

/* Set default buffer size for specific USART if not set by user */
#ifndef USART1_BUFFER_SIZE
#define USART1_BUFFER_SIZE				USART_BUFFER_SIZE
#endif
#ifndef USART2_BUFFER_SIZE
#define USART2_BUFFER_SIZE				USART_BUFFER_SIZE
#endif
#ifndef USART3_BUFFER_SIZE
#define USART3_BUFFER_SIZE				USART_BUFFER_SIZE
#endif
#ifndef UART4_BUFFER_SIZE
#define UART4_BUFFER_SIZE				USART_BUFFER_SIZE
#endif
#ifndef UART5_BUFFER_SIZE
#define UART5_BUFFER_SIZE				USART_BUFFER_SIZE
#endif
#ifndef USART6_BUFFER_SIZE
#define USART6_BUFFER_SIZE				USART_BUFFER_SIZE
#endif
#ifndef UART7_BUFFER_SIZE
#define UART7_BUFFER_SIZE				USART_BUFFER_SIZE
#endif
#ifndef UART8_BUFFER_SIZE
#define UART8_BUFFER_SIZE				USART_BUFFER_SIZE
#endif

/* STM32F0xx related */
#ifndef USART4_BUFFER_SIZE
#define USART4_BUFFER_SIZE				USART_BUFFER_SIZE
#endif
#ifndef USART5_BUFFER_SIZE
#define USART5_BUFFER_SIZE				USART_BUFFER_SIZE
#endif
#ifndef USART7_BUFFER_SIZE
#define USART7_BUFFER_SIZE				USART_BUFFER_SIZE
#endif
#ifndef USART8_BUFFER_SIZE
#define USART8_BUFFER_SIZE				USART_BUFFER_SIZE
#endif

/* NVIC Global Priority */
#ifndef USART_NVIC_PRIORITY
#define USART_NVIC_PRIORITY					0x06
#endif

/* U(S)ART settings, can be changed in your defines.h project file */
/* USART1 default settings */
#ifndef USART1_HARDWARE_FLOW_CONTROL
#define USART1_HARDWARE_FLOW_CONTROL		USART_HardwareFlowControl_None
#endif
#ifndef USART1_MODE
#define USART1_MODE						USART_MODE_TX_RX
#endif
#ifndef USART1_PARITY
#define USART1_PARITY					USART_PARITY_NONE
#endif
#ifndef USART1_STOP_BITS
#define USART1_STOP_BITS					USART_STOPBITS_1
#endif
#ifndef USART1_WORD_LENGTH
#define USART1_WORD_LENGTH				UART_WORDLENGTH_8B
#endif

/* USART2 default settings */
#ifndef USART2_HARDWARE_FLOW_CONTROL
#define USART2_HARDWARE_FLOW_CONTROL		USART_HardwareFlowControl_None
#endif
#ifndef USART2_MODE
#define USART2_MODE						USART_MODE_TX_RX
#endif
#ifndef USART2_PARITY
#define USART2_PARITY					USART_PARITY_NONE
#endif
#ifndef USART2_STOP_BITS
#define USART2_STOP_BITS					USART_STOPBITS_1
#endif
#ifndef USART2_WORD_LENGTH
#define USART2_WORD_LENGTH				UART_WORDLENGTH_8B
#endif

/* USART3 default settings */
#ifndef USART3_HARDWARE_FLOW_CONTROL
#define USART3_HARDWARE_FLOW_CONTROL		USART_HardwareFlowControl_None
#endif
#ifndef USART3_MODE
#define USART3_MODE						USART_MODE_TX_RX
#endif
#ifndef USART3_PARITY
#define USART3_PARITY					USART_PARITY_NONE
#endif
#ifndef USART3_STOP_BITS
#define USART3_STOP_BITS					USART_STOPBITS_1
#endif
#ifndef USART3_WORD_LENGTH
#define USART3_WORD_LENGTH				UART_WORDLENGTH_8B
#endif

/* UART4 default settings */
#ifndef UART4_HARDWARE_FLOW_CONTROL
#define UART4_HARDWARE_FLOW_CONTROL		USART_HardwareFlowControl_None
#endif
#ifndef UART4_MODE
#define UART4_MODE						USART_MODE_TX_RX
#endif
#ifndef UART4_PARITY
#define UART4_PARITY						USART_PARITY_NONE
#endif
#ifndef UART4_STOP_BITS
#define UART4_STOP_BITS					USART_STOPBITS_1
#endif
#ifndef UART4_WORD_LENGTH
#define UART4_WORD_LENGTH				UART_WORDLENGTH_8B
#endif

/* UART5 default settings */
#ifndef UART5_HARDWARE_FLOW_CONTROL
#define UART5_HARDWARE_FLOW_CONTROL		USART_HardwareFlowControl_None
#endif
#ifndef UART5_MODE
#define UART5_MODE						USART_MODE_TX_RX
#endif
#ifndef UART5_PARITY
#define UART5_PARITY						USART_PARITY_NONE
#endif
#ifndef UART5_STOP_BITS
#define UART5_STOP_BITS					USART_STOPBITS_1
#endif
#ifndef UART5_WORD_LENGTH
#define UART5_WORD_LENGTH				UART_WORDLENGTH_8B
#endif

/* USART6 default settings */
#ifndef USART6_HARDWARE_FLOW_CONTROL
#define USART6_HARDWARE_FLOW_CONTROL		USART_HardwareFlowControl_None
#endif
#ifndef USART6_MODE
#define USART6_MODE						USART_MODE_TX_RX
#endif
#ifndef USART6_PARITY
#define USART6_PARITY					USART_PARITY_NONE
#endif
#ifndef USART6_STOP_BITS
#define USART6_STOP_BITS					USART_STOPBITS_1
#endif
#ifndef USART6_WORD_LENGTH
#define USART6_WORD_LENGTH				UART_WORDLENGTH_8B
#endif

/* UART7 default settings */
#ifndef UART7_HARDWARE_FLOW_CONTROL
#define UART7_HARDWARE_FLOW_CONTROL		USART_HardwareFlowControl_None
#endif
#ifndef UART7_MODE
#define UART7_MODE						USART_MODE_TX_RX
#endif
#ifndef UART7_PARITY
#define UART7_PARITY						USART_PARITY_NONE
#endif
#ifndef UART7_STOP_BITS
#define UART7_STOP_BITS					USART_STOPBITS_1
#endif
#ifndef UART7_WORD_LENGTH
#define UART7_WORD_LENGTH				UART_WORDLENGTH_8B
#endif

/* UART8 default settings */
#ifndef UART8_HARDWARE_FLOW_CONTROL
#define UART8_HARDWARE_FLOW_CONTROL		USART_HardwareFlowControl_None
#endif
#ifndef UART8_MODE
#define UART8_MODE						USART_MODE_TX_RX
#endif
#ifndef UART8_PARITY
#define UART8_PARITY						USART_PARITY_NONE
#endif
#ifndef UART8_STOP_BITS
#define UART8_STOP_BITS					USART_STOPBITS_1
#endif
#ifndef UART8_WORD_LENGTH
#define UART8_WORD_LENGTH				UART_WORDLENGTH_8B
#endif

/* STM32F0xx related */
/* USART4 default settings */
#ifndef USART4_HARDWARE_FLOW_CONTROL
#define USART4_HARDWARE_FLOW_CONTROL   USART_HardwareFlowControl_None
#endif
#ifndef USART4_MODE
#define USART4_MODE            USART_MODE_TX_RX
#endif
#ifndef USART4_PARITY
#define USART4_PARITY          USART_PARITY_NONE
#endif
#ifndef USART4_STOP_BITS
#define USART4_STOP_BITS         USART_STOPBITS_1
#endif
#ifndef USART4_WORD_LENGTH
#define USART4_WORD_LENGTH       UART_WORDLENGTH_8B
#endif

/* USART5 default settings */
#ifndef USART5_HARDWARE_FLOW_CONTROL
#define USART5_HARDWARE_FLOW_CONTROL   USART_HardwareFlowControl_None
#endif
#ifndef USART5_MODE
#define USART5_MODE            USART_MODE_TX_RX
#endif
#ifndef USART5_PARITY
#define USART5_PARITY          USART_PARITY_NONE
#endif
#ifndef USART5_STOP_BITS
#define USART5_STOP_BITS         USART_STOPBITS_1
#endif
#ifndef USART5_WORD_LENGTH
#define USART5_WORD_LENGTH       UART_WORDLENGTH_8B
#endif

/* USART7 default settings */
#ifndef USART7_HARDWARE_FLOW_CONTROL
#define USART7_HARDWARE_FLOW_CONTROL   USART_HardwareFlowControl_None
#endif
#ifndef USART7_MODE
#define USART7_MODE            USART_MODE_TX_RX
#endif
#ifndef USART7_PARITY
#define USART7_PARITY          USART_PARITY_NONE
#endif
#ifndef USART7_STOP_BITS
#define USART7_STOP_BITS         USART_STOPBITS_1
#endif
#ifndef USART7_WORD_LENGTH
#define USART7_WORD_LENGTH       UART_WORDLENGTH_8B
#endif

/* USART8 default settings */
#ifndef USART8_HARDWARE_FLOW_CONTROL
#define USART8_HARDWARE_FLOW_CONTROL   USART_HardwareFlowControl_None
#endif
#ifndef USART8_MODE
#define USART8_MODE            USART_MODE_TX_RX
#endif
#ifndef USART8_PARITY
#define USART8_PARITY          USART_PARITY_NONE
#endif
#ifndef USART8_STOP_BITS
#define USART8_STOP_BITS         USART_STOPBITS_1
#endif
#ifndef USART8_WORD_LENGTH
#define USART8_WORD_LENGTH       UART_WORDLENGTH_8B
#endif


/* Define ISR if not already */
#if !defined(USART_ISR_RXNE)
#define USART_ISR_RXNE                      USART_SR_RXNE
#endif

/**
 * @brief  Default string delimiter for USART
 */
#define USART_STRING_DELIMITER              '\n'

/* Configuration */
//#if defined(STM32F4XX)
#define USART_TX_REG(USARTx)                ((USARTx)->DR)
#define USART_WRITE_DATA(USARTx, data)      ((USARTx)->DR = (data))
#define USART_READ_DATA(USARTx)             ((USARTx)->DR)
#define GPIO_AF_UART5                       (GPIO_AF8_UART5)
#define USART_STATUS_REG                    SR

//#else
//#define USART_TX_REG(USARTx)                ((USARTx)->TDR)
//#define USART_WRITE_DATA(USARTx, data)      ((USARTx)->TDR = (data))
//#define USART_READ_DATA(USARTx)             ((USARTx)->RDR)
//#if defined(STM32F7xx)
//#define GPIO_AF_UART5                       (GPIO_AF8_UART5)
//#else
//#define GPIO_AF_UART5                       (GPIO_AF7_UART5)
//#endif /* STM32F7xx */
//#define USART_STATUS_REG                    ISR
//#endif /* STM32F4XX */

/* Wait for TX empty */
#define USART_TXEMPTY(USARTx)               ((USARTx)->USART_STATUS_REG & USART_FLAG_TXE)
#define USART_WAIT(USARTx)                  while (!USART_TXEMPTY(USARTx))

 /**
 * @}
 */

/**
 * @defgroup USART_Functions
 * @brief    USART Functions
 * @{
 */

/**
 * @brief  Initializes USARTx peripheral and corresponding pins
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @param  pinspack: This parameter can be a value of @ref USART_PinsPack_t enumeration
 * @param  baudrate: Baudrate number for USART communication
 * @retval None
 */
void USART_Init(USART_TypeDef* USARTx, USART_PinsPack_t pinspack, uint32_t baudrate);


/**
 * @brief  Puts character to USART port
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @param  c: character to be send over USART
 * @retval None
 */
static __INLINE void USART_Putc(USART_TypeDef* USARTx, volatile char c) {
	/* Check USART */
	if ((USARTx->CR1 & USART_CR1_UE)) {	
		/* Wait to be ready, buffer empty */
		USART_WAIT(USARTx);
		/* Send data */
		USART_WRITE_DATA(USARTx, (uint16_t)(c & 0x01FF));
		/* Wait to be ready, buffer empty */
		USART_WAIT(USARTx);
	}
}

/**
 * @brief  Puts string to USART port
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @param  *str: Pointer to string to send over USART
 * @retval None
 */
void USART_Puts(USART_TypeDef* USARTx, char* str);

/**
 * @brief  Sends data array to USART port
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @param  *DataArray: Pointer to data array to be sent over USART
 * @param  count: Number of elements in data array to be send over USART
 * @retval None
 */
void USART_Send(USART_TypeDef* USARTx, uint8_t* DataArray, uint16_t count);

/**
 * @brief  Gets character from internal USART buffer
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @retval Character from buffer, or 0 if nothing in buffer
 */
uint8_t USART_Getc(USART_TypeDef* USARTx);

/**
 * @brief  Get string from USART
 *
 *         This function can create a string from USART received data.
 *
 *         It generates string until "\n" is not recognized or buffer length is full.
 * 
 * @note   As of version 1.5, this function automatically adds 0x0A (Line feed) at the end of string.
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @param  *buffer: Pointer to buffer where data will be stored from buffer
 * @param  bufsize: maximal number of characters we can add to your buffer, including leading zero
 * @retval Number of characters in buffer
 */
uint16_t USART_Gets(USART_TypeDef* USARTx, char* buffer, uint16_t bufsize);

/**
 * @brief  Check if character c is available in internal buffer
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @param  c: character to check if it is in USARTx's buffer
 * @retval Character status:
 *            -  < 0: Character was not found
 *            - >= 0: Character has been found in buffer
 */
int16_t USART_FindCharacter(USART_TypeDef* USARTx, uint8_t c);

/**
 * @brief  Checks if internal USARTx buffer is empty
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @retval Buffer empty status:
 *            - 0: Buffer is not empty
 *            - > 0: Buffer is empty
 */
uint8_t USART_BufferEmpty(USART_TypeDef* USARTx);

/**
 * @brief  Checks if internal USARTx buffer is full
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @retval Buffer full status:
 *            - 0: Buffer is not full
 *            - > 0: Buffer is full
 */
uint8_t USART_BufferFull(USART_TypeDef* USARTx);

/**
 * @brief  Gets number of bytes in USART buffer
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @retval Number of elements in buffer
 */
uint16_t USART_BufferCount(USART_TypeDef* USARTx);

/**
 * @brief  Clears internal USART buffer
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @retval None
 */
void USART_ClearBuffer(USART_TypeDef* USARTx);

/**
 * @brief  Sets custom character for @ref USART_Gets() function to detect when string ends
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @param  Character: Character value to be used as string end
 * @note   Character will also be added at the end for your buffer when calling @ref USART_Gets() function
 * @retval None
 */
void USART_SetCustomStringEndCharacter(USART_TypeDef* USARTx, uint8_t Character);

///**
// * @brief  Search for string in USART buffer if exists
// * @param  *USARTx: Pointer to USARTx peripheral you will use
// * @param  *str: String to be searched
// * @retval Search status:
// *            -  < 0: String is not in buffer
// *            - >= 0: String is in buffer
// */
//int16_t USART_FindString(USART_TypeDef* USARTx, char* str);

/**
 * @brief  Callback for custom pins initialization for USARTx.
 *
 *         When you call @ef USART_Init() function, and if you pass @arg USART_PinsPack_Custom to function,
 *         then this function will be called where you can initialize custom pins for USART peripheral.
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  *USARTx: Pointer to USARTx peripheral you will use for initialization
 * @param  AlternateFunction: Alternate function number which should be used for GPIO pins
 * @retval None
 */
void USART_InitCustomPinsCallback(USART_TypeDef* USARTx, uint16_t AlternateFunction);

/**
 * @brief  Callback function for receive interrupt on USART1 in case you have enabled custom USART handler mode 
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  c: character received via USART
 * @retval None
 */
void USART1_ReceiveHandler(uint8_t c);

/**
 * @brief  Callback function for receive interrupt on USART2 in case you have enabled custom USART handler mode 
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  c: character received via USART
 * @retval None
 */
void USART2_ReceiveHandler(uint8_t c);

/**
 * @brief  Callback function for receive interrupt on USART3 in case you have enabled custom USART handler mode 
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  c: character received via USART
 * @retval None
 */
void USART3_ReceiveHandler(uint8_t c);

/**
 * @brief  Callback function for receive interrupt on UART4 in case you have enabled custom USART handler mode 
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  c: character received via USART
 * @retval None
 */
void UART4_ReceiveHandler(uint8_t c);

/**
 * @brief  Callback function for receive interrupt on UART5 in case you have enabled custom USART handler mode 
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  c: character received via USART
 * @retval None
 */
void UART5_ReceiveHandler(uint8_t c);

/**
 * @brief  Callback function for receive interrupt on USART6 in case you have enabled custom USART handler mode 
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  c: character received via USART
 * @retval None
 */
void USART6_ReceiveHandler(uint8_t c);

/**
 * @brief  Callback function for receive interrupt on UART7 in case you have enabled custom USART handler mode 
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  c: character received via USART
 * @retval None
 */
void UART7_ReceiveHandler(uint8_t c);

/**
 * @brief  Callback function for receive interrupt on UART8 in case you have enabled custom USART handler mode 
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  c: character received via USART
 * @retval None
 */
void UART8_ReceiveHandler(uint8_t c);

/**
 * @}
 */
 
/**
 * @}
 */
 
/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
