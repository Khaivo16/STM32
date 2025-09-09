#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f103xb.h"
#include "delay.h"

//uint32_t SystemCoreClock = 8000000;
//const uint8_t AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
//const uint8_t APBPrescTable[8U] =  {0, 0, 0, 0, 1, 2, 3, 4};
/***************************************************************************************/
#include <stdint.h>

/* Base addresses */
//#define PERIPH_BASE           0x40000000UL
//#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)  /*!< 0x40010000 */
//#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000UL)  /*!< 0x40020000 */

///* GPIO bases (STM32F1-style) */
//#define GPIOA_BASE            (APB2PERIPH_BASE + 0x00000800UL)
//#define GPIOB_BASE            (APB2PERIPH_BASE + 0x00000C00UL)
//#define GPIOC_BASE            (APB2PERIPH_BASE + 0x00001000UL)
//#define GPIOD_BASE            (APB2PERIPH_BASE + 0x00001400UL)
//#define GPIOE_BASE            (APB2PERIPH_BASE + 0x00001800UL)
//#define GPIOF_BASE            (APB2PERIPH_BASE + 0x00001A00UL)
//#define GPIOG_BASE            (APB2PERIPH_BASE + 0x00001E00UL)

///* Bit-band region for peripherals */
//#define PERIPH_BB_BASE        0x42000000UL

/* Helpers: bit-band calculation */


/***************************************************************************************/
#define RCC_CFGR_PPRE1_Pos                   (8U)                              
#define RCC_CFGR_PPRE1_Msk                   (0x7UL << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000700 */
#define RCC_CFGR_PPRE1                       RCC_CFGR_PPRE1_Msk

#define RCC_CFGR_PPRE2_Pos                   (11U)                             
#define RCC_CFGR_PPRE2_Msk                   (0x7UL << RCC_CFGR_PPRE2_Pos)      /*!< 0x00003800 */
#define RCC_CFGR_PPRE2                       RCC_CFGR_PPRE2_Msk 


#define	INPUT_ANALOG 										((uint8_t) 0x00)
#define	INPUT_FLOATING 									((uint8_t) 0x01)
#define	INPUT_PUPD											((uint8_t) 0x02)

#define	OUTPUT_PP												((uint8_t) 0x00)
#define	OUTPUT_OD 											((uint8_t) 0x01)
#define	OUTPUT_AF_PP 										((uint8_t) 0x02)
#define	OUTPUT_AF_OD 										((uint8_t) 0x03)

#define	MODE_INPUT 											((uint8_t) 0x00)
#define	MODE_OUTPUT_10MHZ 							((uint8_t) 0x01)
#define	MODE_OUTPUT_2MHZ 								((uint8_t) 0x02)
#define	MODE_OUTPUT_50MHZ 							((uint8_t) 0x03)

#define	NOPULL 													((uint8_t) 0x00)
#define	PU 															((uint8_t) 0x01)
#define	PD 															((uint8_t) 0x02)

#define	RESET 													((uint8_t) 0x00)
#define	SET 														((uint8_t) 0x01)

void NVICx_Init(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);

void GPIOx_Init(GPIO_TypeDef * GPIOx, uint8_t Pin, uint8_t Mode, uint8_t Pull, uint8_t Speed);
void GPIOx_WritePin(GPIO_TypeDef *GPIOx, uint8_t Pin, uint8_t PinState);
uint8_t GPIOx_ReadPin(GPIO_TypeDef *GPIOx, uint8_t Pin);
void GPIOx_TogglePin(GPIO_TypeDef *GPIOx, uint8_t Pin, uint16_t TimeMs);

#endif



