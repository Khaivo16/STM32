#ifndef __SPI_H_
#define __SPI_H_

#include "gpio.h"

#define SPI_SPEED_2   	 										0
#define SPI_SPEED_4   	 										1
#define SPI_SPEED_8   	 										2
#define SPI_SPEED_16  	 										3
#define SPI_SPEED_32 												4
#define SPI_SPEED_64 		 										5
#define SPI_SPEED_128 	 										6
#define SPI_SPEED_256 	 										7

#define SPI_DATASIZE_8BIT       			      0
#define SPI_DATASIZE_16BIT     			        1

#define SPI_CR1_MSTR_Pos                    (2U)                               
#define SPI_CR1_MSTR_Msk                    (0x1UL << SPI_CR1_MSTR_Pos)         /*!< 0x00000004 */
#define SPI_CR1_MSTR                        SPI_CR1_MSTR_Msk  

#define SPI_CR1_SSI_Pos                     (8U)                               
#define SPI_CR1_SSI_Msk                     (0x1UL << SPI_CR1_SSI_Pos)          /*!< 0x00000100 */
#define SPI_CR1_SSI													SPI_CR1_SSI_Msk

#define SPI_MODE_SLAVE        	 		        (0x00000000U)
#define SPI_MODE_MASTER         		        (SPI_CR1_MSTR | SPI_CR1_SSI)


void SPIx_Init(SPI_TypeDef *SPIx, uint32_t Mode, uint8_t Baud, uint8_t Datasize);
void SPI_Send(SPI_TypeDef *SPIx, uint8_t Data);

uint8_t SPI_Rec(SPI_TypeDef *SPIx);

uint8_t SPI_Send_Rec(SPI_TypeDef *SPIx, uint8_t Data);

#endif








