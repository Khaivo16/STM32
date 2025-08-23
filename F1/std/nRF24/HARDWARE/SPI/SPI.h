#ifndef __SPI_H
#define __SPI_H 			   
#include "sys.h"  
#include "delay.h"

void SPI_init(SPI_TypeDef *spi,uint16_t SPI_Mode,uint16_t BaudRate);
void s_data_spi(SPI_TypeDef *spi,uint8_t data);
uint8_t r_data_spi(SPI_TypeDef *spi);
void SPI_ReadMulti(SPI_TypeDef* SPIx, uint8_t* dataIn, uint8_t dummy, uint16_t count);
uint8_t SPI_Send(SPI_TypeDef* SPIx, uint8_t data);
void SPI_SendMulti(SPI_TypeDef* SPIx, uint8_t* dataOut, uint8_t* dataIn, uint16_t count);
void SPI_WriteMulti(SPI_TypeDef* SPIx, uint8_t* dataOut, uint16_t count);
#endif