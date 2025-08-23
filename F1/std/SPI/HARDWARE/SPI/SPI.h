#ifndef __SPI_H
#define __SPI_H 			   
#include "sys.h"  
#include "delay.h"

void SPI_init(SPI_TypeDef *spi,uint16_t SPI_Mode,uint16_t BaudRate);
void s_data_spi(SPI_TypeDef *spi,uint8_t data);
uint8_t r_data_spi(SPI_TypeDef *spi);
#endif