#ifndef __spi_H
#define __spi_H 			   
#include "sys.h" 
#include "stm32f1xx.h"
#include "stm32f1xx_hal_spi.h"

void SPI_init(SPI_HandleTypeDef *hspi,SPI_TypeDef *spi,uint16_t SPI_Mode,uint16_t BaudRate);
void s_data_spi(SPI_HandleTypeDef *hspi,uint8_t *data);
void r_data_spi(SPI_HandleTypeDef *hspi,uint8_t *receivedData);
void rs_data_spi(SPI_HandleTypeDef *hspi,uint8_t *data_write,uint8_t *data_read);
#endif