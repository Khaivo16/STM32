#include "spi.h"


void SPI_init(SPI_HandleTypeDef *hspi,SPI_TypeDef *spi,uint16_t SPI_Mode,uint16_t BaudRate){
	if ((spi==SPI1)&&(SPI_Mode==SPI_MODE_MASTER)) 
	{
			__HAL_RCC_SPI1_CLK_ENABLE(); // B?t clock cho SPI1
			/* Configure SPI1 pins: SCK, MISO and MOSI ---------------------------------*/
			/* Confugure SCK and MOSI pins as Alternate Function Push Pull */
			GPIO_Set(GPIOA,GPIO_PIN_5 | GPIO_PIN_7,GPIO_MODE_AF_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);
			/* Confugure MISO pin as Input Floating  */
			GPIO_Set(GPIOA,GPIO_PIN_6,GPIO_MODE_INPUT,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);
		
	}
	else if ((spi==SPI1)&&(SPI_Mode==SPI_MODE_SLAVE)) 
	{
			__HAL_RCC_SPI1_CLK_ENABLE(); // B?t clock cho SPI1
			GPIO_Set(GPIOA,GPIO_PIN_5 | GPIO_PIN_7,GPIO_MODE_INPUT,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);// SCK-PA5 MOSI-PA7
		
			GPIO_Set(GPIOA,GPIO_PIN_6,GPIO_MODE_AF_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);//MISO
			
	}
	else if ((spi==SPI2)&&(SPI_Mode==SPI_MODE_MASTER)) 
	{
			__HAL_RCC_SPI2_CLK_ENABLE(); // B?t clock cho SPI2
			GPIO_Set(GPIOB,GPIO_PIN_13 | GPIO_PIN_15,GPIO_MODE_AF_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);//SCK-PB13 MOSI-PB15
			GPIO_Set(GPIOB,GPIO_PIN_14,GPIO_MODE_INPUT,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);//MISO
	}
	else if ((spi==SPI2)&&(SPI_Mode==SPI_MODE_SLAVE)) 
	{
				/* SPI2 Periph clock enable */
			__HAL_RCC_SPI2_CLK_ENABLE(); // B?t clock cho SPI1
			/* Configure SPI2 pins: SCK, MISO and MOSI ---------------------------------*/
			/* Confugure SCK and MOSI pins as Alternate Function Push Pull */
			GPIO_Set(GPIOB,GPIO_PIN_13 | GPIO_PIN_15,GPIO_MODE_INPUT,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);
			/* Confugure MISO pin as Input Floating  */
			GPIO_Set(GPIOB,GPIO_PIN_14,GPIO_MODE_AF_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);
	}
	
	hspi->Instance = spi;
  hspi->Init.Mode = SPI_Mode;
  hspi->Init.Direction = SPI_DIRECTION_2LINES;
  hspi->Init.DataSize = SPI_DATASIZE_8BIT;
  hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi->Init.NSS = SPI_NSS_SOFT;
  hspi->Init.BaudRatePrescaler = BaudRate;
  hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi->Init.CRCPolynomial = 10;
	HAL_SPI_Init(hspi); // Kh?i t?o SPI1 ho?c SPI2
	
}
void s_data_spi(SPI_HandleTypeDef *hspi,uint8_t *data){
	HAL_SPI_Transmit(hspi, data, 1, HAL_MAX_DELAY);
}

void r_data_spi(SPI_HandleTypeDef *hspi,uint8_t *receivedData){
		HAL_SPI_Receive(hspi, receivedData, 1, HAL_MAX_DELAY);
}
void rs_data_spi(SPI_HandleTypeDef *hspi,uint8_t *data_write,uint8_t *data_read){
		HAL_SPI_TransmitReceive(hspi,data_write,data_read,1,HAL_MAX_DELAY);
}