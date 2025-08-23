#include "SPI.h"

void SPI_init(SPI_TypeDef *spi,uint16_t SPI_Mode,uint16_t BaudRate){
	SPI_InitTypeDef  SPI_InitStructure;
	
	if ((spi==SPI1)&&(SPI_Mode==SPI_Mode_Master)) 
	{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
			/* Configure SPI1 pins: SCK, MISO and MOSI ---------------------------------*/
			/* Confugure SCK and MOSI pins as Alternate Function Push Pull */
			GPIO_SETUP(GPIOA,GPIO_Pin_5 | GPIO_Pin_7,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
			/* Confugure MISO pin as Input Floating  */
			GPIO_SETUP(GPIOA,GPIO_Pin_6,GPIO_Mode_IN_FLOATING,GPIO_Speed_50MHz);
		
	}
	else if ((spi==SPI1)&&(SPI_Mode==SPI_Mode_Slave)) 
	{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
			GPIO_SETUP(GPIOA,GPIO_Pin_5 | GPIO_Pin_7,GPIO_Mode_IN_FLOATING,GPIO_Speed_50MHz);// SCK-PA5 MOSI-PA7
		
			GPIO_SETUP(GPIOA,GPIO_Pin_6,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);//MISO
			
	}
	else if ((spi==SPI2)&&(SPI_Mode==SPI_Mode_Master)) 
	{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
			GPIO_SETUP(GPIOB,GPIO_Pin_13 | GPIO_Pin_15,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);//SCK-PB13 MOSI-PB15
			GPIO_SETUP(GPIOB,GPIO_Pin_14,GPIO_Mode_IN_FLOATING,GPIO_Speed_50MHz);//MISO
	}
	else if ((spi==SPI2)&&(SPI_Mode==SPI_Mode_Slave)) 
	{
				/* SPI2 Periph clock enable */
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
			/* Configure SPI2 pins: SCK, MISO and MOSI ---------------------------------*/
			/* Confugure SCK and MOSI pins as Alternate Function Push Pull */
			GPIO_SETUP(GPIOB,GPIO_Pin_13 | GPIO_Pin_15,GPIO_Mode_IN_FLOATING,GPIO_Speed_50MHz);
			/* Confugure MISO pin as Input Floating  */
			GPIO_SETUP(GPIOB,GPIO_Pin_14,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
	}
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = BaudRate;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
	
	SPI_Init(spi, &SPI_InitStructure);
	SPI_CalculateCRC(spi, ENABLE);
	SPI_Cmd(spi, ENABLE);
	
}
void s_data_spi(SPI_TypeDef *spi,uint8_t data){
	 /* Wait for SPI1 Tx buffer empty */
    while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_TXE) == RESET);
			SPI_I2S_SendData(spi, data);
}

uint8_t r_data_spi(SPI_TypeDef *spi){
		while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_RXNE) == RESET);
		return SPI_I2S_ReceiveData(spi);
}