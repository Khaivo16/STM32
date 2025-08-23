#include "stm32f10x.h"
#include "LED.h"
#include "delay.h"
#include "stdlib.h"
#include "EXTI.h"
#include "uart.h"
#include "pwm.h"
#include "string.h"
#include <math.h>
#include "timer.h"
#include "INPUT_CAPTURE.h" 
#include "SPI.h"

uint32_t TxIdx = 0;
uint16_t a,b=0;
int main(void)
{	
	delay_init();
	LED_init();
	uartx_config(USART1,pins_PA9_PA10,9600);
	SPI_init(SPI1,SPI_Mode_Master,SPI_BaudRatePrescaler_8);	
	SPI_init(SPI2,SPI_Mode_Slave,SPI_BaudRatePrescaler_8);


  while(1)
		
	{		
			s_data_spi(SPI1,200);
			a=r_data_spi(SPI2);
			s_data_spi(SPI2,1);
			b=r_data_spi(SPI1);
  } 
}



