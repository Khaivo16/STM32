#include "stm32f10x.h"
#include "LED.h"
#include "delay.h"
#include "stdlib.h"
#include "uart.h"
#include "SPI.h"

#include "nrf24l01.h"
//STM32VN.TK
/* My address */
uint8_t MyAddress[] = {
	0xE7,
	0xE7,
	0xE7,
	0xE7,
	0xE7
};
/* Receiver address */
uint8_t TxAddress[] = {
	0x7E,
	0x7E,
	0x7E,
	0x7E,
	0x7E
};

uint8_t dataOut[32], dataIn[32];
uint8_t key,mode;
int main(void)
{	
	delay_init();
	LED_init();
	uartx_config(USART1,pins_PA9_PA10,9600);
	NRF24L01_Transmit_Status_t transmissionStatus;
	GPIO_Set(GPIOC,GPIO_Pin_13,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
	delay_init();
	
	
	/* Initialize NRF24L01+ on channel 15 and 32bytes of payload */
	/* By default 2Mbps data rate and 0dBm output power */
	/* NRF24L01 goes to RX mode by default */
	NRF24L01_Init(15, 32);
	
	/* Set 2MBps data rate and -18dBm output power */
	NRF24L01_SetRF(NRF24L01_DataRate_1M, NRF24L01_OutputPower_M12dBm);
	
	/* Set my address, 5 bytes */
	NRF24L01_SetMyAddress(MyAddress);
	/* Set TX address, 5 bytes */
	NRF24L01_SetTxAddress(TxAddress);
	
	mode=1;

  while(1)
		
	{		
		
		if(mode==1)	//TX
		{
			while (1) {
		
			/* Fill data with something 0bcdefghijklmnoszxABCDEFCBDA*/
			sprintf((char *)dataOut, "abcdefghijklmnoszxABCDEFCBDA\n");
	
			
			/* Transmit data, goes automatically to TX mode */
			NRF24L01_Transmit(dataOut);
			
			/* Turn on led to indicate sending */
			LED0=1;
			/* Wait for data to be sent */
			do {
				transmissionStatus = NRF24L01_GetTransmissionStatus();
				send_num(USART1,transmissionStatus);
			} while (transmissionStatus == NRF24L01_Transmit_Status_Sending);
			/* Turn off led */
			LED0=0;
			send_string(USART1,dataOut);

			/* Go back to RX mode */
			NRF24L01_PowerUpRx();
			delayms(2000);
			}
		}
		
//		else	//mode=0//RX
//		{
//			while (1) {
//		/* If data is ready on NRF24L01+ */
//		if (NRF24L01_DataReady()) {
//			/* Get data from NRF24L01+ */
//			NRF24L01_GetData(dataIn);

//			if (dataIn[0]=='a'&&dataIn[27]=='A')				LED0=!LED0;		//kiem tra data
//			
//			/* Go back to RX Mode */
//			NRF24L01_PowerUpRx();		
//			delayms(1000);	
//			}
//		
//		}
					
		
  } 
}


