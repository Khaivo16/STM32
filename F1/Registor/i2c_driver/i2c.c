/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "i2c.h"

uint32_t I2C_Timeout=0;
#define I2C_TIMOUT 100000

void I2Cx_Init(I2C_TypeDef* I2Cx,Pins_I2C pin,uint32_t Speed){
	
	uint32_t fPCLKx=(SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);
	
	RCC->APB2ENR |= 1<<0;// AFIO
	
	if(I2Cx==I2C1)      RCC->APB1ENR |= 1<<21;// I2C1
	else if(I2Cx==I2C2) RCC->APB1ENR |= 1<<22;// I2C2
	
	if(pin==Pin_PB6PB7) {
		GPIOx_Init(GPIOB,6 , OUTPUT_AF_OD,NOPULL,MODE_OUTPUT_50MHZ);//TX
		GPIOx_Init(GPIOB,7 , OUTPUT_AF_OD,NOPULL,MODE_OUTPUT_50MHZ);//RX
	}
	else if(pin==Pin_PB8PB9) {
		AFIO->MAPR |= 1<<1;//REMAP I2C1
		GPIOx_Init(GPIOB,6 , OUTPUT_AF_OD,NOPULL,MODE_OUTPUT_50MHZ);//TX
		GPIOx_Init(GPIOB,7 , OUTPUT_AF_OD,NOPULL,MODE_OUTPUT_50MHZ);//RX
	}
	
	else if(pin==Pin_PB10PB11) {		
		GPIOx_Init(GPIOB,10 , OUTPUT_AF_OD,NOPULL,MODE_OUTPUT_50MHZ);//TX
		GPIOx_Init(GPIOB,11 , OUTPUT_AF_OD,NOPULL,MODE_OUTPUT_50MHZ);//RX
	}
	
	I2Cx->CR2 |= fPCLKx/1000000;//36M
	
	I2Cx->CCR= fPCLKx/(Speed*2);//36-> 180
	
	I2Cx->TRISE = (fPCLKx/1000000)+1;//
	
	I2Cx->CR1	 |= 1<<0; //EN I2C

}

void I2Cx_WriteMulti(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint8_t reg, uint8_t * data, uint8_t size){
	
		I2Cx->CR1	 |= (1<<8)|(1<<10);  //Start generation  + ACK en
		I2C_Timeout= I2C_TIMOUT;
		while((I2Cx->SR1 &(1<<0))==0){if(--I2C_Timeout==0) break;}//Start
		
		I2C_Timeout= I2C_TIMOUT;
		I2Cx->DR= (DevAddress<<1) | 0;
		while((I2Cx->SR1 &(1<<1))==0  | (I2Cx->SR2 &(1<<1))==0){if(--I2C_Timeout==0) break;}//BUSY
	
		I2C_Timeout= I2C_TIMOUT;
		I2Cx->DR= reg;
		while((I2Cx->SR1 &(1<<7))==0){if(--I2C_Timeout==0) break;}//TXE		
		
		for(int i=0; i<size; i++){
			I2Cx->DR= data[i];
			I2C_Timeout= I2C_TIMOUT;
			while((I2Cx->SR1 &(1<<7))==0){if(--I2C_Timeout==0) break;}//TXE		
		}
	  I2Cx->CR1	 &= ~(1<<10); // dis ACK 
		
		I2Cx->CR1	 |= (1<<4); // STOP	
}

void I2Cx_ReadMulti(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint8_t reg, uint8_t * data, uint8_t size){
	
		I2C_Timeout = I2C_TIMOUT;
		I2Cx->CR1	 |= (1<<8)|(1<<10);  //Start generation  + ACK en
		while((I2Cx->SR1 &(1<<0))==0){if(--I2C_Timeout==0) break;}//Start
		
		I2C_Timeout = I2C_TIMOUT;
		I2Cx->DR= (DevAddress<<1) | 0;
		while((I2Cx->SR1 &(1<<1))==0  | (I2Cx->SR2 &(1<<1))==0){if(--I2C_Timeout==0) break;}//BUSY
		
		I2C_Timeout = I2C_TIMOUT;
		I2Cx->DR= reg;
		while((I2Cx->SR1 &(1<<7))==0){if(--I2C_Timeout==0) break;}//TXE		
	
		I2C_Timeout = I2C_TIMOUT;
	  I2Cx->CR1	 |= (1<<8)|(1<<10);  //Start generation  + ACK en
		while((I2Cx->SR1 &(1<<0))==0){if(--I2C_Timeout==0) break;}//Start
		
		I2C_Timeout = I2C_TIMOUT;
		I2Cx->DR= (DevAddress<<1) | 1;
		while((I2Cx->SR1 &(1<<1))==0  | (I2Cx->SR2 &(1<<1))==0){if(--I2C_Timeout==0) break;}//BUSY
	
		if(size>0){
			for(int i=0; i<size-1; i++){
				I2C_Timeout= I2C_TIMOUT;
				while((I2Cx->SR1 &(1<<6))==0){if(--I2C_Timeout==0) break;}//RXE	
				data[i]=I2Cx->DR;			
			}
		}
	
		I2C_Timeout = I2C_TIMOUT;
		while((I2Cx->SR1 &(1<<6))==0){if(--I2C_Timeout==0) break;}//RXE
		data[size-1]=I2Cx->DR;		
	
	  I2Cx->CR1	 &= ~(1<<10); // dis ACK
		
		I2Cx->CR1	 |= (1<<4); // STOP	
}


// Start step
void i2c_start(I2C_TypeDef *I2Cx)
{
//	if(i2c==1)
//	{
//		I2C1->CR1 |= 0x100;
//		while (!(I2C1->SR1 & 1)){};
//	}
	I2C_Timeout = I2C_TIMOUT;
	I2Cx->CR1	 |= (1<<8)|(1<<10);  //Start generation  + ACK en
	while((I2Cx->SR1 &(1<<0))==0){if(--I2C_Timeout==0) break;}//Start
	
}
// Sending the address + R or Write	
void i2c_add(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint8_t RW)
{
	volatile int tmp;
//	if(i2c==1)
//	{
//		I2C1->DR = (address|RW);
//		while((I2C1->SR1 & 2)==0){};
//		while((I2C1->SR1 & 2)){
//			tmp = I2C1->SR1;
//			tmp = I2C1->SR2;
//			if((I2C1->SR1 & 2)==0)
//			{
//				break;
//			}
//		}
//	}
	 
		I2Cx->DR = (DevAddress|RW);
		while((I2Cx->SR1 & 2)==0){};
		while((I2Cx->SR1 & 2)){
			tmp = I2Cx->SR1;
			tmp = I2Cx->SR2;
			if((I2Cx->SR1 & 2)==0)
			{
				break;
			}
		}
	
//	I2C_Timeout = I2C_TIMOUT;
//	I2Cx->DR= (DevAddress<<1) | 0;
//	while((I2Cx->SR1 &(1<<1))==0  | (I2Cx->SR2 &(1<<1))==0){if(--I2C_Timeout==0) break;}//BUSY
//	
//	I2C_Timeout = I2C_TIMOUT;
//	I2Cx->DR= RW;
//	while((I2Cx->SR1 &(1<<7))==0){if(--I2C_Timeout==0) break;}//TXE	

}
// Sending data step
void i2c_data(I2C_TypeDef *I2Cx,char data)
{
//	if(i2c==1)
//	{
//		while((I2C1->SR1 & 0x80) == 0){}
//			I2C1->DR = data;
//		while((I2C1->SR1 & 0x80) == 0){}
//	}
	
	while((I2Cx->SR1 & (1<<7)) == 0){}
	I2C1->DR = data;
	while((I2Cx->SR1 & (1<<7)) == 0){}
	
}
// Stop step
void i2c_stop(I2C_TypeDef *I2Cx)
{
	volatile int tmp;
//	if(i2c==1)
//	{
//		tmp = I2C1->SR1;
//		tmp = I2C1->SR2;
//		I2C1->CR1 |= 0x200;
//	}
	
	tmp = I2Cx->SR1;
	tmp = I2Cx->SR2;
	I2C1->CR1 |= 1<<9;
}

// i2c_write()
void i2c_write(I2C_TypeDef *I2Cx, uint16_t DevAddress,char data[])
{
	int i = 0;
	
	i2c_start(I2Cx);
	
	i2c_add(I2Cx, DevAddress,0);
	
	while(data[i])
		{
			i2c_data(I2Cx,data[i]);
			i++;
		}
	i2c_stop(I2Cx);
}


