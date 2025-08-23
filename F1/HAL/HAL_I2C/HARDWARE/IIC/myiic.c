#include "myiic.h"
//#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
/*
 *	Pinout
 *	
 *			
 *	I2CX	|SCL	SDA		|SCL	SDA		|SCL	SDA
 *			|				|				|
 *	I2C1	|PB6	PB7		|PB8	PB9		|PB6	PB9
 *	I2C2	|PB10	PB11	|PF1	PF0		|PH4	PH5
 *	I2C3	|PA8	PC9		|PH7	PH8		|-		-
 *
////////////////////////////////////////////////////////////////////////////////////// 
						 SCL:SDA
					Pin_PB6PB7,	////I2C1
					Pin_PB8PB9,	////I2C1
					Pin_PB6PB9,	////1
					Pin_PB10PB11,	////I2C2
					Pin_PF1PF0,	////I2C2
					Pin_PA8PC9,	////I2C3
					Pin_PH7PH8	////I2C3

 */								  
////////////////////////////////////////////////////////////////////////////////// 	

uint32_t I2C_Timeout;

/* Handle values for I2C */

static I2C_HandleTypeDef I2C1Handle = {I2C1};

static I2C_HandleTypeDef I2C2Handle = {I2C2};

//static I2C_HandleTypeDef I2C3Handle = {I2C3};

//static I2C_HandleTypeDef I2C4Handle = {I2C4};

I2C_HandleTypeDef* I2C_GetHandle(I2C_TypeDef* I2Cx) {

	if (I2Cx == I2C1) {
		return &I2C1Handle;
	}

	if (I2Cx == I2C2) {
		return &I2C2Handle;
	}

//	if (I2Cx == I2C3) {
//		return &I2C3Handle;
//	}

//	if (I2Cx == I2C4) {
//		return &I2C4Handle;
//	}

	
	/* Return invalid */
	return 0;
}


GPIO_InitTypeDef GPIO_I2C_InitStruct;

/* Private functions */
void I2C1_InitPins(I2C_PinsPack_t pinspack);
void I2C2_InitPins(I2C_PinsPack_t pinspack);
void I2C3_InitPins(I2C_PinsPack_t pinspack);

void IIC_Init(I2C_TypeDef* I2Cx, I2C_PinsPack_t pinspack, uint32_t clockSpeed) {
	
	I2C_HandleTypeDef* Handle = I2C_GetHandle(I2Cx);//I2C_InitTypeDef I2C_InitStruct;
	
	__HAL_RCC_AFIO_CLK_ENABLE();//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	
	/* Set default I2C GPIO	settings */
	GPIO_I2C_InitStruct.Mode = GPIO_MODE_AF_OD;	
	GPIO_I2C_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	
	
	if (I2Cx == I2C1) {
		/* Enable clock */
		__HAL_RCC_I2C1_CLK_ENABLE();//RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
		__HAL_RCC_I2C1_FORCE_RESET();
		__HAL_RCC_I2C1_RELEASE_RESET();
		
		/* Enable pins */
		I2C1_InitPins(pinspack);

		
//		/* Set values */
//		I2C_InitStruct.I2C_ClockSpeed = clockSpeed;
//		I2C_InitStruct.I2C_AcknowledgedAddress = I2C1_ACKNOWLEDGED_ADDRESS;
//		I2C_InitStruct.I2C_Mode = I2C1_MODE;
//		I2C_InitStruct.I2C_OwnAddress1 = I2C1_OWN_ADDRESS;
//		I2C_InitStruct.I2C_Ack = I2C1_ACK;
//		I2C_InitStruct.I2C_DutyCycle = I2C1_DUTY_CYCLE;
		
		
	} else if (I2Cx == I2C2) {
		/* Enable clock */
		__HAL_RCC_I2C2_CLK_ENABLE();//RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
		__HAL_RCC_I2C2_FORCE_RESET();
    __HAL_RCC_I2C2_RELEASE_RESET();
		/* Enable pins */
		I2C2_InitPins(pinspack);
		
		
//		/* Set values */
//		I2C_InitStruct.I2C_ClockSpeed = clockSpeed;
//		I2C_InitStruct.I2C_AcknowledgedAddress = I2C2_ACKNOWLEDGED_ADDRESS;
//		I2C_InitStruct.I2C_Mode = I2C2_MODE;
//		I2C_InitStruct.I2C_OwnAddress1 = I2C2_OWN_ADDRESS;
//		I2C_InitStruct.I2C_Ack = I2C2_ACK;
//		I2C_InitStruct.I2C_DutyCycle = I2C2_DUTY_CYCLE;
	} 
	
	/* Fill settings */
			Handle->Instance = I2Cx;
	
			Handle->Init.ClockSpeed = clockSpeed;
			Handle->Init.DutyCycle = I2C_DUTYCYCLE_2;
			Handle->Init.OwnAddress2 = 0x00;
			Handle->Init.OwnAddress1 = 0x00;
			Handle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
			Handle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
			Handle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
			Handle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	
			/* Initialize I2C */
			HAL_I2C_Init(Handle);
	
//	/* Disable I2C first */
//	I2C_Cmd(I2Cx, DISABLE);
//	/* Initialize I2C */
//	I2C_Init(I2Cx, &I2C_InitStruct);
//	/* Enable I2C */
//	I2C_Cmd(I2Cx, ENABLE);
}

uint8_t I2C_Read(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg) {
	uint8_t received_data=0;
//	I2C_Start(I2Cx, address, I2C_Direction_Transmitter, 0);
//	I2C_WriteData(I2Cx, reg);
//	I2C_Stop(I2Cx);
//	I2C_Start(I2Cx, address, I2C_Direction_Receiver, 1);
//	received_data = I2C_ReadNack(I2Cx);
	
	I2C_HandleTypeDef* Handle = I2C_GetHandle(I2Cx);
	
	/* Send address */
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}		
	
	}
	
	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(Handle, address,&received_data, 1, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
	}	

	
	return received_data;
}

void I2C_Write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t data) {
//	I2C_Start(I2Cx, address, I2C_Direction_Transmitter, 0);
//	I2C_WriteData(I2Cx, reg);
//	I2C_WriteData(I2Cx, data);
//	I2C_Stop(I2Cx);
	uint8_t d[2];
	I2C_HandleTypeDef* Handle = I2C_GetHandle(I2Cx);
		
	/* Format array to send */
	d[0] = reg;
	d[1] = data;
	/* Try to transmit via I2C */
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)address, (uint8_t *)d, 2, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}		
		
	} 
	
	
}

uint8_t I2C_ReadMulti(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {
	I2C_HandleTypeDef* Handle = I2C_GetHandle(I2Cx);
	
	/* Send register address */
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return 1;
	}
	
	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(Handle, address, data, count, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return 1;
	}
	
	/* Return OK */
	return 0;
}

uint8_t I2C_WriteMulti(I2C_TypeDef* I2Cx, uint8_t address, uint16_t reg, uint8_t* data, uint16_t count) {
//	uint8_t i;
//	I2C_Start(I2Cx, address, I2C_Direction_Transmitter, 0);
//	I2C_WriteData(I2Cx, reg);
//	for (i = 0; i < count; i++) {
//		I2C_WriteData(I2Cx, data[i]);
//	}
//	I2C_Stop(I2Cx);
	I2C_HandleTypeDef* Handle = I2C_GetHandle(I2Cx);

	/* Try to transmit via I2C */
	if (HAL_I2C_Mem_Write(Handle, address, reg, reg > 0xFF ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT, data, count, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return 1;
	}
	
	/* Return OK */
	return 0;
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void I2C_ReadNoRegister(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* data) {
	I2C_HandleTypeDef* Handle = I2C_GetHandle(I2Cx);

	/* Receive single byte without specifying  */
	if (HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 1, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}		
		
	}
}

void I2C_ReadMultiNoRegister(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* data, uint16_t count) {
//	uint8_t i;
//	I2C_Start(I2Cx, address, I2C_Direction_Receiver, 1);
//	for (i = 0; i < count; i++) {
//		if (i == (count - 1)) {
//			//Last byte
//			data[i] = I2C_ReadNack(I2Cx);
//		} else {
//			data[i] = I2C_ReadAck(I2Cx);
//		}
//	}
	
		I2C_HandleTypeDef* Handle = I2C_GetHandle(I2Cx);

	/* Receive multi bytes without specifying  */
	if (HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, count, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
	}
	

}

uint8_t TM_I2C_WriteNoRegister(I2C_TypeDef* I2Cx, uint8_t address, uint8_t data) {
	I2C_HandleTypeDef* Handle = I2C_GetHandle(I2Cx);
	
	/* Try to transmit via I2C */
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &data, 1, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return 1;
	} 
	
	/* Return OK */
	return 0;
}

uint8_t TM_I2C_WriteMultiNoRegister(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* data, uint16_t count) {
	I2C_HandleTypeDef* Handle = I2C_GetHandle(I2Cx);
	
	/* Try to transmit via I2C */
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)address, data, count, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return 1;
	} 
	
	/* Return OK */
	return 0;
}



//int16_t I2C_Start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction, uint8_t ack) {
//	I2C_GenerateSTART(I2Cx, ENABLE);
//	
//	I2C_Timeout = I2C_TIMEOUT;
//	while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) && I2C_Timeout) {
//		if (--I2C_Timeout == 0x00) {
//			return 1;
//		}
//	}

//	if (ack) {
//		I2C_AcknowledgeConfig(I2C1, ENABLE);
//	}
//	
//	I2C_Send7bitAddress(I2Cx, address, direction);

//	if (direction == I2C_Direction_Transmitter) {
//		I2C_Timeout = I2C_TIMEOUT;
//		while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR) && I2C_Timeout) {
//			if (--I2C_Timeout == 0x00) {
//				return 1;
//			}
//		}
//	} else if (direction == I2C_Direction_Receiver) {
//		I2C_Timeout = I2C_TIMEOUT;
//		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && I2C_Timeout) {
//			if (--I2C_Timeout == 0x00) {
//				return 1;
//			}
//		}
//	}
//	I2Cx->SR2;
//	
//	return 0;
//}


//void I2C_WriteData(I2C_TypeDef* I2Cx, uint8_t data) {
//	I2C_Timeout = I2C_TIMEOUT;
//	while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE) && I2C_Timeout) {
//		I2C_Timeout--;
//	}
//	I2C_SendData(I2Cx, data);
//}


//uint8_t I2C_ReadAck(I2C_TypeDef* I2Cx) {
//	uint8_t data;
//	I2C_AcknowledgeConfig(I2Cx, ENABLE);
//	
//	I2C_Timeout = I2C_TIMEOUT;
//	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) && I2C_Timeout) {
//		I2C_Timeout--;
//	}

//	data = I2C_ReceiveData(I2Cx);
//	return data;
//}


//uint8_t I2C_ReadNack(I2C_TypeDef* I2Cx) {
//	uint8_t data;
//	
//	I2C_AcknowledgeConfig(I2Cx, DISABLE);
//	
//	I2C_GenerateSTOP(I2Cx, ENABLE);
//	
//	I2C_Timeout = I2C_TIMEOUT;
//	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) && I2C_Timeout) {
//		I2C_Timeout--;
//	}

//	data = I2C_ReceiveData(I2Cx);
//	return data;
//}

//uint8_t I2C_Stop(I2C_TypeDef* I2Cx) {	
//	I2C_Timeout = I2C_TIMEOUT;
//	while (((!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE)) || (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF))) && I2C_Timeout) {
//		if (--I2C_Timeout == 0x00) {
//			return 1;
//		}
//	}
//	I2C_GenerateSTOP(I2Cx, ENABLE);
//	
//	return 0;
//}

uint8_t I2C_IsDeviceConnected(I2C_TypeDef* I2Cx, uint8_t address) {
	uint8_t connected = 0;
//	if (!I2C_Start(I2Cx, address, I2C_Direction_Transmitter, 1)) {
//		connected = 1;
//	}
//	I2C_Stop(I2Cx);
	
	I2C_HandleTypeDef* Handle = I2C_GetHandle(I2Cx);
	
	/* Check if device is ready for communication */
	if (HAL_I2C_IsDeviceReady(Handle, address, 2, 5) != HAL_OK) {
		/* Return error */
		return 1;
	}
	
	return connected;
}

/* Private functions */
void I2C1_InitPins(I2C_PinsPack_t pinspack) {
	/* Enable clock */
	__HAL_RCC_GPIOB_CLK_ENABLE();//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	if (pinspack == Pin_PB6PB7) {
		//                      SCL          SDA
		GPIO_I2C_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
		HAL_GPIO_Init(GPIOB, &GPIO_I2C_InitStruct);
		
	} else if (pinspack == Pin_PB8PB9) {
		//                      SCL          SDA
		GPIO_I2C_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
		HAL_GPIO_Init(GPIOB, &GPIO_I2C_InitStruct);
		
		 __HAL_AFIO_REMAP_I2C1_ENABLE();//GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
//		GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
//		GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
	} else if (pinspack == Pin_PB6PB9) {
		//                      SCL          SDA
		GPIO_I2C_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_9;
		HAL_GPIO_Init(GPIOB, &GPIO_I2C_InitStruct);
		
		 __HAL_AFIO_REMAP_I2C1_ENABLE();//GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
		
//		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
//		GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
	}
	/* Init pins */
	
}

void I2C2_InitPins(I2C_PinsPack_t pinspack) {
	
	if (pinspack == Pin_PB10PB11) {
		__HAL_RCC_GPIOB_CLK_ENABLE();//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		
		//                      SCL           SDA
		GPIO_I2C_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
		
//		GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
//		GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);
		
		HAL_GPIO_Init(GPIOB, &GPIO_I2C_InitStruct);
	} 
}



