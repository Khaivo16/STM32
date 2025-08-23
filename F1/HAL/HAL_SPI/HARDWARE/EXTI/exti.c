#include "exti.h"
//////////////////////////////////////////////////////////////////////////////////	 
					uint8_t t=0;			  
////////////////////////////////////////////////////////////////////////////////// 
void EXTI0_IRQHandler(void)
{
  
	
  
	//Chuong trinh ngat
  //HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
	
	/* EXTI line interrupt detected */
   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
   {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
      HAL_GPIO_EXTI_Callback(GPIO_PIN_0);
			
		 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0));
			
    }
}

//void EXTI9_5_IRQHandler(void)
//{

//}

//GPIO_InitTypeDef GPIO_InitStruct;

//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOA_CLK_ENABLE();

//  /*Configure GPIO pin : PA0 */
//  GPIO_InitStruct.Pin = GPIO_PIN_0;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


//  /* EXTI interrupt init*/
//  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

//////////////////////////////////////////////////////////////////////////////////////////////////////



void EXTIx_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, EXTI_Trigger_t trigger) {
	GPIO_InitTypeDef GPIO_InitStruct;
	
	uint8_t pinsource;
	IRQn_Type irqchannel;
	
	
	/* Enable clock */
	__HAL_RCC_AFIO_CLK_ENABLE();
	
	/* Get proper settings */
	if (GPIOx == GPIOA) {
		// Enable clock for GPIOA
		__HAL_RCC_GPIOA_CLK_ENABLE();

	} else if (GPIOx == GPIOB) {
		// Enable clock for GPIOB
		__HAL_RCC_GPIOB_CLK_ENABLE();

	} else if (GPIOx == GPIOC) {
		// Enable clock for GPIOC
		__HAL_RCC_GPIOC_CLK_ENABLE();

	}else if (GPIOx == GPIOD) {
		// Enable clock for GPIOD
		__HAL_RCC_GPIOD_CLK_ENABLE();

	}
		else {
		;
	}
	
	switch (GPIO_Pin) {
		case GPIO_PIN_0:
			pinsource = 0;
			irqchannel = EXTI0_IRQn;
			break;
		case GPIO_PIN_1:
			pinsource = 1;
			irqchannel = EXTI1_IRQn;
			break;
		case GPIO_PIN_2:
			pinsource = 2;
			irqchannel = EXTI2_IRQn;
			break;
		case GPIO_PIN_3:
			pinsource = 3;
			irqchannel = EXTI3_IRQn;
			break;
		case GPIO_PIN_4:
			pinsource = 4;
			irqchannel = EXTI4_IRQn;
			break;
		case GPIO_PIN_5:
			pinsource = 5;
			irqchannel = EXTI9_5_IRQn;
			break;
		case GPIO_PIN_6:
			pinsource = 6;
			irqchannel = EXTI9_5_IRQn;
			break;
		case GPIO_PIN_7:
			pinsource = 7;
			irqchannel = EXTI9_5_IRQn;
			break;
		case GPIO_PIN_8:
			pinsource = 8;
			irqchannel = EXTI9_5_IRQn;
			break;
		case GPIO_PIN_9:
			pinsource = 9;
			irqchannel = EXTI9_5_IRQn;
			break;
		case GPIO_PIN_10:
			pinsource = 0X0A;
			irqchannel = EXTI15_10_IRQn;
			break;
		case GPIO_PIN_11:
			pinsource = 0X0B;
			irqchannel = EXTI15_10_IRQn;
			break;
		case GPIO_PIN_12:
			pinsource = 0X0C;
			irqchannel = EXTI15_10_IRQn;
			break;
		case GPIO_PIN_13:
			pinsource = 0X0D;
			irqchannel = EXTI15_10_IRQn;
			break;
		case GPIO_PIN_14:
			pinsource = 0X0E;
			irqchannel = EXTI15_10_IRQn;
			break;
		case GPIO_PIN_15:
			pinsource = 0X0F;
			irqchannel = EXTI15_10_IRQn;
			break;
		default:
			break;
	}

	
	/* Set pin as input */
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH ;
	
	if (trigger == Trigger_Falling) {
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
	} else if (trigger == Trigger_Rising) {
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	} else {
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	
	/* Initialize pin */
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);	


 /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(irqchannel, 0, pinsource);
  HAL_NVIC_EnableIRQ(irqchannel);


	
}







