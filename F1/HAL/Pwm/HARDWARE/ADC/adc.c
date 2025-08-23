/**	
 * |----------------------------------------------------------------------
 * |  *  02, 2014
 *	- Added support for measure Vbat pin with ADC
 *	
 * Pinout
 *	CHANNEL			ADC1	ADC2	ADC3
 *	0							PA0		PA0		PA0
 *	1							PA1		PA1		PA1
 *	2							PA2		PA2		PA2
 *	3							PA3		PA3		PA3
 *	4							PA4		PA4		PF6
 *	5							PA5		PA5		PF7
 *	6							PA6		PA6		PF8
 *	7							PA7		PA7		PF9
 *	8							PB0		PB0		PF10
 *	9							PB1		PB1		PF3
 *	10						PC0		PC0		PC0
 *	11						PC1		PC1		PC1
 *	12						PC2		PC2		PC2
 *	13						PC3		PC3		PC3
 *	14						PC4		PC4		PF4
 *	15						PC5		PC5		PF5
 *
 * |----------------------------------------------------------------------
 */
#include "adc.h"
//#include "delay.h"

//uint16_t ADC_ConvertedValue=0;

/* Internal functions */
void ADC_INT_Channel_0_Init(ADC_TypeDef* ADCx);
void ADC_INT_Channel_1_Init(ADC_TypeDef* ADCx);
void ADC_INT_Channel_2_Init(ADC_TypeDef* ADCx);
void ADC_INT_Channel_3_Init(ADC_TypeDef* ADCx);
void ADC_INT_Channel_4_Init(ADC_TypeDef* ADCx);
void ADC_INT_Channel_5_Init(ADC_TypeDef* ADCx);
void ADC_INT_Channel_6_Init(ADC_TypeDef* ADCx);
void ADC_INT_Channel_7_Init(ADC_TypeDef* ADCx);
void ADC_INT_Channel_8_Init(ADC_TypeDef* ADCx);
void ADC_INT_Channel_9_Init(ADC_TypeDef* ADCx);
void ADC_INT_Channel_10_Init(ADC_TypeDef* ADCx);
void ADC_INT_Channel_11_Init(ADC_TypeDef* ADCx);
void ADC_INT_Channel_12_Init(ADC_TypeDef* ADCx);
void ADC_INT_Channel_13_Init(ADC_TypeDef* ADCx);
void ADC_INT_Channel_14_Init(ADC_TypeDef* ADCx);
void ADC_INT_Channel_15_Init(ADC_TypeDef* ADCx);
void ADC_INT_InitPin( GPIO_TypeDef* GPIOx, uint16_t PinX);

uint8_t vbatEnabled = 0;
uint8_t mult_vref = 0;
uint8_t mult_vbat = 0;

ADC_HandleTypeDef ADC;

void Adc_Init(ADC_TypeDef* ADCx, uint8_t channel) {
	
	//ADC_InitTypeDef ADC_InitDef;
	//DMA_InitTypeDef DMA_InitStructure;/////////////////////////////
	
	ADC_ChannelConfTypeDef sConfig;

	
	if (channel == ADC_CHANNEL_0) {
		ADC_INT_Channel_0_Init(ADCx);
	} else if (channel == ADC_CHANNEL_1) {
		ADC_INT_Channel_1_Init(ADCx);
	} else if (channel == ADC_CHANNEL_2) {
		ADC_INT_Channel_2_Init(ADCx);
	} else if (channel == ADC_CHANNEL_3) {
		ADC_INT_Channel_3_Init(ADCx);
	} else if (channel == ADC_CHANNEL_4) {
		ADC_INT_Channel_4_Init(ADCx);
	} else if (channel == ADC_CHANNEL_5) {
		ADC_INT_Channel_5_Init(ADCx);
	} else if (channel == ADC_CHANNEL_6) {
		ADC_INT_Channel_6_Init(ADCx);
	} else if (channel == ADC_CHANNEL_7) {
		ADC_INT_Channel_7_Init(ADCx);
	} else if (channel == ADC_CHANNEL_8) {
		ADC_INT_Channel_8_Init(ADCx);
	} else if (channel == ADC_CHANNEL_9) {
		ADC_INT_Channel_9_Init(ADCx);
	} else if (channel == ADC_CHANNEL_10) {
		ADC_INT_Channel_10_Init(ADCx);
	} else if (channel == ADC_CHANNEL_11) {
		ADC_INT_Channel_11_Init(ADCx);
	} else if (channel == ADC_CHANNEL_12) {
		ADC_INT_Channel_12_Init(ADCx);
	} else if (channel == ADC_CHANNEL_13) {
		ADC_INT_Channel_13_Init(ADCx);
	} else if (channel == ADC_CHANNEL_14) {
		ADC_INT_Channel_14_Init(ADCx);
	} else if (channel == ADC_CHANNEL_15) {
		ADC_INT_Channel_15_Init(ADCx);
	}
	
	/* Init ADC */
	//ADC_InitADC(ADCx);	
	
	if (ADCx == ADC1) {
		__HAL_RCC_ADC1_CLK_ENABLE();		
	} else if (ADCx == ADC2) {
		__HAL_RCC_ADC2_CLK_ENABLE();
	} 
#ifdef ADC3

	else if (ADCx == ADC3) {
		__HAL_RCC_ADC3_CLK_ENABLE();
	}
	
#endif
	///////////////////////////////////////////////////////////////////////////////////////////
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//	/* DMA channel1 configuration ----------------------------------------------*/
//  DMA_DeInit(DMA1_Channel1);
//  DMA_InitStructure.DMA_PeripheralBaseAddr = ((uint32_t)0x4001244C);
//  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//  DMA_InitStructure.DMA_BufferSize = 1;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
//  
//  /* Enable DMA1 channel1 */
//  DMA_Cmd(DMA1_Channel1, ENABLE);
	
	////////////////////////////////////////////////////////////////////////////////////
		
//	ADC_InitDef.ADC_Mode = ADC_Mode_Independent;
//  ADC_InitDef.ADC_ScanConvMode = ENABLE;
//  ADC_InitDef.ADC_ContinuousConvMode = ENABLE;
//  ADC_InitDef.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
//  ADC_InitDef.ADC_DataAlign = ADC_DataAlign_Right;
//  ADC_InitDef.ADC_NbrOfChannel = 1;	
//	/* Initialize ADC */
//	ADC_Init(ADCx, &ADC_InitDef);

		ADC.Instance = ADCx;
			ADC.Init.ScanConvMode = ADC_SCAN_DISABLE;
			ADC.Init.ContinuousConvMode = DISABLE;
			ADC.Init.DiscontinuousConvMode = DISABLE;
			ADC.Init.ExternalTrigConv = ADC_SOFTWARE_START;
			ADC.Init.DataAlign = ADC_DATAALIGN_RIGHT;
			ADC.Init.NbrOfDiscConversion = 0;
			ADC.Init.NbrOfConversion = 1;
		HAL_ADC_Init(&ADC);
		
		/**Configure Regular Channel 
				*/
			sConfig.Channel = (uint8_t)channel;
			sConfig.Rank = 1;
			sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
			
			//ADC.Instance = ADCx;
			//HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			/* Return zero */
			if (HAL_ADC_ConfigChannel(&ADC, &sConfig) != HAL_OK) {
				//return 0;
			}	
	
	/* Enable ADCx DMA */
  //ADC_DMACmd(ADCx, ENABLE);//////////////////////////////////////////////
	
//	/* Enable ADC */
//	ADC_Cmd(ADCx, ENABLE);
	
	while(HAL_ADCEx_Calibration_Start(&ADC) != HAL_OK);           // calibrate AD convertor
	
	
	//Calib ADC
//	ADC_ResetCalibration(ADCx);	 
//	while(ADC_GetResetCalibrationStatus(ADCx));	
//	
//	ADC_StartCalibration(ADCx);
//  while(ADC_GetCalibrationStatus(ADCx));	
	
	
//	/* Start ADC1 Software Conversion */ 
//  ADC_SoftwareStartConvCmd(ADCx, ENABLE);
			
			/* Start conversion */  
	if (HAL_ADC_Start(&ADC) != HAL_OK) {
		//return 0;
	}
}

void ADC_InitADC(ADC_TypeDef* ADCx) {
	
}

uint16_t ADC_Read(ADC_TypeDef* ADCx, uint8_t channel) {
	ADC_ChannelConfTypeDef sConfig;
	
	
	/**Configure Regular Channel 
				*/
			sConfig.Channel = (uint8_t)channel;
			sConfig.Rank = 1;
			sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
			
			//ADC.Instance = ADCx;
			//HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			/* Return zero */
			if (HAL_ADC_ConfigChannel(&ADC, &sConfig) != HAL_OK) {
				return 0;
			}	
			
						/* Start conversion */  
	if (HAL_ADC_Start(&ADC) != HAL_OK) {
		return 0;
	}
//	ADC_RegularChannelConfig(ADCx, channel, 1, ADC_SampleTime_239Cycles5);
//	ADC_SoftwareStartConvCmd(ADCx,ENABLE);
//	
//	/* Wait till done */
//	while (ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC) == RESET);
//	
//	/* Return result */
//	return ADC_GetConversionValue(ADCx);
			
			/* Poll for end */
	if (HAL_ADC_PollForConversion(&ADC, 10) == HAL_OK) {
		/* Get the converted value of regular channel */		
		// Read the value
	//if (HAL_ADC_GetState(&hadc1) == HAL_ADC_STATE_EOC_REG){	

	return HAL_ADC_GetValue(&ADC);//}
	}


	return 0;
}

uint16_t Get_Adc_Average(ADC_TypeDef* ADCx, uint8_t channel,uint8_t times)
{
	uint32_t temp_val=0;
	uint8_t t;
	for(t=0;t<times;t++)
	{
		temp_val+=ADC_Read( ADCx,channel);
		delay(5);
	}
	return temp_val/times;
} 

void ADC_EnableVbat(void) {
//	/* Enable VBAT */
//	ADC_VBATCmd(ENABLE);
//	
//	/* Store vbat enabled */
//	vbatEnabled = 1;
}

void ADC_DisableVbat(void) {
//	/* Disable VBAT */
//	ADC_VBATCmd(DISABLE);
//	
//	/* Store vbat enabled */
//	vbatEnabled = 0;
}

uint16_t ADC_ReadVbat(ADC_TypeDef* ADCx) {
	uint32_t result;
//	/* Read battery voltage data */
//	/* Start conversion */
//	ADC_RegularChannelConfig(ADCx, ADC_CHANNEL_Vbat, 1, ADC_SampleTime_112Cycles);
//	ADC_SoftwareStartConv(ADCx);
//	
//	/* Wait till done */
//	while (ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC) == RESET);
//	
//	/* Get result */
//	result = ADC_GetConversionValue(ADCx);
//	
//	/* Convert to voltage */
//	result = result * ADC_VBAT_MULTI * ADC_SUPPLY_VOLTAGE / 0xFFF;
//	
//	/* Return value in mV */
	return (uint16_t) result;
}

/* Private functions */
void ADC_INT_Channel_0_Init(ADC_TypeDef* ADCx) {
	ADC_INT_InitPin( GPIOA, GPIO_PIN_0);
}
void ADC_INT_Channel_1_Init(ADC_TypeDef* ADCx) {
	ADC_INT_InitPin( GPIOA, GPIO_PIN_1);
}
void ADC_INT_Channel_2_Init(ADC_TypeDef* ADCx) {
	ADC_INT_InitPin( GPIOA, GPIO_PIN_2);
}
void ADC_INT_Channel_3_Init(ADC_TypeDef* ADCx) {
	ADC_INT_InitPin( GPIOA, GPIO_PIN_3);
}
void ADC_INT_Channel_4_Init(ADC_TypeDef* ADCx) {
	if (ADCx == ADC1 || ADCx == ADC2) {
		ADC_INT_InitPin( GPIOA, GPIO_PIN_4);
	}
#ifdef	ADC3
	else if (ADCx == ADC3) {
		ADC_INT_InitPin( GPIOF, GPIO_PIN_6);
	}
#endif
}
void ADC_INT_Channel_5_Init(ADC_TypeDef* ADCx) {
	if (ADCx == ADC1 || ADCx == ADC2) {
		ADC_INT_InitPin( GPIOA, GPIO_PIN_5);
	} 
#ifdef	AD3
	else if (ADCx == ADC3) {
		ADC_INT_InitPin( GPIOF, GPIO_PIN_7);
	}
#endif
}
void ADC_INT_Channel_6_Init(ADC_TypeDef* ADCx) {
	if (ADCx == ADC1 || ADCx == ADC2) {
		ADC_INT_InitPin( GPIOA, GPIO_PIN_6);
	} 
#ifdef ADC3	
	else if (ADCx == ADC3) {
		ADC_INT_InitPin( GPIOF, GPIO_PIN_8);
	}
#endif
}
void ADC_INT_Channel_7_Init(ADC_TypeDef* ADCx) {
	if (ADCx == ADC1 || ADCx == ADC2) {
		ADC_INT_InitPin( GPIOA, GPIO_PIN_7);
	} 
#ifdef ADC3	
	else if (ADCx == ADC3) {
		ADC_INT_InitPin( GPIOF, GPIO_PIN_9);
	}
#endif
}
void ADC_INT_Channel_8_Init(ADC_TypeDef* ADCx) {
	if (ADCx == ADC1 || ADCx == ADC2) {
		ADC_INT_InitPin( GPIOB, GPIO_PIN_0);
	} 
#ifdef ADC3	
	else if (ADCx == ADC3) {
		ADC_INT_InitPin( GPIOF, GPIO_PIN_10);
	}
#endif
}
void ADC_INT_Channel_9_Init(ADC_TypeDef* ADCx) {
	if (ADCx == ADC1 || ADCx == ADC2) {
		ADC_INT_InitPin( GPIOB, GPIO_PIN_1);
	} 
#ifdef ADC3		
	else if (ADCx == ADC3) {
		ADC_INT_InitPin( GPIOF, GPIO_PIN_3);
	}
#endif
}
void ADC_INT_Channel_10_Init(ADC_TypeDef* ADCx) {
	ADC_INT_InitPin( GPIOC, GPIO_PIN_0);
}
void ADC_INT_Channel_11_Init(ADC_TypeDef* ADCx) {
	ADC_INT_InitPin( GPIOC, GPIO_PIN_1);
}
void ADC_INT_Channel_12_Init(ADC_TypeDef* ADCx) {
	ADC_INT_InitPin( GPIOC, GPIO_PIN_2);
}
void ADC_INT_Channel_13_Init(ADC_TypeDef* ADCx) {
	ADC_INT_InitPin( GPIOC, GPIO_PIN_3);
}
void ADC_INT_Channel_14_Init(ADC_TypeDef* ADCx) {
	if (ADCx == ADC1 || ADCx == ADC2) {
		ADC_INT_InitPin( GPIOC, GPIO_PIN_3);
	} 
#ifdef ADC3			
	else if (ADCx == ADC3) {
		ADC_INT_InitPin( GPIOF, GPIO_PIN_4);
	}
#endif
}
void ADC_INT_Channel_15_Init(ADC_TypeDef* ADCx) {
	if (ADCx == ADC1 || ADCx == ADC2) {
		ADC_INT_InitPin( GPIOC, GPIO_PIN_5);
	} 
#ifdef ADC3			
	else if (ADCx == ADC3) {
		ADC_INT_InitPin( GPIOF, GPIO_PIN_5);
	}
#endif
}

void ADC_INT_InitPin( GPIO_TypeDef* GPIOx, uint16_t PinX) {
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/* Enable GPIO clock */
	//RCC_APB2PeriphClockCmd(RCCx, ENABLE);
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
		//else if (GPIOx == GPIOE) {
//		// Enable clock for GPIOE
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

//	}else if (GPIOx == GPIOF) {
//		// Enable clock for GPIOF
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);

//	}else if (GPIOx == GPIOG) {
//		// Enable clock for GPIOG
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);

//	}

	/* Set GPIO settings */
	GPIO_InitStruct.Pin= PinX;
	//GPIO_InitStruct.GPIO_Speed = (GPIOSpeed_TypeDef)0;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

	/* Initialize GPIO pin */
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
