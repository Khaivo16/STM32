#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//m?t s? ch?c nang
void GPIO_Set(GPIO_TypeDef* GPIOx,uint32_t  BITx,uint32_t  MODE,uint32_t PULL,uint32_t  OSPEED)
{  
		GPIO_InitTypeDef GPIO_InitStruct;
      
	
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
	 
  GPIO_InitStruct.Pin = BITx;
  GPIO_InitStruct.Mode = MODE;
  GPIO_InitStruct.Speed = OSPEED;
	GPIO_InitStruct.Pull = PULL;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
        
} 
//////////////////////////////////////////////////////////////////////////////////	 
const PinDescription g_APinDescription[]=
{

  { GPIOA, GPIO_PIN_0,RCC_CLK_GPIOA, ADC_CHANNEL_0, NULL, TIM_CHANNEL_1}, // 0     //      
  { GPIOA, GPIO_PIN_1,RCC_CLK_GPIOA, ADC_CHANNEL_1, TIM2, TIM_CHANNEL_2}, // 1         // 
  { GPIOA, GPIO_PIN_2,RCC_CLK_GPIOA, ADC_CHANNEL_2, TIM2, TIM_CHANNEL_3}, // 2
	{ GPIOA, GPIO_PIN_3,RCC_CLK_GPIOA, ADC_CHANNEL_3, TIM2, TIM_CHANNEL_4}, // 3     //      
  { GPIOA, GPIO_PIN_4,RCC_CLK_GPIOA, ADC_CHANNEL_4, NULL, NULL}, // 4         // 
  { GPIOA, GPIO_PIN_5,RCC_CLK_GPIOA, ADC_CHANNEL_5, NULL, NULL}, // 5
	{ GPIOA, GPIO_PIN_6,RCC_CLK_GPIOA, ADC_CHANNEL_6, TIM3, TIM_CHANNEL_1}, // 6     //      
  { GPIOA, GPIO_PIN_7,RCC_CLK_GPIOA, ADC_CHANNEL_7, TIM3, TIM_CHANNEL_2}, // 7         // 
  { GPIOA, GPIO_PIN_8,RCC_CLK_GPIOA, NULL, TIM1, TIM_CHANNEL_1}, // 8
	{ GPIOA, GPIO_PIN_9,RCC_CLK_GPIOA, NULL, TIM1, TIM_CHANNEL_2}, // 9     //      
  { GPIOA, GPIO_PIN_10,RCC_CLK_GPIOA,NULL, TIM1, TIM_CHANNEL_3}, // 10         // 
  { GPIOA, GPIO_PIN_11,RCC_CLK_GPIOA,NULL, TIM1, TIM_CHANNEL_4}, // 11
	{ GPIOA, GPIO_PIN_12,RCC_CLK_GPIOA,NULL, NULL, NULL}, // 12    //      
  { GPIOA, GPIO_PIN_13,RCC_CLK_GPIOA,NULL, NULL, NULL}, // 13        // 
  { GPIOA, GPIO_PIN_14,RCC_CLK_GPIOA,NULL, NULL, NULL}, // 14
	{ GPIOA, GPIO_PIN_15,RCC_CLK_GPIOA,NULL, NULL, NULL}, // 15
	
	{ GPIOB, GPIO_PIN_0,RCC_CLK_GPIOB, ADC_CHANNEL_8, TIM3, TIM_CHANNEL_3}, // 0     //      
  { GPIOB, GPIO_PIN_1,RCC_CLK_GPIOB, ADC_CHANNEL_9, TIM3, TIM_CHANNEL_4}, // 1         // 
  { GPIOB, GPIO_PIN_2,RCC_CLK_GPIOB,NULL, NULL, NULL}, // 2
	{ GPIOB, GPIO_PIN_3,RCC_CLK_GPIOB, NULL, TIM2, TIM_CHANNEL_2}, // 3     //      
  { GPIOB, GPIO_PIN_4,RCC_CLK_GPIOB, NULL, TIM3, TIM_CHANNEL_1}, // 4         // 
  { GPIOB, GPIO_PIN_5,RCC_CLK_GPIOB, NULL, TIM3, TIM_CHANNEL_2}, // 5
	{ GPIOB, GPIO_PIN_6,RCC_CLK_GPIOB, NULL, TIM4, TIM_CHANNEL_1}, // 6     //      
  { GPIOB, GPIO_PIN_7,RCC_CLK_GPIOB, NULL, TIM4, TIM_CHANNEL_2}, // 7         // 
  { GPIOB, GPIO_PIN_8,RCC_CLK_GPIOB, NULL, NULL, TIM_CHANNEL_1}, // 8//TIM10
	{ GPIOB, GPIO_PIN_9,RCC_CLK_GPIOB, NULL, NULL, TIM_CHANNEL_1}, // 9     //TIM11      
  { GPIOB, GPIO_PIN_10,RCC_CLK_GPIOB, NULL, TIM2, TIM_CHANNEL_3}, // 10         // 
  { GPIOB, GPIO_PIN_11,RCC_CLK_GPIOB, NULL, TIM2, TIM_CHANNEL_4}, // 11
	{ GPIOB, GPIO_PIN_12,RCC_CLK_GPIOB,NULL, NULL, NULL}, // 12    //      
  { GPIOB, GPIO_PIN_13,RCC_CLK_GPIOB,NULL, NULL, NULL}, // 13        // 
  { GPIOB, GPIO_PIN_14,RCC_CLK_GPIOB, NULL, NULL, TIM_CHANNEL_1}, // 14//TIM12
	{ GPIOB, GPIO_PIN_15,RCC_CLK_GPIOB, NULL, NULL, TIM_CHANNEL_2}, // 15//TIM12
	
	{ GPIOC, GPIO_PIN_0,RCC_CLK_GPIOC, ADC_CHANNEL_10, NULL, NULL}, // 0     //      
  { GPIOC, GPIO_PIN_1,RCC_CLK_GPIOC, ADC_CHANNEL_11, NULL, NULL}, // 1         // 
  { GPIOC, GPIO_PIN_2,RCC_CLK_GPIOC, ADC_CHANNEL_12, NULL, NULL}, // 2
	{ GPIOC, GPIO_PIN_3,RCC_CLK_GPIOC, ADC_CHANNEL_13, NULL, NULL}, // 3     //      
  { GPIOC, GPIO_PIN_4,RCC_CLK_GPIOC, ADC_CHANNEL_14, NULL, NULL}, // 4         // 
  { GPIOC, GPIO_PIN_5,RCC_CLK_GPIOC, ADC_CHANNEL_14, NULL, NULL}, // 5
	{ GPIOC, GPIO_PIN_6,RCC_CLK_GPIOC, NULL, NULL, TIM_CHANNEL_1}, // 6     // TIM8     
  { GPIOC, GPIO_PIN_7,RCC_CLK_GPIOC, NULL, NULL, TIM_CHANNEL_2}, // 7         // TIM8
  { GPIOC, GPIO_PIN_8,RCC_CLK_GPIOC, NULL, NULL, TIM_CHANNEL_3}, // 8		//TIM8
	{ GPIOC, GPIO_PIN_9,RCC_CLK_GPIOC, NULL, NULL, TIM_CHANNEL_4}, // 9     //   TIM8   
  { GPIOC, GPIO_PIN_10,RCC_CLK_GPIOC,NULL, NULL, NULL}, // 10         // 
  { GPIOC, GPIO_PIN_11,RCC_CLK_GPIOC,NULL, NULL, NULL}, // 11
	{ GPIOC, GPIO_PIN_12,RCC_CLK_GPIOC,NULL, NULL, NULL}, // 12    //      
  { GPIOC, GPIO_PIN_13,RCC_CLK_GPIOC,NULL, NULL, NULL}, // 13        // 
  { GPIOC, GPIO_PIN_14,RCC_CLK_GPIOC,NULL, NULL, NULL}, // 14
	{ GPIOC, GPIO_PIN_15,RCC_CLK_GPIOC,NULL, NULL, NULL}, // 15
	
	{ GPIOD, GPIO_PIN_0,RCC_CLK_GPIOD,NULL, NULL, NULL}, // 0     //      
  { GPIOD, GPIO_PIN_1,RCC_CLK_GPIOD,NULL, NULL, NULL}, // 1         // 
  { GPIOD, GPIO_PIN_2,RCC_CLK_GPIOD,NULL, NULL, NULL}, // 2
	{ GPIOD, GPIO_PIN_3,RCC_CLK_GPIOD,NULL, NULL, NULL}, // 3     //      
  { GPIOD, GPIO_PIN_4,RCC_CLK_GPIOD,NULL, NULL, NULL}, // 4         // 
  { GPIOD, GPIO_PIN_5,RCC_CLK_GPIOD,NULL, NULL, NULL}, // 5
	{ GPIOD, GPIO_PIN_6,RCC_CLK_GPIOD,NULL, NULL, NULL}, // 6     //      
  { GPIOD, GPIO_PIN_7,RCC_CLK_GPIOD,NULL, NULL, NULL}, // 7         // 
  { GPIOD, GPIO_PIN_8,RCC_CLK_GPIOD,NULL, NULL, NULL}, // 8
	{ GPIOD, GPIO_PIN_9,RCC_CLK_GPIOD,NULL, NULL, NULL}, // 9     //      
  { GPIOD, GPIO_PIN_10,RCC_CLK_GPIOD,NULL, NULL, NULL}, // 10         // 
  { GPIOD, GPIO_PIN_11,RCC_CLK_GPIOD,NULL, NULL, NULL}, // 11
	{ GPIOD, GPIO_PIN_12,RCC_CLK_GPIOD, NULL, TIM4, TIM_CHANNEL_1}, // 12    //      
  { GPIOD, GPIO_PIN_13,RCC_CLK_GPIOD, NULL, TIM4, TIM_CHANNEL_2}, // 13        // 
  { GPIOD, GPIO_PIN_14,RCC_CLK_GPIOD, NULL, TIM4, TIM_CHANNEL_3}, // 14
	{ GPIOD, GPIO_PIN_15,RCC_CLK_GPIOD, NULL, TIM4, TIM_CHANNEL_4}, // 15
	
	{ GPIOE, GPIO_PIN_0,RCC_CLK_GPIOE,NULL, NULL, NULL}, // 0     //      
  { GPIOE, GPIO_PIN_1,RCC_CLK_GPIOE,NULL, NULL, NULL}, // 1         // 
  { GPIOE, GPIO_PIN_2,RCC_CLK_GPIOE,NULL, NULL, NULL}, // 2
	{ GPIOE, GPIO_PIN_3,RCC_CLK_GPIOE,NULL, NULL, NULL}, // 3     //      
  { GPIOE, GPIO_PIN_4,RCC_CLK_GPIOE,NULL, NULL, NULL}, // 4         // 
  { GPIOE, GPIO_PIN_5,RCC_CLK_GPIOE, NULL, NULL, TIM_CHANNEL_1}, // 5
	{ GPIOE, GPIO_PIN_6,RCC_CLK_GPIOE, NULL, NULL, TIM_CHANNEL_2}, // 6     //      
  { GPIOE, GPIO_PIN_7,RCC_CLK_GPIOE,NULL, NULL, NULL}, // 7         // 
  { GPIOE, GPIO_PIN_8,RCC_CLK_GPIOE,NULL, NULL, NULL}, // 8
	{ GPIOE, GPIO_PIN_9,RCC_CLK_GPIOE, NULL, TIM1, TIM_CHANNEL_1}, // 9     //      
  { GPIOE, GPIO_PIN_10,RCC_CLK_GPIOE,NULL, NULL, NULL}, // 10         // 
  { GPIOE, GPIO_PIN_11,RCC_CLK_GPIOE, NULL, TIM1, TIM_CHANNEL_2}, // 11
	{ GPIOE, GPIO_PIN_12,RCC_CLK_GPIOE,NULL, NULL, NULL}, // 12    //      
  { GPIOE, GPIO_PIN_13,RCC_CLK_GPIOE, NULL, TIM1, TIM_CHANNEL_3}, // 13        // 
  { GPIOE, GPIO_PIN_14,RCC_CLK_GPIOE, NULL, TIM1, TIM_CHANNEL_4}, // 14
	{ GPIOE, GPIO_PIN_15,RCC_CLK_GPIOE,NULL, NULL, NULL}, // 15
	
//	{ GPIOF, GPIO_PIN_0,RCC_CLK_GPIOF}, // 0     //      
//  { GPIOF, GPIO_PIN_1,RCC_CLK_GPIOF}, // 1         // 
//  { GPIOF, GPIO_PIN_2,RCC_CLK_GPIOF}, // 2
//	{ GPIOF, GPIO_PIN_3,RCC_CLK_GPIOF}, // 3     //      
//  { GPIOF, GPIO_PIN_4,RCC_CLK_GPIOF}, // 4         // 
//  { GPIOF, GPIO_PIN_5,RCC_CLK_GPIOF}, // 5
//	{ GPIOF, GPIO_PIN_6,RCC_CLK_GPIOF}, // 6     //      
//  { GPIOF, GPIO_PIN_7,RCC_CLK_GPIOF}, // 7         // 
//  { GPIOF, GPIO_PIN_8,RCC_CLK_GPIOF}, // 8
//	{ GPIOF, GPIO_PIN_9,RCC_CLK_GPIOF}, // 9     //      
//  { GPIOF, GPIO_PIN_10,RCC_CLK_GPIOF}, // 10         // 
//  { GPIOF, GPIO_PIN_11,RCC_CLK_GPIOF}, // 11
//	{ GPIOF, GPIO_PIN_12,RCC_CLK_GPIOF}, // 12    //      
//  { GPIOF, GPIO_PIN_13,RCC_CLK_GPIOF}, // 13        // 
//  { GPIOF, GPIO_PIN_14,RCC_CLK_GPIOF}, // 14
//	{ GPIOF, GPIO_PIN_15,RCC_CLK_GPIOF}, // 15		//95
} ;
////////////////////////////////////////////////////////////////////////
//m?t s? ch?c nang

void pinMode(uint32_t PINx,uint32_t MODE)
{  
   //u32 pinpos=0,pos=0,curpin=0;
	
	 GPIO_TypeDef *GPIOx = g_APinDescription[PINx].pPort;
	 uint16_t BITx = g_APinDescription[PINx].ulPin;
	 GPIO_InitTypeDef GPIO_InitStructure;	
	 uint32_t CLK = g_APinDescription[PINx].ulPeripheral;
	
	if (CLK == RCC_CLK_GPIOA) {
		// 
		__HAL_RCC_GPIOA_CLK_ENABLE();

	} else if (CLK == RCC_CLK_GPIOB) {
		// 
		__HAL_RCC_GPIOB_CLK_ENABLE();

	} else if (CLK == RCC_CLK_GPIOC) {
		// 
		__HAL_RCC_GPIOC_CLK_ENABLE();

	}else if (CLK == RCC_CLK_GPIOD) {
		__HAL_RCC_GPIOD_CLK_ENABLE();

	}else if (CLK == RCC_CLK_GPIOE) {
		__HAL_RCC_GPIOE_CLK_ENABLE();

	}
	else if (CLK == RCC_CLK_GPIOF) {
		//__HAL_RCC_GPIOF_CLK_ENABLE();

	}
	//////////////////////////////////////////////////////

  GPIO_InitStructure.Pin = BITx;
	
	if (MODE == INPUT) {
		// 
		GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
		GPIO_InitStructure.Pull = GPIO_NOPULL;

	} else if (MODE == INPUT_PULLUP) {
		// 
		GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
		GPIO_InitStructure.Pull = GPIO_PULLUP;

	} else if (MODE == INPUT_PULLDOWN) {
		// 
		GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
		GPIO_InitStructure.Pull = GPIO_PULLDOWN;

	}else if (MODE == OUTPUT) {
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

	}else if (MODE == OUTPUT_OD) {
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

	} else if (MODE == AF_OD) {
		//
		GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

	}else if (MODE == AF_PP) {
		// 
		 GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
		 GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

	}else if (MODE == AN_INPUT) {
		// 
		GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStructure.Pull = GPIO_PULLDOWN;

	}else if (MODE == CHANGE) {
		// 
		 GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
		 GPIO_InitStructure.Pull = GPIO_NOPULL;

	}else if (MODE == FALLING) {
		// 
		GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStructure.Pull = GPIO_PULLUP;

	}else if (MODE == RISING) {
		// 
		GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
		 GPIO_InitStructure.Pull = GPIO_PULLDOWN;

	}
	HAL_GPIO_Init(GPIOx, &GPIO_InitStructure);  
	
        
} 

void digitalWrite( uint32_t ulPin, GPIO_PinState ulVal )
{
  /* Handle */
	HAL_GPIO_WritePin(g_APinDescription[ulPin].pPort, g_APinDescription[ulPin].ulPin, ulVal);
   
}

int digitalRead( uint32_t ulPin )
{
	
	return HAL_GPIO_ReadPin(g_APinDescription[ulPin].pPort, g_APinDescription[ulPin].ulPin);
}
///////////////////////////////////

uint32_t millis(void)
{
  // ToDo: ensure no interrupts
  return HAL_GetTick();
}

__STATIC_INLINE uint32_t LL_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}
// Interrupt-compatible version of micros
uint32_t micros(void)
{
  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  //LL_SYSTICK_IsActiveCounterFlag();
  uint32_t m = HAL_GetTick();
  const uint32_t tms = SysTick->LOAD + 1;
  __IO uint32_t u = tms - SysTick->VAL;
  if (LL_SYSTICK_IsActiveCounterFlag()) {
    m = HAL_GetTick();
    u = tms - SysTick->VAL;
  }
  return (m * 1000 + (u * 1000) / tms);
}

void delay(uint32_t ms)
{
  if (ms != 0) {
    uint32_t start = millis();
    do {
      yield();
    } while (millis() - start < ms);
  }
}
static void __empty() {
	// Empty
}
void yield(void) __attribute__ ((weak, alias("__empty")));
void delay_us(uint32_t nus)
{
	uint32_t end = micros() + nus;
    while (micros() < end);
}

//********************************************************************************  

