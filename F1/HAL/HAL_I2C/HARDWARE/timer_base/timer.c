#include "timer.h"

void Timerbase_init(TIM_HandleTypeDef *htim,TIM_TypeDef * TIMx,uint16_t ARR,uint16_t PSC)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
	if (TIMx==TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
	else if (TIMx==TIM3)  __HAL_RCC_TIM3_CLK_ENABLE();
	else if (TIMx==TIM3)  __HAL_RCC_TIM4_CLK_ENABLE();
	/* Set TIMx instance */
  htim->Instance = TIMx;

  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  htim->Init.Period            = ARR - 1;
  htim->Init.Prescaler         = PSC-1;
  htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim->Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim->Init.RepetitionCounter = 0;
	
	HAL_TIM_Base_Init(htim);
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(htim, &sClockSourceConfig) ;
	HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig);
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  // Enable the TIM2 global Interrupt
	if (TIMx==TIM2) { 
			HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
			HAL_NVIC_EnableIRQ(TIM2_IRQn);
	}
	else if (TIMx==TIM3) {
			HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
			HAL_NVIC_EnableIRQ(TIM3_IRQn);
	}
	else if (TIMx==TIM4) {
			HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
			HAL_NVIC_EnableIRQ(TIM4_IRQn);
	}

}