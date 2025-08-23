#include "timer.h"

void Timerbase_init(TIM_TypeDef * TIMx,uint16_t ARR,uint16_t PSC)
{
		NVIC_InitTypeDef NVIC_InitStructure;
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		uint8_t NVIC_IRQChannel;  
	
		if (TIMx==TIM2) {RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);NVIC_IRQChannel= TIM2_IRQn; }
		else if (TIMx==TIM3) {RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);NVIC_IRQChannel= TIM3_IRQn; }
		else if (TIMx==TIM4) {RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);NVIC_IRQChannel= TIM4_IRQn; }

		
		/* Enable the TIM2 global Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = NVIC_IRQChannel;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

		NVIC_Init(&NVIC_InitStructure);
		

		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = ARR-1;
		TIM_TimeBaseStructure.TIM_Prescaler = PSC-1;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

		TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

		

		
		/* TIM IT enable */
		TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
		/* TIM2 enable counter */
		TIM_Cmd(TIMx, ENABLE);
}