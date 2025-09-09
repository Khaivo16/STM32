#ifndef __TIM_H
#define __TIM_H

#include "gpio.h"

typedef enum
{
  PWM_Pin1,
	PWM_Pin2,
	PWM_Pin3
} PWM_Pin;

typedef enum
{
  PWM_Channel1,
	PWM_Channel2,
	PWM_Channel3,
	PWM_Channel4
} PWM_Channel;

void TIM2_PinInit( PWM_Channel Channel, PWM_Pin Pin);
void TIM3_PinInit( PWM_Channel Channel, PWM_Pin Pin);
void TIM4_PinInit( PWM_Channel Channel, PWM_Pin Pin);

void PWMx_Init(TIM_TypeDef *TIMx, PWM_Channel Channel, PWM_Pin Pin, uint16_t ARR, uint16_t PSC);

void TIMERx_Init(TIM_TypeDef *TIMx, uint16_t ARR, uint16_t PSC);

void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);

long map(long x, long in_min, long in_max, long out_min, long out_max);


#endif













