#ifndef __timer_H
#define __timer_H 			   
#include "sys.h" 
#include "stm32f1xx.h"

void Timerbase_init(TIM_HandleTypeDef *htim,TIM_TypeDef * TIMx,uint16_t ARR,uint16_t PSC);
#endif