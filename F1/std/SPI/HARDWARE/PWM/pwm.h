#ifndef __pwm_H
#define __pwm_H 			   
#include "sys.h" 

typedef enum
{ 
	PP1,
	PP2,
	PP3
}pin_pwm;

typedef enum
{ 
	CH1,
	CH2,
	CH3,
	CH4
}ch_pwm;


void PWMx_init(TIM_TypeDef * TIMx,ch_pwm Chanelx,pin_pwm Pin,uint16_t ARR,uint16_t PSC);
#endif