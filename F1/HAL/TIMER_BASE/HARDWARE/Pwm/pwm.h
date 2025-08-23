#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
							  
////////////////////////////////////////////////////////////////////////////////// 	
typedef enum {
	PWM_Channel_1,
	PWM_Channel_2,
	PWM_Channel_3,
	PWM_Channel_4
} PWM_Channel_t;

/**
 * Pin selected for corresponding channel on specific channel
 *
 */
typedef enum {
	PWM_PinsPack_1,
	PWM_PinsPack_2,
	PWM_PinsPack_3
}PWM_PinsPack_t;

void TIMER_PROPERTIES_EnableClock(TIM_TypeDef* TIMx);
void PWM_INT_InitTIM1Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
void PWM_INT_InitTIM2Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
void PWM_INT_InitTIM3Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
void PWM_INT_InitTIM4Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
void PWM_INT_InitTIM5Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
void PWM_INT_InitTIM8Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
void PWM_INT_InitTIM9Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
void PWM_INT_InitTIM10Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
void PWM_INT_InitTIM11Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
void PWM_INT_InitTIM12Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
void PWM_INT_InitTIM13Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
void PWM_INT_InitTIM14Pins(PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);

void BasicPWM1(void);
void BasicPWM2(void);
void  TIM_OC(TIM_HandleTypeDef  TIMx,PWM_Channel_t Channel);
void PWM_InitChannel(TIM_TypeDef* TIMx, PWM_Channel_t Channel, PWM_PinsPack_t PinsPack);
void PWM_Init(TIM_TypeDef* TIMx,PWM_Channel_t Channel, PWM_PinsPack_t PinsPack,uint32_t arr,uint32_t psc);
void PWM_Set(TIM_TypeDef* TIMx,PWM_Channel_t Channel,uint32_t pwm);
#endif
