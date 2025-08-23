
#include "pwm.h"


/*
 * PWM pins 
 *
 * 	TIMER	|CHANNEL 1				|CHANNEL 2				|CHANNEL 3				|CHANNEL 4
 * 				|PP1	PP2		PP3		|PP1	PP2		PP3		|PP1	PP2		PP3		|PP1	PP2		PP3
 *
 * 	TIM 2	|PA0	PA5		PA15	|PA1	PB3		-			|PA2	PB10	-			|PA3	PB11	-
 * 	TIM 3	|PA6	PB4		PC6		|PA7	PB5		PC7		|PB0	PC8		-			|PB1	PC9		-
 * 	TIM 4	|PB6	PD12	-			|PB7	PD13	-			|PB8	PD14	-			|PB9	PD15	-
 * 	TIM 5	|PA0	PH10	-			|PA1	PH11	-			|PA2	PH12	-			|PA3	PI0		-
 * 	TIM 8	|PC6	PI5		-			|PC7	PI6		-			|PC8	PI7		-			|PC9	PI2		-
 * 	TIM 9	|PA2	PE5		-			|PA3	PE6		-			|-		-		-				|-		-		-
 * 	TIM 10|PB8	PF6		-			|-		-		-				|-		-		-				|-		-		-
 * 	TIM 11|PB9	PF7		-			|-		-		-				|-		-		-				|-		-		-
 * 	TIM 12|PB14	PH6		-			|PB15	PH9		-			|-		-		-				|-		-		-
 * 	TIM 13|PA6	PF8		-			|-		-		-				|-		-		-				|-		-		-
 * 	TIM 14|PA7	PF9		-			|-		-		-				|-		-		-				|-		-		-
 *
*/

void TIM2_pin(ch_pwm Chanelx,pin_pwm Pin){
	if (Chanelx==CH1){
		if (Pin==PP1) GPIO_SETUP(GPIOA,GPIO_Pin_0,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
		else if (Pin==PP2) GPIO_SETUP(GPIOA,GPIO_Pin_5,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
		else if (Pin==PP3) GPIO_SETUP(GPIOA,GPIO_Pin_15,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
		}
	else if (Chanelx==CH2){
		if (Pin==PP1) GPIO_SETUP(GPIOA,GPIO_Pin_1,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
		else if (Pin==PP2) GPIO_SETUP(GPIOB,GPIO_Pin_3,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
		}
	else if (Chanelx==CH3){
		if (Pin==PP1) GPIO_SETUP(GPIOA,GPIO_Pin_2,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
		else if (Pin==PP2) GPIO_SETUP(GPIOA,GPIO_Pin_10,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
	}
	else if (Chanelx==CH4){
		if(Pin==PP1) GPIO_SETUP(GPIOA,GPIO_Pin_3,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
		else if(Pin==PP2) GPIO_SETUP(GPIOB,GPIO_Pin_11,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
	}
}	
	
void TIM3_pin(ch_pwm Chanelx,pin_pwm Pin){
	if (Chanelx==CH1){
		if (Pin==PP1) GPIO_SETUP(GPIOA,GPIO_Pin_6,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
		else if (Pin==PP2) GPIO_SETUP(GPIOB,GPIO_Pin_4,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
		else if (Pin==PP3) GPIO_SETUP(GPIOC,GPIO_Pin_6,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
		}
	else if (Chanelx==CH2){
		if (Pin==PP1) GPIO_SETUP(GPIOA,GPIO_Pin_7,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
		else if (Pin==PP2) GPIO_SETUP(GPIOB,GPIO_Pin_5,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
		else if (Pin==PP3) GPIO_SETUP(GPIOC,GPIO_Pin_7,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
		}
	else if (Chanelx==CH3){
		if (Pin==PP1) GPIO_SETUP(GPIOB,GPIO_Pin_0,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
		else if (Pin==PP2) GPIO_SETUP(GPIOC,GPIO_Pin_8,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
	}
	else if (Chanelx==CH4){
			if (Pin==PP1) GPIO_SETUP(GPIOB,GPIO_Pin_1,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
			else if (Pin==PP2) GPIO_SETUP(GPIOC,GPIO_Pin_9,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
	}	
}
		
void TIM4_pin(ch_pwm Chanelx,pin_pwm Pin){
	if (Chanelx==CH1){GPIO_SETUP(GPIOB,GPIO_Pin_6,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);}
	else if (Chanelx==CH2){GPIO_SETUP(GPIOB,GPIO_Pin_7,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);	}
	else if (Chanelx==CH3){	GPIO_SETUP(GPIOB,GPIO_Pin_8,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);}
	else if (Chanelx==CH4){	GPIO_SETUP(GPIOB,GPIO_Pin_9,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);}	
}
		
		


void PWMx_init(TIM_TypeDef * TIMx,ch_pwm Chanelx,pin_pwm Pin,uint16_t ARR,uint16_t PSC){

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	if (TIMx==TIM2) {RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); TIM2_pin(Chanelx,Pin);}
	else if (TIMx==TIM3) {RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); TIM3_pin(Chanelx,Pin);}
	else if (TIMx==TIM4) {RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); TIM4_pin(Chanelx,Pin);}
	
	
	

	/* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = ARR-1;
  TIM_TimeBaseStructure.TIM_Prescaler = PSC-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	
	if (Chanelx==CH1){TIM_OC1Init(TIMx, &TIM_OCInitStructure); TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);}
	else if (Chanelx==CH2){TIM_OC2Init(TIMx, &TIM_OCInitStructure); TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);}
	else if (Chanelx==CH3){TIM_OC3Init(TIMx, &TIM_OCInitStructure); TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);}
	else if (Chanelx==CH4){TIM_OC4Init(TIMx, &TIM_OCInitStructure); TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);}


  TIM_ARRPreloadConfig(TIMx, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIMx, ENABLE);
}


