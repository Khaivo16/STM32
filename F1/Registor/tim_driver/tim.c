
#include "tim.h"

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

/*
	PA0 -> TIM2_CH1
	PA1 -> TIM2_CH2
	PA2 -> TIM2_CH3
	PA3 -> TIM2_CH4

	PA6 -> TIM3_CH1
	PA7 -> TIM3_CH2
	PB0 -> TIM3_CH3
	PB1 -> TIM3_CH4

	PA8 -> TIM1_CH1
	PA9 -> TIM1_CH2
	PA10 -> TIM1_CH3
	PA11 -> TIM1_CH4

*/

void TIM2_PinInit( PWM_Channel Channel, PWM_Pin Pin){
	if(Channel==PWM_Channel1){
		if(Pin==PWM_Pin1) GPIOx_Init(GPIOA, 0,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		if(Pin==PWM_Pin2) {AFIO->MAPR &=~(3<<8); AFIO->MAPR |=2<<8;GPIOx_Init(GPIOA, 5,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);}
		if(Pin==PWM_Pin3) {AFIO->MAPR &=~(3<<8); AFIO->MAPR |=3<<8;GPIOx_Init(GPIOA, 15,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);}
	}
	if(Channel==PWM_Channel2){
		if(Pin==PWM_Pin1) GPIOx_Init(GPIOA, 1,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		if(Pin==PWM_Pin2) {AFIO->MAPR &=~(3<<8); AFIO->MAPR |=3<<8;GPIOx_Init(GPIOB, 3,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);}
	}
	if(Channel==PWM_Channel3){
		if(Pin==PWM_Pin1) GPIOx_Init(GPIOA, 2,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		if(Pin==PWM_Pin2) {AFIO->MAPR &=~(3<<8); AFIO->MAPR |=3<<8;GPIOx_Init(GPIOB, 10,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);}
	}
	if(Channel==PWM_Channel4){
		if(Pin==PWM_Pin1) GPIOx_Init(GPIOA, 3,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		if(Pin==PWM_Pin2) {AFIO->MAPR &=~(3<<8); AFIO->MAPR |=3<<8;GPIOx_Init(GPIOB, 11,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);}
	}

}

void TIM3_PinInit( PWM_Channel Channel, PWM_Pin Pin){
	if(Channel==PWM_Channel1){
		if(Pin==PWM_Pin1) GPIOx_Init(GPIOA, 6,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		if(Pin==PWM_Pin2) {AFIO->MAPR &=~(3<<10); AFIO->MAPR |=2<<10;GPIOx_Init(GPIOB, 4,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);}
	  if(Pin==PWM_Pin3) {AFIO->MAPR &=~(3<<11); AFIO->MAPR |=3<<10;GPIOx_Init(GPIOC, 6,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);}
	}
	if(Channel==PWM_Channel2){
		if(Pin==PWM_Pin1) GPIOx_Init(GPIOA, 7,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		if(Pin==PWM_Pin2) {AFIO->MAPR &=~(3<<10); AFIO->MAPR |=2<<10;GPIOx_Init(GPIOB, 5,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);}
	  if(Pin==PWM_Pin3) {AFIO->MAPR &=~(3<<10); AFIO->MAPR |=3<<10;GPIOx_Init(GPIOC, 7,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);}
	}
	if(Channel==PWM_Channel3){
		if(Pin==PWM_Pin1) GPIOx_Init(GPIOB, 0,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		if(Pin==PWM_Pin2) {AFIO->MAPR &=~(3<<10); AFIO->MAPR |=3<<10;GPIOx_Init(GPIOC, 8,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);}
	}
	if(Channel==PWM_Channel4){
		if(Pin==PWM_Pin1) GPIOx_Init(GPIOB, 1,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		if(Pin==PWM_Pin2) {AFIO->MAPR &=~(3<<10); AFIO->MAPR |=3<<10;GPIOx_Init(GPIOC, 9,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);}
	}

}
void TIM4_PinInit( PWM_Channel Channel, PWM_Pin Pin){
	if(Channel==PWM_Channel1){
		if(Pin==PWM_Pin1) GPIOx_Init(GPIOB, 6,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		if(Pin==PWM_Pin2) { AFIO->MAPR |=1<<12;GPIOx_Init(GPIOD, 12,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);}
	}
	if(Channel==PWM_Channel2){
		if(Pin==PWM_Pin1) GPIOx_Init(GPIOB, 7,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		if(Pin==PWM_Pin2) {AFIO->MAPR |=1<<12;GPIOx_Init(GPIOD, 13,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);}
	}
	if(Channel==PWM_Channel3){
		if(Pin==PWM_Pin1) GPIOx_Init(GPIOB, 8,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
if(Pin==PWM_Pin2) {AFIO->MAPR |=1<<12;GPIOx_Init(GPIOD, 14,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);}
	}
	if(Channel==PWM_Channel4){
		if(Pin==PWM_Pin1) GPIOx_Init(GPIOB, 9,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);
		if(Pin==PWM_Pin2) {AFIO->MAPR |=1<<12;GPIOx_Init(GPIOD, 15,OUTPUT_AF_PP,NOPULL,MODE_OUTPUT_50MHZ);}
	}

}
void PWMx_Init(TIM_TypeDef *TIMx, PWM_Channel Channel, PWM_Pin Pin, uint16_t ARR, uint16_t PSC)
{

	RCC->APB2ENR |= 1<<0; //AFIO

	if(TIMx== TIM1)RCC->APB2ENR |= 1<<11; // TIM1
	if(TIMx== TIM2){RCC->APB1ENR |= 1<<0; TIM2_PinInit( Channel, Pin);}// TIM2
	if(TIMx== TIM3){RCC->APB1ENR |= 1<<1; TIM3_PinInit( Channel, Pin);}// TIM3
	if(TIMx== TIM4){RCC->APB1ENR |= 1<<2; TIM4_PinInit( Channel, Pin);}// TIM4

	TIMx->PSC=PSC-1;
	TIMx->ARR=ARR-1;
	
	TIMx->CR1 |= 0<<4;//Counter used as upcounter
	TIMx->CR1 |= 1<<7;//Auto-reload preload enable
	
	if(Channel==PWM_Channel1)
	{
		TIMx->CCMR1 &= ~(3<<0);TIMx->CCMR1 |= 0<<0;//CC1 channel is configured as output.
		TIMx->CCMR1 |= 0<<2;//TIM_OCFAST_DISABLE
		TIMx->CCMR1 |= 1<<3;//Output compare 1 preload enable
		TIMx->CCMR1 &= ~(7<<4);TIMx->CCMR1 |= 6<<4;//: PWM mode 1
		TIMx->CCER |=(0<<1)|(1<<0);//OC1 active high.OC1 signal is output
	}
	if(Channel==PWM_Channel2)
	{
		TIMx->CCMR1 &= ~(3<<8);TIMx->CCMR1 |= 0<<8;//CC1 channel is configured as output.
		TIMx->CCMR1 |= 0<<10;//TIM_OCFAST_DISABLE
		TIMx->CCMR1 |= 1<<11;//Output compare 1 preload enable
		TIMx->CCMR1 &= ~(7<<12);TIMx->CCMR1 |= 6<<12;//: PWM mode 1
		TIMx->CCER |=(0<<5)|(1<<4);//OC1 active high.OC1 signal is output
	}
	
	if(Channel==PWM_Channel3)
	{
		TIMx->CCMR2 &= ~(3<<0);TIMx->CCMR1 |= 0<<0;//CC1 channel is configured as output.
		TIMx->CCMR2 |= 0<<2;//TIM_OCFAST_DISABLE
		TIMx->CCMR2 |= 1<<3;//Output compare 1 preload enable
		TIMx->CCMR2 &= ~(7<<4);TIMx->CCMR1 |= 6<<4;//: PWM mode 1
		TIMx->CCER |=(0<<9)|(1<<8);//OC1 active high.OC1 signal is output
	}
	if(Channel==PWM_Channel4)
	{
		TIMx->CCMR2 &= ~(3<<8);TIMx->CCMR1 |= 0<<8;//CC1 channel is configured as output.
		TIMx->CCMR2 |= 0<<10;//TIM_OCFAST_DISABLE
		TIMx->CCMR2 |= 1<<11;//Output compare 1 preload enable
		TIMx->CCMR2 &= ~(7<<12);TIMx->CCMR1 |= 6<<12;//: PWM mode 1
		TIMx->CCER |=(0<<13)|(1<<12);//OC1 active high.OC1 signal is output
	}
	
	TIMx->CR1 |= 1<<0;//Counter/TIMx enabled
}

// TIMER


void TIMERx_Init(TIM_TypeDef *TIMx, uint16_t ARR, uint16_t PSC)
{
	uint8_t t = 0;
	IRQn_Type IRQn;
	
	RCC->APB2ENR |= 1<<0; //AFIO

	if(TIMx== TIM1)RCC->APB2ENR |= 1<<11; // TIM1
	if(TIMx== TIM2){RCC->APB1ENR |= 1<<0; t = 2; IRQn = TIM2_IRQn;}// TIM2
	if(TIMx== TIM3){RCC->APB1ENR |= 1<<1; t = 3; IRQn = TIM3_IRQn;}// TIM3
	if(TIMx== TIM4){RCC->APB1ENR |= 1<<2; t = 4; IRQn = TIM4_IRQn;}// TIM4

	TIMx->PSC=PSC-1;
	TIMx->ARR=ARR-1;
	
	TIMx->DIER |= 1<<0; // IT
	
	NVICx_Init(IRQn, 0, t);

	TIMx->CR1 |= 1<<0; // Counter - timer enable
}

void TIM2_IRQHandler(void)
{
	if(TIM2->SR & (1<<0))
	{
		// xy ly ngat
		GPIOC->ODR ^= 1<<13;
	}
	TIM2->SR &= ~(1<<0);
}

void TIM3_IRQHandler(void)
{
	if(TIM3->SR & (1<<0))
	{
		// xy ly ngat
		GPIOA->ODR ^= 1<<0;
	}
	TIM3->SR &= ~(1<<0);
}

void TIM4_IRQHandler(void)
{
	if(TIM4->SR & (1<<0))
	{
		// xy ly ngat
		GPIOA->ODR ^= 1<<1;
	}
	TIM4->SR &= ~(1<<0);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}











