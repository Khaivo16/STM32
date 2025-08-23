#include "delay.h"
#include "sys.h"
volatile uint32_t counter_ms = 0;
volatile uint32_t counter_us = 0;

__IO uint32_t TimingMillis;
static void __empty() {
	// Empty
}
void yield(void) __attribute__ ((weak, alias("__empty")));

//void SysTick_Handler(void) {
//	//neu ngat 1ms su dung 2 ham millis() và micros()
//  //counter_ms++;//1ms
//	TimingMillis++;
//	//neu ngat 1us
//	//counter_us++; counter_ms %= counter_us;//1us
//}

//uint32_t millis(void) {
//  return counter_ms;
//}

//uint32_t micros(void)//thong qua ms
//{

//int Micros = millis()*1000 + (SystemCoreClock/1000-SysTick->VAL)/168;
//return Micros;

//}

void delay_init(void){
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	SysTick_Config(SystemCoreClock / 1000);	// ngat 1ms
	//SysTick_Config (SystemCoreClock / 1000000); //1us per interrupt
}


//////////////////////////////////////////////////
uint32_t millis( void )
{
// todo: ensure no interrupts
	//return GetTickCount() ;
  //return TimingMillis;
	return HAL_GetTick();
}

__STATIC_INLINE uint32_t LL_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}
// Interrupt-compatible version of micros
// Theory: repeatedly take readings of SysTick counter, millis counter and SysTick interrupt pending flag.
// When it appears that millis counter and pending is stable and SysTick hasn't rolled over, use these 
// values to calculate micros. If there is a pending SysTick, add one to the millis counter in the calculation.
uint32_t micros( void )
{
   /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  LL_SYSTICK_IsActiveCounterFlag();
  uint32_t m = HAL_GetTick();
  const uint32_t tms = SysTick->LOAD + 1;
  __IO uint32_t u = tms - SysTick->VAL;
  if (LL_SYSTICK_IsActiveCounterFlag()) {
    m = HAL_GetTick();
    u = tms - SysTick->VAL;
  }
  return (m * 1000 + (u * 1000) / tms);
}

// original function:
// uint32_t micros( void )
// {
//     uint32_t ticks ;
//     uint32_t count ;
// 
//     SysTick->CTRL;
//     do {
//         ticks = SysTick->VAL;
//         count = GetTickCount();
//     } while (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);
// 
//     return count * 1000 + (SysTick->LOAD + 1 - ticks) / (SystemCoreClock/1000000) ;
// }

void delay_ms (uint32_t nms)
{
//uint32_t Time = millis();
//while ((nms - (millis()-Time))> 0);
	delay( nms );
}

void delay_us(uint32_t nus)
{
//uint32_t Time = micros();
//while((nus-(micros()-Time)) > 0);
	const uint32_t count_1us = HAL_RCC_GetSysClockFreq() / 4000000;
		for (int i = 0; i < nus; i++) {
			for (uint32_t count = 0; ++count <= count_1us;) {
			}
		}
}
void delay( uint32_t ms )
{
//    uint32_t end = millis() + ms;
//    while (millis() < end)
//    	yield();
	const uint32_t count_1ms = HAL_RCC_GetSysClockFreq() / 4000;
		for (int i = 0; i < ms; i++) {
			for (uint32_t count = 0; ++count <= count_1ms;) {
			}
		}
}			 

