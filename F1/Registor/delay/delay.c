#include "delay.h"


void Systick_Init(void)
{
	SysTick->CTRL = 0;
	SysTick->LOAD = 0x00FFFFFF;
	SysTick->VAL = 0;
	SysTick->CTRL |= 5;
}
void DelayMillis(void)
{
	SysTick->LOAD = 0x11940; // clock 72Mhz
	SysTick->VAL = 0;
	while((SysTick->CTRL & (1<<16)) == 0);
}
void DelayMs(unsigned long t)
{
	for(; t > 0; t--)
	{
		DelayMillis();
	}
}


