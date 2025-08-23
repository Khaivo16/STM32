#include "delay.h"
uint32_t ms;

void delay_init(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	SysTick_Config(SystemCoreClock / 1000);	// ngat 1ms
	//SysTick_Config (SystemCoreClock / 1000000); //1us per interrupt
}

void SysTick_Handler(void)
{	
	ms++;
}

uint32_t millis(void){
	//uint32_t millis =ms;
	return ms;
}

u32 micros(void)//thong qua ms
{

int Micros = millis()*1000 + (SystemCoreClock/1000-SysTick->VAL)/72;
return Micros;

}

void delayms(uint32_t time) {
		uint32_t end=ms+time;
		while (ms<end) {}
}

void delayus(uint32_t time) {
		uint32_t end=micros()+time;
		while (micros()<end) {}
}


























