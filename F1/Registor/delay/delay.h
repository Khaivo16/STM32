#ifndef __DELAY_H_
#define __DELAY_H_

#include "stm32f103xb.h"

void Systick_Init(void);
void DelayMillis(void);
void DelayMs(unsigned long t);

#endif


