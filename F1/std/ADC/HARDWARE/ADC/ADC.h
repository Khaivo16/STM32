#ifndef __ADC_H
#define __ADC_H 			   
#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 



////////////////////////////////////////////////////////////////////////////////// 
	 
void ADCx_init(ADC_TypeDef* ADCx,uint8_t channel);
uint16_t ADC_read(ADC_TypeDef* ADCx,uint8_t channel);
uint16_t ADC_avr(ADC_TypeDef* ADCx,uint8_t channel,uint8_t lan);

#endif
