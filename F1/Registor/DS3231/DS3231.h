#ifndef __DS3231_H
#define __DS3231_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
uint8_t BCDtoBIN(uint8_t bcd);

uint8_t BINtoBCD(uint8_t bin);

uint8_t Read_SS(void);
	 
////////////////////
void DS3231_Init(void);
void DS3231Read(uint8_t *, uint8_t *, uint8_t *,uint8_t *,uint8_t *,uint8_t *,uint8_t *);
void DS3231Set(uint8_t, uint8_t, uint8_t,uint8_t,uint8_t,uint8_t,uint8_t);	 
	 
	 
	 
#ifdef __cplusplus
 }
#endif
	 
#endif
 
 


 