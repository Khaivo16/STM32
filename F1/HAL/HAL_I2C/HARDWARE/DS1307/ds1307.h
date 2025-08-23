#ifndef __RTC_H
#define __RTC_H
/**
 *  Library for STM32F4xx

 */
/**
 * Includes
 */
#include "stm32f1xx_hal.h"
#include "myiic.h"


//I2C slave address for DS1307
#define DS1307_I2C_ADDR			0xD0

//Registers 
//#define DS1307_SECONDS			0x00
//#define DS1307_MINUTES			0x01
//#define DS1307_HOURS				0x02
//#define DS1307_DAY				0x03
//#define DS1307_DATE				0x04
//#define DS1307_MONTH				0x05
//#define DS1307_YEAR				0x06
//#define DS1307_CONTROL			0x07


/**
 * Convert BCD to BIN data
 *
 * Returns BIN value of given BCD
 */
extern uint8_t RTC_Bcd2Bin(uint8_t bcd);

/**
 * Convert BIN to BCD data
 * Returns BCD value of given BIN
 */
extern uint8_t RTC_Bin2Bcd(uint8_t bin);

/**

 * Returns value between min and max
 */
extern uint8_t RTC_CheckMinMax(uint8_t val, uint8_t min, uint8_t max);
////////////////////////
void DS1307_Init(void);
extern void RTC_GetDateTime(uint8_t *gio,uint8_t *phut,uint8_t *giay,uint8_t *thu,uint8_t *ngay,uint8_t *thang,uint8_t *nam) ;
extern void RTC_SetDateTime(uint8_t gio,uint8_t phut,uint8_t giay,uint8_t thu,uint8_t ngay,uint8_t thang,uint8_t nam);

#endif

