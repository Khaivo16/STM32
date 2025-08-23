#ifndef __RTC_H
#define __RTC_H
/**
 *  Library for STM32F4xx

 */
/**
 * Includes
 */
#include "stm32f10x.h"
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
extern u8 RTC_Bcd2Bin(u8 bcd);

/**
 * Convert BIN to BCD data
 * Returns BCD value of given BIN
 */
extern u8 RTC_Bin2Bcd(u8 bin);

/**

 * Returns value between min and max
 */
extern u8 RTC_CheckMinMax(u8 val, u8 min, u8 max);
////////////////////////
void DS1307_Init(void);
extern void RTC_GetDateTime(u8 *gio,u8 *phut,u8 *giay,u8 *thu,u8 *ngay,u8 *thang,u8 *nam) ;
extern void RTC_SetDateTime(u8 gio,u8 phut,u8 giay,u8 thu,u8 ngay,u8 thang,u8 nam);

#endif

