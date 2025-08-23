
#ifndef __TOUCH_7846_H
#define __TOUCH_7846_H

#ifdef __cplusplus
 extern "C" {
#endif
 
#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"	

#include "string.h" 
#include "stdlib.h"
#include "math.h"
#include "widget.h"	 
typedef struct 
{
	u16 X0;
	u16 Y0;
	u16 X; 
	u16 Y;						   	    
	u8 TouchDetected;

	float xfac;
	float yfac;
	short xoff;
	short yoff;
}Pen_Holder;

extern Pen_Holder Pen_Point;
#define TOUCH_CS_PORT	 GPIOA
#define TOUCH_CS_PIN	 GPIO_Pin_15

#define T_CS()   GPIO_ResetBits(TOUCH_CS_PORT, TOUCH_CS_PIN);
#define T_DCS()  GPIO_SetBits(TOUCH_CS_PORT, TOUCH_CS_PIN);

#define CMD_RDY 0X90  //0B10010000
#define CMD_RDX	0XD0  //0B11010000  											 
 
//#define PEN  GPIOD->IDR&(1<<6) //
//#define NPEN !(0x0080&PEN)      //!PEN


unsigned char SPI_WriteByte(u8 num);
void SpiDelay(unsigned int DelayCnt);
u16 TPReadX(void);
u16 TPReadY(void);	   
u8 read_once(void);	
u8 Read_Ads7846(void); 
u8 Convert_Pos(void);

void EXTI9_5_IRQHandler(void);
void NVIC_TOUCHConfiguration(void);
void Touch_init(void);				  
void LCD_ShowNum(uint8_t x,uint16_t y,uint16_t data);

u8 TOADO_CHAM(u16 TOADO_X1,u16 TOADO_Y1,u16 TOADO_X2,u16 TOADO_Y2);
u8 SOSANH_TOADO(u16 TOADO_X,u16 TOADO_Y);
u8 touch_scan();
void TouchScreenCallbackSet(long (*pfnCallback)(unsigned long ulMessage, long lX,long lY));

Pen_Holder* IOE_TS_GetState(void);
void Touch_Adjust(void);
uint16_t XPT2046_Press(void);
#ifdef __cplusplus
}
#endif

#endif 

///////////////////////////////////////////////////////////////////////////////////////////////////















