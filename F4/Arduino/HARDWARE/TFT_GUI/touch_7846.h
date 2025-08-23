
#ifndef __TOUCH_7846_H
#define __TOUCH_7846_H

#ifdef __cplusplus
 extern "C" {
#endif
 
#include "sys.h"

#include "widget.h"  
#include "grlib.h"
#include <stdlib.h>
#include <math.h>
	 
typedef struct 
{
	uint16_t X0;
	uint16_t Y0;
	uint16_t X; 
	uint16_t Y;						   	    
	uint8_t TouchDetected;

	float xfac;
	float yfac;
	short xoff;
	short yoff;
}Pen_Holder;
extern Pen_Holder Pen_Point;

//#define TOUCH_CS_PORT	 GPIOA
//#define TOUCH_CS_PIN	 GPIO_Pin_15
#define T_CS()   digitalWrite(PA15, LOW)
#define T_DCS()  digitalWrite(PA15, HIGH)

#define CMD_RDY 0X90  //0B10010000
#define CMD_RDX	0XD0  //0B11010000  											 
 
//#define PEN  GPIOD->IDR&(1<<6) //
//#define NPEN !(0x0080&PEN)      //!PEN

unsigned char SPI_WriteByte(uint8_t num);
void SpiDelay(unsigned int DelayCnt);
uint16_t TPReadX(void);
uint16_t TPReadY(void);	   
uint8_t read_once(void);	
uint8_t Read_Ads7846(void); 
uint8_t Convert_Pos(void);


void NVIC_TOUCHConfiguration(void);
void TOUCH_Init(void);				  
void LCD_ShowNum(uint8_t x,uint16_t y,uint16_t data);

uint8_t TOADO_CHAM(uint16_t TOADO_X1,uint16_t TOADO_Y1,uint16_t TOADO_X2,uint16_t TOADO_Y2);
uint8_t SOSANH_TOADO(uint16_t TOADO_X,uint16_t TOADO_Y);
uint8_t touch_scan(void);


Pen_Holder* IOE_TS_GetState(void);
void Touch_Adjust(void);
uint16_t XPT2046_Press(void);
#ifdef __cplusplus
}
#endif
void TouchScreenCallbackSet(long (*pfnCallback)(unsigned long ulMessage, long lX,long lY));
#endif 

///////////////////////////////////////////////////////////////////////////////////////////////////















