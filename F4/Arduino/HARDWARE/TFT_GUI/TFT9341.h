
/*
 * gs_ili9341.h
 *
 *  Created on: 23 Jun 2016
 *      Author: STM32VN
 */

#ifndef GS_ILI9341_H_
#define GS_ILI9341_H_
#include "sys.h"
//#include "delay.h"
#include "TOUCH_7846.h"
#include "ili9341.h"

#include "grlib.h"
#include "widget.h"
#include "String.h"
#include "stdlib.h"



#include "nut_play.h"
#include "nut_play_s.h"
#include "nut_next.h"
#include "nut_next_s.h"
#include "nut_prw.h"
#include "nut_prw_s.h"
#include "nut_pause.h"
#include "anh_nen_about.h"



typedef struct  
{										    
	uint16_t width;			
	uint16_t height;			
	uint16_t id;				
	uint8_t  dir;			
	uint16_t	 wramcmd;		
	uint16_t  rramcmd;   
	uint16_t  setxcmd;		
	uint16_t  setycmd;		
}_lcd_dev; 	


extern _lcd_dev lcddev;	
////////////////////////////////////////////////////////////////////////	 

#define USE_HORIZONTAL  	  0 //0-0 1-90 2-180 3-270
#define LCD_USE8BIT_MODEL   0	

//////////////////////////////////////////////////////////////////////////////////	  

#define LCD_W 240
#define LCD_H 320

		   
extern uint16_t  POINT_COLOR;   
extern uint16_t  BACK_COLOR; 




#define WHITE       0xFFFF
#define BLACK      	0x0000	  
#define BLUE       	0x001F  
#define BRED        0XF81F
#define GRED 			 	0XFFE0
#define GBLUE			 	0X07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
//#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN 			0XBC40 
#define BRRED 			0XFC07 
#define GRAY  			0X8430 


#define DARKBLUE      	 0X01CF	
#define LIGHTBLUE      	 0X7D7C	  
#define GRAYBLUE       	 0X5458 

 
#define LIGHTGREEN     	0X841F 
#define LIGHTGRAY     0XEF5B 
#define LGRAY 			 		0XC618 

#define LGRAYBLUE      	0XA651 
#define LBBLUE          0X2B12 
	    															  
void GUI_Init(void);
uint16_t LCD_read(void);
void LCD_Clr(uint16_t Color);	 
void LCD_Clear(unsigned int dat);
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);
void LCD_DrawPoint(uint16_t x,uint16_t y);
uint16_t  LCD_ReadPoint(uint16_t x,uint16_t y);    
void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd);
uint16_t LCD_RD_DATA(void);						    
void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue);
void LCD_WR_REG(uint16_t data);
void LCD_WR_DATA(uint16_t data);
void LCD_ReadReg(uint16_t LCD_Reg,uint8_t *Rval,int n);
void LCD_WriteRAM_Prepare(void);
void LCD_ReadRAM_Prepare(void);   
void Lcd_WriteData_16Bit(uint16_t Data);
uint16_t Lcd_ReadData_16Bit(void);
void LCD_direction(uint8_t direction );
uint16_t Color_To_565(uint8_t r, uint8_t g, uint8_t b);
uint16_t LCD_Read_ID(void);
/////////////////////////////////////////////////////////////////
#define RGB565CONVERT(red, green, blue) (int) (((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3))
#define DPYCOLORTRANSLATE(c)    ((((c) & 0x00f80000) >> 8) | (((c) & 0x0000fc00) >> 5) |(((c) & 0x000000f8) >> 3))

 void ili9320DrawMultiple(void *pvDisplayData, long lX,
                                           long lY, long lX0, long lCount,
                                           long lBPP,
                                           const unsigned char *pucData,
                                           const unsigned char *pucPalette);
void ili9320PixelDraw(void *pvDisplayData, long x, long y,unsigned long point);
static unsigned long ili9320ColorTranslate(void *pvDisplayData,unsigned long c)
{return(DPYCOLORTRANSLATE(c));}
void ili9320Flush(void *pvDisplayData);																					 
void ili9320RectFill(void *pvDisplayData, const tRectangle *pRect,unsigned long ulValue);
void ili9320LineDrawH(void *pvDisplayData, long lX1, long lX2,
                                   long lY, unsigned long ulValue);
void ili9320LineDrawV(void *pvDisplayData, long lX, long lY1,
                                   long lY2, unsigned long ulValue);
void GrContextInit(tContext *pContext, const tDisplay *pDisplay);
void GrContextClipRegionSet(tContext *pContext, tRectangle *pRect);
void GrLineDrawH(const tContext *pContext, long lX1, long lX2, long lY);
void GrLineDrawV(const tContext *pContext, long lX, long lY1, long lY2);
static long
GrClipCodeGet(const tContext *pContext, long lX, long lY);
static long GrLineClip(const tContext *pContext, long *plX1, long *plY1, long *plX2,
           long *plY2);
void
GrLineDraw(const tContext *pContext, long lX1, long lY1, long lX2, long lY2);
void LineBMP(unsigned char hang, unsigned int x0,unsigned int d0, unsigned char *mang );
void GrTriangleDrawFill(const tContext *pContext,uint16_t x1,uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3);
void GrBar(const tContext *pContext,unsigned short x1, unsigned short y1,unsigned short x2, unsigned short y2, unsigned char width);


#include "canvas.h"
#include "container.h"
#include "checkbox.h"
#include "imgbutton.h"
#include "listbox.h"
#include "pushbutton.h"
#include "radiobutton.h"
#include "slider.h"
//#include "touch.h"
#include "bmp.h"

#endif /* GS_ILI9341_H_ */
