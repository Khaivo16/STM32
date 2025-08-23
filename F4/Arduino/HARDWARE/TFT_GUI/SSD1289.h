#ifndef __SSD1289_H
#define __SSD1289_H
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "stm32f4xx.h"
#include "AsciiLib.h" 

#include "grlib.h"
#include "widget.h"
#include "String.h"
#include "stdlib.h"
/* Private define ------------------------------------------------------------*/
#define USE_16BIT_PMP
 
 #define SSD1289_BACKLIGHT_PORT		GPIOB
 #define SSD1289_BACKLIGHT_PIN		GPIO_Pin_0
 #define SSD1289_Backlight(x)	x ? GPIO_SetBits(SSD1289_BACKLIGHT_PORT,SSD1289_BACKLIGHT_PIN): GPIO_ResetBits(SSD1289_BACKLIGHT_PORT,SSD1289_BACKLIGHT_PIN)
 
/*********************************************************************
* Overview: Horizontal and vertical display resolution
*                  (from the glass datasheet).
*********************************************************************/
//#define DISP_HOR_RESOLUTION				320
//#define DISP_VER_RESOLUTION				240

//#define DISP_ORIENTATION					0
//#define DISP_ORIENTATION					90
//#define DISP_ORIENTATION					180
#define DISP_ORIENTATION					90

/* Private define ------------------------------------------------------------*/

#if  ( DISP_ORIENTATION == 90 ) || ( DISP_ORIENTATION == 270 )

#define  MAX_X  320
#define  MAX_Y  240   

#elif  ( DISP_ORIENTATION == 0 ) || ( DISP_ORIENTATION == 180 )

#define  MAX_X  240
#define  MAX_Y  320   

#endif

/*********************************************************************
* Overview: Horizontal synchronization timing in pixels
*                  (from the glass datasheet).
*********************************************************************/
//#define DISP_HOR_PULSE_WIDTH		20    /* 20 */
//#define DISP_HOR_BACK_PORCH			51	  /* 48	*/
//#define DISP_HOR_FRONT_PORCH		20	  /* 20 */

/*********************************************************************
* Overview: Vertical synchronization timing in lines
*                  (from the glass datasheet).
*********************************************************************/
//#define DISP_VER_PULSE_WIDTH		2	  /* 2 */
//#define DISP_VER_BACK_PORCH			12	  /* 16 */
//#define DISP_VER_FRONT_PORCH		4	  /* 4 */

/*********************************************************************
* Definition for SPI interface for HIMAX 8238-A relevant to hardware 
* layout; Hardware dependent!
*********************************************************************/
#define GPIO3 3
#define GPIO2 2
#define GPIO1 1
#define GPIO0 0
#define LCD_RESET (1<<GPIO3)	   /* LCD Reset signal (Reset for display panel, NOT ssd1963) */
#define LCD_SPENA (1<<GPIO0)	   /* SPI EN signal */
#define LCD_SPCLK (1<<GPIO1)	   /* SPI CLK */
#define LCD_SPDAT (1<<GPIO2)	   /* SPI DATA */

/* LCD color */
#define   BLACK        0x0000
#define   NAVY         0x000F
#define   DGREEN       0x03E0
#define   DCYAN        0x03EF
#define   MAROON       0x7800
#define   PURPLE       0x780F
#define   OLIVE        0x7BE0
#define   GREY         0xF7DE
#define   LGRAY        0xC618
#define   DGRAY        0x7BEF
#define   BLUE         0x001F
#define   GREEN        0x07E0
#define   CYAN         0x07FF
#define   RED          0xF800
#define   MAGENTA      0xF81F
#define   YELLOW       0xFFE0
#define   WHITE        0xFFFF

#define RGB565CONVERT(red, green, blue) (int) (((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3))
#define DPYCOLORTRANSLATE(c)    ((((c) & 0x00f80000) >> 8) | (((c) & 0x0000fc00) >> 5) |(((c) & 0x000000f8) >> 3))

/* Private function prototypes -----------------------------------------------*/
void LCD_Initializtion(void);
void LCD_Clear(uint16_t Color);
void TIM_Config(void);	
//void LCD_SetBacklight(uint8_t intensity);
uint16_t LCD_GetPoint(uint16_t Xpos,uint16_t Ypos);
void LCD_SetPoint(uint16_t Xpos,uint16_t Ypos,uint16_t point);
void PutChar(uint16_t Xpos,uint16_t Ypos,uint8_t c,uint16_t charColor,uint16_t bkColor);
void LCD_DrawLine( uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 , uint16_t color );
void PutChinese(uint16_t Xpos,uint16_t Ypos,uint8_t *str,uint16_t Color,uint16_t bkColor); 
void GUI_Text(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor);
void GUI_Chinese(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor);	
void LCD_DrawPicture(uint16_t StartX,uint16_t StartY,uint16_t EndX,uint16_t EndY,uint16_t *pic);
void SSD1289_TextFont(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor, uint16_t FONTx);
void SSD1289_Text(uint16_t Xpos, uint16_t Ypos, uint8_t *str, uint16_t Color, uint16_t bkColor);
void SSD1289_PutCharFont( uint16_t Xpos, uint16_t Ypos, uint8_t ASCII, uint16_t charColor, uint16_t bkColor, uint16_t FONTx);
void SSD1289_PutChar(uint16_t Xpos, uint16_t Ypos, uint8_t ASCII, uint16_t charColor, uint16_t bkColor);
void SSD1289_CleanTextFont(uint16_t Xpos, uint16_t Ypos, uint8_t *str, uint16_t Color, uint16_t FONTx);
void SSD1289_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void SSD1289_FillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void SSD1289_DrawPicture(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *pic);
void SSD1289_DrawPicture8bit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pic);

void LCD_WriteIndex(uint16_t index);
 void LCD_WriteData(uint16_t data);
 uint16_t LCD_ReadData(void);
 uint16_t LCD_ReadReg(uint16_t LCD_Reg);
 void LCD_WriteReg(uint16_t LCD_Reg,uint16_t LCD_RegValue);
static void LCD_SetCursor( uint16_t Xpos, uint16_t Ypos );
void delay_ms(uint16_t ms);
/////////////////////////////////////////////////////////////////
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
void GrTriangleDrawFill(const tContext *pContext,u16 x1,u16 y1, u16 x2, u16 y2, u16 x3, u16 y3);
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
#endif 
