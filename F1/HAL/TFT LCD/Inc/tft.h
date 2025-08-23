/**
  ******************************************************************************
  * @file    hx8347g.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06-May-2014
  * @brief   This file contains all the functions prototypes for the hx8347g.c
  *          driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TFT_H
#define __TFT_H

#include "sys.h"
#include "stm32f1xx_ll_gpio.h"

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "fonts.h"

#define true	1
#define false	0
/*
To use a different resolution TFT, change this or use setResolution().
*/
#define TFTWIDTH  240
#define TFTHEIGHT 320

#define  WIDTH    ((uint16_t)240)
#define  HEIGHT   ((uint16_t)320)

// Color definitions
#define BLACK       0x0000      /*   0,   0,   0 */
#define NAVY        0x000F      /*   0,   0, 128 */
#define DARKGREEN   0x03E0      /*   0, 128,   0 */
#define DARKCYAN    0x03EF      /*   0, 128, 128 */
#define MAROON      0x7800      /* 128,   0,   0 */
#define PURPLE      0x780F      /* 128,   0, 128 */
#define OLIVE       0x7BE0      /* 128, 128,   0 */
#define LIGHTGRAY   0xC618      /* 192, 192, 192 */
#define DARKGRAY    0x7BEF      /* 128, 128, 128 */
#define BLUE        0x001F      /*   0,   0, 255 */
#define GREEN       0x07E0      /*   0, 255,   0 */
#define CYAN        0x07FF      /*   0, 255, 255 */
#define RED         0xF800      /* 255,   0,   0 */
#define MAGENTA     0xF81F      /* 255,   0, 255 */
#define YELLOW      0xFFE0      /* 255, 255,   0 */
#define WHITE       0xFFFF      /* 255, 255, 255 */
#define ORANGE      0xFD20      /* 255, 165,   0 */
#define GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define PINK        0xF81F
#define GRAY        0x5AEB

/*
Define pins and Output Data Registers
*/

#if 1
#define TFT_DATA       GPIOA
#define TFT_D0         LL_GPIO_PIN_0
#define TFT_D1         LL_GPIO_PIN_1
#define TFT_D2         LL_GPIO_PIN_2
#define TFT_D3         LL_GPIO_PIN_3
#define TFT_D4         LL_GPIO_PIN_4
#define TFT_D5         LL_GPIO_PIN_5
#define TFT_D6         LL_GPIO_PIN_6
#define TFT_D7         LL_GPIO_PIN_7
//Port data |D7 |D6 |D5 |D4 |D3 |D2 |D1 |D0 |
//Pin stm32 |PA7|PA6|PA5|PA4|PA3|PA2|PA1|PA0|
#endif

#if 0
#define TFT_DATA       GPIOA
#define TFT_D0         GPIO_PIN_8
#define TFT_D1         GPIO_PIN_9
#define TFT_D2         GPIO_PIN_10
#define TFT_D3         GPIO_PIN_11
#define TFT_D4         GPIO_PIN_12
#define TFT_D5         GPIO_PIN_13
#define TFT_D6         GPIO_PIN_14
#define TFT_D7         GPIO_PIN_15
//Port data |D7  |D6  |D5  |D4  |D3  |D2  |D1 |D0 |
//Pin stm32 |PA15|PA14|PA13|PA12|PA11|PA10|PA9|PA8|
#endif


#if 0
#define TFT_DATA       GPIOB
#define TFT_D0         GPIO_PIN_0
#define TFT_D1         GPIO_PIN_1
#define TFT_D2         GPIO_PIN_2
#define TFT_D3         GPIO_PIN_3
#define TFT_D4         GPIO_PIN_4
#define TFT_D5         GPIO_PIN_5
#define TFT_D6         GPIO_PIN_6
#define TFT_D7         GPIO_PIN_7
//Port data |D7 |D6 |D5 |D4 |D3 |D2 |D1 |D0 |
//Pin stm32 |PB7|PB6|PB5|PB4|PB3|PB2|PB1|PB0|
#endif

#if 0
#define TFT_DATA       GPIOB
#define TFT_D0         GPIO_PIN_8
#define TFT_D1         GPIO_PIN_9
#define TFT_D2         GPIO_PIN_10
#define TFT_D3         GPIO_PIN_11
#define TFT_D4         GPIO_PIN_12
#define TFT_D5         GPIO_PIN_13
#define TFT_D6         GPIO_PIN_14
#define TFT_D7         GPIO_PIN_15
//Port data |D7  |D6  |D5  |D4  |D3  |D2  |D1 |D0 |
//Pin stm32 |PB15|PB14|PB13|PB12|PB11|PB10|PB9|PB8|
#endif

#if 0
#define TFT_DATA       GPIOC
#define TFT_D0         GPIO_PIN_0
#define TFT_D1         GPIO_PIN_1
#define TFT_D2         GPIO_PIN_2
#define TFT_D3         GPIO_PIN_3
#define TFT_D4         GPIO_PIN_4
#define TFT_D5         GPIO_PIN_5
#define TFT_D6         GPIO_PIN_6
#define TFT_D7         GPIO_PIN_7
//Port data |D7 |D6 |D5 |D4 |D3 |D2 |D1 |D0 |
//Pin stm32 |PC7|PC6|PC5|PC4|PC3|PC2|PC1|PC0|
#endif

#if 0
#define TFT_DATA       GPIOC
#define TFT_D0         GPIO_PIN_8
#define TFT_D1         GPIO_PIN_9
#define TFT_D2         GPIO_PIN_10
#define TFT_D3         GPIO_PIN_11
#define TFT_D4         GPIO_PIN_12
#define TFT_D5         GPIO_PIN_13
#define TFT_D6         GPIO_PIN_14
#define TFT_D7         GPIO_PIN_15
//Port data |D7  |D6  |D5  |D4  |D3  |D2  |D1 |D0 |
//Pin stm32 |PC15|PC14|PC13|PC12|PC11|PC10|PC9|PC8|
#endif

#if 0
#define TFT_DATA       GPIOD
#define TFT_D0         GPIO_PIN_0
#define TFT_D1         GPIO_PIN_1
#define TFT_D2         GPIO_PIN_2
#define TFT_D3         GPIO_PIN_3
#define TFT_D4         GPIO_PIN_4
#define TFT_D5         GPIO_PIN_5
#define TFT_D6         GPIO_PIN_6
#define TFT_D7         GPIO_PIN_7
//Port data |D7 |D6 |D5 |D4 |D3 |D2 |D1 |D0 |
//Pin stm32 |PD7|PD6|PD5|PD4|PD3|PD2|PD1|PD0|
#endif

#if 0
#define TFT_DATA       GPIOD
#define TFT_D0         GPIO_PIN_8
#define TFT_D1         GPIO_PIN_9
#define TFT_D2         GPIO_PIN_10
#define TFT_D3         GPIO_PIN_11
#define TFT_D4         GPIO_PIN_12
#define TFT_D5         GPIO_PIN_13
#define TFT_D6         GPIO_PIN_14
#define TFT_D7         GPIO_PIN_15
//Port data |D7  |D6  |D5  |D4  |D3  |D2  |D1 |D0 |
//Pin stm32 |PD15|PD14|PD13|PD12|PD11|PD10|PD9|PD8|
#endif

#if 0
#define TFT_DATA       GPIOE
#define TFT_D0         GPIO_PIN_0
#define TFT_D1         GPIO_PIN_1
#define TFT_D2         GPIO_PIN_2
#define TFT_D3         GPIO_PIN_3
#define TFT_D4         GPIO_PIN_4
#define TFT_D5         GPIO_PIN_5
#define TFT_D6         GPIO_PIN_6
#define TFT_D7         GPIO_PIN_7
//Port data |D7 |D6 |D5 |D4 |D3 |D2 |D1 |D0 |
//Pin stm32 |PE7|PE6|PE5|PE4|PE3|PE2|PE1|PE0|
#endif

#if 0
#define TFT_DATA       GPIOE
#define TFT_D0         GPIO_PIN_8
#define TFT_D1         GPIO_PIN_9
#define TFT_D2         GPIO_PIN_10
#define TFT_D3         GPIO_PIN_11
#define TFT_D4         GPIO_PIN_12
#define TFT_D5         GPIO_PIN_13
#define TFT_D6         GPIO_PIN_14
#define TFT_D7         GPIO_PIN_15
//Port data |D7  |D6  |D5  |D4  |D3  |D2  |D1 |D0 |
//Pin stm32 |PE15|PE14|PE13|PE12|PE11|PE10|PE9|PE8|
#endif


#define TFT_CNTRL      GPIOB
#define LL_LOW(GPIO_PIN)  LL_GPIO_WriteOutputPort(TFT_CNTRL, (LL_GPIO_ReadOutputPort(TFT_CNTRL) & ~(GPIO_PIN)))
#define LL_HIGH(GPIO_PIN) LL_GPIO_WriteOutputPort(TFT_CNTRL, (LL_GPIO_ReadOutputPort(TFT_CNTRL) | GPIO_PIN))


// Note:
// PA15 PB3 PB4 is assigned to JTAG debug port by default on some boards.
// Therefore, it may not be available by default.
#define TFT_RD         LL_GPIO_PIN_0 // Px0
#define TFT_WR         LL_GPIO_PIN_1 // Px1
#define TFT_RS         LL_GPIO_PIN_5 // Px5
#define TFT_CS         LL_GPIO_PIN_6 // Px6
#define TFT_RST        LL_GPIO_PIN_7 // Px7

//#define DELAY          delayMicroseconds(10);
#define DELAY          (void)0  // NOP

#define RD_ACTIVE      LL_LOW(TFT_RD)
#define RD_IDLE        LL_HIGH(TFT_RD)
#define WR_ACTIVE      LL_LOW(TFT_WR)
#define WR_IDLE        LL_HIGH(TFT_WR)
#define CD_COMMAND     LL_LOW(TFT_RS)
#define CD_DATA        LL_HIGH(TFT_RS)
#define CS_ACTIVE      LL_LOW(TFT_CS)
#define CS_IDLE        LL_HIGH(TFT_CS)
#define RST_ACTIVE     LL_LOW(TFT_RST)
#define RST_IDLE       LL_HIGH(TFT_RST)

#define RD_STROBE      {RD_ACTIVE; RD_IDLE;} // Not use
#define WR_STROBE      {WR_ACTIVE; WR_IDLE;} // Not use





  void STM32_TFT_8bit(void);
  void     TFT_enablePortClock(GPIO_TypeDef *gpio);
  void     TFT_setResolution(int16_t width, int16_t height);
  void     TFT_setOffset(int16_t offset);
  void     TFT_begin(uint16_t ID);
  void     TFT_setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
  void     TFT_fillScreen(uint16_t color);
  void     TFT_drawLine(int16_t x0, int16_t y0,int16_t x1, int16_t y1, uint16_t color);
	void 		 TFT_drawCircleHelper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
  void     TFT_drawPixel(int16_t x, int16_t y, uint16_t color);
	void 		 TFT_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
	void 		 TFT_fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color);
  void 		 TFT_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
	void     TFT_pushColors8(uint8_t * block, int16_t n, uint8_t first);
  void     TFT_pushColors(uint16_t * block, int16_t n, uint8_t first);
  void     TFT_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  void     TFT_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  void     TFT_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
	void 		 TFT_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
	void 		 TFT_drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
	void 		 TFT_fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
	void 		 TFT_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
  void 		 TFT_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
	uint16_t TFT_color565(uint8_t r, uint8_t g, uint8_t b);
  int16_t  TFT_readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h);
  uint16_t TFT_readPixel(int16_t x, int16_t y);
  void     TFT_setRotation(uint8_t r);
  void     TFT_vertScroll(int16_t top, int16_t scrollines, int16_t offset);
  void     TFT_invertDisplay(uint8_t i);
  uint16_t TFT_readID(void);
	uint16_t width(void);
	uint16_t height(void);
	///////////////////////////////////////////////////////////////////////////////////////
void TFT_drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
size_t TFT_write(uint8_t c);
void TFT_setFont(const GFXfont *f) ;
void TFT_setCursor(int16_t x, int16_t y);
void TFT_printnewtstr (int row, uint16_t txtcolor, const GFXfont *f, uint8_t txtsize, uint8_t *str);
void TFT_printstr (uint8_t *str);
void TFT_setTextWrap(uint8_t w);
void TFT_setTextBgColor (uint16_t color);
void TFT_setTextColor (uint16_t color);
void TFT_setTextSize (uint8_t size);
uint8_t TFT_getRotation (void);
void TFT_scrollup (uint16_t speed);
void TFT_scrolldown (uint16_t speed);

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#define pgm_read_pointer(addr) ((void *)pgm_read_word(addr))
	

 

#endif /* __TFT_H */

