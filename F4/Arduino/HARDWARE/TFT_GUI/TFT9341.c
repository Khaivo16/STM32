#include "TFT9341.h"
#include <string.h>

/****************************************************************************
 *! \brief
 *gsili9341
 *My ili9341 code.
 *See GUIDRV_Template.c for my code that add the required
 *and some optional EMWIN driver code
 ***************************************************************************/
#include <stdint.h>
#include<assert.h>

#include <fsmc.h>

/*You have to first calculate your base address. For example base address of Bank1 of NOR/SRAM Bank1~4 is 0x60000000.
This address (0x60000000) is your command address.
Then according to your Address line pin you have to calculate data address.For A16 you can callculate like this;
RAM base address = 0X60020000 = 0X60000000 +2 ^ 16 * 2 = 0X60000000 + 0X20000 = 0X60020000
This address is your data address.
Example code:*/

//#define LCD_REG      (*((volatile unsigned short *) 0x60000000)) 
//#define LCD_RAM      (*((volatile unsigned short *) 0x60020000)) 

#define LCD_REG  (*((volatile unsigned short *) 0x60000000)) // RS = 0
#define LCD_RAM  (*((volatile unsigned short *) 0x60100000)) // RS = 1
	
//#define LCD_REG (*fsmcCommand)
//#define LCD_RAM    (*fsmcData)


//****************************************************************************//
#include <Adafruit_TFTLCD_16bit_STM32.h> // Hardware-specific library


extern Adafruit_TFTLCD_16bit_STM32 tft;
//****************************************************************************//

//****************************************************************************//

///////////////////////////////////////////////////////////////////////

_lcd_dev lcddev;


uint16_t POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  
uint16_t DeviceCode;	 

 	 
void GUI_Init(void)
{  
//	fsmc_lcd_init();
//	delay_ms(100);
////*************3.2inch ILI9341**********//	
//	LCD_WR_REG(0xCF);  
//	LCD_WR_DATA(0x00); 
//	LCD_WR_DATA(0xD9); //C1 
//	LCD_WR_DATA(0X30); 
//	LCD_WR_REG(0xED);  
//	LCD_WR_DATA(0x64); 
//	LCD_WR_DATA(0x03); 
//	LCD_WR_DATA(0X12); 
//	LCD_WR_DATA(0X81); 
//	LCD_WR_REG(0xE8);  
//	LCD_WR_DATA(0x85); 
//	LCD_WR_DATA(0x10); 
//	LCD_WR_DATA(0x7A); 
//	LCD_WR_REG(0xCB);  
//	LCD_WR_DATA(0x39); 
//	LCD_WR_DATA(0x2C); 
//	LCD_WR_DATA(0x00); 
//	LCD_WR_DATA(0x34); 
//	LCD_WR_DATA(0x02); 
//	LCD_WR_REG(0xF7);  
//	LCD_WR_DATA(0x20); 
//	LCD_WR_REG(0xEA);  
//	LCD_WR_DATA(0x00); 
//	LCD_WR_DATA(0x00); 
//	LCD_WR_REG(0xC0);    //Power control 
//	LCD_WR_DATA(0x1B);   //VRH[5:0] 
//	LCD_WR_REG(0xC1);    //Power control 
//	LCD_WR_DATA(0x12);   //SAP[2:0];BT[3:0] //0x01
//	LCD_WR_REG(0xC5);    //VCM control 
//	LCD_WR_DATA(0x26); 	 //3F
//	LCD_WR_DATA(0x26); 	 //3C
//	LCD_WR_REG(0xC7);    //VCM control2 
//	LCD_WR_DATA(0XB0); 
//	
//	LCD_WR_REG(0x36);    // Memory Access Control 
//	LCD_WR_DATA(0x08); 
//	
//	LCD_WR_REG(0x3A);   
//	LCD_WR_DATA(0x55); 
//	LCD_WR_REG(0xB1);   
//	LCD_WR_DATA(0x00);   
//	LCD_WR_DATA(0x1A); 
//	LCD_WR_REG(0xB6);    // Display Function Control 
//	LCD_WR_DATA(0x0A); 
//	LCD_WR_DATA(0xA2); 
//	LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
//	LCD_WR_DATA(0x00); 
//	LCD_WR_REG(0x26);    //Gamma curve selected 
//	LCD_WR_DATA(0x01); 
//	LCD_WR_REG(0xE0); //Set Gamma
//	LCD_WR_DATA(0x1F);
//	LCD_WR_DATA(0x24);
//	LCD_WR_DATA(0x24);
//	LCD_WR_DATA(0x0D);
//	LCD_WR_DATA(0x12);
//	LCD_WR_DATA(0x09);
//	LCD_WR_DATA(0x52);
//	LCD_WR_DATA(0xB7);
//	LCD_WR_DATA(0x3F);
//	LCD_WR_DATA(0x0C);
//	LCD_WR_DATA(0x15);
//	LCD_WR_DATA(0x06);
//	LCD_WR_DATA(0x0E);
//	LCD_WR_DATA(0x08);
//	LCD_WR_DATA(0x00);
//	LCD_WR_REG(0XE1); //Set Gamma
//	LCD_WR_DATA(0x00);
//	LCD_WR_DATA(0x1B);
//	LCD_WR_DATA(0x1B);
//	LCD_WR_DATA(0x02);
//	LCD_WR_DATA(0x0E);
//	LCD_WR_DATA(0x06);
//	LCD_WR_DATA(0x2E);
//	LCD_WR_DATA(0x48);
//	LCD_WR_DATA(0x3F);
//	LCD_WR_DATA(0x03);
//	LCD_WR_DATA(0x0A);
//	LCD_WR_DATA(0x09);
//	LCD_WR_DATA(0x31);
//	LCD_WR_DATA(0x37);
//	LCD_WR_DATA(0x1F);
//	LCD_WR_REG(0x2B); 
//	LCD_WR_DATA(0x00);
//	LCD_WR_DATA(0x00);
//	LCD_WR_DATA(0x01);
//	LCD_WR_DATA(0x3f);
//	LCD_WR_REG(0x2A); 
//	LCD_WR_DATA(0x00);
//	LCD_WR_DATA(0x00);
//	LCD_WR_DATA(0x00);
//	LCD_WR_DATA(0xef);	 
//	LCD_WR_REG(0x11); //Exit Sleep
//	delay_ms(120);
//	LCD_WR_REG(0x29); //display on		

//  LCD_direction(3);//270*
// 
//	LCD_Clear(WHITE);

	
			
			lcddev.width=tft.width();
			lcddev.height=tft.height();	
			lcddev.setxcmd=0x2A;
			lcddev.setycmd=0x2B;
			lcddev.wramcmd=0x2C;
			lcddev.rramcmd=0x2E;		
			

		
}
 


/*****************************************************************************
 * @name       :void LCD_WR_REG(uint16_t data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :data:Command value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_REG(uint16_t data)
{ 
	data=data;  
#if LCD_USE8BIT_MODEL
	LCD_REG=(data<<8); 
#else
	LCD_REG=data; 
#endif	
}

/*****************************************************************************
 * @name       :void LCD_WR_DATA(uint16_t data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit data to the LCD screen
 * @parameters :data:data value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_DATA(uint16_t data)
{
	data=data;			
#if LCD_USE8BIT_MODEL
	LCD_RAM=(data<<8);
#else
	LCD_RAM=data; 
#endif
}

/*****************************************************************************
 * @name       :uint16_t LCD_RD_DATA(void)
 * @date       :2018-11-13 
 * @function   :Read an 16-bit value from the LCD screen
 * @parameters :None
 * @retvalue   :read value
******************************************************************************/
uint16_t LCD_RD_DATA(void)
{
	return LCD_read();
}

uint16_t LCD_read(void)
{
	uint16_t data;  
	data=LCD_RAM;	
#if LCD_USE8BIT_MODEL
	return (data>>8);  
#else
	return data;
#endif
}
/*****************************************************************************
 * @name       :void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue)
 * @date       :2018-08-09 
 * @function   :Write data into registers
 * @parameters :LCD_Reg:Register address
                LCD_RegValue:Data to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue)
{
#if LCD_USE8BIT_MODEL	
	LCD_REG = (LCD_Reg<<8);		 
	LCD_RAM = (LCD_RegValue<<8);
#else
	LCD_REG = LCD_Reg;		 
	LCD_RAM = LCD_RegValue;    		 
#endif
}	   

/*****************************************************************************
 * @name       :uint16_t LCD_ReadReg(uint16_t LCD_Reg)
 * @date       :2018-11-13 
 * @function   :read value from specially registers
 * @parameters :LCD_Reg:Register address
 * @retvalue   :read value
******************************************************************************/
void LCD_ReadReg(uint16_t LCD_Reg,uint8_t *Rval,int n)
{
	LCD_WR_REG(LCD_Reg); 
	while(n--)
	{		
		*(Rval++) = LCD_RD_DATA();
		delay_us(100);
	}
}

/*****************************************************************************
 * @name       :void LCD_WriteRAM_Prepare(void)
 * @date       :2018-08-09 
 * @function   :Write GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
}	 

/*****************************************************************************
 * @name       :void LCD_ReadRAM_Prepare(void)
 * @date       :2018-11-13 
 * @function   :Read GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_ReadRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.rramcmd);
}

/*****************************************************************************
 * @name       :void Lcd_WriteData_16Bit(uint16_t Data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :Data:Data to be written
 * @retvalue   :None
******************************************************************************/	 
void Lcd_WriteData_16Bit(uint16_t Data)
{
#if LCD_USE8BIT_MODEL	
	 LCD_RAM = Data;
	 LCD_RAM = Data<<8;   
#else
	 LCD_RAM = Data; 
#endif
}

uint16_t Color_To_565(uint8_t r, uint8_t g, uint8_t b)
{
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
}

/*****************************************************************************
 * @name       :uint16_t Lcd_ReadData_16Bit(void)
 * @date       :2018-11-13 
 * @function   :Read an 16-bit value from the LCD screen
 * @parameters :None
 * @retvalue   :read value
******************************************************************************/	
uint16_t Lcd_ReadData_16Bit(void)
{
	uint16_t r,g,b;
	//dummy data
	r = LCD_RD_DATA();
	delay_us(1);//±1us
	//8bit:red data	
	//16bit:red and green data
	r = LCD_RD_DATA();
	delay_us(1);//±1us	
	//8bit:green data
	//16bit:blue data
	g = LCD_RD_DATA();
#if LCD_USE8BIT_MODEL
	delay_us(1);//±1us	
	//8bit:blue data
	b = LCD_RD_DATA();
#else
	b = g>>8;
	g = r&0xFF; 
	r = r>>8;
#endif
	return Color_To_565(r, g, b);
}

/*****************************************************************************
 * @name       :void LCD_DrawPoint(uint16_t x,uint16_t y)
 * @date       :2018-08-09 
 * @function   :Write a pixel data at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/	
void LCD_DrawPoint(uint16_t x,uint16_t y)
{
	LCD_SetCursor(x,y); 
	Lcd_WriteData_16Bit(POINT_COLOR); 
}

/*****************************************************************************
 * @name       :uint16_t LCD_ReadPoint(uint16_t x,uint16_t y)
 * @date       :2018-11-13 
 * @function   :Read a pixel color value at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :the read color value
******************************************************************************/	
uint16_t LCD_ReadPoint(uint16_t x,uint16_t y)
{
	uint16_t color;
	if(x>=lcddev.width||y>=lcddev.height)
	{
		return 0;	
	}
	LCD_SetCursor(x,y);
	LCD_ReadRAM_Prepare();
	color = Lcd_ReadData_16Bit();
	return color;
}

/*****************************************************************************
 * @name       :void LCD_Clear(uint16_t Color)
 * @date       :2018-08-09 
 * @function   :Full screen filled LCD screen
 * @parameters :color:Filled color
 * @retvalue   :None
******************************************************************************/	
void LCD_Clr(uint16_t Color)
{
  unsigned int i;//,m;  
	uint32_t total_point=lcddev.width*lcddev.height;
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);   
	for(i=0;i<total_point;i++)
	{ 
#if LCD_USE8BIT_MODEL	
		LCD_RAM = Color;
		LCD_RAM = Color<<8;
#else
		LCD_RAM = Color;
#endif
	}
} 

void LCD_Clear(unsigned int dat){LCD_Clr(dat);}
/*****************************************************************************
 * @name       :void LCD_Init(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
/*****************************************************************************
 * @name       :void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd)
 * @date       :2018-08-09 
 * @function   :Setting LCD display window
 * @parameters :xStar:the bebinning x coordinate of the LCD display window
								yStar:the bebinning y coordinate of the LCD display window
								xEnd:the endning x coordinate of the LCD display window
								yEnd:the endning y coordinate of the LCD display window
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd)
{	
	LCD_WR_REG(lcddev.setxcmd);	
	LCD_WR_DATA(xStar>>8);
	LCD_WR_DATA(0x00FF&xStar);		
	LCD_WR_DATA(xEnd>>8);
	LCD_WR_DATA(0x00FF&xEnd);

	LCD_WR_REG(lcddev.setycmd);	
	LCD_WR_DATA(yStar>>8);
	LCD_WR_DATA(0x00FF&yStar);		
	LCD_WR_DATA(yEnd>>8);
	LCD_WR_DATA(0x00FF&yEnd);

	LCD_WriteRAM_Prepare();				
}   

/*****************************************************************************
 * @name       :void LCD_direction(uint8_t direction)
 * @date       :2018-08-09 
 * @function   :Setting the display direction of LCD screen
 * @parameters :direction:0-0 degree
                          1-90 degree
													2-180 degree
													3-270 degree
 * @retvalue   :None
******************************************************************************/ 
void LCD_direction(uint8_t direction)
{ 
			lcddev.setxcmd=0x2A;
			lcddev.setycmd=0x2B;
			lcddev.wramcmd=0x2C;
			lcddev.rramcmd=0x2E;
	switch(direction){		  
		case 0:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;		
			LCD_WriteReg(0x36,(1<<3));
		break;
		case 1:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<5)|(1<<6));
		break;
		case 2:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;	
			LCD_WriteReg(0x36,(1<<3)|(1<<7)|(1<<4)|(1<<6));
		break;
		case 3:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<7)|(1<<5)|(1<<4));
		break;	
		default:break;
	}		
}	 

/*****************************************************************************
 * @name       :uint16_t LCD_Read_ID(void)
 * @date       :2018-11-13 
 * @function   :Read ID
 * @parameters :None
 * @retvalue   :ID value
******************************************************************************/ 
uint16_t LCD_Read_ID(void)
{
	uint8_t val[4] = {0};
	LCD_ReadReg(0xD3,val,4);
	return (val[2]<<8)|val[3];
}


////////////////////////////////////////////////////////////////////////////////////////////////
/*****************************************************************************
 * @name       :void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
 * @date       :2018-08-09 
 * @function   :Set coordinate value
 * @parameters :Xpos:the  x coordinate of the pixel
								Ypos:the  y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/ 
//void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
//{	  	    			
//	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);	
//} 
void LCD_SetCursor( uint16_t Xpos, uint16_t Ypos )
{				   
    LCD_WR_REG(lcddev.setxcmd); // Column addr set
    LCD_WR_DATA((Xpos >> 8));   // XSTART
    LCD_WR_DATA((Xpos & 0x00ff));   // XSTART
    //-----------------------------------
    LCD_WR_REG(lcddev.setycmd); // Row addr set
    LCD_WR_DATA((Ypos >>8));   // YSTART
    LCD_WR_DATA((Ypos &0x00ff));   // YSTART
	//tft.setCursor(Xpos, Ypos);

}

uint16_t  ili9320_BRG2RGB(uint32_t Color)
{
  uint16_t  r, g, b, rgb;

  b = (Color>>0)  & 0x1f;
  g = (Color>>5)  & 0x3f;
  r = (Color>>11) & 0x1f;
  
  rgb =  (b<<11) + (g<<5) + (r<<0);

  return( rgb );
} 


void ili9320PixelDraw(void *pvDisplayData, long x, long y,unsigned long point)
{
  if ( (x>320)||(y>240) ) return;
  LCD_SetCursor(x,y);
  //Clr_Cs;
  LCD_WR_REG(lcddev.wramcmd);
  LCD_WR_DATA(point);
  //Set_Cs;
	//tft.drawPixel(x, y, point);
}


 void ili9320DrawMultiple(void *pvDisplayData, long lX,
                                           long lY, long lX0, long lCount,
                                           long lBPP,
                                           const unsigned char *pucData,
                                           const unsigned char *pucPalette)
{    unsigned int ulByte;
	//LCD_WR_CMD(0x0036,0x0008); ///////LCD_WR_CMD(0x03,0x1018);///////mac dinh:0x1030
	   LCD_SetCursor(lX,lY);
		 //Clr_Cs;
     LCD_WR_REG(lcddev.wramcmd);
		switch(lBPP)
    {
        //
        // The pixel data is in 1 bit per pixel format.
        //
        case 1:
        {
            //
            // Loop while there are more pixels to draw.
            //
            while(lCount)
            {
                //
                // Get the next byte of image data.
                //
                ulByte = *pucData++;

                //
                // Loop through the pixels in this byte of image data.
                //
                for(; (lX0 < 8) && lCount; lX0++, lCount--)
                {
                    //
                    // Draw this pixel in the appropriate color.
                    //
                    LCD_WR_DATA(((unsigned long *)pucPalette)[(ulByte >>
                                                             (7 - lX0)) & 1]);
                }

                //
                // Start at the beginning of the next byte of image data.
                //
                lX0 = 0;
            }

            //
            // The image data has been drawn.
            //
            break;
        }

        //
        // The pixel data is in 4 bit per pixel format.
        //
        case 4:
        {
            //
            // Loop while there are more pixels to draw.  "Duff's device" is
            // used to jump into the middle of the loop if the first nibble of
            // the pixel data should not be used.  Duff's device makes use of
            // the fact that a case statement is legal anywhere within a
            // sub-block of a switch statement.  See
            // http://en.wikipedia.org/wiki/Duff's_device for detailed
            // information about Duff's device.
            //
            switch(lX0 & 1)
            {
                case 0:
                    while(lCount)
                    {
                        //
                        // Get the upper nibble of the next byte of pixel data
                        // and extract the corresponding entry from the
                        // palette.
                        //
                        ulByte = (*pucData >> 4) * 3;
                        ulByte = (*(unsigned long *)(pucPalette + ulByte) &
                                  0x00ffffff);

                        //
                        // Translate this palette entry and write it to the
                        // screen.
                        //
                         LCD_WR_DATA(DPYCOLORTRANSLATE(ulByte));

                        //
                        // Decrement the count of pixels to draw.
                        //
                        lCount--;

                        //
                        // See if there is another pixel to draw.
                        //
                        if(lCount)
                        {
                case 1:
                            //
                            // Get the lower nibble of the next byte of pixel
                            // data and extract the corresponding entry from
                            // the palette.
                            //
                            ulByte = (*pucData++ & 15) * 3;
                            ulByte = (*(unsigned long *)(pucPalette + ulByte) &
                                      0x00ffffff);

                            //
                            // Translate this palette entry and write it to the
                            // screen.
                            //
                             LCD_WR_DATA(DPYCOLORTRANSLATE(ulByte));

                            //
                            // Decrement the count of pixels to draw.
                            //
                            lCount--;
                        }
                    }
            }

            //
            // The image data has been drawn.
            //
            break;
        }

        //
        // The pixel data is in 8 bit per pixel format.
        //
        case 8:
        {
            //
            // Loop while there are more pixels to draw.
            //
            while(lCount--)
            {
                //
                // Get the next byte of pixel data and extract the
                // corresponding entry from the palette.
                //
                ulByte = *pucData++ * 3;
                ulByte = *(unsigned long *)(pucPalette + ulByte) & 0x00ffffff;

                //
                // Translate this palette entry and write it to the screen.
                //
                 LCD_WR_DATA(DPYCOLORTRANSLATE(ulByte));
            }

            //
            // The image data has been drawn.
            //
            break;
        }

        //
        // We are being passed data in the display's native format.  Merely
        // write it directly to the display.  This is a special case which is
        // not used by the graphics library but which is helpful to
        // applications which may want to handle, for example, JPEG images.
        //
        case 16:
        {
            unsigned short usByte;

            //
            // Loop while there are more pixels to draw.
            //
            while(lCount--)
            {
                //
                // Get the next byte of pixel data and extract the
                // corresponding entry from the palette.
                //
                usByte = *((unsigned short *)pucData);
                pucData += 2;

                //
                // Translate this palette entry and write it to the screen.
                //
                 LCD_WR_DATA(usByte);
            }
        }
    }
   //Set_Cs; 
		//LCD_SetWindows(0,0,319,239);
}

 void ili9320RectFill(void *pvDisplayData, const tRectangle *pRect,unsigned long ulValue)
{ 
    long lCount;
    for(lCount = pRect->sYMin ; lCount <= pRect->sYMax; lCount++)ili9320LineDrawH(&pvDisplayData, pRect->sXMin, pRect->sXMax,lCount,ulValue);		
}

 void ili9320LineDrawH(void *pvDisplayData, long lX1, long lX2,
                                   long lY, unsigned long ulValue)
{    //LCD_WR_CMD(0x0036,0x0008);//LCD_WR_CMD(0x03,0x1038); //
	   LCD_SetCursor(lX1,lY);
     //Clr_Cs;
     LCD_WR_REG(lcddev.wramcmd);
     while(lX1++ <= lX2)LCD_WR_DATA(ulValue);
     //Set_Cs;
	   //LCD_WR_CMD(0x03,0x1030); //reset ve mac dinh
     //LCD_WR_REG(lcddev.wramcmd);
}

 void ili9320LineDrawV(void *pvDisplayData, long lX, long lY1,
                                   long lY2, unsigned long ulValue)
{ //LCD_WR_CMD(0x0036,0x0008);//LCD_WR_CMD(0x03,0x1018); //
  LCD_SetCursor(lX,lY1);
  //Clr_Cs;
  LCD_WR_REG(lcddev.wramcmd);
  while(lY1++ <= lY2)LCD_WR_DATA(ulValue);
  //Set_Cs;
	//LCD_WR_CMD(0x03,0x1030); //reset ve mac dinh
  //LCD_WR_REG(0x0022);
}

void ili9320Flush(void *pvDisplayData){}

void GrContextInit(tContext *pContext, const tDisplay *pDisplay)
{
    pContext->lSize = sizeof(tContext);
    pContext->pDisplay = pDisplay;
    pContext->sClipRegion.sXMin = 0;
    pContext->sClipRegion.sYMin = 0;
    pContext->sClipRegion.sXMax = pDisplay->usWidth - 1;
    pContext->sClipRegion.sYMax = pDisplay->usHeight - 1;
    pContext->ulForeground = 0;
    pContext->ulBackground = 0;
    pContext->pFont = 0;
}
void GrContextClipRegionSet(tContext *pContext, tRectangle *pRect)
{
    unsigned long ulW, ulH;
    ulW = DpyWidthGet(pContext->pDisplay);
    ulH = DpyHeightGet(pContext->pDisplay);
    pContext->sClipRegion.sXMin = ((pRect->sXMin < 0) ? 0 :
                                   ((pRect->sXMin >= ulW) ? (ulW - 1) :
                                    pRect->sXMin));
    pContext->sClipRegion.sYMin = ((pRect->sYMin < 0) ? 0 :
                                   ((pRect->sYMin >= ulH) ? (ulH - 1) :
                                    pRect->sYMin));
    pContext->sClipRegion.sXMax = ((pRect->sXMax < 0) ? 0 :
                                   ((pRect->sXMax >= ulW) ? (ulW - 1) :
                                    pRect->sXMax));
    pContext->sClipRegion.sYMax = ((pRect->sYMax < 0) ? 0 :
                                   ((pRect->sYMax >= ulH) ? (ulH - 1) :
                                    pRect->sYMax));
}
void GrLineDrawH(const tContext *pContext, long lX1, long lX2, long lY)
{
    long lTemp;
    if((lY < pContext->sClipRegion.sYMin) ||
       (lY > pContext->sClipRegion.sYMax))
    {
        return;
    }
    if(lX1 > lX2)
    {
        lTemp = lX1;
        lX1 = lX2;
        lX2 = lTemp;
    }
    if((lX1 > pContext->sClipRegion.sXMax) ||
       (lX2 < pContext->sClipRegion.sXMin))
    {
        return;
    }
    if(lX1 < pContext->sClipRegion.sXMin)
    {
        lX1 = pContext->sClipRegion.sXMin;
    }
    if(lX2 > pContext->sClipRegion.sXMax)
    {
        lX2 = pContext->sClipRegion.sXMax;
    }
    DpyLineDrawH(pContext->pDisplay, lX1, lX2, lY, pContext->ulForeground);
}

void
GrLineDrawV(const tContext *pContext, long lX, long lY1, long lY2)
{
    long lTemp;
    if((lX < pContext->sClipRegion.sXMin) ||
       (lX > pContext->sClipRegion.sXMax))
    {
        return;
    }
    if(lY1 > lY2)
    {
        lTemp = lY1;
        lY1 = lY2;
        lY2 = lTemp;
    }
    if((lY1 > pContext->sClipRegion.sYMax) ||
       (lY2 < pContext->sClipRegion.sYMin))
    {
        return;
    }
    if(lY1 < pContext->sClipRegion.sYMin)
    {
        lY1 = pContext->sClipRegion.sYMin;
    }
    if(lY2 > pContext->sClipRegion.sYMax)
    {
        lY2 = pContext->sClipRegion.sYMax;
    }
    DpyLineDrawV(pContext->pDisplay, lX, lY1, lY2, pContext->ulForeground);
}
static long
GrClipCodeGet(const tContext *pContext, long lX, long lY)
{
    long lCode;
    lCode = 0;
    if(lY < pContext->sClipRegion.sYMin)
    {
        lCode |= 1;
    }
    if(lY > pContext->sClipRegion.sYMax)
    {
        lCode |= 2;
    }
    if(lX < pContext->sClipRegion.sXMin)
    {
        lCode |= 4;
    }
    if(lX > pContext->sClipRegion.sXMax)
    {
        lCode |= 8;
    }
    return(lCode);
}
static long
GrLineClip(const tContext *pContext, long *plX1, long *plY1, long *plX2,
           long *plY2)
{
    long lCode, lCode1, lCode2, lX, lY;
    lCode1 = GrClipCodeGet(pContext, *plX1, *plY1);
    lCode2 = GrClipCodeGet(pContext, *plX2, *plY2);
    while(1)
    {
        if((lCode1 == 0) && (lCode2 == 0))
        {
            return(1);
        }
        if((lCode1 & lCode2) != 0)
        {
            return(0);
        }
        if(lCode1)
        {
            lCode = lCode1;
        }
        else
        {
            lCode = lCode2;
        }
        if(lCode & 1)
        {
            lX = (*plX1 + (((*plX2 - *plX1) *
                            (pContext->sClipRegion.sYMin - *plY1)) /
                           (*plY2 - *plY1)));
            lY = pContext->sClipRegion.sYMin;
        }
        else if(lCode & 2)
        {
            lX = (*plX1 + (((*plX2 - *plX1) *
                            (pContext->sClipRegion.sYMax - *plY1)) /
                           (*plY2 - *plY1)));
            lY = pContext->sClipRegion.sYMax;
        }
        else if(lCode & 4)
        {
            lX = pContext->sClipRegion.sXMin;
            lY = (*plY1 + (((*plY2 - *plY1) *
                            (pContext->sClipRegion.sXMin - *plX1)) /
                           (*plX2 - *plX1)));
        }
        else
        {
            lX = pContext->sClipRegion.sXMax;
            lY = (*plY1 + (((*plY2 - *plY1) *
                            (pContext->sClipRegion.sXMax - *plX1)) /
                           (*plX2 - *plX1)));
        }
        if(lCode1)
        {
            *plX1 = lX;
            *plY1 = lY;
            lCode1 = GrClipCodeGet(pContext, lX, lY);
        }
        else
        {
            *plX2 = lX;
            *plY2 = lY;

            lCode2 = GrClipCodeGet(pContext, lX, lY);
        }
    }
}

void
GrLineDraw(const tContext *pContext, long lX1, long lY1, long lX2, long lY2)
{
    long deltax,deltay,heso,i,tam;
    if(lX1 == lX2)
    {
        GrLineDrawV(pContext, lX1, lY1, lY2);
        return;
    }

    if(lY1 == lY2)
    {
      
        GrLineDrawH(pContext, lX1, lX2, lY1);

    
        return;
    }
    if(GrLineClip(pContext, &lX1, &lY1, &lX2, &lY2) == 0)
    {
        return;
    }
		deltax= lX2-lX1; deltay =lY2-lY1;
		heso=lX2*lY1 -lX1*lY2;
    if(((lY2 > lY1) ? (lY2 - lY1) : (lY1 - lY2)) >
       ((lX2 > lX1) ? (lX2 - lX1) : (lX1 - lX2)))
    {
     if(lY1>lY2){i=lY1;lY1=lY2;lY2=i;}
     for(i=lY1;i<=lY2;i++)
     {   tam = (deltax*i - heso)/deltay;
         DpyPixelDraw(pContext->pDisplay, tam, i, pContext->ulForeground);
     }
				
    }
    else
    {
      if(lX1>lX2){i=lX1;lX1=lX2;lX2=i;}
      for(i=lX1;i<=lX2;i++)
      {   tam = (deltay*i + heso)/deltax;
          DpyPixelDraw(pContext->pDisplay, i, tam, pContext->ulForeground);
      }
    }
   
}
void LineBMP(unsigned char hang, unsigned int x0,unsigned int d0, unsigned char *mang )
{  
	 uint16_t *bitmap_ptr = (uint16_t*)mang,i;
	 LCD_SetCursor(x0,hang);
	 //LCD_WR_CMD(0x0036,0x0008);//LCD_WR_CMD(0x03,0x1018); 
   //Clr_Cs;
   LCD_WR_REG(lcddev.wramcmd);
   for(i=0;i<=d0;i++)LCD_WR_DATA(*bitmap_ptr++);
   //Set_Cs; 
}
void GrTriangleDrawFill(const tContext *pContext,uint16_t x1,uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3)
{
	   signed short x[3],y[3],n,yt1,yt2,yt,dx1,dx2,dx3,dy1,dy2,dy3; 
		 signed long sh1,sh2,sh3;
		 x[0]=x1; x[1]=x2; x[2]=x3;y[0]=y1; y[1]=y2; y[2]=y3;
		 if(x[0]>x[1]){yt=x[0];x[0]=x[1];x[1]=yt;yt=y[0];y[0]=y[1];y[1]=yt;}
		 if(x[0]>x[2]){yt=x[0];x[0]=x[2];x[2]=yt;yt=y[0];y[0]=y[2];y[2]=yt;}
		 if(x[1]>x[2]){yt=x[1];x[1]=x[2];x[2]=yt;yt=y[1];y[1]=y[2];y[2]=yt;}
     dx1= x[2]-x[0]; dx2= x[1]-x[0];dx3=x[2]-x[1];
		 dy1= y[2]-y[0]; dy2= y[1]-y[0];dy3=y[2]-y[1];
		 sh1=x[2]*y[0] - x[0]*y[2]; sh2=x[1]*y[0] - x[0]*y[1]; sh3=x[2]*y[1] - x[1]*y[2];
		 
     for(n=x[0] ; n<x[1]; n++)
		 {
			  yt1= (dy1*n + sh1)/dx1;
				yt2= (dy2*n + sh2)/dx2;
			  if(yt2<yt1){yt= yt1; yt1=yt2; yt2=yt;}
		    GrLineDrawV(pContext, n, yt1, yt2);
		 }
		 for(n=x[1] ; n<x[2]; n++)
		 {
			  yt1= (dy1*n + sh1)/dx1;
				yt2= (dy3*n + sh3)/dx3;
			  if(yt2<yt1){yt= yt1; yt1=yt2; yt2=yt;}
		    GrLineDrawV(pContext, n, yt1, yt2);
		 }	 
}

void GrBar(const tContext *pContext,unsigned short x1, unsigned short y1,unsigned short x2, unsigned short y2, unsigned char width)
{
   unsigned char        half_width;
   signed long  addx=1, addy=1, j,P, diff, c1, c2,dy, dx;
   unsigned short i=0;
   if(x2>x1)dx = x2 - x1; else dx = x1 - x2;
   if(y2>y1)dy = y2 - y1; else dy = y1 - y2;

   half_width = width/2;
   c1 = -(dx*x1 + dy*y1);
   c2 = -(dx*x2 + dy*y2);

   if(x1 > x2)
   {
      signed short temp;
      temp = c1;
      c1 = c2;
      c2 = temp;
      addx = -1;
   }
   if(y1 > y2)
   {
      signed short temp;
      temp = c1;
      c1 = c2;
      c2 = temp;
      addy = -1;
   }

   if(dx >= dy)
   {
      P = 2*dy - dx;
      diff = P - dx;

      for(i=0; i<=dx; ++i)
      {
         for(j=-half_width; j<half_width+width%2; ++j)
         {
							 DpyPixelDraw(pContext->pDisplay, x1, y1+j, pContext->ulForeground);
         }
         if(P < 0)
         {
            P  += 2*dy;
            x1 += addx;
         }
         else
         {
            P  += diff;
            x1 += addx;
            y1 += addy;
         }
      }
   }
   else
   {
      P = 2*dx - dy;
      diff = P - dy;

      for(i=0; i<=dy; ++i)
      {
         if(P < 0)
         {
            P  += 2*dx;
            y1 += addy;
         }
         else
         {
            P  += diff;
            x1 += addx;
            y1 += addy;
         }
         for(j=-half_width; j<half_width+width%2; ++j)
         {
							 DpyPixelDraw(pContext->pDisplay, x1+j, y1, pContext->ulForeground);
         }
      }
   }
}

const tDisplay ili9320 =
{
    sizeof(tDisplay),
    0,
    320,
    240,
    ili9320PixelDraw,
    ili9320DrawMultiple,
    ili9320LineDrawH,
    ili9320LineDrawV,
    ili9320RectFill,
    ili9320ColorTranslate,
    ili9320Flush
};




/*********************************************************************************************************
      END FILE
*********************************************************************************************************/




