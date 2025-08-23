#ifndef __ili9320_H__
#define __ili9320_H__
#include "stm32f10x.h"                    
#include "grlib.h"
#include "widget.h"

#define Set_Cs  GPIOD->BSRR  = 0x00001000;
#define Clr_Cs  GPIOD->BRR   = 0x00001000;
#define Set_Rs  GPIOD->BSRR  = 0x00002000;
#define Clr_Rs  GPIOD->BRR   = 0x00002000;
#define Set_nWr GPIOD->BSRR  = 0x00004000;
#define Clr_nWr GPIOD->BRR   = 0x00004000;
#define Set_nRd GPIOD->BSRR  = 0x00008000;
#define Clr_nRd GPIOD->BRR   = 0x00008000;
#define DPYCOLORTRANSLATE(c)    ((((c) & 0x00f80000) >> 8) | (((c) & 0x0000fc00) >> 5) |(((c) & 0x000000f8) >> 3)) 
 void Lcd_Configuration(void)
{ 
   GPIO_InitTypeDef GPIO_InitStructure;
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE, ENABLE);  
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOE, &GPIO_InitStructure);
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
   GPIO_Init(GPIOD, &GPIO_InitStructure);
} 
 void ili9320_Delay(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}
 void ili9320_WriteIndex(u16 idx)
{
    Clr_Rs;
   Set_nRd;
   GPIOE->ODR = idx;
   Clr_nWr;
   Set_nWr;
}
 void ili9320_WriteData(u16 data)
{
   Set_Rs;
   Set_nRd;
     GPIOE->ODR = data;
   Clr_nWr;
   Set_nWr;
}
 u16 ili9320_ReadData(void)
{
   unsigned short val = 0;
   Set_Rs;
   Set_nWr;
   Clr_nRd;
   GPIOE->CRH = 0x44444444;
   GPIOE->CRL = 0x44444444;
   val = GPIOE->IDR;
   val = GPIOE->IDR;
   GPIOE->CRH = 0x33333333;
   GPIOE->CRL = 0x33333333;
   Set_nRd;
   return val;
}
 void ili9320_WriteRegister(u16 index,u16 dat)
{
   Clr_Cs;
   ili9320_WriteIndex(index);      
   ili9320_WriteData(dat);    
   Set_Cs; 
}
 u16 ili9320_ReadRegister(u16 index)
{ 
    Clr_Cs;
   ili9320_WriteIndex(index);     
   index = ili9320_ReadData();         
   Set_Cs;
   return index;
}
 void ili9320_SetCursor(u16 x,u16 y)
{
       ili9320_WriteRegister(0x004e,y);       
       ili9320_WriteRegister(0x004f,x);  
}
 void ili9320_SetWindows(u16 StartX,u16 StartY,u16 EndX,u16 EndY)
{
     ili9320_SetCursor(StartX,StartY);
     ili9320_WriteRegister(0x44,StartY|(EndY<<8));
     ili9320_WriteRegister(0x45,StartX);
     ili9320_WriteRegister(0x46,EndX);
}
 void ili9320_Clear(unsigned int dat)
{
  u32  i;

  ili9320_SetCursor(0x0000, 0x0000);  
  Clr_Cs; 
  ili9320_WriteIndex(0x0022);    
  for(i=0;i<76800;i++)
  {
    ili9320_WriteData(dat);
  }
  Set_Cs;
}

void ili9320_Init()
{
      Lcd_Configuration() ; 
      ili9320_Delay(50000); 
         //************* Start Initial Sequence **********//
      ili9320_WriteRegister(0x00, 0x0001); // Start internal OSC.
      ili9320_WriteRegister(0x01, 0x3B3F); // Driver output control, RL=0;REV=1;GD=1;BGR=0;SM=0;TB=1
      ili9320_WriteRegister(0x02, 0x0600); // set 1 line inversion
      //************* Power control setup ************/
      ili9320_WriteRegister(0x0C, 0x0007); // Adjust VCIX2 output voltage
      ili9320_WriteRegister(0x0D, 0x0006); // Set amplitude magnification of VLCD63
      ili9320_WriteRegister(0x0E, 0x3200); // Set alternating amplitude of VCOM
      ili9320_WriteRegister(0x1E, 0x00BB); // Set VcomH voltage
      ili9320_WriteRegister(0x03, 0x6A64); // Step-up factor/cycle setting
      //************ RAM position control **********/
      ili9320_WriteRegister(0x0F, 0x0000); // Gate scan position start at G0.
      ili9320_WriteRegister(0x44, 0xEF00); // Horizontal RAM address position
      ili9320_WriteRegister(0x45, 0x0000); // Vertical RAM address start position
      ili9320_WriteRegister(0x46, 0x013F); // Vertical RAM address end position
      // ----------- Adjust the Gamma Curve ----------//
      ili9320_WriteRegister(0x30, 0x0000);
      ili9320_WriteRegister(0x31, 0x0706);
      ili9320_WriteRegister(0x32, 0x0206);
      ili9320_WriteRegister(0x33, 0x0300);
      ili9320_WriteRegister(0x34, 0x0002);
      ili9320_WriteRegister(0x35, 0x0000);
      ili9320_WriteRegister(0x36, 0x0707);
      ili9320_WriteRegister(0x37, 0x0200);
      ili9320_WriteRegister(0x3A, 0x0908);
      ili9320_WriteRegister(0x3B, 0x0F0D);
      //************* Special command **************/  (0x6830&0xff00)|0x28
      ili9320_WriteRegister(0x28, 0x0006); // Enable test command
      ili9320_WriteRegister(0x2F, 0x12EB); // RAM speed tuning
      ili9320_WriteRegister(0x26, 0x7000); // Internal Bandgap strength
      ili9320_WriteRegister(0x20, 0xB0E3); // Internal Vcom strength
      ili9320_WriteRegister(0x27, 0x0044); // Internal Vcomh/VcomL timing
      ili9320_WriteRegister(0x2E, 0x7E45); // VCOM charge sharing time  
      //************* Turn On display ******************/
      ili9320_WriteRegister(0x10, 0x0000); // Sleep mode off.
      ili9320_Delay(50000); // Wait 30mS
      ili9320_WriteRegister(0x11, 0x6870); // Entry mode setup. 262K type B, take care on the data bus with 16it only
      ili9320_WriteRegister(0x07, 0x0033); // Display ON   */
      ili9320_Delay(10000);
      ili9320_Clear(DPYCOLORTRANSLATE(ClrBlack));
}
 void ili9320PixelDraw(void *pvDisplayData, long x, long y,unsigned long point)
{
  if ( (x>320)||(y>240) ) return;
  ili9320_SetCursor(319-x,y);
  Clr_Cs;
  ili9320_WriteIndex(0x0022);
  ili9320_WriteData(point);
  Set_Cs;
}

 void ili9320DrawMultiple(void *pvDisplayData, long lX,
                                           long lY, long lX0, long lCount,
                                           long lBPP,
                                           const unsigned char *pucData,
                                           const unsigned char *pucPalette)
{    unsigned int ulByte;
     ili9320_WriteRegister(0x11,0x6808); 
	   ili9320_SetCursor(319-lX,lY);
		 Clr_Cs;
     ili9320_WriteIndex(0x0022);
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
                    ili9320_WriteData(((unsigned long *)pucPalette)[(ulByte >>
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
                         ili9320_WriteData(DPYCOLORTRANSLATE(ulByte));

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
                             ili9320_WriteData(DPYCOLORTRANSLATE(ulByte));

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
                 ili9320_WriteData(DPYCOLORTRANSLATE(ulByte));
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
                 ili9320_WriteData(usByte);
            }
        }
    }
   Set_Cs;   ili9320_SetWindows(0,0,319,239);
}

 void ili9320RectFill(void *pvDisplayData, const tRectangle *pRect,unsigned long ulValue)
{
    long lCount;
    ili9320_SetWindows(319-pRect->sXMax,pRect->sYMin,319-pRect->sXMin,pRect->sYMax);
    Clr_Cs;
    ili9320_WriteIndex(0x0022);
    for(lCount = ((pRect->sXMax - pRect->sXMin + 1) *(pRect->sYMax - pRect->sYMin + 1)); lCount >= 0; lCount--)ili9320_WriteData(ulValue);
    Set_Cs;  
    ili9320_SetWindows(0,0,319,239);
}

 void ili9320LineDrawH(void *pvDisplayData, long lX1, long lX2,
                                   long lY, unsigned long ulValue)
{    ili9320_WriteRegister(0x11,0x6828); 
	   ili9320_SetCursor(319-lX2,lY);
     Clr_Cs;
     ili9320_WriteIndex(0x0022);
     while(lX1++ <= lX2)ili9320_WriteData(ulValue);
     Set_Cs;
	   
}

 void ili9320LineDrawV(void *pvDisplayData, long lX, long lY1,
                                   long lY2, unsigned long ulValue)
{ ili9320_WriteRegister(0x11,0x6820); 
  ili9320_SetCursor(319-lX,lY2);
  Clr_Cs;
  ili9320_WriteIndex(0x0022);
  while(lY1++ <= lY2)ili9320_WriteData(ulValue);
  Set_Cs;
}

 
static unsigned long ili9320ColorTranslate(void *pvDisplayData,unsigned long c)
{return(DPYCOLORTRANSLATE(c));}

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
	 u16 *bitmap_ptr = (u16*)mang,i;
	 ili9320_SetCursor(x0,hang);
	 ili9320_WriteRegister(0x11,0x6818); 
   Clr_Cs;
   ili9320_WriteIndex(0x0022);
   for(i=0;i<=d0;i++)ili9320_WriteData(*bitmap_ptr++);
   Set_Cs; 
}
void GrTriangleDrawFill(const tContext *pContext,u16 x1,u16 y1, u16 x2, u16 y2, u16 x3, u16 y3)
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
#include "canvas.h"
#include "container.h"
#include "checkbox.h"
#include "imgbutton.h"
#include "listbox.h"
#include "pushbutton.h"
#include "radiobutton.h"
#include "slider.h"
#include "touch.h"
//#include "bmp.h"
#endif 


