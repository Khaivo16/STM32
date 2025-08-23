#include "TOUCH_7846.h"
#include <stdlib.h>
#include <math.h>

#include <SPI.h>

Pen_Holder Pen_Point;

uint8_t TOUCH=0;
uint16_t TAM_X0=0,TAM_Y0=0;
long (*g_pfnTSHandler)(unsigned long ulMessage, long lX, long lY);

unsigned char flag=0;

														
unsigned char SPI_WriteByte(uint8_t num)    
{  
  return SPI_3.transfer(num); 	 				   
} 	

void SpiDelay(unsigned int DelayCnt)
{
 unsigned int i;
 for(i=0;i<DelayCnt;i++);
}

uint16_t TPReadX(void)
{ 
   uint16_t x=0;
   T_CS();
   SpiDelay(10);
   SPI_WriteByte(0x90);
   SpiDelay(10);  
   x=SPI_WriteByte(0xFF);
   x<<=8;
   x+=SPI_WriteByte(0x0);	
   T_DCS(); 
   //SpiDelay(10);    
   x=x>>4;
   x  = x&0xFFF; //fff
   return (x);
}


uint16_t TPReadY(void)
{
  uint16_t y=0;
  T_CS();
  SpiDelay(10);
  SPI_WriteByte(0xd0);
  SpiDelay(10);	 
  y=SPI_WriteByte(0x0);
  y<<=8;
  y+=SPI_WriteByte(0x0); 
  T_DCS(); 
  y = y>>4;
  y = y&0xFFF;	 //fff
  return (y);
}

		   
uint8_t read_once(void)
{
   Pen_Point.X=TPReadX(); 
   Pen_Point.Y=TPReadY(); 
   return 1;
}	 

 


void TOUCH_Init(void)
{	
	pinMode(PA15,OUTPUT);
	pinMode(PB15,INPUT_PULLUP); 	
	T_DCS();
	SPI_3.begin();
  SPI_3.setClock(2000000); // 2 MHz (speed!)
}

  	   
uint8_t Read_Ads7846(void)
{
       uint8_t t,t1,count=0;
       uint16_t databuffer[2][10];
       uint16_t temp=0;         
                         
        if(digitalRead(PB15)==0)        
        {
          read_once(); 
       while(count<10)                                             
       {                               
          {        if(read_once())
                  {          
                       databuffer[0][count]=Pen_Point.X;
                       databuffer[1][count]=Pen_Point.Y;
                       count++;  
                   }
               }
       }

       if(count==10)
       {  
           do
               {        
                       t1=0;                  
                       for(t=0;t<count-1;t++)
                       {
                               if(databuffer[0][t]>databuffer[0][t+1])
                               {
                                       temp=databuffer[0][t+1];
                                       databuffer[0][t+1]=databuffer[0][t];
                                       databuffer[0][t]=temp;
                                       t1=1; 
                               }  
                       }
               }while(t1);           
               do
               {        
                       t1=0;                 
                       for(t=0;t<count-1;t++)
                       {
                               if(databuffer[1][t]>databuffer[1][t+1])
                               {
                                       temp=databuffer[1][t+1];
                                       databuffer[1][t+1]=databuffer[1][t];
                                       databuffer[1][t]=temp;
                                       t1=1;         
                               }  
                       }
               }while(t1);                   
                                           
               Pen_Point.X=2047-((databuffer[0][3]+databuffer[0][4]+databuffer[0][5])/3);
               Pen_Point.Y=((databuffer[1][3]+databuffer[1][4]+databuffer[1][5])/3);         
               flag=1;
               return 1;                                                                           
       }
       flag=0;
        }
       return 0;
        
}


//void Drow_Touch_Point(uint16_t x,uint16_t y)
//{
//	LCD_DrawUniLine(x-12,y,x+13,y);
//	LCD_DrawUniLine(x,y-12,x,y+13);
//	Pixel(x+1,y+1,BLUE);
//	Pixel(x-1,y+1,BLUE);
//	Pixel(x+1,y-1,BLUE);
//	Pixel(x-1,y-1,BLUE);
//	LCD_DrawCircle(x,y,6);
//}	  
//void Draw_Big_Point(uint16_t x,uint16_t y)
//{	    
//	Pixel(x,y,BLUE);
//	Pixel(x+1,y,BLUE);
//	Pixel(x,y+1,BLUE);
//	Pixel(x+1,y+1,BLUE);
//}		
 uint8_t Convert_Pos(void)
{                    
               Read_Ads7846();
               Pen_Point.Y0=240-(int)((Pen_Point.Y-103)/7.7);
               Pen_Point.X0=330-(int)((Pen_Point.X-104)/5.56); 
               if(Pen_Point.X0>320)
               {
                 Pen_Point.X0=320;
               }
               if(Pen_Point.Y0>240)
               {
                 Pen_Point.Y0=240;
               }
				return	1;													 
} 


//void Touch_Adjust(void)
//{								 
//	uint16_t pos_temp[4][2];
//	uint8_t  cnt=0;	
//	uint16_t d1,d2;
//	uint32_t tem1,tem2;
//	float fac; 	   
//	cnt=0;				
//	LCD_SetTextColor(BLUE);
//	LCD_SetTextColor(WHITE);
//	LCD_Clear(WHITE);
//	Drow_Touch_Point(20,20);
//	Pen_Point.xfac=0;
//	while(1)
//	{
//                if(1==1)
//		//if(Pen_Point.Key_Sta==Key_Down)
//		{
//			if(Read_Ads7846())
//			{  								   
//				pos_temp[cnt][0]=Pen_Point.X;
//				pos_temp[cnt][1]=Pen_Point.Y;
//				cnt++;
//                                SpiDelay(0xFFFF);
//			}			 
//			switch(cnt)
//			{			   
//				case 1:
//					LCD_Clear(WHITE);
//					Drow_Touch_Point(210,20);
//					break;
//				case 2:
//					LCD_Clear(WHITE);
//					Drow_Touch_Point(20,300);
//					break;
//				case 3:
//					LCD_Clear(WHITE);
//					Drow_Touch_Point(210,300);
//					break;
//				case 4:
//					tem1=abs(pos_temp[0][0]-pos_temp[1][0]);
//					tem2=abs(pos_temp[0][1]-pos_temp[1][1]);
//					tem1*=tem1;
//					tem2*=tem2;
//					d1=sqrt(tem1+tem2);
//					
//					tem1=abs(pos_temp[2][0]-pos_temp[3][0]);
//					tem2=abs(pos_temp[2][1]-pos_temp[3][1]);
//					tem1*=tem1;
//					tem2*=tem2;
//					d2=sqrt(tem1+tem2);
//					fac=(float)d1/d2;
//					if(fac<0.75||fac>1.25||d1==0||d2==0)
//					{
//						cnt=0;
//						LCD_Clear(WHITE);
//						Drow_Touch_Point(20,20);
//						continue;
//					}
//					tem1=abs(pos_temp[0][0]-pos_temp[2][0]);
//					tem2=abs(pos_temp[0][1]-pos_temp[2][1]);
//					tem1*=tem1;
//					tem2*=tem2;
//					d1=sqrt(tem1+tem2);
//					
//					tem1=abs(pos_temp[1][0]-pos_temp[3][0]);
//					tem2=abs(pos_temp[1][1]-pos_temp[3][1]);
//					tem1*=tem1;
//					tem2*=tem2;
//					d2=sqrt(tem1+tem2);
//					fac=(float)d1/d2;
//					if(fac<0.75||fac>1.25)
//					{
//						cnt=0;
//						LCD_Clear(WHITE);
//						Drow_Touch_Point(20,20);
//						continue;
//					}
//					tem1=abs(pos_temp[1][0]-pos_temp[2][0]);
//					tem2=abs(pos_temp[1][1]-pos_temp[2][1]);
//					tem1*=tem1;
//					tem2*=tem2;
//					d1=sqrt(tem1+tem2);
//	
//					tem1=abs(pos_temp[0][0]-pos_temp[3][0]);
//					tem2=abs(pos_temp[0][1]-pos_temp[3][1]);
//					tem1*=tem1;
//					tem2*=tem2;
//					d2=sqrt(tem1+tem2);
//					fac=(float)d1/d2;
//					if(fac<0.75||fac>1.25)
//					{
//						cnt=0;
//						LCD_Clear(WHITE);
//						Drow_Touch_Point(20,20);
//						continue;
//					}
//					Pen_Point.xfac=(float)200/(pos_temp[1][0]-pos_temp[0][0]);
//					Pen_Point.xoff=(240-Pen_Point.xfac*(pos_temp[1][0]+pos_temp[0][0]))/2;

//					Pen_Point.yfac=(float)280/(pos_temp[2][1]-pos_temp[0][1]);
//					Pen_Point.yoff=(320-Pen_Point.yfac*(pos_temp[2][1]+pos_temp[0][1]))/2;
//					LCD_Clear(WHITE);
//					return ;


//			}
//		}
//	}
//}

/////////////////////////////////////////////////////////////////////////////////////////
uint8_t TOADO_CHAM(uint16_t TOADO_X1,uint16_t TOADO_Y1,uint16_t TOADO_X2,uint16_t TOADO_Y2)
{

		if((Pen_Point.X0>TOADO_X1)&&(Pen_Point.Y0>TOADO_Y1)&&(Pen_Point.X0<TOADO_X2)&&(Pen_Point.Y0<(TOADO_Y2)))
		{
			return 1;
		}
	return 0;
}
uint8_t SOSANH_TOADO(uint16_t TOADO_X,uint16_t TOADO_Y)
{
	if(((Pen_Point.X0!=TOADO_X)||(Pen_Point.Y0!=TOADO_Y)))
	{
		return 1;
	}
	return 0;
}

uint8_t touch_scan()
{ uint8_t tt_tam;
			Pen_Point.TouchDetected =XPT2046_Press();
			tt_tam=!Pen_Point.TouchDetected;
			Convert_Pos();	 // QUET CAM UNG
						if((digitalRead(PB15)==0)&&(tt_tam==1)&&(TOUCH==0)) // NHAN XUONG
						{	
							if((TOADO_CHAM(0,0,319,239)))
							{
							TAM_X0 = Pen_Point.X0;
							TAM_Y0 = Pen_Point.Y0;
							g_pfnTSHandler(WIDGET_MSG_PTR_DOWN, TAM_X0, TAM_Y0);WidgetMessageQueueProcess();
								TOUCH=1;//SSD1289_Text(76,20,"Development",WHITE,RED);
							}
						}
						else if((digitalRead(PB15)==0)&&(tt_tam==1)&&(TOUCH==1)) // DI CHUYEN
						{
							if((TOADO_CHAM(0,0,319,239)))
							{
								if(SOSANH_TOADO(TAM_X0,TAM_Y0)) 
								{//SSD1289_Text(76,100,"Development Board",WHITE,RED);
									TAM_X0 = Pen_Point.X0;
									TAM_Y0 = Pen_Point.Y0;
									g_pfnTSHandler(WIDGET_MSG_PTR_MOVE, TAM_X0, TAM_Y0);WidgetMessageQueueProcess();
								}	
							}
						}
						else if((digitalRead(PB15)==1)&&(tt_tam==0)&&(TOUCH==1)) // NHA PHIM
							{
								//if(TOADO_CHAM(0,0,111,35,128)==0)
								//{
									g_pfnTSHandler(WIDGET_MSG_PTR_UP, TAM_X0, TAM_Y0);WidgetMessageQueueProcess();
								TOUCH=0;//SSD1289_Text(76,120,"Development Board V1.0",WHITE,RED);
								//}
							}
							return tt_tam;
}

void TouchScreenCallbackSet(long (*pfnCallback)(unsigned long ulMessage, long lX,long lY))
{
    g_pfnTSHandler = pfnCallback;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////


static uint16_t read_IRQ(void)
{
  return digitalRead(PB15);
}

uint16_t XPT2046_Press(void)
{
  return read_IRQ();
}

/**
  * @brief  Returns Status and positions of the Touch screen.
  * @param  None
  * @retval Pointer to TS_STATE structure holding Touch Screen information.
  */
//Pen_Holder* IOE_TS_GetState(void)
//{
////  uint32_t xDiff, yDiff , x , y;
////  static uint32_t _x = 0, _y = 0;
////  
////  /* Check if the Touch detect event happened */
////   
////	Pen_Point.TouchDetected =XPT2046_Press();
////  		
////  if(Pen_Point.TouchDetected==0)
////  {	
////		Convert_Pos();
////    x=Pen_Point.X0,y=Pen_Point.Y0;
////    xDiff = x > _x? (x - _x): (_x - x);
////    yDiff = y > _y? (y - _y): (_y - y);       
////    if (xDiff + yDiff > 5)
////    {	
////      _x = x;
////      _y = y;       
////    }
////  }  
////  /* Update the X position */
////  Pen_Point.X0 = _x;
////    
////  /* Update the Y position */  
////  Pen_Point.Y0 = _y;
////    
////  
////  /* Return pointer to the updated structure */
////  return &Pen_Point; 
//}
