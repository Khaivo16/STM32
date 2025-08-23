//*****************************************************************************************
// KHAI BAO THU VIEN
//*****************************************************************************************
#include "stm32f4xx.h"
#include "widget.h"
#include "grlib.h"
#include "TFT9341.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

extern tCanvasWidget g_nen_tab_home;

extern void WidgetAdd_about(void);
void WidgetRemove_home(void);
void NUT_OK(tWidget *pWidget);
void NUT_NEXT(tWidget *pWidget);
uint8_t tt_nut_ok=0,tt_nut_next=0;


ImageButton(g_nut_ok, &g_nen_tab_home, 0, 0, &ili9320, 30, 20,       
									69, 75, IB_STYLE_RELEASE_NOTIFY, 0, 0,    
									0, 0, 0, nut_play, nut_play_s,    
									0, 0, 0, 0,        
									0, NUT_OK);
ImageButton(g_nut_next, &g_nen_tab_home, 0, 0, &ili9320, 130, 20,       
									69, 75, IB_STYLE_RELEASE_NOTIFY, 0, 0,    
									0, 0, 0, nut_next, nut_next_s,    
									0, 0, 0, 0,        
									0, NUT_NEXT);
									
Canvas(g_nen_tab_home, WIDGET_ROOT, 0, 0, &ili9320, 0, 0, 361,    
               338, CANVAS_STYLE_IMG, 0, 0, 0, 
               0, 0, 0, 0);  

void WidgetAdd_home(void)
{

		WidgetAdd((tWidget *)&g_nen_tab_home, (tWidget *)&g_nut_ok);
		WidgetAdd((tWidget *)&g_nen_tab_home, (tWidget *)&g_nut_next);
		WidgetAdd(WIDGET_ROOT,(tWidget *)&g_nen_tab_home);
		WidgetPaint((tWidget *)&g_nen_tab_home);
		WidgetMessageQueueProcess();
}
void WidgetRemove_home(void)
{
		WidgetRemove((tWidget *)&g_nen_tab_home);
		LCD_Clear(DPYCOLORTRANSLATE(ClrBlack));
		WidgetMessageQueueProcess();
}


void NUT_OK(tWidget *pWidget) {

		
	if(tt_nut_ok==0)
	{
		tt_nut_ok=1;
		ImageButtonImageSet(&g_nut_ok,nut_pause);		
	}
	else
	{
		ImageButtonImageSet(&g_nut_ok,nut_play);
		tt_nut_ok=0;
	}
	
	WidgetPaint((tWidget *)&g_nut_ok);
	WidgetMessageQueueProcess();
}

void NUT_NEXT(tWidget *pWidget) {

		 WidgetRemove_home();
		 WidgetAdd_about();
	
}


