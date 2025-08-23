//*****************************************************************************************
// KHAI BAO THU VIEN
//*****************************************************************************************
#include "sys.h"
#include "widget.h"
#include "grlib.h"
#include "TFT9341.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"


extern tCanvasWidget g_nen_tab_about;

extern void WidgetAdd_home(void);
const char *g_list_about[30];	
void listbox_about(tWidget *pWidget,short sSelIndex);
void WidgetRemove_about(void);
void NUT_PLAY(tWidget *pWidget);
uint8_t tt_play=0;

//Canvas(g_time, &g_nen_tab_about, 0, 0, &ili9320, 20, 120, 206,    
//               26, (CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_LEFT), ClrBlack, 0, ClrWhite, 
//               &g_sFontCmss24b, "Set Time/Date:", 0, 0);  								 
//Canvas(g_bright, &g_nen_tab_system, 0, 0, &ili9320, 20, 120+30, 206,    
//               26, (CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_LEFT), ClrBlack, 0, ClrWhite, 
//               &g_sFontCmss24b, "Set Brightness:", 0, 0);  							
//Slider(g_Set_pwm_manghinh, &g_nen_tab_system, 0, 0,
//       &ili9320, 226, 120+30, 258, 26, 0, 100,
//       100, (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT), 
//			 0x3E65FF, 0x1F1F1F, 0,
//      ClrWhite, ClrWhite, &g_sFontCm18b, "100%", 0, 0, Set_pwm_manghinh);

//ImageButton(g_nut_save_system, &g_nen_tab_system, 0, 0, &ili9320, 510, 410,       
//									90, 35, ( IB_STYLE_FILL | IB_STYLE_RELEASE_NOTIFY | IB_STYLE_TEXT), ClrBlack, 0x86C6E8,    
//									0xB9E3FB, &g_sFontCmss18b, "Save", 0, 0,    
//									0, 0, 0, 0,        
//									0, NUT_SAVE_SYSTEM);
ImageButton(g_nut_play, &g_nen_tab_about, 0, 0, &ili9320, 30, 20,       
									69, 75, IB_STYLE_RELEASE_NOTIFY, 0, 0,    
									0, 0, 0, nut_prw, nut_prw_s,    
									0, 0, 0, 0,        
									0, NUT_PLAY);
									

ListBox(g_listbox_about, &g_nen_tab_about, 0, 0, &ili9320, 0, 0, 360, 337, (LISTBOX_STYLE_OUTLINE) , 0xD9F28A,
				0x00A2E8, ClrBlack, ClrWhite, ClrWhite, &g_sFontCmss30b, g_list_about, 30, 
				0, listbox_about);	
Canvas(g_nen_tab_about, WIDGET_ROOT, 0, 0, &ili9320, 0, 0, 361,    
               338, CANVAS_STYLE_IMG, 0, 0, 0, 
               0, 0, 0, 0);  	
void listbox_about(tWidget *pWidget,short sSelIndex)
{
}	
/*void NUT_SET_TIME(tWidget *pWidget)
{
	u16 i;
		tt_shift=2;
		for(i=0;i<30;i++)
			{
				chuoinhap[i]=00;
			}
	WidgetRemove_system();
	WidgetAdd_keyboard();
	banphim_123();
}
void Set_pwm_manghinh(tWidget *pWidget, long lValue)
{
	bien_pwm_manghinh=lValue;
	DO_SANG_NEN_LCD(bien_pwm_manghinh);
	sprintf(char_pwm_thietbi,"%02d",lValue); 
	strcat(char_pwm_thietbi,"%");
	SliderTextSet(&g_Set_pwm_manghinh,char_pwm_thietbi);
	
	WidgetPaint((tWidget *)&g_Set_pwm_manghinh);
	WidgetMessageQueueProcess();
}
void NUT_SAVE_SYSTEM(tWidget *pWidget)
{
	WriteFlash();
}
*/
void WidgetAdd_about(void)
{

		//WidgetAdd((tWidget *)&g_nen_tab_system, (tWidget *)&g_time);	
		//WidgetAdd((tWidget *)&g_nen_tab_system, (tWidget *)&g_bright);	
		//WidgetAdd((tWidget *)&g_nen_tab_system, (tWidget *)&g_Set_pwm_manghinh);	
	//	WidgetAdd((tWidget *)&g_nen_tab_system, (tWidget *)&g_nut_save_system);	
	/*	WidgetAdd((tWidget *)&g_nen_tab_about, (tWidget *)&g_listbox_about);	
		ListBoxClear(&g_listbox_about);
		ListBoxTextAdd(&g_listbox_about, "***************************************"); 
		ListBoxTextAdd(&g_listbox_about, "********* DO AN TOT NGHIEP *********"); 
		ListBoxTextAdd(&g_listbox_about, "***************************************"); 
		ListBoxTextAdd(&g_listbox_about, " "); 
		ListBoxTextAdd(&g_listbox_about, "DE TAI: HOP DIEU KHIEN THONG MINH"); 
		ListBoxTextAdd(&g_listbox_about, " "); 
		ListBoxTextAdd(&g_listbox_about, " "); 
		ListBoxTextAdd(&g_listbox_about, "GVHD: PHAN VAN HOAN"); 
		ListBoxTextAdd(&g_listbox_about, "SVTH: BUI QUANG THO - 11141423"); 
		ListBoxTextAdd(&g_listbox_about, "        PHAM VAN NHAN - 11141151"); 
		ListBoxTextAdd(&g_listbox_about, "***************************************"); 
		ListBoxTextAdd(&g_listbox_about, "************* Version 1.0.0 ************"); 
		ListBoxTextAdd(&g_listbox_about, "***************************************"); 
		ListBoxTextAdd(&g_listbox_about, " "); 
		ListBoxTextAdd(&g_listbox_about, " "); 
		ListBoxTextAdd(&g_listbox_about, " "); 
		ListBoxTextAdd(&g_listbox_about, " "); 
		ListBoxTextAdd(&g_listbox_about, " "); 
		ListBoxTextAdd(&g_listbox_about, " "); 
		ListBoxTextAdd(&g_listbox_about, " "); 
		ListBoxTextAdd(&g_listbox_about, " "); 
		ListBoxTextAdd(&g_listbox_about, " "); 
		ListBoxTextAdd(&g_listbox_about, " "); */
		WidgetAdd((tWidget *)&g_nen_tab_about, (tWidget *)&g_nut_play);
		WidgetAdd(WIDGET_ROOT,(tWidget *)&g_nen_tab_about);
		WidgetPaint((tWidget *)&g_nen_tab_about);
		WidgetMessageQueueProcess();
}
void WidgetRemove_about(void)
{
		WidgetRemove((tWidget *)&g_nen_tab_about);
		LCD_Clear(DPYCOLORTRANSLATE(ClrBlack));
		WidgetMessageQueueProcess();
}


void NUT_PLAY(tWidget *pWidget) {

		WidgetRemove_about();
		WidgetAdd_home();
//	if(tt_play==0)
//	{
//		tt_play=1;
//		ImageButtonImageSet(&g_nut_play,nut_pause);
//		//ImageButtonImagePressedSet(&g_nut_play, nut_pause);
//	}
//	else
//	{
//		ImageButtonImageSet(&g_nut_play,nut_play);
//		tt_play=0;
//	}
//	
//	WidgetPaint((tWidget *)&g_nut_play);
//	WidgetMessageQueueProcess();
}
