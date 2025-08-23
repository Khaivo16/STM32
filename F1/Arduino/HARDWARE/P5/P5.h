#ifndef __P5_H__
#define __P5_H__

#include "stm32f1xx_hal.h"

#define MAX_BIT       5


//#define A         1   //B
//#define B         0   //B
//#define C         10  //B
//#define D         5   //B2\

//#define R2        11
//#define B2        8
//#define G2        10

//#define R1        2
//#define B1        5
//#define G1        3
///////////////////////////////////////////////////
#define LAT       12  //B
#define OE        11  //B
#define clk       7  //A7

#define A         1   //B
#define B         0   //B
#define C         10  //B
#define D         2   //B2

#define R2        5//A5
#define G2        4//A4
#define B2        6//A6


#define R1        1//A/1
#define G1        2//A2
#define B1        3//A3


#define OE_P       GPIOB
#define xuat_P     GPIOB
#define clk_P      GPIOA
#define Control_P  GPIOB
#define data_PORT  GPIOA

#define P5_W  127
#define P5_H  31

#define S (unsigned char [])

void ngatquetled(void);
void math(int hang);
void hang(char so);
void SET_dosang(uint16_t sang);
uint16_t GET_dosang(void);

void P5_chonvitri(int x,int y);
void P5_sendnumber_font(char x,char y,char num,int font); //gui so ra man hinh;
char readbit_font1(int x,int y,char num,int font);
char readbit_font2(int x,int y,char num,int font);
char readbit_font3(int x,int y,unsigned char txt);
char readbit_font4(int x,int y,unsigned char txt);
char readbit_font5(int x,int y,unsigned char txt);
void Set_color(char cR,char cG,char cB);
void P5_sendnumber_font1(char x,char y,char num,int font);
void P5_sendnumber_font2(char x,char y,char num,int font);
void P5_sendnumber_font4(char x,char y,char num);
void P5_sendnumber_font5(char x,char y,char num);
void P5_sendnumber_F5(unsigned char num,unsigned char cR,unsigned char cG,unsigned char cB);
void P5_sendnumber_textfont5(unsigned char *s, unsigned char *c);

void P5_sendtext_wColor(unsigned char txt,char cR,char cG,char cB);
void P5_sendString_wColor(unsigned char *s,char cR,char cG,char cB);
void P5_sendString_wColorRun(int vitri,char kt);
void P5_sendtext(unsigned char txt);
void P5_sendString(unsigned char *s);
void P5_clear_alltext(void);
void P5_SetStringRun(unsigned char *s);
unsigned char P5_GetStringRun(int vitri);
void P5_SetStringRunY(int y);
void P5_StringRun(void);
void Getchu(int vitri);
void String_runSetTxt(int vitri,unsigned char txt);
void String_runSetColor(int txt,char cR,char cG,char cB);
unsigned char String_runGetColor(int vitri,int mau);
void String_runSetALLColor(char cR,char cG,char cB);
void P5_image(int sizex,int sizey,int x,int y,unsigned char *img); //in anh ra man hinh
void lattrang_number_font1(int x,int y,int sizex,int sizey,int vitri_y,int num,int font);
void lattrang_number_font2(int x,int y,int sizex,int sizey,int vitri_y,int num,int font);

void P5_sendtextFontAS57_wColor(unsigned char txt,char cR,char cG,char cB);
void P5_sendStringFontAS57_wColor(unsigned char *s,char cR,char cG,char cB);
void display_QR_code(unsigned char txt,char cR,char cG,char cB);


void set_px(int x, int y, char cR,char cG,char cB);
void P5_veduongngang(unsigned char x, unsigned char y,unsigned char dodai,char cR,char cG,char cB);
void P5_veduongdoc(unsigned char x, unsigned char y,unsigned char dodai,char cR,char cG,char cB);
void P5_veduongthang(int x, int y,int x1, int y1,char cR,char cG,char cB);
void Clock_Analog_BackGround(void);
void Clock_Analog(int gio,int phut,int giay,unsigned char *c);

int Random(int n);
void P5_ve_hinh_tron_kin(int x0,int y0,int r, char cR,char cG,char cB);
void P5_ve_hinh_tron(int x0,int y0,int r,char cR,char cG,char cB) ;
void P5_outtro(char effect);
void P5_clear(void);
void P5_clearTOP(void);
void P5_vehinhchunhat_kin(unsigned char x, unsigned char y,unsigned char rong, unsigned char cao,char cR,char cG,char cB);
void P5_vehinhchunhat(unsigned char x, unsigned char y,unsigned char rong, unsigned char cao,char cR,char cG,char cB);
void P5_vehinhchunhat_Vien(unsigned char x, unsigned char y,unsigned char rong, unsigned char cao,char cR,char cG,char cB);
void P5_special(char i);
	
#endif  //__P5_H__
