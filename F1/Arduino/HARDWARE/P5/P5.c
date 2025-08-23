#include "P5.h"
#include "font.h"
#include "math.h"
#include "stdlib.h"
#include "Arduino.h"
extern TIM_HandleTypeDef htim2;

unsigned int const timer_clock[8]={15,30,60,320,640,1280}; // he so chia nap vao timer
unsigned char vitri_bit=0,row=0; // kiem tra xem dang quet den bit nao
unsigned char hang_led;
uint16_t led[MAX_BIT][128];
unsigned char display[3][32][128];

unsigned char *displayX=&display[0][0][0];

signed int P5_x,P5_y;
unsigned char COLOR[2][3]={0,0,0,0,0,0};

int dosang;

int String_RUN_Y=0; //vi tri chu chay theo truc Y 
#define String_maxHEIGHT 16
#define String_maxLEN 128
unsigned char String_RUN[String_maxLEN];
unsigned char String_COLOR[3][String_maxLEN];

//---------------------------------Chuong trinh quet LED ma tran --------------------------------------------------------//

// bat sang hang tu 1->16 ( do tam matrix P5 quet 1/16) (1 thoi diem chi co duy nhat 1 hang duoc sang len)
void hang(char so)
{
	switch(so)
	{
		case 0:Control_P->BSRR=(1<<(D+16));Control_P->BSRR=(1<<(C+16));Control_P->BSRR=(1<<(B+16));Control_P->BSRR=(1<<(A+16));break;
		case 1:Control_P->BSRR=(1<<(D+16));Control_P->BSRR=(1<<(C+16));Control_P->BSRR=(1<<(B+16));Control_P->BSRR=(1<<A);break;
		case 2:Control_P->BSRR=(1<<(D+16));Control_P->BSRR=(1<<(C+16));Control_P->BSRR=(1<<B);Control_P->BSRR=(1<<(A+16));break;
		case 3:Control_P->BSRR=(1<<(D+16));Control_P->BSRR=(1<<(C+16));Control_P->BSRR=(1<<B);Control_P->BSRR=(1<<A);break;
		case 4:Control_P->BSRR=(1<<(D+16));Control_P->BSRR=(1<<C);Control_P->BSRR=(1<<(B+16));Control_P->BSRR=(1<<(A+16));break;
		case 5:Control_P->BSRR=(1<<(D+16));Control_P->BSRR=(1<<C);Control_P->BSRR=(1<<(B+16));Control_P->BSRR=(1<<A);break;
		case 6:Control_P->BSRR=(1<<(D+16));Control_P->BSRR=(1<<C);Control_P->BSRR=(1<<B);Control_P->BSRR=(1<<(A+16));break;
		case 7:Control_P->BSRR=(1<<(D+16));Control_P->BSRR=(1<<C);Control_P->BSRR=(1<<B);Control_P->BSRR=(1<<A);break;
		case 8:Control_P->BSRR=(1<<D);Control_P->BSRR=(1<<(C+16));Control_P->BSRR=(1<<(B+16));Control_P->BSRR=(1<<(A+16));break;
		case 9:Control_P->BSRR=(1<<D);Control_P->BSRR=(1<<(C+16));Control_P->BSRR=(1<<(B+16));Control_P->BSRR=(1<<A);break;
		case 10:Control_P->BSRR=(1<<D);Control_P->BSRR=(1<<(C+16));Control_P->BSRR=(1<<B);Control_P->BSRR=(1<<(A+16));break;
		case 11:Control_P->BSRR=(1<<D);Control_P->BSRR=(1<<(C+16));Control_P->BSRR=(1<<B);Control_P->BSRR=(1<<A);break;
		case 12:Control_P->BSRR=(1<<D);Control_P->BSRR=(1<<C);Control_P->BSRR=(1<<(B+16));Control_P->BSRR=(1<<(A+16));break;
		case 13:Control_P->BSRR=(1<<D);Control_P->BSRR=(1<<C);Control_P->BSRR=(1<<(B+16));Control_P->BSRR=(1<<A);break;
		case 14:Control_P->BSRR=(1<<D);Control_P->BSRR=(1<<C);Control_P->BSRR=(1<<B);Control_P->BSRR=(1<<(A+16));break;
		case 15:Control_P->BSRR=(1<<D);Control_P->BSRR=(1<<C);Control_P->BSRR=(1<<B);Control_P->BSRR=(1<<A);break;
	}
}
void math(int hang)
{  
	for(int ts=0;ts<MAX_BIT;ts++)
	{
	 for(int i=0;i<128;i++)
	 {
		 led[ts][i]=0;
		 if((display[1][16+hang][i] & (1<<ts)) != 0 ){led[ts][i]|=(1<<R2);} //ghi bit4  G2 
		 if((display[1][0+hang][i] & (1<<ts)) != 0 ){led[ts][i]|=(1<<G1);}   //ghi bit2  G1
		 if((display[0][0+hang][i] & (1<<ts)) != 0 ){led[ts][i]|=(1<<R1);}   //ghi bit1  R1
		 if((display[2][0+hang][i] & (1<<ts)) != 0 ){led[ts][i]|=(1)<<B1;}   //ghi bit3  B1
		 if((display[0][16+hang][i] & (1<<ts)) != 0 ){led[ts][i]|=(1)<<G2;} //ghi bit5  R2
		 if((display[2][16+hang][i] & (1<<ts)) != 0 ){led[ts][i]|=(1)<<B2;} //ghi bit6  B2
	 }
	} 
}
void ngatquetled(void)
{
//	TIM4 -> PSC = timer_clock[vitri_bit]; 	 // nap gia tri chia moi 
//	TIM4->ARR=38;
//	TIM4->EGR = 1;                           //set EGR len 1 de load lai PSC
//	TIM4 ->SR = 0xFFFFFFFE;                  //xoa co ngat

	Control_P->BSRR=(1<<(LAT+16));	
TIM2->CCR4 = 0; //__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);
	for(int i=0;i<128;i++){data_PORT->ODR=led[vitri_bit][i];clk_P->BSRR=(1<<clk);}	 								
	hang(hang_led);
	Control_P->BSRR=(1<<LAT); // chot data
TIM2->CCR4 = dosang;//__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,dosang);
	
	vitri_bit++;
	if(vitri_bit==MAX_BIT){vitri_bit=0;hang_led++;if(hang_led==16)hang_led=0;math(hang_led);}	
}
void SET_dosang(uint16_t sang)
{
	 dosang=sang;
}
uint16_t GET_dosang(void)
{
	 return dosang;
}
// ------------------------CHuong trinh xu li do hoa ------------------------------------------------------//
void P5_chonvitri(int x,int y)
{
	P5_x  = x;
	P5_y  = y;
}
char readbit_font1(int x,int y,char num,int font)
{
	char temp = x%8;
	return (number_font1[font][num][y][x/8] & (0x80 >> temp)) >> (7-temp);
}
char readbit_font2(int x,int y,char num,int font)
{
	char temp = x%8;
	return (number_font2[font][num][y][x/8] & (0x80 >> temp)) >> (7-temp);
}
char readbit_font3(int x,int y,unsigned char txt)
{
	char temp = x%8;
	return (font11[txt][y][x/8] & (0x80 >> temp)) >> (7-temp);
}
char readbit_font4(int x,int y,unsigned char txt)
{
	char temp = x%8;
	return (number_font4[txt][y][x/8] & (0x80 >> temp)) >> (7-temp);
}
char readbit_font5(int x,int y,unsigned char txt)
{
	char temp = x%8;
	return (number_font5[txt][y][x/8] & (0x80 >> temp)) >> (7-temp);
}

#define arrD1 (32*64)
#define arrD2 (64)

char readbit_fontAS57(int x,int y,unsigned char txt)
{
	char temp = x%8;
	return (fontAs57[txt*7*1 + y*1 + x/8]& (0x80 >> temp)) >> (7-temp);
}
char readbit_fontQR(int x,int y,unsigned char txt)
{
	char temp = x%8;
	return (QR_code[txt*32*4 + y*4 + x/8]& (0x80 >> temp)) >> (7-temp);
}


void Set_color(char cR,char cG,char cB)
{
	COLOR[1][0] = cR;
	COLOR[1][1] = cG;
	COLOR[1][2] = cB;
}
void P5_sendtextFontAS57_wColor(unsigned char txt,char cR,char cG,char cB) 
{
	Set_color(cR,cG,cB);
	if(P5_y>=P5_H)return;
	if(P5_x>=P5_W)
	{
			 P5_x=0;
			 P5_y+=8;
	}
	if(txt == '\n')
	{
			 P5_x=0;
			 P5_y+=8;
		return;
	}
	int sizew=6;
	if(txt==' ')sizew=2;
	 for(int mau=0;mau<3;mau++)
	 {
		  for(int i=P5_y;i<7+P5_y;i++)
		  {
				 for(int k=P5_x;k<P5_x+sizew;k++)
				 {
					  if(k > P5_W)break;
					  displayX[mau*arrD1 + i*arrD2 + k] = COLOR[readbit_fontAS57(k-P5_x,i-P5_y,txt-32)][mau]; 
				 }
			}
	 }
	 P5_x += sizew;
}
void display_QR_code(unsigned char txt,char cR,char cG,char cB)
{
	Set_color(cR,cG,cB);
	for(int mau=0;mau<3;mau++)
	 {
		  for(int i=P5_y;i<32+P5_y;i++)
		  {
				 for(int k=P5_x;k<P5_x+32;k++)
				 {
					  if(k > P5_W)break;
					  displayX[mau*arrD1 + i*arrD2 + k] = COLOR[readbit_fontQR(k-P5_x,i-P5_y,txt)][mau];
				 }
			}
	 }
}
void P5_sendStringFontAS57_wColor(unsigned char *s,char cR,char cG,char cB) 
{
	while(*s)
	 {
		  P5_sendtextFontAS57_wColor(*s,cR,cG,cB);
		  s++;
	 }
}

//void Set_color(char cR,char cG,char cB)
//{
//	COLOR[1][0] = cR;
//	COLOR[1][1] = cG;
//	COLOR[1][2] = cB;
//}
void P5_sendnumber_font1(char x,char y,char num,int font) //gui so ra man hinh
{
	 char width=13;
	 if(num>=10)width=5;
	 for(int mau=0;mau<3;mau++)
	 {
		  for(int i=y;i<20+y;i++)
		  {
				 for(int k=x;k<width+x;k++)
				 {
					  if(COLOR[1][0] < 32)display[mau][i][k] = COLOR[readbit_font1(k-x,i-y,num,font)][mau];
					  else 
		        {
							if(readbit_font1(k-x,i-y,num,font)!=0)display[mau][i][k] = Color_Font[255-COLOR[1][0]][mau][i-y][k-x];
							else display[mau][i][k] = 0;
						}
				 }
			}
	 }
}
void P5_sendnumber_font2(char x,char y,char num,int font) //gui so ra man hinh
{
	 char width=9;
	 char f = 0;
	 if(font != 0 ) f=1;
	 if(num>=10)width=3;
	 for(int mau=0;mau<3;mau++)
	 {
		  for(int i=y;i<12+y;i++)
		  {
				 for(int k=x;k<width+x;k++)
				 {
					  display[mau][i][k] = COLOR[readbit_font2(k-x,i-y,num,f)][mau];
				 }
			}
	 }
}
void P5_sendnumber_font4(char x,char y,char num) //gui so ra man hinh
{
	 char width=7;
	 if(num==10)width=3;
	 for(int mau=0;mau<3;mau++)
	 {
		  for(int i=y;i<10+y;i++)
		  {
				 for(int k=x;k<width+x;k++)
				 {
					  display[mau][i][k] = COLOR[readbit_font4(k-x,i-y,num)][mau];
				 }
			}
	 }
}
void P5_sendnumber_font5(char x,char y,char num) //gui so ra man hinh
{
	 char width=6;
	 if(num==10)width=4;
	 for(int mau=0;mau<3;mau++)
	 {
		  for(int i=y;i<7+y;i++)
		  {
				 for(int k=x;k<width+x;k++)
				 {
					  display[mau][i][k] = COLOR[readbit_font5(k-x,i-y,num)][mau];
				 }
			}
	 }
}
void P5_sendnumber_F5(unsigned char num,unsigned char cR,unsigned char cG,unsigned char cB) //gui so ra man hinh
{
	 unsigned char CL[2][3]={0,0,0,cR,cG,cB};
	 char width=6;
	 if(num==10)width=4;
	 for(int mau=0;mau<3;mau++)
	 {
		  for(int i=P5_y;i<7+P5_y;i++)
		  {
				 for(int k=P5_x;k<width+P5_x;k++)
				 {
					  if(k>P5_W)break;
					  if(i>P5_H)break;
					  if(k<0)continue;
					  if(i<0)continue;
					  display[mau][i][k] = CL[readbit_font5(k-P5_x,i-P5_y,num)][mau];
				 }
			}
	 }
	 P5_x +=6;
}
void P5_sendnumber_textfont5(unsigned char *s, unsigned char *c)
{
	int i=0;
	while(*s)
	 {
		  P5_sendnumber_F5(*s -1,c[i],c[i+1],c[i+2]);
		  s++;
		  i+=3;
	 }
}
void P5_sendtext(unsigned char txt) //gui chu tieng Viet ra man hinh
{
	 if(P5_x==P5_W)return;
	 for(int mau=0;mau<3;mau++)
	 {
		  for(int i=P5_y;i<16+P5_y;i++)
		  {
				 for(int k=P5_x;k<P5_x+Width_font11[txt];k++)
				 {
					  if(k > P5_W)break;
					  display[mau][i][k] = COLOR[readbit_font3(k-P5_x,i-P5_y,txt)][mau];
				 }
			}
	 }
	 P5_x += Width_font11[txt];
	 if(P5_x>P5_W)P5_x=P5_W;
}
void P5_sendString(unsigned char *s)
{
	 while(*s)
	 {
		  P5_sendtext(*s);
		  s++;
	 }
}
void P5_sendtext_wColor(unsigned char txt,char cR,char cG,char cB) //gui chu tieng Viet ra man hinh
{
	  char cl[2][3]={0,0,0,cR,cG,cB};
		if(P5_x==P5_W)return;
		for(int mau=0;mau<3;mau++)
	  {
			for(int i=P5_y;i<16+P5_y;i++)
			{
				 for(int k=P5_x;k<P5_x+Width_font11[txt];k++)
				 {
						if(k > P5_W)break;
						display[mau][i][k] = cl[readbit_font3(k-P5_x,i-P5_y,txt)][mau];
				 }
			}
		}
	P5_x += Width_font11[txt];
	if(P5_x>P5_W)P5_x=P5_W;
}
void P5_sendString_wColor(unsigned char *s,char cR,char cG,char cB)
{
	 while(*s)
	 {
		  P5_sendtext_wColor(*s,cR,cG,cB);
		  s++;
	 }
}
void P5_sendString_wColorRun(int vitri,char kt)
{
	 int dem=0;
	 char W = Width_font11[String_RUN[vitri]];
	 if(W==0)W=16;
	 if(kt==1)
	 {

				for(int mau=0;mau<3;mau++)
					{
						for(int i=P5_y;i<16+P5_y;i++)
						{
							 for(int k=P5_x;k<P5_x+W;k++)
							 {
									if(i==13)display[mau][i][k] = 31;
								  else display[mau][i][k] = 0;
							 }
						}
					}
				P5_x += W;
				vitri++;dem++;

	 }
	 while(String_RUN[vitri]) 
	 {
		  P5_sendtext_wColor(String_RUN[vitri],String_runGetColor(dem,0),String_runGetColor(dem,1),String_runGetColor(dem,2));
		  vitri++;dem++;
		  if(dem>10)return;
	 }
}
int chu,kiemtraktdich;
void Getchu(int vitri)
{
	chu=vitri;
	kiemtraktdich=0;
}
void P5_clear_alltext(void)
{
	for(int i=0;i<String_maxLEN;i++) //copy data
	 {
		  String_RUN[i]=0;
	 }
}
void P5_SetStringRun(unsigned char *s)
{	 
	 for(int i=0;i<String_maxLEN;i++)String_RUN[i]=0;//clear data
	 for(int i=0;i<String_maxLEN;i++) //copy data
	 {
		  if((*s) == 0)break;
		  String_RUN[i] = (*s);
		  s++;
	 }
	 chu=kiemtraktdich=0;
}
unsigned char P5_GetStringRun(int vitri)
{
	return String_RUN[vitri];
}
void P5_SetStringRunY(int y)
{
	String_RUN_Y=y; 
}
void P5_StringRun(void)
{
	if(String_RUN[0] == 0) return;
	for(int mau=0;mau<3;mau++)
	{
	 for(int x=0;x<P5_W;x++)                       //dich trai toan bo mang du lieu
		 {         
				 for(int y=String_RUN_Y;y<String_maxHEIGHT+String_RUN_Y;y++)
				 {                   
						display[mau][y][x] = display[mau][y][x+1];
				 }   
		 }
	}
	
	//dua du lieu moi vao cot cuoi cung
	for(int mau=0;mau<3;mau++)
	{
		 for(int y=String_RUN_Y;y<String_maxHEIGHT+String_RUN_Y;y++)
				 {                   
					  if(readbit_font3(kiemtraktdich,y-String_RUN_Y,String_RUN[chu]) == 1)
						{
							display[mau][y][P5_W] = String_COLOR[mau][chu];
						}
						else display[mau][y][P5_W]=0;
				 } 
	}
	kiemtraktdich++;
	if(kiemtraktdich==Width_font11[String_RUN[chu]])
  {
		chu++;
		kiemtraktdich=0;
		if(String_RUN[chu] == 0)  //STOP
			{
				 chu=0;
			}
	}
}
void String_runSetTxt(int vitri,unsigned char txt)
{
	String_RUN[vitri] = txt;
}
void String_runSetColor(int txt,char cR,char cG,char cB)
{
	 String_COLOR[0][txt] = cR;
	 String_COLOR[1][txt] = cG;
	 String_COLOR[2][txt] = cB;
}
unsigned char String_runGetColor(int vitri,int mau)
{
	 return String_COLOR[mau][vitri];
}
void String_runSetALLColor(char cR,char cG,char cB)
{
	for(int i=0;i<String_maxLEN;i++)
	{
		 String_COLOR[0][i] = cR;
	   String_COLOR[1][i] = cG;
	   String_COLOR[2][i] = cB;
	}
}

void P5_image(int sizex,int sizey,int x,int y,unsigned char *img) //in anh ra man hinh
{
	for(int mau=0;mau<3;mau++)
	{
		for(int Start_x=x;Start_x<sizex+x;Start_x++)
		{
			  for(int Start_y=y;Start_y<sizey+y;Start_y++)
				{
					 display[mau][Start_y][Start_x] = img[(sizex*sizey*mau) + Start_x + (Start_y*sizex)];
				}
		}
	}
}


void lattrang_number_font1(int x,int y,int sizex,int sizey,int vitri_y,int num,int font)
{
	for(int i=x;i<x+sizex;i++)
	{
		 for(int k=y+sizey-1;k>y;k--)
		 {
			  display[0][k][i] = display[0][k-1][i];
			  display[1][k][i] = display[1][k-1][i];
			  display[2][k][i] = display[2][k-1][i];
		 }
	}
	if(vitri_y == 9999)
	{
		 for(int i=x;i<x+sizex;i++)
		 {
        display[0][y][i] = 0;
			  display[1][y][i] = 0;
			  display[2][y][i] = 0;
		 }
	}
	else
	{
		 for(int mau=0;mau<3;mau++)
		 for(int i=x;i<x+sizex;i++)
		 {
			  if(COLOR[1][0] < 32)display[mau][y][i] = COLOR[readbit_font1(i-x,vitri_y,num,font)][mau];
				else 
				{
					if(readbit_font1(i-x,vitri_y,num,font)!=0)display[mau][y][i] = Color_Font[255-COLOR[1][0]][mau][vitri_y][i-x];
					else display[mau][y][i] = 0;
				}
		 }
	}
}

void lattrang_number_font2(int x,int y,int sizex,int sizey,int vitri_y,int num,int font)
{
	for(int i=x;i<x+sizex;i++)
	{
		 for(int k=y+sizey-1;k>y;k--)
		 {
			  display[0][k][i] = display[0][k-1][i];
			  display[1][k][i] = display[1][k-1][i];
			  display[2][k][i] = display[2][k-1][i];
		 }
	}
	if(vitri_y == 9999)
	{
		 for(int i=x;i<x+sizex;i++)
		 {
        display[0][y][i] = 0;
			  display[1][y][i] = 0;
			  display[2][y][i] = 0;
		 }
	}
	else
	{
		 for(int mau=0;mau<3;mau++)
		 for(int i=x;i<x+sizex;i++)
		 {
			  display[mau][y][i] = COLOR[readbit_font2(i-x,vitri_y,num,font)][mau];
		 }
	}
}

//----------------------------------Cac ham de tao ra dong ho kim  analog ------------------------------------//

#define swap(a, b) {int t = a; a = b; b = t; }    // chuong trinh con ( hoan doi gia tri a va b) 
#define ONE_RADIAN 0.0174533

#define CLOCK_RADIUS  15
#define SECOND_HAND_SIZE  14
#define MINUTE_HAND_SIZE  10
#define HOUR_HAND_SIZE    6

void set_px(int x, int y, char cR,char cG,char cB)
{
	if(x<0 || y<0 || x>63 ||y>31)return;
	display[0][y][x]=cR;
	display[1][y][x]=cG;
	display[2][y][x]=cB;
}
void P5_veduongngang(unsigned char x, unsigned char y,unsigned char dodai,char cR,char cG,char cB)   // ve 1 duong ke ngang
{
  char dem;
  if(dodai>P5_W)dodai=P5_W;
  for(dem=0;dem<dodai;dem++)
	{
		 display[0][y][x+dem] = cR;
		 display[1][y][x+dem] = cG;
		 display[2][y][x+dem] = cB;
	}
} 

void P5_veduongdoc(unsigned char x, unsigned char y,unsigned char dodai,char cR,char cG,char cB)      // ve 1 duong ke doc
{
char dem;
  if(dodai>P5_H) dodai=P5_H;
  for(dem=0;dem<dodai;dem++)
	{
		 display[0][y+dem][x] = cR;
		 display[1][y+dem][x] = cG;
		 display[2][y+dem][x] = cB;
	}
}
void P5_veduongthang(int x, int y,int x1, int y1,char cR,char cG,char cB)
{
 int dx,dy,err,ystep; 
 int steep=abs(y1-y) > abs(x1-x);   
 int xtd,ytd;
 if(x==x1)
 {
  ytd=y1-y;  
  ytd=abs(ytd);
  if(y1>y)P5_veduongdoc(x,y,ytd+1,cR,cG,cB);    
  else P5_veduongdoc(x1,y1,ytd+1,cR,cG,cB);  
  return;
 } 
 
 if(y==y1)
 {    
  xtd=x1-x;
  xtd=abs(xtd);
  if(x1>x)P5_veduongngang(x,y,xtd+1,cR,cG,cB);    
  else P5_veduongngang(x1,y1,xtd+1,cR,cG,cB);
  return;
 }
 if (steep) 
  {
    swap(x, y);
    swap(x1, y1);
  }    
  
  if (x > x1) 
  {
    swap(x, x1);
    swap(y, y1);
  }
 dx=x1-x;
 dy=abs(y1-y);
 err=dx/2;  
  
  if (y < y1)ystep = 1;    
  else ystep = -1;
    
  for (; x<=x1; x++) 
  {
    if (steep)
		{
				 display[0][x][y] = cR;
				 display[1][x][y] = cG;
				 display[2][x][y] = cB;
		}
    else 
		{
				 display[0][y][x] = cR;
				 display[1][y][x] = cG;
				 display[2][y][x] = cB;
    }
		
    err -= dy;      
    if (err < 0) 
    {
      y += ystep;
      err += dx;
    }
  }
}

void P5_vehinhchunhat(unsigned char x, unsigned char y,unsigned char rong, unsigned char cao,char cR,char cG,char cB)
{
   unsigned char x1,y1;
   x1=x+rong-1;
   y1=y+cao-1;
   P5_veduongdoc(x,y,cao,cR,cG,cB); P5_veduongdoc(x1,y,cao,cR,cG,cB);     
   P5_veduongngang(x,y,rong,cR,cG,cB); P5_veduongngang(x,y1,rong,cR,cG,cB);  
}
void P5_vehinhchunhat_kin(unsigned char x, unsigned char y,unsigned char rong, unsigned char cao,char cR,char cG,char cB)
{
   unsigned char x1;
   x1=x+rong;        
   for(;x<x1;x++)P5_veduongdoc(x,y,cao,cR,cG,cB);
}
void P5_vehinhchunhat_Vien(unsigned char x, unsigned char y,unsigned char rong, unsigned char cao,char cR,char cG,char cB)
{
	P5_vehinhchunhat(x,y,rong,cao,31,31,31);
	P5_vehinhchunhat_kin(x+1,y+1,rong-2,cao-2,cR,cG,cB);
}

#define CLOCK_X_Center 15
#define CLOCK_Y_Center 15
void Clock_Analog_BackGround(void)
{
	P5_image(32,32,0,0,(unsigned char *)clock_khung);
}
void Clock_Analog(int gio,int phut,int giay,unsigned char *c)
{
	//ve kim giay
	float angle = ONE_RADIAN*giay*6;
	float x =  CLOCK_X_Center + sin(angle)*SECOND_HAND_SIZE;
  float y =  CLOCK_Y_Center - cos(angle)*SECOND_HAND_SIZE;
	P5_veduongthang(CLOCK_X_Center,CLOCK_Y_Center,x+0.5,y+0.5,c[6],c[7],c[8]);
	
	// ve kim phut
	angle = ONE_RADIAN*phut*6;
	x = CLOCK_X_Center + sin(angle)*MINUTE_HAND_SIZE;
	y = CLOCK_Y_Center - cos(angle)*MINUTE_HAND_SIZE;
  P5_veduongthang(CLOCK_X_Center,CLOCK_Y_Center,x+0.5,y+0.5,c[3],c[4],c[5]);
	
	// ve kim gio
	angle = ONE_RADIAN * (gio+phut/60)*30;
	x = CLOCK_X_Center + sin(angle)*HOUR_HAND_SIZE;
	y = CLOCK_Y_Center - cos(angle)*HOUR_HAND_SIZE;
  P5_veduongthang(CLOCK_X_Center,CLOCK_Y_Center,x+0.5,y+0.5,c[0],c[1],c[2]);
}
//-------------1 vai ham tao hieu ung co ban----------------------//


void P5_ve_hinh_tron(int x0,int y0,int r,char cR,char cG,char cB) 
 {
  int ddF_x,f,x0x,x0y ,x0_x,x0_y,y0x,y0y,y0_x,y0_y,ddF_y,x,y;
 
  if(r<1){return;}//thoát

  f = 1 - r;
  ddF_x = 1;
  ddF_y = -2 * r;
  x = 0;
  y = r;

  set_px(x0, y0+r, cR,cG,cB);
  set_px(x0, y0-r, cR,cG,cB);
  set_px(x0+r, y0, cR,cG,cB);
  set_px(x0-r, y0, cR,cG,cB);

  while (x<y) 
   {
    if (f >= 0) 
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;      

    ddF_x += 2;
    f += ddF_x;
    x0x=x0+x;x0y=x0+y; x0_x=x0-x; x0_y=x0-y;
    y0x=y0+x; y0y=y0+y; y0_x=y0-x; y0_y=y0-y;
    set_px(x0x, y0_y, cR,cG,cB);
    set_px(x0y, y0_x, cR,cG,cB);
    set_px(x0_x, y0_y, cR,cG,cB);
    set_px(x0_y, y0_x, cR,cG,cB);
    set_px(x0_y, y0x, cR,cG,cB);
    set_px(x0_x, y0y, cR,cG,cB);
    set_px(x0y, y0x, cR,cG,cB); 
    set_px(x0x, y0y, cR,cG,cB);   
  }
}
 
void P5_ve_hinh_tron_kin(int x0,int y0,int r, char cR,char cG,char cB) 
{
  int i,f,y0x,y0y,ddF_x,ddF_y,x,y;
  if(r<1){return;}//thoát
  f = 1 - r;
  ddF_x = 1;
  ddF_y = -2 * r;
  x = 0;
  y = r;

  for (i=y0-r; i<=y0+r; i++)set_px(x0, i, cR,cG,cB);
  
  while (x<y) 
   {
    if (f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
   y0y=y0+y;
   y0x=y0+x;
    for (i=y0-y; i<=y0y; i++) 
    {
      set_px(x0+x, i, cR,cG,cB);
      set_px(x0-x, i, cR,cG,cB);    
    } 
    for (i=y0-x; i<=y0x; i++) 
    {
      set_px(x0+y, i, cR,cG,cB);
      set_px(x0-y, i, cR,cG,cB);
    }    
  }
}
 
int Random(int n)
{
   return rand() %(n+1);
}
void P5_outtro(char effect) //xoa man hinh voi hieu ung tuy chon
{
	char cR= Random(31);
	char cG= Random(31);
	char cB= Random(31);
	if(effect==0)
	{
		for(int i=0;i<36;i++)
		  {
				P5_ve_hinh_tron_kin(32,16,i,cR,cG,cB);
				HAL_Delay(30);
			}
		for(int i=0;i<36;i++)
		  {
				P5_ve_hinh_tron_kin(32,16,i,0,0,0);
				HAL_Delay(30);
			}
	}
	else
	{
		//dich man hinh sang trai
		for(int x=0;x<64;x++)
		{
			for(int i=0;i<63;i++)
			{
				for(int y=0;y<32;y++)
				 {
					 display[0][y][i]=display[0][y][i+1];
					 display[1][y][i]=display[1][y][i+1];
					 display[2][y][i]=display[2][y][i+1];
				 }
			}
			for(int y=0;y<32;y++)
				 {
					 display[0][y][63]=0;
					 display[1][y][63]=0;
					 display[2][y][63]=0;
				 }
		  HAL_Delay(12);
		}
	}
}
void P5_clear(void)
{
	for(int i=0;i<64;i++)
  {
		for(int y=0;y<32;y++)
		 {
			 display[0][y][i]=0;
			 display[1][y][i]=0;
			 display[2][y][i]=0;
		 }
	}
}

void P5_clearTOP(void)
{
	for(int i=0;i<64;i++)
  {
		for(int y=0;y<16;y++)
		 {
			 display[0][y][i]=0;
			 display[1][y][i]=0;
			 display[2][y][i]=0;
		 }
	}
}
void P5_special(char i)
{
	if(i==0)for(int i=19;i<29;i++)for(int y=25;y<35;y++)set_px(y,i,img_B[0][i-19][y-25],img_B[1][i-19][y-25],img_B[2][i-19][y-25]);
	else P5_vehinhchunhat(25,19,10,10,0,0,0);
}

//end P5 Lib


char readbit_font(int x,int y,char num,int font)
{
	char temp = x%8;
	return (number_font[font][num][y][x/8] & (0x80 >> temp)) >> (7-temp);
}

void P5_sendnumber_font(char x,char y,char num,int font) //gui so ra man hinh
{
	 char width=64;
	 if(num>=10)return;
	 for(int mau=0;mau<3;mau++)
	 {
		  for(int i=y;i<32+y;i++)
		  {
				 for(int k=x;k<width+x;k++)
				 {
					  if(COLOR[1][0] < 32)display[mau][i][k] = COLOR[readbit_font(k-x,i-y,num,font)][mau];
					  else 
		        {
							if(readbit_font(k-x,i-y,num,font)!=0)display[mau][i][k] = Color_Font[255-COLOR[1][0]][mau][i-y][k-x];
							else display[mau][i][k]  = 0;
						}
				 }
			}
	 }
}
