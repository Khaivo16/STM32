#include "bmp.h"
#include "ff.h"
FIL *bmpfil;
UINT dat;
unsigned char buff[960]; // Working buffer
BHEADERTYPE ftype;
extern void LineBMP(unsigned char hang, unsigned int x0,unsigned int d0, unsigned char *mang );
extern void LCD_Clear(unsigned int dat);
BRESULT openBMP(FIL *fil,unsigned int x, unsigned int y) {
	unsigned int  j, dibsiz, imgofs,k,bitpixel,gttamx,gttamy;
	signed int i;
  unsigned long width, height;
	bmpfil = fil;
	imgofs = 0;
	
	// Read header
	f_read(bmpfil, buff, 14, &dat);
	if (dat == 14) {
		if(!((buff[0] == 0x42) && (buff[1] == 0x4D))) { f_close(bmpfil); return BR_HEADER_ERR; }
		imgofs = ( unsigned long)buff[10] | (( unsigned long)buff[11] << 8) | (( unsigned long)buff[12] << 16) | (( unsigned long)buff[13] << 24);
	} else { f_close(bmpfil); return BR_HEADER_ERR; }
	
	// Read DIB header
	f_read(bmpfil, buff, 4, &dat);
	dibsiz = ( unsigned long)buff[0] | (( unsigned long)buff[1] << 8) | (( unsigned long)buff[2] << 16) | (( unsigned long)buff[3] << 24);
	if (dibsiz == 40) ftype = BITMAPINFOHEADER;
	
	// Width and Height
	f_read(bmpfil, buff, 8, &dat);
	width = ( unsigned long)buff[0] | (( unsigned long)buff[1] << 8) | (( unsigned long)buff[2] << 16) | (( unsigned long)buff[3] << 24);
	height = ( unsigned long)buff[4] | (( unsigned long)buff[5] << 8) | (( unsigned long)buff[6] << 16) | (( unsigned long)buff[7] << 24);
	if((width > 320) || (height > 240)) { f_close(bmpfil); return BR_PAR_ERR; }
	f_read(bmpfil, buff, imgofs - 26, &dat);
	bitpixel = buff[2];
	if((x<320)&&(y<240))
	{
		if((height+y)>240)gttamy=240;else gttamy=height+y;
		if((x+width)>320)gttamx=319-x;else gttamx=width-1;
		for (i = gttamy ; i > y; i--) 
		{
				if(bitpixel ==24)
				{ 	
				 {  f_read(bmpfil, buff, 3*width , &dat);
						for (j = 0,k=0; j < 3*width; j+=3,k+=2) 
						 {buff[k] =   ((buff[j+1]&0xfc)*8)|(buff[j]/8);
							buff[k+1] = (buff[j+2]&0xf8)|(buff[j+1]/32);
						 }
					}
				 }
				 else if(bitpixel ==16)f_read(bmpfil, buff, 2*width , &dat);
				 LineBMP(i-1,319-x,gttamx ,buff);
		}
  }
	else 
	{ LCD_Clear(0xffff);
		for (i = 120+(height/2) ; i > 120-(height/2); i--) 
		{
				if(bitpixel ==24)
				{ 	
				 {  f_read(bmpfil, buff, 3*width , &dat);
						for (j = 0,k=0; j < 3*width; j+=3,k+=2) 
						 {buff[k] =   ((buff[j+1]&0xfc)*8)|(buff[j]/8);
							buff[k+1] = (buff[j+2]&0xf8)|(buff[j+1]/32);
						 }
					}
				 }
				 else if(bitpixel ==16)f_read(bmpfil, buff, 2*width , &dat);
				 if(i>-1)LineBMP(i-1,159+(width/2),width-1 ,buff);
		} 
	}
	f_close(bmpfil);	
	return BR_OK;
	
}
