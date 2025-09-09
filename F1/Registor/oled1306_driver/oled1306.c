#include "oled1306.h"



//oled_cmd_1byte() 
void oled_cmd_1byte(I2C_TypeDef *I2Cx,char data)
{
	
	i2c_start(I2Cx);
	
	i2c_add(I2Cx, 0x78,0); //0x78 is the primary address
	
	i2c_data(I2Cx,0x00); // Control function for a command
	i2c_data(I2Cx,data);
	
	i2c_stop(I2Cx);
	
//	I2Cx_WriteMulti(I2Cx, 0x78, 0x00,  );
}


//oled_cmd_2byte()
void oled_cmd_2byte(I2C_TypeDef *I2Cx,char data[])
{
	int i = 0;
	
	i2c_start(I2Cx);
	
	i2c_add(I2Cx, 0x78,0); //0x78 is the primary address
	
	i2c_data(I2Cx,0x00); // Control function for a command
	for (i=0;i<2;i++)
	{
		i2c_data(I2Cx,data[i]);
	}
	
	i2c_stop(I2Cx);
}


//oled_init()
void oled_init(I2C_TypeDef *I2Cx, char screen_size)
{
	I2Cx_Init(I2C1, Pin_PB6PB7 , 100000);
	
	char cmd[] = {0xA8,0x3F};
	oled_cmd_2byte(I2Cx,cmd);
	char cmd1[] = {0xD3,0x00};
	oled_cmd_2byte(I2Cx,cmd1);
	
	oled_cmd_1byte(I2Cx,0x40);
	oled_cmd_1byte(I2Cx,0xA1);
	
	oled_cmd_1byte(I2Cx,0xC8);
	
	char cmd2[] = {0xDA,screen_size};
	oled_cmd_2byte(I2Cx,cmd2);
	
	char cmd3[] = {0x81,0x7F};
	oled_cmd_2byte(I2Cx,cmd3);
	
	oled_cmd_1byte(I2Cx,0xA4);
	oled_cmd_1byte(I2Cx,0xA6);
	
	char cmd4[] = {0xD5,0x80};
	oled_cmd_2byte(I2Cx,cmd4);
	
	char cmd5[] = {0x8D,0x14};
	oled_cmd_2byte(I2Cx,cmd5);
	
	oled_cmd_1byte(I2Cx,0xAF);
	
	char cmd6[] = {0x20,0x10};
	oled_cmd_2byte(I2Cx,cmd6);
	
}
void oled_init_64(I2C_TypeDef *I2Cx)
{
	oled_init(I2Cx, 0x12);
}

void oled_init_32(I2C_TypeDef *I2Cx)
{
	oled_init(I2Cx, 0x22);
}

//oled_data()
void oled_data(I2C_TypeDef *I2Cx,char data)
{
	
	i2c_start(I2Cx);
	
	i2c_add(I2Cx, 0x78,0); //0x78 is the primary address
	
	i2c_data(I2Cx,0x40); // Control function for a data
	i2c_data(I2Cx,data);
	
	i2c_stop(I2Cx);
}
// oled_pos()

void oled_pos(I2C_TypeDef *I2Cx,char Ypos, char Xpos)
{
	oled_cmd_1byte(I2Cx,0x00 + (0x0F & Xpos));
	oled_cmd_1byte(I2Cx,0x10 + (0x0F & (Xpos>>4)));
	oled_cmd_1byte(I2Cx,0xB0 + Ypos);
}

// oled_blank()
void oled_blank(I2C_TypeDef *I2Cx)
{
	int i,j;
	oled_pos(I2Cx,0, 0);
	for(i=0;i<8;i++)
	{
		for(j=0;j<128;j++)
		{
			oled_data(I2Cx,0x00);
		}
	}
	oled_pos(I2Cx,0, 0);
}

//oled_print
void oled_print(I2C_TypeDef *I2Cx,char str[])
{
	int i,j;
	i=0;
	while(str[i])
	{
		for(j=0;j<5;j++)
		{
			oled_data(I2Cx,ASCII[(str[i]-32)][j]);
		}
		i++;
	}
}

//oled_msg
void oled_msg(I2C_TypeDef *I2Cx,char Ypos, char Xpos,char str[])
{
	oled_pos(I2Cx,Ypos,Xpos);
	oled_print(I2Cx,str);
}






















