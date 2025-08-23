#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
							  
////////////////////////////////////////////////////////////////////////////////// 	

//IIC
void IIC_Init(void)
{			
//  GPIO_InitTypeDef  GPIO_InitStructure;

//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

//  //GPIOB8,B9
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
	
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;				 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	IIC_SCL=1;
	IIC_SDA=1;
}
//
void IIC_Start(void)
{
	SDA_OUT();     //sda
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;
}	  
//
void IIC_Stop(void)
{
	SDA_OUT();//sda
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//G?i t¨ªn hi?u k?t th¨²c bus I2C
	delay_us(4);							   	
}
// Ch? t¨ªn hi?u ph?n h?i ??n
// Gi¨¢ tr? tr? v?: 1, nh?n ???c ph?n h?i th?t b?i
// 0, ph?n h?i th¨¤nh c?ng
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA 
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;// 	   
	return 0;  
} 
//T?o ph?n h?i ACK
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//Kh?ng t?o ra ph?n h?i ACK		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
// IIC g?i m?t byte
//slave xem c¨® ph?n h?i kh?ng
// 1, c¨® ph?n h?i
// 0, kh?ng c¨® cau tr? l?i	  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;// K¨¦o xu?ng ?? b?t ??u truy?n d? li?u
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//??c 1 byte, ack = 1, g?i ACK, ack = 0, g?i NACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//nACK
    else
        IIC_Ack(); //ACK   
    return receive;
}
////////////////////////////////////////////////
void IIC_Cmd_Write(u8 add ,u8 reg,u8 data)
{
    IIC_Start();                  
    IIC_Send_Byte(add+0);   
	while(IIC_Wait_Ack());
    IIC_Send_Byte(reg);    
	while(IIC_Wait_Ack());
    IIC_Send_Byte(data);       
	while(IIC_Wait_Ack());
    IIC_Stop();                   
	delay_ms(5);
}
u8 IIC_Write(u8 addr,u8 reg,u8 data) 		//ham tuong duong	IIC_Cmd_Write(u8 add ,u8 reg,u8 data) 
{ 
    IIC_Start(); 
	IIC_Send_Byte(addr|0);
	if(IIC_Wait_Ack())	
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	
    IIC_Wait_Ack();		
	IIC_Send_Byte(data);
	if(IIC_Wait_Ack())	
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}
u8 Read_IIC(u8 addr, u8 reg)
{   
u8 rec;	
    IIC_Start();                          
    IIC_Send_Byte(addr|0);         
	while(IIC_Wait_Ack());
	
	IIC_Send_Byte(reg);         
	while(IIC_Wait_Ack());
	
	IIC_Start();                          
  IIC_Send_Byte(addr|1);         
	while(IIC_Wait_Ack());
	//BUF[0]=IIC_Read_Byte(1);  
	rec=IIC_Read_Byte(0);  

    IIC_Stop();                         
    delay_ms(5);
	return rec;
}
u8 IIC_ReadMulti(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	IIC_Start(); 
	IIC_Send_Byte((addr)|0);	
	if(IIC_Wait_Ack())	
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	
    IIC_Wait_Ack();		
    IIC_Start();
	IIC_Send_Byte((addr)|1);	
    IIC_Wait_Ack();		
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);
		else *buf=IIC_Read_Byte(1);		
		len--;
		buf++; 
	}    
    IIC_Stop();	
	return 0;	
}
u8 IIC_WriteMulti(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    IIC_Start(); 
	IIC_Send_Byte(addr|0);	
	if(IIC_Wait_Ack())	
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	
    IIC_Wait_Ack();		
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	
		if(IIC_Wait_Ack())		
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Stop();	 
	return 0;	
} 



























