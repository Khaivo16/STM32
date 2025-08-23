
#include "myiic.h"  
// I2C PCf8574 address slave 0x40,0x4E
void IIC_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;			    //LED0-->PB.5 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_Out_PP; 	 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_Speed_50MHz;	 
  GPIO_Init(GPIOB, &GPIO_InitStructure);			     //GPIOB.5
	IIC_SCL=1;
	IIC_SDA =1;
}

void IIC_Start(void){
	SDA_OUT();
	IIC_SCL=1;
	IIC_SDA =1;
	delay_us(4);
	IIC_SDA =0;
	delay_us(4);
	IIC_SCL=0;	
}

void IIC_Stop(void){	
	SDA_OUT();
	IIC_SCL=0;
	IIC_SDA =0;
	delay_us(4);
	IIC_SDA =1;
	IIC_SCL=1;
	delay_us(4);		
}

void IIC_Send_Byte(u8 txd){//11111111&10000000
	int i=0;
	SDA_OUT();
	IIC_SCL=0;
	for(i=0;i<8;i++){
		IIC_SDA=(txd&0x80)>>7;
		txd<<=1;
		delay_us(2);
		IIC_SCL=1;
		delay_us(2);
		IIC_SCL=0;
		
	}
	
}


void IIC_Ack(void){
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					
void IIC_NAck(void){
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}	

u8 IIC_Read_Byte(unsigned char ack){
	int i=0;
	u8 rec=0;
	SDA_IN();
	for(i=0;i<8;i++){//11111111
		IIC_SCL=0;
		delay_us(2);
		IIC_SCL=1;
		delay_us(2);
		rec<<=1;
		if(READ_SDA) rec++;
	}
	if(!ack){
		IIC_Ack();
	}
		else{
		IIC_NAck();	
		}
	return rec;
}

u8 IIC_Wait_Ack(void){
	u8 time=0;
	SDA_IN();
	IIC_SDA=1;delay_us(1);
	IIC_SCL=1;delay_us(1);
	while(READ_SDA){
		time++;
		if(time>250) {IIC_Stop(); return 1;}
	}
	IIC_SCL=0;
	return 0; 
}

void IIC_Cmd_Write(u8 add ,u8 reg,u8 data){
	IIC_Start();
	
	IIC_Send_Byte(add|0);
	while(IIC_Wait_Ack());
	IIC_Send_Byte(reg);
	while(IIC_Wait_Ack());
	IIC_Send_Byte(data);
	while(IIC_Wait_Ack());
	
	IIC_Stop();
	delay_ms(2);
}

u8 IIC_Write(u8 addr,u8 reg,u8 data){
	IIC_Start();
	
	IIC_Send_Byte(addr|0);
	if(IIC_Wait_Ack()){IIC_Stop(); return 1;}
	IIC_Send_Byte(reg);
	if(IIC_Wait_Ack()){IIC_Stop(); return 1;}
	IIC_Send_Byte(data);
	if(IIC_Wait_Ack()){IIC_Stop(); return 1;}
	
	IIC_Stop();
	delay_ms(2);
	return 0;
}

u8 Read_IIC(u8 addr, u8 reg){
	IIC_Start();
	IIC_Send_Byte(addr|0);
	while(IIC_Wait_Ack());
	IIC_Send_Byte(reg);
	while(IIC_Wait_Ack());
	
	IIC_Start();
	IIC_Send_Byte(addr|1);
	while(IIC_Wait_Ack());
	reg=IIC_Read_Byte(1);
	IIC_Stop();
	delay_ms(2);
	return reg;
}

u8 IIC_ReadMulti(u8 addr,u8 reg,u8 len,u8 *buf){
	
	IIC_Start();
	IIC_Send_Byte(addr|0);
	while(IIC_Wait_Ack());
	IIC_Send_Byte(reg);
	while(IIC_Wait_Ack());
	
	IIC_Start();
	IIC_Send_Byte(addr|1);
	while(IIC_Wait_Ack());
	
	while(len){
		if(len==1) *buf= IIC_Read_Byte(1);
		else *buf= IIC_Read_Byte(0);
		len--;
		buf++;
	}	
	
	IIC_Stop();
	return 0;

}

u8 IIC_WriteMulti(u8 addr,u8 reg,u8 len,u8 *buf){
	u8 i=0;
	IIC_Start();
	IIC_Send_Byte(addr|0);
	while(IIC_Wait_Ack());
	IIC_Send_Byte(reg);
	while(IIC_Wait_Ack());
	
	for(i=0;i<len;i++){
		IIC_Send_Byte(buf[i]);
		if(IIC_Wait_Ack()){IIC_Stop(); return 1;}
	}
	
	IIC_Stop();	
	return 0;
}
