#ifndef __MYIIC_H
#define __MYIIC_H 			   
#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 


////////////////////////////////////////////////////////////////////////////////// 
//IO
#define SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=8<<4;}	//PB9IN
#define SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=3<<4;} //PB9OUT
 

#define IIC_SCL    PBout(8) //SCL
#define IIC_SDA    PBout(9) //SDA	 
#define READ_SDA   PBin(9)  //SDA IN

//IIC
void IIC_Init(void);                			 
void IIC_Start(void);				
void IIC_Stop(void);	  			
void IIC_Send_Byte(u8 txd);		
u8 IIC_Read_Byte(unsigned char ack);
u8 IIC_Wait_Ack(void); 				
void IIC_Ack(void);					
void IIC_NAck(void);	
///////////////////////////////
void IIC_Cmd_Write(u8 add ,u8 reg,u8 data);
u8 IIC_Write(u8 addr,u8 reg,u8 data);
u8 Read_IIC(u8 addr, u8 reg);
u8 IIC_ReadMulti(u8 addr,u8 reg,u8 len,u8 *buf);
u8 IIC_WriteMulti(u8 addr,u8 reg,u8 len,u8 *buf);

#endif

