#include "sys.h"
#include "usart.h"	  
////////////////////////////////////////////////////////////////////////////////// 	 
//

//////////////////////////////////////////////////////////////////////////////////	 
//
////////////////////////////////////////////////////////////////////////////////// 	  
 /**	
 * |----------------------------------------------------------------------
 * | 	Pins_PA9PA10,	//USART1
			Pins_PB6PB7,	////USART1
			Pins_PA2PA3,	//USART2
			Pins_PD5PD6,	//USART2
			Pins_PB10PB11,	//USART3
			Pins_PC10PC11,	//USART3//UART4
			Pins_PD8PD9,	//USART3
			Pins_PA0PA1,	//UART4
			Pins_PC12PD2,	//UART5
			Pins_PC6PC7,	//USART6
			Pins_PG14PG9,	//USART6
			Pins_PB8PE7,	//UART7
			Pins_PF7PF6,	//UART7
			Pins_PE1PE0		//UART8
*/



typedef struct
{
  uint8_t *Buffer;             

  uint16_t Buffer_size;  
	
	uint16_t num;
	
	uint16_t in;
	
	u16 out;

  GPIOMode_TypeDef GPIO_Mode;    
}USART_t;

u8 Buffer_USART1[100];
u8 Buffer_USART2[100];
u8 Buffer_USART3[100];
u8 Buffer_USART4[100];
u8 Buffer_USART5[100];
u8 Buffer_USART6[100];
u8 Buffer_USART7[100];
u8 Buffer_USART8[100];

USART_t ST_USART1 = { Buffer_USART1, 100,0,0,0};
USART_t ST_USART2 = { Buffer_USART2, 100,0,0,0};
USART_t ST_USART3 = { Buffer_USART3, 100,0,0,0};
USART_t ST_USART4 = { Buffer_USART4, 100,0,0,0};
USART_t ST_USART5 = { Buffer_USART5, 100,0,0,0};
USART_t ST_USART6 = { Buffer_USART6, 100,0,0,0};
USART_t ST_USART7 = { Buffer_USART7, 100,0,0,0};
USART_t ST_USART8 = { Buffer_USART8, 100,0,0,0};


void USARTtoBUF(USART_t *u, char c ){
	
	if(u->in<=u->Buffer_size){
		if(u->in==u->Buffer_size) u->in=0;
		u->Buffer[u->in] = c;//USART_Putc(USART1, c);
		u->in++;
		u->num++;
//		if(strstr((const char *)u->Buffer,"ABC")!=NULL)
//			//if(buff[0]=='A'&&buff[1]=='B'&&buff[2]=='C')
//			{ 
//				USART_Puts(USART1, (char *)u->Buffer);
//				//for(i=0;i<3;i++) buff[i] = 0;
//			}
	}
	
	
}

u8 USART_Getc(USART_TypeDef * USARTx )
	{ u8 c=0;
		USART_t *u;	
			
		if(USARTx==USART1) u = & ST_USART1;
		else if(USARTx==USART2) u = & ST_USART2;
		else if(USARTx==USART3) u = & ST_USART3;
		else if(USARTx==UART4) u = & ST_USART4;
		else if(USARTx==UART5) u = & ST_USART5;
//		else if(USARTx==USART6) u = & ST_USART6;
//		else if(USARTx==UART7) u = & ST_USART7;
//		else if(USARTx==UART8) u = & ST_USART8;
		
		if(u->num>0){
			if(u->out==u->Buffer_size) u->out=0;
			 c=u->Buffer[u->out] ;//USART_Putc(USART1, c);
			u->out++;
			u->num--;
		}
		return c;
	}
	
//uint8_t USART_BufferEmpty(USART_TypeDef* USARTx) {
//	USART_t* u ;
//	if(USARTx==USART1) u = & ST_USART1;
//		else if(USARTx==USART2) u = & ST_USART2;
//		else if(USARTx==USART3) u = & ST_USART3;
//		else if(USARTx==UART4) u = & ST_USART4;
//		else if(USARTx==UART5) u = & ST_USART5;
//	
//	return (u->num == 0);
//}

//uint8_t USART_BufferFull(USART_TypeDef* USARTx) {
//	USART_t* u ;
//	if(USARTx==USART1) u = & ST_USART1;
//		else if(USARTx==USART2) u = & ST_USART2;
//		else if(USARTx==USART3) u = & ST_USART3;
//		else if(USARTx==UART4) u = & ST_USART4;
//		else if(USARTx==UART5) u = & ST_USART5;
//	
//	return (u->num == u->Buffer_size);
//}

//void USART_ClearBuffer(USART_TypeDef* USARTx) {
//	USART_t* u ;
//	if(USARTx==USART1) u = & ST_USART1;
//		else if(USARTx==USART2) u = & ST_USART2;
//		else if(USARTx==USART3) u = & ST_USART3;
//		else if(USARTx==UART4) u = & ST_USART4;
//		else if(USARTx==UART5) u = & ST_USART5;
//	
//	u->num = 0;
//	u->in = 0;
//	u->out = 0;
//}

//uint8_t USART_FindCharacter(USART_TypeDef* USARTx, volatile char c) {
//	uint16_t num, out;
//	USART_t* u ;
//	
//	if(USARTx==USART1) u = & ST_USART1;
//		else if(USARTx==USART2) u = & ST_USART2;
//		else if(USARTx==USART3) u = & ST_USART3;
//		else if(USARTx==UART4) u = & ST_USART4;
//		else if(USARTx==UART5) u = & ST_USART5;
//	
//	/* Temp variables */
//	num = u->num;
//	out = u->out;
//	
//	while (num > 0) {
//		/* Check overflow */
//		if (out == u->Buffer_size) {
//			out = 0;
//		}
//		if (u->Buffer[out] == c) {
//			/* Character found */
//			return 1;
//		}
//		out++;
//		num--;
//	}
//	
//	/* Character is not in buffer */
//	return 0;
//}
	
u8 USART_Gets(USART_TypeDef * USARTx , char *buffer, u16 len)
	{ u16 i=0;
		u8 c=0;
		
		//if(USART_BufferEmpty(USART1)|| (!USART_FindCharacter(USARTx,'\n') && !USART_BufferFull(USARTx) )) return 0;
		
//		while(i<len)
//		{
//		buffer[i]=USART_Getc(USARTx);
//		if (buffer[i]=='\n') {i++;break;}
//			else i++;
//		
//		}
//		buffer[i]=0;
		while(i<len)
		{
			c= USART_Getc(USARTx);
			if(c) //printf("%c",c);
			{	
								
				buffer[i]=c;
				if (buffer[i]=='\n')
				{				
					//if(strstr(a,"ON")!=NULL) { printf("%s\n",a);for (z=0;z<100;z++) a[z]=0; i=0; }
					i++; break;
					
				}
				else i++;
				
				if (i>=len) i=0;
			}
		}		
		buffer[i]=0;
		return i;
	}
//////////////////////////////////////////////////////////////////
//printf  

#pragma import(__use_no_semihosting)             
               
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
  
_sys_exit(int x) 
{ 
	x = x; 
} 
//fputc
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);
    USART1->DR = (u8) ch;      
	return ch;
}


/*microLib*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 


u8 USARTx_GetSubPriority(USART_TypeDef * USARTx)
{
	u8 u=0;
	
	if(USARTx==USART1) u=0;
	else if(USARTx==USART2) u=1;
	else if(USARTx==USART3) u=2;
	#ifdef	USE_UART4
	else if(USARTx==UART4) u=3;
	#endif
	#ifdef	USE_UART5
	else if(USARTx==UART5) u=4;
	#endif
	#ifdef	USE_USART6
	else if(USARTx==USART6) u=5;
	#endif
	#ifdef	USE_UART7
	else if(USARTx==UART7) u=6;
	#endif
	#ifdef	USE_UART8
	else if(USARTx==UART8) u=7;
	#endif
	
	return u;
}
	
void USARTx_Init(USART_TypeDef * USARTx, USART_Pins pins, u32 bound){
	//GPIO
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint8_t IRQChannel;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	if(USARTx==USART1) {RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);IRQChannel = USART1_IRQn;}
	else if(USARTx==USART2) {RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);IRQChannel = USART2_IRQn;}
	else if(USARTx==USART3) {RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);IRQChannel = USART3_IRQn;}
		
	if(pins==Pins_PA9PA10)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//USART1-GPIOA
		//USART1_TX   GPIOA.9
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);//GPIOA.9
		 
		//USART1_RX	  GPIOA.10
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);//GPIOA.10  
	}
	else if(pins==Pins_PB6PB7)
		{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//USART1-GPIOA
		//USART1_TX   GPIOB.9
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //PB.6
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
		GPIO_Init(GPIOB, &GPIO_InitStructure);//GPIOB.6
		 
		//USART1_RX	  GPIOB.7
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//PB7
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOB, &GPIO_InitStructure);//GPIOB.7 
	}
	else if(pins==Pins_PA2PA3)
		{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//USART1-GPIOA
		//USART1_TX   GPIOA.2
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);//GPIOA.2
		 
		//USART1_RX	  GPIOA.3
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);//GPIOA.3
	}
	#ifdef	USE_USART3
	else if(pins==Pins_PD5PD6)
		{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);	//USART1-GPIOD
		//USART1_TX   GPIOD.5
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //PD.5
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
		GPIO_Init(GPIOD, &GPIO_InitStructure);//GPIOD.5
		 
		//USART1_RX	  GPIOD.6
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PD6
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOD, &GPIO_InitStructure);//GPIOD.6 
	}
	#endif
	else if(pins==Pins_PB10PB11)
		{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//USART1-GPIOB
		//USART1_TX   GPIOB.10
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
		GPIO_Init(GPIOB, &GPIO_InitStructure);//GPIOB.10
		 
		//USART1_RX	  GPIOB.11
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOB, &GPIO_InitStructure);//GPIOB.11  
	}
	else if(pins==Pins_PC10PC11)
		{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	//USART1-GPIOC
		//USART1_TX   GPIOC.10
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PC.10
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
		GPIO_Init(GPIOC, &GPIO_InitStructure);//GPIOC.10
		 
		//USART1_RX	  GPIOC.11
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PC11
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOC, &GPIO_InitStructure);//GPIOC.11  
	}
	#ifdef	USE_UART4
	else if(pins==Pins_PA0PA1)
		{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//USART1-GPIOC
		//USART1_TX   GPIOA.0
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //PA0
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);//GPIOA.0
		 
		//USART1_RX	  GPIOA.1
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//PA1
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);//GPIOA1  
	}
	#endif
	
  
	

  //Usart1 NVIC 
  NVIC_InitStructure.NVIC_IRQChannel = IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = USARTx_GetSubPriority(USARTx);		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
  
   //USART 

	USART_InitStructure.USART_BaudRate = bound;////////
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;////
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//////

  USART_Init(USARTx, &USART_InitStructure); 
	
  USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
  USART_Cmd(USARTx, ENABLE);     
}
 
	


//////////////////////////////////////////////////////////////////////////////////////////////////
u8 USART_RX_BUF[USART_REC_LEN];     

u16 USART_RX_STA=0;        
  
void uart_init(u32 bound){
  //GPIO
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//USART1£¬GPIOA
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
  GPIO_Init(GPIOA, &GPIO_InitStructure);//GPIOA.9
   
  //USART1_RX	  GPIOA.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//GPIOA.10  

  //Usart1 NVIC 
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
  
   //USART 

	USART_InitStructure.USART_BaudRate = bound;////////
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;////
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//////

  USART_Init(USART1, &USART_InitStructure); 
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART1, ENABLE);                    

}


void USART_Putc(USART_TypeDef* USARTx, volatile char c) {
	/* Wait to be ready */
	//while (!USART_GetFlagStatus(USARTx, USART_FLAG_TXE));
	while (!(USARTx->SR & USART_FLAG_TXE));
	/* Send data */
	USARTx->DR = (uint16_t)(c & 0x01FF);
}	

void USART_Puts(USART_TypeDef* USARTx, char* str) {
	while (*str) {
		/* Send char by char */
		USART_Putc(USARTx, *str++);
	}
}

char buff[3];
void USART1_IRQHandler(void)                	
	{
	u8 Res;
	u8 i=0;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  
		{
//		Res =USART_ReceiveData(USART1);	//USART_Putc(USART1, Res);
//			for( i=0; i<3;i++)  buff[i] =buff[i+1];
//			buff[2]= Res;
//			if(strstr(buff,"ABC")!=NULL)
//			//if(buff[0]=='A'&&buff[1]=='B'&&buff[2]=='C')
//			{ 
//				USART_Puts(USART1, buff);
//				for(i=0;i<3;i++) buff[i] = 0;
//			}
			
			USARTtoBUF(&ST_USART1, USART1->DR );
		  		 
     } 

} 
	




