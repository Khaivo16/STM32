//UART1 PA9-PA10
//UART2 PA2-PA3
//UART3 PB10-PB11
#include "uart.h"

typedef struct
{
  char* buffer;
	uint8_t size;
	uint8_t in;
	uint8_t out;
	uint8_t num;
} USART_t;   

char USART1_buffer[100];
char USART2_buffer[100];
char USART3_buffer[100];

USART_t USART1_st={USART1_buffer,100,0,0,0};
USART_t USART2_st={USART2_buffer,100,0,0,0};
USART_t USART3_st={USART3_buffer,100,0,0,0};

int fputc(int ch, FILE *f)
{ 
//	while((USART1->SR&0X40)==0);
//    USART1->DR = (u8) ch;   
send_data(USART1,ch);	
	return ch;
}

void uartx_config(USART_TypeDef* USARTx,pin_uart Pin,uint32_t baund)
{
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		/* System Clocks Configuration */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		uint8_t NVIC_IRQChannel;
		if (USARTx==USART1) {RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); NVIC_IRQChannel=USART1_IRQn; ;}
		else if (USARTx==USART2) {RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);NVIC_IRQChannel=USART2_IRQn;}
		else if (USARTx==USART3) {RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);NVIC_IRQChannel=USART3_IRQn;}
		
		if (Pin==pins_PA9_PA10) {GPIO_SETUP(GPIOA,GPIO_Pin_9,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);GPIO_SETUP(GPIOA,GPIO_Pin_10,GPIO_Mode_IN_FLOATING,GPIO_Speed_50MHz);}		
		else if (Pin==pins_PA2_PA3) {GPIO_SETUP(GPIOA,GPIO_Pin_2,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);GPIO_SETUP(GPIOA,GPIO_Pin_3,GPIO_Mode_IN_FLOATING,GPIO_Speed_50MHz);}	
		else if (Pin==pins_PB10_PB11) {GPIO_SETUP(GPIOB,GPIO_Pin_10,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);GPIO_SETUP(GPIOB,GPIO_Pin_11,GPIO_Mode_IN_FLOATING,GPIO_Speed_50MHz);}
		
		USART_InitStructure.USART_BaudRate = baund;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		
		NVIC_InitStructure.NVIC_IRQChannel =NVIC_IRQChannel ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		/* Configure USARTy */
		USART_Init(USARTx, &USART_InitStructure);

  	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
		/* Enable the USARTy */
		USART_Cmd(USARTx, ENABLE);
		
}
void send_data(USART_TypeDef* USARTx,char data) 
{
	/* Send one byte from USARTy to USARTz */
		USART_SendData(USARTx,data);
    
    /* Loop until USARTy DR register is empty */ 
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
    {
    }
     
	}
void send_string(USART_TypeDef* USARTx,char *str)
{
	while(*str) {
				send_data(USARTx,*str++);
	}
}

void send_num(USART_TypeDef* USARTx,uint32_t num)
{
		char str[100];
		sprintf(str,"%d\n",num);
		send_string(USART1,str);	

}

void send_num_float(USART_TypeDef* USARTx,float num)
{
		char str[100];
		sprintf(str,"%.2f\n",num);
		send_string(USART1,str);	

}

void USART_to_buffer(USART_t *u,char c)
{
	if (u->in==u->size) {u->in=0;}
	u->buffer[u->in]=c;
	u->in++;
	u->num++;	
}

char get_c(USART_TypeDef* USARTx)
{
	USART_t *u;
	char c=0;
	if (USARTx==USART1) {u=&USART1_st;}
	if (USARTx==USART2) {u=&USART2_st;}
	if (USARTx==USART3) {u=&USART3_st;}
	
	if (u->num>0) {
	if (u->out==u->size) {u->out=0;}	
	c=u->buffer[u->out];
	u->out++;
	u->num--;
	}
	return c;
}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
		
		USART_to_buffer(&USART1_st,USART1->DR);
//		send_data(USART1,USART1->DR);
	}
																							
}	

uint8_t get_str(USART_TypeDef* USARTx,char *str,uint8_t len)
{
		uint8_t pos=0;;
		char c;
		while (pos<len) {
			c=get_c(USARTx);	
			if (c) {
				str[pos]=c; 
				if (c=='\n') {pos++;break;} else pos++;
				if (pos>len) pos=0;
			}
		}
		str[pos]='\0';
	return pos;
}

uint8_t get_str_1(USART_TypeDef* USARTx,char *str,uint8_t len)
{
		uint8_t pos=0;;
		char c=0;
		for (pos=0;pos<9;pos++){
				str[pos]=str[pos+1];
		}
		c=get_c(USARTx);
		str[9]=c;
	return pos;
}

uint8_t AT_cmd(USART_TypeDef* USARTx,char *command,char *respond,uint32_t timeout) 
{
		uint32_t time=0,i=0;
		char res[100];
		char c;
		memset(res,'\0',100);
		send_string(USARTx,command);
		send_string(USARTx,"\r\n");
		do{	
				if (c) {
				c=get_c(USARTx);	
				res[i++]=c;	
		
			if (strstr(res,respond)){return 1;}		
			}
		time=time+1;
		delayms(1);	
		}while(time<timeout);
		return 0;
}
