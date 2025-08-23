#include "stm32f10x.h"
#include "LED.h"
#include "delay.h"
#include "stdlib.h"
#include "EXTI.h"
#include "uart.h"
#include "string.h"
#include <math.h>

#define LED1 PBout(12)


char x=0;
char str[10];
char send[10];
char respond[10];
char * p;
int val3=0;
float val,val2;
int so[10];
char c;

int main(void)
{	
	delay_init();
	LED_init();
	uartx_config(USART1,pins_PA9_PA10,9600);
	GPIO_SETUP(GPIOB,GPIO_Pin_12,GPIO_Mode_Out_PP,GPIO_Speed_10MHz);
	do{val3=AT_cmd(USART1,"AT","OK",5000);}while(val3==0);
  while(1)
	{
//   if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
//    {
//    
//    /* Store the received byte in RxBuffer */
////   c = (USART_ReceiveData(USART1) & 0x7F);
//		send_data(USART1,USART1->DR);	}
//		x=get_c(USART1);
//		if (x) {send_data(USART1,x);}
//			if (get_str(USART1,str,10)) {
//				
//				}
//					if (strstr(str,"ON")) {LED1=0;}
//					else if(strstr(str,"OFF"))  LED1=1;
//			
//				val = atof(strtok(str, ","));
//				val2 = atof(strtok(NULL, ","));
//				val3 = atoi(strtok(NULL, "\n"));
//				sprintf(str,"%2.2f-%2.2f-%3d\n", val,val2,val3);
//				send_string(USART1,str);
		
//					c=get_c(USART1);
//					
//					if(c){
//							for (	int i=0;i<9;i++){ str[i]=str[i+1];
//									str[9]=c;
//								}
//					delay_ms(50);
//					if ((str[0]==0x01) && (str[9]==0x02)){
//					send_string(USART1,str);	send_string(USART1,"\n");}
//			}
				
				LED1=!LED1;
				delay_ms(50);
				
  } 
}

