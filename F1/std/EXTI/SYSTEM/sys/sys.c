#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//m?t s? ch?c nang
void GPIO_Set(GPIO_TypeDef* GPIOx,u16 BITx,GPIOMode_TypeDef MODE,GPIOSpeed_TypeDef OSPEED)
{  
		GPIO_InitTypeDef  GPIO_InitStructure;
      
	
	if (GPIOx == GPIOA) {
		// Enable clock for GPIOA
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	} else if (GPIOx == GPIOB) {
		// Enable clock for GPIOB
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	} else if (GPIOx == GPIOC) {
		// Enable clock for GPIOC
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	}else if (GPIOx == GPIOD) {
		// Enable clock for GPIOD
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	}else if (GPIOx == GPIOE) {
		// Enable clock for GPIOE
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

	}else if (GPIOx == GPIOF) {
		// Enable clock for GPIOF
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);

	}else if (GPIOx == GPIOG) {
		// Enable clock for GPIOG
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);

	}
	 
  GPIO_InitStructure.GPIO_Pin = BITx;			    //LED0-->PB.5 
  GPIO_InitStructure.GPIO_Mode = MODE;//GPIO_Mode_Out_PP; 	 
  GPIO_InitStructure.GPIO_Speed = OSPEED;//GPIO_Speed_50MHz;	 
  GPIO_Init(GPIOx, &GPIO_InitStructure);			     //GPIOB.5
        
} 


void PIN_Init(GPIO_TypeDef * GPIOx,uint16_t Pin,GPIOMode_TypeDef Mode,GPIOSpeed_TypeDef Speed){
	
	GPIO_InitTypeDef  GPIO_InitStructure; 		
	
	if (GPIOx == GPIOA) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	    
	}else if (GPIOx == GPIOB) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	    
	}
	else if (GPIOx == GPIOB) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	    
	}
	else if (GPIOx == GPIOC) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	    
	}
	else if (GPIOx == GPIOD) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);	    
	}
	
  GPIO_InitStructure.GPIO_Pin = Pin;			    //LED0-->PB.5 
  GPIO_InitStructure.GPIO_Mode = Mode; 	 
  GPIO_InitStructure.GPIO_Speed = Speed;	 
  GPIO_Init(GPIOx, &GPIO_InitStructure);	
}
//********************************************************************************  
void GPIO_SETUP(GPIO_TypeDef * GPIOx , uint16_t Pin, GPIOMode_TypeDef Mode, GPIOSpeed_TypeDef Speed ){
	
	GPIO_InitTypeDef  GPIO_InitStructure; 		
	
	if(GPIOx==GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	    		
	else if(GPIOx==GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	    	
	else if(GPIOx==GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	
	
  GPIO_InitStructure.GPIO_Pin = Pin;			    
  GPIO_InitStructure.GPIO_Mode = Mode; 	 
  GPIO_InitStructure.GPIO_Speed = Speed;	 
  GPIO_Init(GPIOx, &GPIO_InitStructure);			     
	
}




void WFI_SET(void)
{
	__ASM volatile("wfi");		  
}

void INTX_DISABLE(void)
{		  
	__ASM volatile("cpsid i");
}

void INTX_ENABLE(void)
{
	__ASM volatile("cpsie i");		  
}

__asm void MSR_MSP(u32 addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}
