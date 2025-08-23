#ifndef __SYS_H
#define __SYS_H	
#include "stm32f1xx_hal.h"
#include "stdio.h" 
#include "string.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//STM32VN.COM	
////////////////////////////////////////////////////////////////////////////////// 	 

//0, ucos
//1, ucos
#define SYSTEM_SUPPORT_OS		0		//UCOS
																	    
	 

//IO
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//Dia chi
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //ngo ra
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //ngo vao

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)   

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //ngo ra
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //ngo vao

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////
/* Types used for the tables below */
typedef struct _PinDescription
{
  GPIO_TypeDef* pPort ;
  uint32_t ulPin ;
  uint32_t ulPeripheral ;
  /*
  EPioType ulPinType ;
  uint32_t ulPinConfiguration ;
  uint32_t ulPinAttribute ;
  */
  /*
  EAnalogChannel ulAnalogChannel ; *//* Analog pin in the Arduino context (label on the board) */
  uint8_t ulADCChannelNumber ; /* ADC Channel number in the SAM device */
  TIM_TypeDef* ulTimerPeripheral;
  uint16_t ulTimerChannel;
  /*
  EPWMChannel ulPWMChannel ;
  ETCChannel ulTCChannel ;
  */
} PinDescription ;

//extern const PinDescription g_APinDescription[] ;

#define RCC_CLK_GPIOA           (0u)
#define RCC_CLK_GPIOB           (1u)
#define RCC_CLK_GPIOC           (2u)
#define RCC_CLK_GPIOD           (3u)
#define RCC_CLK_GPIOE           (4u)
#define RCC_CLK_GPIOF           (5u)

#define PA0           (0u)
#define PA1           (1u)
#define PA2           (2u)
#define PA3           (3u)
#define PA4           (4u)
#define PA5           (5u)
#define PA6           (6u)
#define PA7           (7u)
#define PA8           (8u)
#define PA9           (9u)
#define PA10          (10u)
#define PA11          (11u)
#define PA12           (12u)
#define PA13           (13u)
#define PA14           (14u)
#define PA15           (15u)

#define PB0           (16u)
#define PB1           (17u)
#define PB2           (18u)
#define PB3           (19u)
#define PB4           (20u)
#define PB5           (21u)
#define PB6           (22u)
#define PB7           (23u)
#define PB8           (24u)
#define PB9           (25u)
#define PB10          (26u)
#define PB11          (27u)
#define PB12           (28u)
#define PB13           (29u)
#define PB14           (30u)
#define PB15           (31u)

#define PC0           (32u)
#define PC1           (33u)
#define PC2           (34u)
#define PC3           (35u)
#define PC4           (36u)
#define PC5           (37u)
#define PC6           (38u)
#define PC7           (39u)
#define PC8           (40u)
#define PC9           (41u)
#define PC10          (42u)
#define PC11          (43u)
#define PC12           (44u)
#define PC13           (45u)
#define PC14           (46u)
#define PC15           (47u)

#define PD0           (48u)
#define PD1           (49u)
#define PD2           (50u)
#define PD3           (51u)
#define PD4           (52u)
#define PD5           (53u)
#define PD6           (54u)
#define PD7           (55u)
#define PD8           (56u)
#define PD9           (57u)
#define PD10          (58u)
#define PD11          (59u)
#define PD12           (60u)
#define PD13           (61u)
#define PD14           (62u)
#define PD15           (63u)

#define PE0           (64u)
#define PE1           (65u)
#define PE2           (66u)
#define PE3           (67u)
#define PE4           (68u)
#define PE5           (69u)
#define PE6           (70u)
#define PE7           (71u)
#define PE8           (72u)
#define PE9           (73u)
#define PE10          (74u)
#define PE11          (75u)
#define PE12           (76u)
#define PE13           (77u)
#define PE14           (78u)
#define PE15           (79u)

#define PF0           (80u)
#define PF1           (81u)
#define PF2           (82u)
#define PF3           (83u)
#define PF4           (84u)
#define PF5           (85u)
#define PF6           (86u)
#define PF7           (87u)
#define PF8           (88u)
#define PF9           (89u)
#define PF10          (90u)
#define PF11          (91u)
#define PF12           (92u)
#define PF13           (93u)
#define PF14           (94u)
#define PF15           (95u)

//#define PD15           (62u)
////////////////////

#define HIGH GPIO_PIN_SET
#define LOW  GPIO_PIN_RESET

#define INPUT 0x0
#define OUTPUT 0x1
#define OUTPUT_OD 0x7
#define INPUT_PULLUP 0x2
#define INPUT_PULLDOWN 0x3
#define AF_OD 0x4
#define AF_PP 0x5
#define AN_INPUT 0x6
//      LOW 0
//      HIGH 1
#define CHANGE 12
#define FALLING 13
#define RISING 14

#define TIMER_PWM  0x8

#define DEFAULT 1
#define EXTERNAL 0

void yield(void);

//void GPIO_Set(GPIO_TypeDef* GPIOx,uint16_t BITx,GPIOMode_TypeDef MODE,GPIOSpeed_TypeDef OSPEED);
void GPIO_Set(GPIO_TypeDef* GPIOx,uint32_t  BITx,uint32_t  MODE,uint32_t PULL,uint32_t  OSPEED);

int digitalRead( uint32_t ulPin );
void digitalWrite( uint32_t ulPin, GPIO_PinState ulVal );
void pinMode(uint32_t PINx,uint32_t MODE);


///////////////////////////////////////////////////
void delay(uint32_t ms);
void delay_us(uint32_t nus);
uint32_t micros(void);
uint32_t millis(void);
#define delay_ms() delay()	
#define delayMicroseconds(us)	delay_us(us)
	
#endif
