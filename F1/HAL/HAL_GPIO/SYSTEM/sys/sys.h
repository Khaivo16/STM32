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
																	    

#define RD_PORT GPIOA
#define RD_PIN  GPIO_PIN_4
#define WR_PORT GPIOA
#define WR_PIN  GPIO_PIN_3
#define CD_PORT GPIOA          // RS PORT
#define CD_PIN  GPIO_PIN_2     // RS PIN
#define CS_PORT GPIOA
#define CS_PIN  GPIO_PIN_1
#define RESET_PORT GPIOA
#define RESET_PIN  GPIO_PIN_0

#define D0_PORT GPIOB
#define D0_PIN GPIO_PIN_0
#define D1_PORT GPIOB
#define D1_PIN GPIO_PIN_1
#define D2_PORT GPIOA
#define D2_PIN GPIO_PIN_15
#define D3_PORT GPIOB
#define D3_PIN GPIO_PIN_3
#define D4_PORT GPIOB
#define D4_PIN GPIO_PIN_4
#define D5_PORT GPIOB
#define D5_PIN GPIO_PIN_5
#define D6_PORT GPIOB
#define D6_PIN GPIO_PIN_6
#define D7_PORT GPIOA
#define D7_PIN GPIO_PIN_5



#define  WIDTH    ((uint16_t)240)
#define  HEIGHT   ((uint16_t)320)


/****************** delay in microseconds ***********************/
//extern TIM_HandleTypeDef htim1;
//void delay (uint32_t time)
//{
//	/* change your code here for the delay in microseconds */
//	__HAL_TIM_SET_COUNTER(&htim1, 0);
//	while ((__HAL_TIM_GET_COUNTER(&htim1))<time);
//}




// configure macros for the data pins.

/* First of all clear all the LCD_DATA pins i.e. LCD_D0 to LCD_D7
 * We do that by writing the HIGHER bits in BSRR Register
 *
 * For example :- To clear Pins B3, B4 , B8, B9, we have to write GPIOB->BSRR = 0b0000001100011000 <<16
 *
 *
 *
 * To write the data to the respective Pins, we have to write the lower bits of BSRR :-
 *
 * For example say the PIN LCD_D4 is connected to PB7, and LCD_D6 is connected to PB2
 *
 * GPIOB->BSRR = (data & (1<<4)) << 3.  Here first select 4th bit of data (LCD_D4), and than again shift left by 3 (Total 4+3 =7 i.e. PB7)
 *
 * GPIOB->BSRR = (data & (1<<6)) >> 4.  Here first select 6th bit of data (LCD_D6), and than again shift Right by 4 (Total 6-4 =2 i.e. PB2)
 *
 *
 */
  #define write_8(d) { \
   GPIOA->BSRR = 0b1000000000100000 << 16; \
   GPIOB->BSRR = 0b0000000001111011 << 16; \
   GPIOA->BSRR = (((d) & (1<<2)) << 13) \
               | (((d) & (1<<7)) >> 2); \
   GPIOB->BSRR = (((d) & (1<<0)) << 0) \
               | (((d) & (1<<1)) << 0) \
			   | (((d) & (1<<3)) << 0) \
			   | (((d) & (1<<4)) << 0) \
			   | (((d) & (1<<5)) << 0) \
			   | (((d) & (1<<6)) << 0); \
    }


  /* To read the data from the Pins, we have to read the IDR Register
   *
   * Take the same example say LCD_D4 is connected to PB7, and LCD_D6 is connected to PB2
   *
   * To read data we have to do the following
   *
   * GPIOB->IDR & (1<<7) >> 3. First read the PIN (1<<7 means we are reading PB7) than shift it to the position, where it is connected to
   * and in this example, that would be 4 (LCD_D4). (i.e. 7-3=4)
   *
   * GPIOB->IDR & (1<<2) << 4. First read the PIN (1<<2 means we are reading PB2) than shift it to the position, where it is connected to
   * and in this case, that would be 6 (LCD_D6). (i.e. 2+4= 6). Shifting in the same direction
   *
   */
  #define read_8() (          (((GPIOB->IDR & (1<<0)) >> 0) \
                           | ((GPIOB->IDR & (1<<1)) >> 0) \
                           | ((GPIOA->IDR & (1<<15)) >> 13) \
                           | ((GPIOB->IDR & (1<<3)) >> 0) \
                           | ((GPIOB->IDR & (1<<4)) >> 0) \
                           | ((GPIOB->IDR & (1<<5)) >> 0) \
                           | ((GPIOB->IDR & (1<<6)) >> 0) \
                           | ((GPIOA->IDR & (1<<5)) << 2)))



/********************* For 180 MHz *****************************/
//#define WRITE_DELAY { WR_ACTIVE8; }
//#define READ_DELAY  { RD_ACTIVE16;}


/************************** For 72 MHZ ****************************/
#define WRITE_DELAY { }
#define READ_DELAY  { RD_ACTIVE;  }


/************************** For 100 MHZ ****************************/
//#define WRITE_DELAY { WR_ACTIVE2; }
//#define READ_DELAY  { RD_ACTIVE4; }


/************************** For 216 MHZ ****************************/
//#define WRITE_DELAY { WR_ACTIVE8; WR_ACTIVE8; } //216MHz
//#define IDLE_DELAY  { WR_IDLE4;WR_IDLE4; }
//#define READ_DELAY  { RD_ACTIVE16;RD_ACTIVE16;RD_ACTIVE16;}


/************************** For 48 MHZ ****************************/
//#define WRITE_DELAY { }
//#define READ_DELAY  { }


/*****************************  DEFINES FOR DIFFERENT TFTs   ****************************************************/

//#define SUPPORT_0139              //S6D0139 +280 bytes
//#define SUPPORT_0154              //S6D0154 +320 bytes
//#define SUPPORT_1289              //SSD1289,SSD1297 (ID=0x9797) +626 bytes, 0.03s
//#define SUPPORT_1580              //R61580 Untested
//#define SUPPORT_1963              //only works with 16BIT bus anyway
//#define SUPPORT_4532              //LGDP4532 +120 bytes.  thanks Leodino
//#define SUPPORT_4535              //LGDP4535 +180 bytes
//#define SUPPORT_68140             //RM68140 +52 bytes defaults to PIXFMT=0x55
//#define SUPPORT_7735
//#define SUPPORT_7781              //ST7781 +172 bytes
//#define SUPPORT_8230              //UC8230 +118 bytes
#define SUPPORT_8347D             //HX8347-D, HX8347-G, HX8347-I, HX8367-A +520 bytes, 0.27s
//#define SUPPORT_8347A             //HX8347-A +500 bytes, 0.27s
//#define SUPPORT_8352A             //HX8352A +486 bytes, 0.27s
//#define SUPPORT_8352B             //HX8352B
//#define SUPPORT_8357D_GAMMA       //monster 34 byte
//#define SUPPORT_9163              //
//#define SUPPORT_9225              //ILI9225-B, ILI9225-G ID=0x9225, ID=0x9226, ID=0x6813 +380 bytes
//#define SUPPORT_9326_5420         //ILI9326, SPFD5420 +246 bytes
//#define SUPPORT_9342              //costs +114 bytes
//#define SUPPORT_9806              //UNTESTED
//#define SUPPORT_9488_555          //costs +230 bytes, 0.03s / 0.19s
//#define SUPPORT_B509_7793         //R61509, ST7793 +244 bytes
//#define OFFSET_9327 32            //costs about 103 bytes, 0.08s

	 

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
