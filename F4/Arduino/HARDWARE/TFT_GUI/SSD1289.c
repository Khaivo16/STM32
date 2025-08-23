/*********************************************************************************************************
*
* File                : LCD.c
* Hardware Environment: 
* Build Environment   : RealView MDK-ARM  Version: 4.20
* Version             : V1.0
* By                  : 
*
*                                  
*                                       
*                                          STM32VN
*
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "SSD1289.h"

/* Private define ------------------------------------------------------------*/

//#define LCD_REG              (*((volatile unsigned short *) 0x60000000)) /* RS = 0 */
//#define LCD_RAM              (*((volatile unsigned short *) 0x60020000)) /* RS = 1 */

#define LCD_REG              (*((volatile unsigned short *) 0x60000000)) /* RS = 0 */
#define LCD_RAM              (*((volatile unsigned short *) 0x60100000)) /* RS = 1 */

/* Private variables ---------------------------------------------------------*/
static uint8_t LCD_Code;
uint16_t TimerPeriod    = 0;
uint16_t Channel3Pulse  = 0;
//****************************************************************************//

//****************************************************************************//
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
//****************************************************************************//

/* Private define ------------------------------------------------------------*/
#define  ILI9320    0  /* 0x9320 */
#define  ILI9325    1  /* 0x9325 */
#define  ILI9328    2  /* 0x9328 */
#define  ILI9331    3  /* 0x9331 */
#define  SSD1298    4  /* 0x8999 */
#define  SSD1289    5  /* 0x8989 */
#define  ST7781     6  /* 0x7783 */
#define  LGDP4531   7  /* 0x4531 */
#define  SPFD5408B  8  /* 0x5408 */
#define  R61505U    9  /* 0x1505 0x0505 */	   
#define  HX8347D    10 /* 0x0047 */
#define  HX8347A    11 /* 0x0047 */	
#define  LGDP4535   12 /* 0x4535 */  
#define  SSD2119    13 /* 3.5 LCD 0x9919 */

/*******************************************************************************
* Function Name  : LCD_CtrlLinesConfig
* Description    : Configures LCD Control lines (FSMC Pins) in alternate function
                   Push-Pull mode.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void LCD_CtrlLinesConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	
	/* Enable GPIOs clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE |RCC_AHB1Periph_GPIOB , ENABLE);
	
	/* Enable FSMC clock */
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE); 

	/*-- GPIOs Configuration ------------------------------------------------------*/
	/*
	+-------------------+--------------------+------------------+------------------+
	+                       SRAM pins assignment                                   +

	+-------------------+--------------------+
	*/
	/* GPIOD configuration */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0,  GPIO_AF_FSMC); // PD0=FSMC_D2   -> DB2
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource1,  GPIO_AF_FSMC); // PD1=FSMC_D3   -> DB3
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource4,  GPIO_AF_FSMC); // PD4=FSMC_NOE  -> RD
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource5,  GPIO_AF_FSMC); // PD5=FSMC_NWE  -> WR
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource7,  GPIO_AF_FSMC); // PD7=FSMC_NE1  -> CS
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8,  GPIO_AF_FSMC); // PD8=FSMC_D13  -> DB15
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9,  GPIO_AF_FSMC); // PD9=FSMC_D14  -> DB16
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC); // PD10=FSMC_D15 -> DB17  
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC); // PD14=FSMC_D0  -> DB0
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC); // PD15=FSMC_D1  -> DB1
  
  // Structur fuer Port-D
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
                                GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                                GPIO_Pin_14 | GPIO_Pin_15;
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  // Config von Port-D
  GPIO_Init(GPIOD, &GPIO_InitStructure);   
  
  //-----------------------------------------
  // Alle Pins von Port-E initialisieren
  //-----------------------------------------
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource3,  GPIO_AF_FSMC); // PE3=FSMC_A19  -> RS
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource7,  GPIO_AF_FSMC); // PE7=FSMC_D4   -> DB4
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource8,  GPIO_AF_FSMC); // PE8=FSMC_D5   -> DB5  
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9,  GPIO_AF_FSMC); // PE9=FSMC_D6   -> DB6
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC); // PE10=FSMC_D7  -> DB7
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC); // PE11=FSMC_D8  -> DB10
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC); // PE12=FSMC_D9  -> DB11
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC); // PE13=FSMC_D10 -> DB12
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC); // PE14=FSMC_D11 -> DB13
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC); // PE15=FSMC_D12 -> DB14

  // Structur fuer Port-E
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 |
                                GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 |
                                GPIO_Pin_14 | GPIO_Pin_15;
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  // Config von Port-E
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	/* configure BACKLIGHT pin */
	GPIO_InitStructure.GPIO_Pin = SSD1289_BACKLIGHT_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(SSD1289_BACKLIGHT_PORT, &GPIO_InitStructure);
	
//		/* Set PD.00(D2), PD.01(D3), PD.04(NOE), PD.05(NWE), PD.08(D13), PD.09(D14),
//	 PD.10(D15), PD.11(A16), PD.14(D0), PD.15(D1) as alternate function push pull */
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);				//////////RS
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | 
//	                            GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |
//								 GPIO_Pin_14 | GPIO_Pin_15;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//	
//	GPIO_Init(GPIOD, &GPIO_InitStructure);
//	
//	/* Set PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
//	 PE.14(D11), PE.15(D12) as alternate function push pull */	
//	/* GPIOE configuration */
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource2 , GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_FSMC);
//	
//	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_2 | GPIO_Pin_7 | GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 |
//	                             GPIO_Pin_11| GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
//	
//	GPIO_Init(GPIOE, &GPIO_InitStructure);
//	
//	/*CS always low*/
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_Init(GPIOD, &GPIO_InitStructure);	
//	GPIO_ResetBits(GPIOD , GPIO_Pin_7);		 //CS=0;
}

/*******************************************************************************
* Function Name  : LCD_FSMCConfig
* Description    : Configures the Parallel interface (FSMC) for LCD(Parallel mode)
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void LCD_FSMCConfig(void)
{
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef FSMC_NORSRAMTimingInitStructure;
	
	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;
//	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 


	
	FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 15;//1;   
	FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 0;	   
	FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = 15;//1;	   
	FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
	FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0x00;
	FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0x00;
	FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;	
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;	  
	
	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 
	
	/* Enable FSMC Bank4_SRAM Bank */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);  
}
void LCD_BackLight(int procentai)
{
  if (procentai>100)
    {procentai=100;}
  else if(procentai<0)
    {procentai=0;}
  Channel3Pulse =(uint16_t)(((uint32_t)procentai*(TimerPeriod-1))/100);
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
}

void TIM_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA  , ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
  TimerPeriod = (SystemCoreClock / 17570 ) - 1;
  Channel3Pulse = (uint16_t) (((uint32_t) 99 * (TimerPeriod - 1)) / 100);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
  TIM_Cmd(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
        
}
/*******************************************************************************
* Function Name  : LCD_Configuration
* Description    : Configure the LCD Control pins and FSMC Parallel interface
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void LCD_Configuration(void)
{
	u32 i=0x1fffff;
	/* Configure the LCD Control pins --------------------------------------------*/
	LCD_CtrlLinesConfig();
	while(i--);
	/* Configure the FSMC Parallel interface -------------------------------------*/
	LCD_FSMCConfig();
}

/*******************************************************************************
* Function Name  : delay_ms
* Description    : Delay Time
* Input          : - cnt: Delay Time
* Output         : None
* Return         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void delay_ms(uint16_t ms)    
{ 
	uint16_t i,j; 
	for( i = 0; i < ms; i++ )
	{ 
		for( j = 0; j < 1141; j++ );
	}
} 

/*******************************************************************************
* Function Name  : LCD_Initializtion
* Description    : SSD1963 Resets
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
	uint16_t DeviceCode;
	uint16_t i;

void LCD_Initializtion(void)
{
	LCD_Configuration();
	TIM_Config();
  	LCD_BackLight(100);

	DeviceCode = 0x0123;
	DeviceCode = LCD_ReadReg(0x0000);		/* Read LCD ID	*/	
	
	if( DeviceCode == 0x9325 || DeviceCode == 0x9328 )	
	{
		LCD_Code = ILI9325;
		LCD_WriteReg(0x00e7,0x0010);      
		LCD_WriteReg(0x0000,0x0001);  	/* start internal osc */
		LCD_WriteReg(0x0001,0x0100);     
		LCD_WriteReg(0x0002,0x0700); 	/* power on sequence */
		LCD_WriteReg(0x0003,(1<<12)|(1<<5)|(1<<4)|(0<<3) ); 	/* importance */
		LCD_WriteReg(0x0004,0x0000);                                   
		LCD_WriteReg(0x0008,0x0207);	           
		LCD_WriteReg(0x0009,0x0000);         
		LCD_WriteReg(0x000a,0x0000); 	/* display setting */        
		LCD_WriteReg(0x000c,0x0001);	/* display setting */        
		LCD_WriteReg(0x000d,0x0000); 			        
		LCD_WriteReg(0x000f,0x0000);
		/* Power On sequence */
		LCD_WriteReg(0x0010,0x0000);   
		LCD_WriteReg(0x0011,0x0007);
		LCD_WriteReg(0x0012,0x0000);                                                                 
		LCD_WriteReg(0x0013,0x0000);                 
		delay_ms(50);  /* delay 50 ms */		
		LCD_WriteReg(0x0010,0x1590);   
		LCD_WriteReg(0x0011,0x0227);
		delay_ms(50);  /* delay 50 ms */		
		LCD_WriteReg(0x0012,0x009c);                  
		delay_ms(50);  /* delay 50 ms */		
		LCD_WriteReg(0x0013,0x1900);   
		LCD_WriteReg(0x0029,0x0023);
		LCD_WriteReg(0x002b,0x000e);
		delay_ms(50);  /* delay 50 ms */		
		LCD_WriteReg(0x0020,0x0000);                                                            
		LCD_WriteReg(0x0021,0x0000);           
		delay_ms(50);  /* delay 50 ms */		
		LCD_WriteReg(0x0030,0x0007); 
		LCD_WriteReg(0x0031,0x0707);   
		LCD_WriteReg(0x0032,0x0006);
		LCD_WriteReg(0x0035,0x0704);
		LCD_WriteReg(0x0036,0x1f04); 
		LCD_WriteReg(0x0037,0x0004);
		LCD_WriteReg(0x0038,0x0000);        
		LCD_WriteReg(0x0039,0x0706);     
		LCD_WriteReg(0x003c,0x0701);
		LCD_WriteReg(0x003d,0x000f);
		delay_ms(50);  /* delay 50 ms */		
		LCD_WriteReg(0x0050,0x0000);        
		LCD_WriteReg(0x0051,0x00ef);   
		LCD_WriteReg(0x0052,0x0000);     
		LCD_WriteReg(0x0053,0x013f);
		LCD_WriteReg(0x0060,0xa700);        
		LCD_WriteReg(0x0061,0x0001); 
		LCD_WriteReg(0x006a,0x0000);
		LCD_WriteReg(0x0080,0x0000);
		LCD_WriteReg(0x0081,0x0000);
		LCD_WriteReg(0x0082,0x0000);
		LCD_WriteReg(0x0083,0x0000);
		LCD_WriteReg(0x0084,0x0000);
		LCD_WriteReg(0x0085,0x0000);
		  
		LCD_WriteReg(0x0090,0x0010);     
		LCD_WriteReg(0x0092,0x0000);  
		LCD_WriteReg(0x0093,0x0003);
		LCD_WriteReg(0x0095,0x0110);
		LCD_WriteReg(0x0097,0x0000);        
		LCD_WriteReg(0x0098,0x0000);  
		/* display on sequence */    
		LCD_WriteReg(0x0007,0x0133);
		
		LCD_WriteReg(0x0020,0x0000);
		LCD_WriteReg(0x0021,0x0000);
	}
	else if( DeviceCode == 0x9320 || DeviceCode == 0x9300 )
	{
	    LCD_Code = ILI9320;
	    LCD_WriteReg(0x00,0x0000);
		LCD_WriteReg(0x01,0x0100);	/* Driver Output Contral */
		LCD_WriteReg(0x02,0x0700);	/* LCD Driver Waveform Contral */
		LCD_WriteReg(0x03,0x1018);	/* Entry Mode Set */
		
		LCD_WriteReg(0x04,0x0000);	/* Scalling Contral */
	    LCD_WriteReg(0x08,0x0202);	/* Display Contral */
		LCD_WriteReg(0x09,0x0000);	/* Display Contral 3.(0x0000) */
		LCD_WriteReg(0x0a,0x0000);	/* Frame Cycle Contal.(0x0000) */
	    LCD_WriteReg(0x0c,(1<<0));	/* Extern Display Interface Contral */
		LCD_WriteReg(0x0d,0x0000);	/* Frame Maker Position */
		LCD_WriteReg(0x0f,0x0000);	/* Extern Display Interface Contral 2. */
		
	    delay_ms(100);  /* delay 100 ms */		
		LCD_WriteReg(0x07,0x0101);	/* Display Contral */
	    delay_ms(100);  /* delay 100 ms */		
	
		LCD_WriteReg(0x10,(1<<12)|(0<<8)|(1<<7)|(1<<6)|(0<<4));	/* Power Control 1.(0x16b0)	*/
		LCD_WriteReg(0x11,0x0007);								/* Power Control 2 */
		LCD_WriteReg(0x12,(1<<8)|(1<<4)|(0<<0));				/* Power Control 3.(0x0138)	*/
		LCD_WriteReg(0x13,0x0b00);								/* Power Control 4 */
		LCD_WriteReg(0x29,0x0000);								/* Power Control 7 */
		
		LCD_WriteReg(0x2b,(1<<14)|(1<<4));
			
		LCD_WriteReg(0x50,0);       /* Set X Start */
		LCD_WriteReg(0x51,239);	    /* Set X End */
		LCD_WriteReg(0x52,0);	    /* Set Y Start */
		LCD_WriteReg(0x53,319);	    /* Set Y End */
		
		LCD_WriteReg(0x60,0x2700);	/* Driver Output Control */
		LCD_WriteReg(0x61,0x0001);	/* Driver Output Control */
		LCD_WriteReg(0x6a,0x0000);	/* Vertical Srcoll Control */
		
		LCD_WriteReg(0x80,0x0000);	/* Display Position? Partial Display 1 */
		LCD_WriteReg(0x81,0x0000);	/* RAM Address Start? Partial Display 1 */
		LCD_WriteReg(0x82,0x0000);	/* RAM Address End-Partial Display 1 */
		LCD_WriteReg(0x83,0x0000);	/* Displsy Position? Partial Display 2 */
		LCD_WriteReg(0x84,0x0000);	/* RAM Address Start? Partial Display 2 */
		LCD_WriteReg(0x85,0x0000);	/* RAM Address End? Partial Display 2 */
		
	    LCD_WriteReg(0x90,(0<<7)|(16<<0));	/* Frame Cycle Contral.(0x0013)	*/
		LCD_WriteReg(0x92,0x0000);	/* Panel Interface Contral 2.(0x0000) */
		LCD_WriteReg(0x93,0x0001);	/* Panel Interface Contral 3. */
	    LCD_WriteReg(0x95,0x0110);	/* Frame Cycle Contral.(0x0110)	*/
		LCD_WriteReg(0x97,(0<<8));	
		LCD_WriteReg(0x98,0x0000);	/* Frame Cycle Contral */
	
	    LCD_WriteReg(0x07,0x0173);
	}
	else if( DeviceCode == 0x9331 )
	{
	    LCD_Code = ILI9331;
		LCD_WriteReg(0x00E7, 0x1014);
		LCD_WriteReg(0x0001, 0x0100);   /* set SS and SM bit */
		LCD_WriteReg(0x0002, 0x0200);   /* set 1 line inversion */
		LCD_WriteReg(0x0003, 0x1030);   /* set GRAM write direction and BGR=1 */
		LCD_WriteReg(0x0008, 0x0202);   /* set the back porch and front porch */
	    LCD_WriteReg(0x0009, 0x0000);   /* set non-display area refresh cycle ISC[3:0] */
		LCD_WriteReg(0x000A, 0x0000);   /* FMARK function */
		LCD_WriteReg(0x000C, 0x0000);   /* RGB interface setting */
		LCD_WriteReg(0x000D, 0x0000);   /* Frame marker Position */
		LCD_WriteReg(0x000F, 0x0000);   /* RGB interface polarity */
		/* Power On sequence */
		LCD_WriteReg(0x0010, 0x0000);   /* SAP, BT[3:0], AP, DSTB, SLP, STB	*/
		LCD_WriteReg(0x0011, 0x0007);   /* DC1[2:0], DC0[2:0], VC[2:0] */
		LCD_WriteReg(0x0012, 0x0000);   /* VREG1OUT voltage	*/
		LCD_WriteReg(0x0013, 0x0000);   /* VDV[4:0] for VCOM amplitude */
	    delay_ms(200);  /* delay 200 ms */		
		LCD_WriteReg(0x0010, 0x1690);   /* SAP, BT[3:0], AP, DSTB, SLP, STB	*/
		LCD_WriteReg(0x0011, 0x0227);   /* DC1[2:0], DC0[2:0], VC[2:0] */
	    delay_ms(50);  /* delay 50 ms */		
		LCD_WriteReg(0x0012, 0x000C);   /* Internal reference voltage= Vci	*/
	    delay_ms(50);  /* delay 50 ms */		
		LCD_WriteReg(0x0013, 0x0800);   /* Set VDV[4:0] for VCOM amplitude */
		LCD_WriteReg(0x0029, 0x0011);   /* Set VCM[5:0] for VCOMH */
		LCD_WriteReg(0x002B, 0x000B);   /* Set Frame Rate */
	    delay_ms(50);  /* delay 50 ms */		
		LCD_WriteReg(0x0020, 0x0000);   /* GRAM horizontal Address */
		LCD_WriteReg(0x0021, 0x0000);   /* GRAM Vertical Address */
		/* Adjust the Gamma Curve */
		LCD_WriteReg(0x0030, 0x0000);
		LCD_WriteReg(0x0031, 0x0106);
		LCD_WriteReg(0x0032, 0x0000);
		LCD_WriteReg(0x0035, 0x0204);
		LCD_WriteReg(0x0036, 0x160A);
		LCD_WriteReg(0x0037, 0x0707);
		LCD_WriteReg(0x0038, 0x0106);
		LCD_WriteReg(0x0039, 0x0707);
		LCD_WriteReg(0x003C, 0x0402);
		LCD_WriteReg(0x003D, 0x0C0F);
		/* Set GRAM area */
		LCD_WriteReg(0x0050, 0x0000);   /* Horizontal GRAM Start Address */
		LCD_WriteReg(0x0051, 0x00EF);   /* Horizontal GRAM End Address */
		LCD_WriteReg(0x0052, 0x0000);   /* Vertical GRAM Start Address */
		LCD_WriteReg(0x0053, 0x013F);   /* Vertical GRAM Start Address */
		LCD_WriteReg(0x0060, 0x2700);   /* Gate Scan Line */
		LCD_WriteReg(0x0061, 0x0001);   /*  NDL,VLE, REV */
		LCD_WriteReg(0x006A, 0x0000);   /* set scrolling line */
		/* Partial Display Control */
		LCD_WriteReg(0x0080, 0x0000);
		LCD_WriteReg(0x0081, 0x0000);
		LCD_WriteReg(0x0082, 0x0000);
		LCD_WriteReg(0x0083, 0x0000);
		LCD_WriteReg(0x0084, 0x0000);
		LCD_WriteReg(0x0085, 0x0000);
		/* Panel Control */
		LCD_WriteReg(0x0090, 0x0010);
		LCD_WriteReg(0x0092, 0x0600);
		LCD_WriteReg(0x0007,0x0021);		
	    delay_ms(50);  /* delay 50 ms */		
		LCD_WriteReg(0x0007,0x0061);
	    delay_ms(50);  /* delay 50 ms */		
		LCD_WriteReg(0x0007,0x0133);    /* 262K color and display ON */
	}
	else if( DeviceCode == 0x9919 )
	{
	    LCD_Code = SSD2119;
	    /* POWER ON &RESET DISPLAY OFF */
		LCD_WriteReg(0x28,0x0006);
		LCD_WriteReg(0x00,0x0001);		
		LCD_WriteReg(0x10,0x0000);		
		LCD_WriteReg(0x01,0x72ef);
		LCD_WriteReg(0x02,0x0600);
		LCD_WriteReg(0x03,0x6a38);	
		LCD_WriteReg(0x11,0x6874);
		LCD_WriteReg(0x0f,0x0000);    /* RAM WRITE DATA MASK */
		LCD_WriteReg(0x0b,0x5308);    /* RAM WRITE DATA MASK */
		LCD_WriteReg(0x0c,0x0003);
		LCD_WriteReg(0x0d,0x000a);
		LCD_WriteReg(0x0e,0x2e00);  
		LCD_WriteReg(0x1e,0x00be);
		LCD_WriteReg(0x25,0x8000);
		LCD_WriteReg(0x26,0x7800);
		LCD_WriteReg(0x27,0x0078);
		LCD_WriteReg(0x4e,0x0000);
		LCD_WriteReg(0x4f,0x0000);
		LCD_WriteReg(0x12,0x08d9);
		/* Adjust the Gamma Curve */
		LCD_WriteReg(0x30,0x0000);
		LCD_WriteReg(0x31,0x0104);	 
		LCD_WriteReg(0x32,0x0100);	
	    LCD_WriteReg(0x33,0x0305);	
	    LCD_WriteReg(0x34,0x0505);	 
		LCD_WriteReg(0x35,0x0305);	
	    LCD_WriteReg(0x36,0x0707);	
	    LCD_WriteReg(0x37,0x0300);	
		LCD_WriteReg(0x3a,0x1200);	
		LCD_WriteReg(0x3b,0x0800);		 
	    LCD_WriteReg(0x07,0x0033);
	}
	else if( DeviceCode == 0x1505 || DeviceCode == 0x0505 )
	{
	    LCD_Code = R61505U;
	   /* second release on 3/5  ,luminance is acceptable,water wave appear during camera preview */
	    LCD_WriteReg(0x0007,0x0000);
	    delay_ms(50);  /* delay 50 ms */		
	    LCD_WriteReg(0x0012,0x011C);    /* why need to set several times?	*/
	    LCD_WriteReg(0x00A4,0x0001);    /* NVM */
	    LCD_WriteReg(0x0008,0x000F);
	    LCD_WriteReg(0x000A,0x0008);
	    LCD_WriteReg(0x000D,0x0008);    
	    /* GAMMA CONTROL */
	    LCD_WriteReg(0x0030,0x0707);
	    LCD_WriteReg(0x0031,0x0007); 
	    LCD_WriteReg(0x0032,0x0603); 
	    LCD_WriteReg(0x0033,0x0700); 
	    LCD_WriteReg(0x0034,0x0202); 
	    LCD_WriteReg(0x0035,0x0002); 
	    LCD_WriteReg(0x0036,0x1F0F);
	    LCD_WriteReg(0x0037,0x0707); 
	    LCD_WriteReg(0x0038,0x0000); 
	    LCD_WriteReg(0x0039,0x0000); 
	    LCD_WriteReg(0x003A,0x0707); 
	    LCD_WriteReg(0x003B,0x0000); 
	    LCD_WriteReg(0x003C,0x0007); 
	    LCD_WriteReg(0x003D,0x0000); 
	    delay_ms(50);  /* delay 50 ms */		
	    LCD_WriteReg(0x0007,0x0001);
	    LCD_WriteReg(0x0017,0x0001);    /* Power supply startup enable */
	    delay_ms(50);  /* delay 50 ms */		
	    /* power control */
	    LCD_WriteReg(0x0010,0x17A0); 
	    LCD_WriteReg(0x0011,0x0217);    /* reference voltage VC[2:0]   Vciout = 1.00*Vcivl */
	    LCD_WriteReg(0x0012,0x011E);    /* Vreg1out = Vcilvl*1.80   is it the same as Vgama1out ?	*/
	    LCD_WriteReg(0x0013,0x0F00);    /* VDV[4:0]-->VCOM Amplitude VcomL = VcomH - Vcom Ampl */
	    LCD_WriteReg(0x002A,0x0000);  
	    LCD_WriteReg(0x0029,0x000A);    /* Vcomh = VCM1[4:0]*Vreg1out    gate source voltage?? */
	    LCD_WriteReg(0x0012,0x013E);    /* power supply on */
	    /* Coordinates Control */
	    LCD_WriteReg(0x0050,0x0000);
	    LCD_WriteReg(0x0051,0x00EF); 
	    LCD_WriteReg(0x0052,0x0000); 
	    LCD_WriteReg(0x0053,0x013F); 
	    /* Pannel Image Control */
	    LCD_WriteReg(0x0060,0x2700); 
	    LCD_WriteReg(0x0061,0x0001); 
	    LCD_WriteReg(0x006A,0x0000); 
	    LCD_WriteReg(0x0080,0x0000); 
	    /* Partial Image Control */
	    LCD_WriteReg(0x0081,0x0000); 
	    LCD_WriteReg(0x0082,0x0000); 
	    LCD_WriteReg(0x0083,0x0000); 
	    LCD_WriteReg(0x0084,0x0000); 
	    LCD_WriteReg(0x0085,0x0000); 
	    /* Panel Interface Control */
	    LCD_WriteReg(0x0090,0x0013);      /* frenqucy */	
	    LCD_WriteReg(0x0092,0x0300); 
	    LCD_WriteReg(0x0093,0x0005); 
	    LCD_WriteReg(0x0095,0x0000); 
	    LCD_WriteReg(0x0097,0x0000); 
	    LCD_WriteReg(0x0098,0x0000); 
	  
	    LCD_WriteReg(0x0001,0x0100); 
	    LCD_WriteReg(0x0002,0x0700); 
	    LCD_WriteReg(0x0003,0x1030); 
	    LCD_WriteReg(0x0004,0x0000); 
	    LCD_WriteReg(0x000C,0x0000); 
	    LCD_WriteReg(0x000F,0x0000); 
	    LCD_WriteReg(0x0020,0x0000); 
	    LCD_WriteReg(0x0021,0x0000); 
	    LCD_WriteReg(0x0007,0x0021); 
	    delay_ms(200);  /* delay 200 ms */		
	    LCD_WriteReg(0x0007,0x0061); 
	    delay_ms(200);  /* delay 200 ms */		
	    LCD_WriteReg(0x0007,0x0173); 
	}							 
	else if( DeviceCode == 0x8989 )
	{
	    LCD_Code = SSD1289;
	    LCD_WriteReg(0x0000,0x0001);    delay_ms(50);   /* Enable LCD Oscillator */
	    LCD_WriteReg(0x0003,0xA8A4);    delay_ms(50);   
	    LCD_WriteReg(0x000C,0x0000);    delay_ms(50);   
	    LCD_WriteReg(0x000D,0x080C);    delay_ms(50);   
	    LCD_WriteReg(0x000E,0x2B00);    delay_ms(50);   
	    LCD_WriteReg(0x001E,0x00B0);    delay_ms(50);   
	    LCD_WriteReg(0x0001,0x2B3F);    delay_ms(50);   /*  320*240 0x2B3F */
	    LCD_WriteReg(0x0002,0x0600);    delay_ms(50);
	    LCD_WriteReg(0x0010,0x0000);    delay_ms(50);
	    LCD_WriteReg(0x0011,0x6830);    delay_ms(50);		//0x6070
	    LCD_WriteReg(0x0005,0x0000);    delay_ms(50);
	    LCD_WriteReg(0x0006,0x0000);    delay_ms(50);
	    LCD_WriteReg(0x0016,0xEF1C);    delay_ms(50);
	    LCD_WriteReg(0x0017,0x0003);    delay_ms(50);
	    LCD_WriteReg(0x0007,0x0133);    delay_ms(50);         
	    LCD_WriteReg(0x000B,0x0000);    delay_ms(50);
	    LCD_WriteReg(0x000F,0x0000);    delay_ms(50);
	    LCD_WriteReg(0x0041,0x0000);    delay_ms(50);
	    LCD_WriteReg(0x0042,0x0000);    delay_ms(50);
	    LCD_WriteReg(0x0048,0x0000);    delay_ms(50);
	    LCD_WriteReg(0x0049,0x013F);    delay_ms(50);
	    LCD_WriteReg(0x004A,0x0000);    delay_ms(50);
	    LCD_WriteReg(0x004B,0x0000);    delay_ms(50);
	    LCD_WriteReg(0x0044,0xEF00);    delay_ms(50);
	    LCD_WriteReg(0x0045,0x0000);    delay_ms(50);
	    LCD_WriteReg(0x0046,0x013F);    delay_ms(50);
	    LCD_WriteReg(0x0030,0x0707);    delay_ms(50);
	    LCD_WriteReg(0x0031,0x0204);    delay_ms(50);
	    LCD_WriteReg(0x0032,0x0204);    delay_ms(50);
	    LCD_WriteReg(0x0033,0x0502);    delay_ms(50);
	    LCD_WriteReg(0x0034,0x0507);    delay_ms(50);
	    LCD_WriteReg(0x0035,0x0204);    delay_ms(50);
	    LCD_WriteReg(0x0036,0x0204);    delay_ms(50);
	    LCD_WriteReg(0x0037,0x0502);    delay_ms(50);
	    LCD_WriteReg(0x003A,0x0302);    delay_ms(50);
	    LCD_WriteReg(0x003B,0x0302);    delay_ms(50);
	    LCD_WriteReg(0x0023,0x0000);    delay_ms(50);
	    LCD_WriteReg(0x0024,0x0000);    delay_ms(50);
	    LCD_WriteReg(0x0025,0x8000);    delay_ms(50);
	    LCD_WriteReg(0x004f,0);
	    LCD_WriteReg(0x004e,0);

		//LCD_WriteReg(0x0011,0x6068);
	}
	else if( DeviceCode == 0x8999 )
	{
		LCD_Code = SSD1298;		
		LCD_WriteReg(0x0028,0x0006);
		LCD_WriteReg(0x0000,0x0001);
		LCD_WriteReg(0x0003,0xaea4);    /* power control 1---line frequency and VHG,VGL voltage */
		LCD_WriteReg(0x000c,0x0004);    /* power control 2---VCIX2 output voltage */
		LCD_WriteReg(0x000d,0x000c);    /* power control 3---Vlcd63 voltage */
		LCD_WriteReg(0x000e,0x2800);    /* power control 4---VCOMA voltage VCOML=VCOMH*0.9475-VCOMA */
		LCD_WriteReg(0x001e,0x00b5);    /* POWER CONTROL 5---VCOMH voltage */   
		LCD_WriteReg(0x0001,0x3b3f);     
		LCD_WriteReg(0x0002,0x0600);
		LCD_WriteReg(0x0010,0x0000);
		LCD_WriteReg(0x0011,0x6830);
		LCD_WriteReg(0x0005,0x0000);
		LCD_WriteReg(0x0006,0x0000);
		LCD_WriteReg(0x0016,0xef1c);  
		LCD_WriteReg(0x0007,0x0033);    /* Display control 1 */
		/* when GON=1 and DTE=0,all gate outputs become VGL */
		/* when GON=1 and DTE=0,all gate outputs become VGH */
		/* non-selected gate wires become VGL */     
		LCD_WriteReg(0x000b,0x0000);
		LCD_WriteReg(0x000f,0x0000);
		LCD_WriteReg(0x0041,0x0000);
		LCD_WriteReg(0x0042,0x0000);
		LCD_WriteReg(0x0048,0x0000);
		LCD_WriteReg(0x0049,0x013f);
		LCD_WriteReg(0x004a,0x0000);
		LCD_WriteReg(0x004b,0x0000); 
		LCD_WriteReg(0x0044,0xef00);	/* Horizontal RAM start and end address */
		LCD_WriteReg(0x0045,0x0000);	/* Vretical RAM start address */
		LCD_WriteReg(0x0046,0x013f);	/* Vretical RAM end address */ 
		LCD_WriteReg(0x004e,0x0000);	/* set GDDRAM x address counter */
		LCD_WriteReg(0x004f,0x0000);    /* set GDDRAM y address counter */   
		/* y control */
		LCD_WriteReg(0x0030,0x0707);
		LCD_WriteReg(0x0031,0x0202);
		LCD_WriteReg(0x0032,0x0204);
		LCD_WriteReg(0x0033,0x0502);
		LCD_WriteReg(0x0034,0x0507);
		LCD_WriteReg(0x0035,0x0204);
		LCD_WriteReg(0x0036,0x0204);
		LCD_WriteReg(0x0037,0x0502);
		LCD_WriteReg(0x003a,0x0302);
		LCD_WriteReg(0x003b,0x0302); 
		LCD_WriteReg(0x0023,0x0000);
		LCD_WriteReg(0x0024,0x0000);
		LCD_WriteReg(0x0025,0x8000);
		LCD_WriteReg(0x0026,0x7000);
		LCD_WriteReg(0x0020,0xb0eb);
		LCD_WriteReg(0x0027,0x007c);
	}
	else if( DeviceCode == 0x5408 )
	{
		LCD_Code = SPFD5408B;
		
		LCD_WriteReg(0x0001,0x0100);	  /* Driver Output Contral Register */ 
		LCD_WriteReg(0x0002,0x0700);      /* LCD Driving Waveform Contral */
		LCD_WriteReg(0x0003,0x1030);
		
		LCD_WriteReg(0x0004,0x0000);	  /* Scalling Control register */
		LCD_WriteReg(0x0008,0x0207);	  /* Display Control 2 */
		LCD_WriteReg(0x0009,0x0000);	  /* Display Control 3 */
		LCD_WriteReg(0x000A,0x0000);	  /* Frame Cycle Control */
		LCD_WriteReg(0x000C,0x0000);	  /* External Display Interface Control 1 */
		LCD_WriteReg(0x000D,0x0000);      /* Frame Maker Position */
		LCD_WriteReg(0x000F,0x0000);	  /* External Display Interface Control 2 */
		delay_ms(50);
		LCD_WriteReg(0x0007,0x0101);	  /* Display Control */
		delay_ms(50);
		LCD_WriteReg(0x0010,0x16B0);      /* Power Control 1 */
		LCD_WriteReg(0x0011,0x0001);      /* Power Control 2 */
		LCD_WriteReg(0x0017,0x0001);      /* Power Control 3 */
		LCD_WriteReg(0x0012,0x0138);      /* Power Control 4 */
		LCD_WriteReg(0x0013,0x0800);      /* Power Control 5 */
		LCD_WriteReg(0x0029,0x0009);	  /* NVM read data 2 */
		LCD_WriteReg(0x002a,0x0009);	  /* NVM read data 3 */
		LCD_WriteReg(0x00a4,0x0000);  
		LCD_WriteReg(0x0050,0x0000);
		LCD_WriteReg(0x0051,0x00EF);
		LCD_WriteReg(0x0052,0x0000);
		LCD_WriteReg(0x0053,0x013F);
		   
		LCD_WriteReg(0x0060,0x2700);	  /* Driver Output Control */

		LCD_WriteReg(0x0061,0x0003);	  /* Driver Output Control */
		LCD_WriteReg(0x006A,0x0000);	  /* Vertical Scroll Control */
		
		LCD_WriteReg(0x0080,0x0000);	  /* Display Position 每 Partial Display 1 */
		LCD_WriteReg(0x0081,0x0000);	  /* RAM Address Start 每 Partial Display 1 */
		LCD_WriteReg(0x0082,0x0000);	  /* RAM address End - Partial Display 1 */
		LCD_WriteReg(0x0083,0x0000);	  /* Display Position 每 Partial Display 2 */
		LCD_WriteReg(0x0084,0x0000);	  /* RAM Address Start 每 Partial Display 2 */
		LCD_WriteReg(0x0085,0x0000);	  /* RAM address End 每 Partail Display2 */
		LCD_WriteReg(0x0090,0x0013);	  /* Frame Cycle Control */
		LCD_WriteReg(0x0092,0x0000); 	  /* Panel Interface Control 2 */
		LCD_WriteReg(0x0093,0x0003);	  /* Panel Interface control 3 */
		LCD_WriteReg(0x0095,0x0110);	  /* Frame Cycle Control */
		LCD_WriteReg(0x0007,0x0173);
	}
	else if( DeviceCode == 0x4531 )
	{	
		LCD_Code = LGDP4531;
		/* Setup display */
		LCD_WriteReg(0x00,0x0001);
		LCD_WriteReg(0x10,0x0628);
		LCD_WriteReg(0x12,0x0006);
		LCD_WriteReg(0x13,0x0A32);
		LCD_WriteReg(0x11,0x0040);
		LCD_WriteReg(0x15,0x0050);
		LCD_WriteReg(0x12,0x0016);
		delay_ms(50);
		LCD_WriteReg(0x10,0x5660);
		delay_ms(50);
		LCD_WriteReg(0x13,0x2A4E);
		LCD_WriteReg(0x01,0x0100);
		LCD_WriteReg(0x02,0x0300);	
		LCD_WriteReg(0x03,0x1030);		
		LCD_WriteReg(0x08,0x0202);
		LCD_WriteReg(0x0A,0x0000);
		LCD_WriteReg(0x30,0x0000);
		LCD_WriteReg(0x31,0x0402);
		LCD_WriteReg(0x32,0x0106);
		LCD_WriteReg(0x33,0x0700);
		LCD_WriteReg(0x34,0x0104);
		LCD_WriteReg(0x35,0x0301);
		LCD_WriteReg(0x36,0x0707);
		LCD_WriteReg(0x37,0x0305);
		LCD_WriteReg(0x38,0x0208);
		LCD_WriteReg(0x39,0x0F0B);
		delay_ms(50);
		LCD_WriteReg(0x41,0x0002);
		LCD_WriteReg(0x60,0x2700);
		LCD_WriteReg(0x61,0x0001);
		LCD_WriteReg(0x90,0x0119);
		LCD_WriteReg(0x92,0x010A);
		LCD_WriteReg(0x93,0x0004);
		LCD_WriteReg(0xA0,0x0100);
		delay_ms(50);
		LCD_WriteReg(0x07,0x0133);
		delay_ms(50);
		LCD_WriteReg(0xA0,0x0000);
	}
	else if( DeviceCode == 0x4535 )
	{	
		LCD_Code = LGDP4535;	
		LCD_WriteReg(0x15, 0x0030); 	 /* Set the internal vcore voltage */                                              
		LCD_WriteReg(0x9A, 0x0010); 	 /* Start internal OSC */
		LCD_WriteReg(0x11, 0x0020);	     /* set SS and SM bit */
		LCD_WriteReg(0x10, 0x3428);	     /* set 1 line inversion */
		LCD_WriteReg(0x12, 0x0002);	     /* set GRAM write direction and BGR=1 */ 
		LCD_WriteReg(0x13, 0x1038);	     /* Resize register */
		delay_ms(40); 
		LCD_WriteReg(0x12, 0x0012);	     /* set the back porch and front porch */
		delay_ms(40); 
		LCD_WriteReg(0x10, 0x3420);	     /* set non-display area refresh cycle ISC[3:0] */
		LCD_WriteReg(0x13, 0x3045);	     /* FMARK function */
		delay_ms(70); 
		LCD_WriteReg(0x30, 0x0000);      /* RGB interface setting */
		LCD_WriteReg(0x31, 0x0402);	     /* Frame marker Position */
		LCD_WriteReg(0x32, 0x0307);      /* RGB interface polarity */
		LCD_WriteReg(0x33, 0x0304);      /* SAP, BT[3:0], AP, DSTB, SLP, STB */
		LCD_WriteReg(0x34, 0x0004);      /* DC1[2:0], DC0[2:0], VC[2:0] */
		LCD_WriteReg(0x35, 0x0401);      /* VREG1OUT voltage */
		LCD_WriteReg(0x36, 0x0707);      /* VDV[4:0] for VCOM amplitude */
		LCD_WriteReg(0x37, 0x0305);      /* SAP, BT[3:0], AP, DSTB, SLP, STB */
		LCD_WriteReg(0x38, 0x0610);      /* DC1[2:0], DC0[2:0], VC[2:0] */
		LCD_WriteReg(0x39, 0x0610);      /* VREG1OUT voltage */
		LCD_WriteReg(0x01, 0x0100);      /* VDV[4:0] for VCOM amplitude */
		LCD_WriteReg(0x02, 0x0300);      /* VCM[4:0] for VCOMH */
		LCD_WriteReg(0x03, 0x1030);      /* GRAM horizontal Address */
		LCD_WriteReg(0x08, 0x0808);      /* GRAM Vertical Address */
		LCD_WriteReg(0x0A, 0x0008);		
		LCD_WriteReg(0x60, 0x2700);	     /* Gate Scan Line */
		LCD_WriteReg(0x61, 0x0001);	     /* NDL,VLE, REV */
		LCD_WriteReg(0x90, 0x013E);
		LCD_WriteReg(0x92, 0x0100);
		LCD_WriteReg(0x93, 0x0100);
		LCD_WriteReg(0xA0, 0x3000);
		LCD_WriteReg(0xA3, 0x0010);
		LCD_WriteReg(0x07, 0x0001);
		LCD_WriteReg(0x07, 0x0021);
		LCD_WriteReg(0x07, 0x0023);
		LCD_WriteReg(0x07, 0x0033);
		LCD_WriteReg(0x07, 0x0133);
	} 		 		
	else if( DeviceCode == 0x0047 )
	{
		LCD_Code = HX8347D;
		/* Start Initial Sequence */
		LCD_WriteReg(0xEA,0x00);                          
		LCD_WriteReg(0xEB,0x20);                                                     
		LCD_WriteReg(0xEC,0x0C);                                                   
		LCD_WriteReg(0xED,0xC4);                                                    
		LCD_WriteReg(0xE8,0x40);                                                     
		LCD_WriteReg(0xE9,0x38);                                                    
		LCD_WriteReg(0xF1,0x01);                                                    
		LCD_WriteReg(0xF2,0x10);                                                    
		LCD_WriteReg(0x27,0xA3);                                                    
		/* GAMMA SETTING */
		LCD_WriteReg(0x40,0x01);                           
		LCD_WriteReg(0x41,0x00);                                                   
		LCD_WriteReg(0x42,0x00);                                                   
		LCD_WriteReg(0x43,0x10);                                                    
		LCD_WriteReg(0x44,0x0E);                                                   
		LCD_WriteReg(0x45,0x24);                                                  
		LCD_WriteReg(0x46,0x04);                                                  
		LCD_WriteReg(0x47,0x50);                                                   
		LCD_WriteReg(0x48,0x02);                                                    
		LCD_WriteReg(0x49,0x13);                                                  
		LCD_WriteReg(0x4A,0x19);                                                  
		LCD_WriteReg(0x4B,0x19);                                                 
		LCD_WriteReg(0x4C,0x16);                                                 
		LCD_WriteReg(0x50,0x1B);                                                   
		LCD_WriteReg(0x51,0x31);                                                    
		LCD_WriteReg(0x52,0x2F);                                                     
		LCD_WriteReg(0x53,0x3F);                                                    
		LCD_WriteReg(0x54,0x3F);                                                     
		LCD_WriteReg(0x55,0x3E);                                                     
		LCD_WriteReg(0x56,0x2F);                                                   
		LCD_WriteReg(0x57,0x7B);                                                     
		LCD_WriteReg(0x58,0x09);                                                  
		LCD_WriteReg(0x59,0x06);                                                 
		LCD_WriteReg(0x5A,0x06);                                                   
		LCD_WriteReg(0x5B,0x0C);                                                   
		LCD_WriteReg(0x5C,0x1D);                                                   
		LCD_WriteReg(0x5D,0xCC);                                                   
		/* Power Voltage Setting */
		LCD_WriteReg(0x1B,0x18);                                                    
		LCD_WriteReg(0x1A,0x01);                                                    
		LCD_WriteReg(0x24,0x15);                                                    
		LCD_WriteReg(0x25,0x50);                                                    
		LCD_WriteReg(0x23,0x8B);                                                   
		LCD_WriteReg(0x18,0x36);                           
		LCD_WriteReg(0x19,0x01);                                                    
		LCD_WriteReg(0x01,0x00);                                                   
		LCD_WriteReg(0x1F,0x88);                                                    
		delay_ms(50);
		LCD_WriteReg(0x1F,0x80);                                                  
		delay_ms(50);
		LCD_WriteReg(0x1F,0x90);                                                   
		delay_ms(50);
		LCD_WriteReg(0x1F,0xD0);                                                   
		delay_ms(50);
		LCD_WriteReg(0x17,0x05);                                                    
		LCD_WriteReg(0x36,0x00);                                                    
		LCD_WriteReg(0x28,0x38);                                                 
		delay_ms(50);
		LCD_WriteReg(0x28,0x3C);                                                
	}
	else if( DeviceCode == 0x7783 )
	{
		LCD_Code = ST7781;
		/* Start Initial Sequence */
		LCD_WriteReg(0x00FF,0x0001);
		LCD_WriteReg(0x00F3,0x0008);
		LCD_WriteReg(0x0001,0x0100);
		LCD_WriteReg(0x0002,0x0700);
		LCD_WriteReg(0x0003,0x1030);  
		LCD_WriteReg(0x0008,0x0302);
		LCD_WriteReg(0x0008,0x0207);
		LCD_WriteReg(0x0009,0x0000);
		LCD_WriteReg(0x000A,0x0000);
		LCD_WriteReg(0x0010,0x0000);  
		LCD_WriteReg(0x0011,0x0005);
		LCD_WriteReg(0x0012,0x0000);
		LCD_WriteReg(0x0013,0x0000);
		delay_ms(50);
		LCD_WriteReg(0x0010,0x12B0);
		delay_ms(50);
		LCD_WriteReg(0x0011,0x0007);
		delay_ms(50);
		LCD_WriteReg(0x0012,0x008B);
		delay_ms(50);	
		LCD_WriteReg(0x0013,0x1700);
		delay_ms(50);	
		LCD_WriteReg(0x0029,0x0022);		
		LCD_WriteReg(0x0030,0x0000);
		LCD_WriteReg(0x0031,0x0707);
		LCD_WriteReg(0x0032,0x0505);
		LCD_WriteReg(0x0035,0x0107);
		LCD_WriteReg(0x0036,0x0008);
		LCD_WriteReg(0x0037,0x0000);
		LCD_WriteReg(0x0038,0x0202);
		LCD_WriteReg(0x0039,0x0106);
		LCD_WriteReg(0x003C,0x0202);
		LCD_WriteReg(0x003D,0x0408);
		delay_ms(50);				
		LCD_WriteReg(0x0050,0x0000);		
		LCD_WriteReg(0x0051,0x00EF);		
		LCD_WriteReg(0x0052,0x0000);		
		LCD_WriteReg(0x0053,0x013F);		
		LCD_WriteReg(0x0060,0xA700);		
		LCD_WriteReg(0x0061,0x0001);
		LCD_WriteReg(0x0090,0x0033);				
		LCD_WriteReg(0x002B,0x000B);		
		LCD_WriteReg(0x0007,0x0133);
	}
	else	/* special ID */
	{
		DeviceCode = 0x0123;
		DeviceCode = LCD_ReadReg(0x67);
		if( DeviceCode == 0x0047 )
		{
	        LCD_Code = HX8347A;
	        LCD_WriteReg(0x0042,0x0008);   
	        /* Gamma setting */  
		    LCD_WriteReg(0x0046,0x00B4); 
		    LCD_WriteReg(0x0047,0x0043); 
		    LCD_WriteReg(0x0048,0x0013); 
		    LCD_WriteReg(0x0049,0x0047); 
		    LCD_WriteReg(0x004A,0x0014); 
		    LCD_WriteReg(0x004B,0x0036); 
		    LCD_WriteReg(0x004C,0x0003); 
		    LCD_WriteReg(0x004D,0x0046); 
		    LCD_WriteReg(0x004E,0x0005);  
		    LCD_WriteReg(0x004F,0x0010);  
		    LCD_WriteReg(0x0050,0x0008);  
		    LCD_WriteReg(0x0051,0x000a);  
		    /* Window Setting */
		    LCD_WriteReg(0x0002,0x0000); 
		    LCD_WriteReg(0x0003,0x0000); 
		    LCD_WriteReg(0x0004,0x0000); 
		    LCD_WriteReg(0x0005,0x00EF); 
		    LCD_WriteReg(0x0006,0x0000); 
		    LCD_WriteReg(0x0007,0x0000); 
		    LCD_WriteReg(0x0008,0x0001); 
		    LCD_WriteReg(0x0009,0x003F); 
		    delay_ms(10); 
		    LCD_WriteReg(0x0001,0x0006); 
		    LCD_WriteReg(0x0016,0x00C8);   
		    LCD_WriteReg(0x0023,0x0095); 
		    LCD_WriteReg(0x0024,0x0095); 
		    LCD_WriteReg(0x0025,0x00FF); 
		    LCD_WriteReg(0x0027,0x0002); 
		    LCD_WriteReg(0x0028,0x0002); 
		    LCD_WriteReg(0x0029,0x0002); 
		    LCD_WriteReg(0x002A,0x0002); 
		    LCD_WriteReg(0x002C,0x0002); 
		    LCD_WriteReg(0x002D,0x0002); 
		    LCD_WriteReg(0x003A,0x0001);  
		    LCD_WriteReg(0x003B,0x0001);  
		    LCD_WriteReg(0x003C,0x00F0);    
		    LCD_WriteReg(0x003D,0x0000); 
		    delay_ms(20); 
		    LCD_WriteReg(0x0035,0x0038); 
		    LCD_WriteReg(0x0036,0x0078); 
		    LCD_WriteReg(0x003E,0x0038); 
		    LCD_WriteReg(0x0040,0x000F); 
		    LCD_WriteReg(0x0041,0x00F0);  
		    LCD_WriteReg(0x0038,0x0000); 
		    /* Power Setting */ 
		    LCD_WriteReg(0x0019,0x0049);  
		    LCD_WriteReg(0x0093,0x000A); 
		    delay_ms(10); 
		    LCD_WriteReg(0x0020,0x0020);   
		    LCD_WriteReg(0x001D,0x0003);   
		    LCD_WriteReg(0x001E,0x0000);  
		    LCD_WriteReg(0x001F,0x0009);   
		    LCD_WriteReg(0x0044,0x0053);  
		    LCD_WriteReg(0x0045,0x0010);   
		    delay_ms(10);  
		    LCD_WriteReg(0x001C,0x0004);  
		    delay_ms(20); 
		    LCD_WriteReg(0x0043,0x0080);    
		    delay_ms(5); 
		    LCD_WriteReg(0x001B,0x000a);    
		    delay_ms(40);  
		    LCD_WriteReg(0x001B,0x0012);    
		    delay_ms(40);   
		    /* Display On Setting */ 
		    LCD_WriteReg(0x0090,0x007F); 
		    LCD_WriteReg(0x0026,0x0004); 
		    delay_ms(40);  
		    LCD_WriteReg(0x0026,0x0024); 
		    LCD_WriteReg(0x0026,0x002C); 
		    delay_ms(40);   
		    LCD_WriteReg(0x0070,0x0008); 
		    LCD_WriteReg(0x0026,0x003C);  
		    LCD_WriteReg(0x0057,0x0002); 
		    LCD_WriteReg(0x0055,0x0000); 
		    LCD_WriteReg(0x0057,0x0000); 
		}	 
	}  						
    delay_ms(50);   /* delay 50 ms */		
}

/*******************************************************************************
* Function Name  : LCD_Clear
* Description    : 
* Input          : - Color: Screen Color
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_Clear(uint16_t Color)
{
	uint32_t index=0;
	
	if( LCD_Code == HX8347D || LCD_Code == HX8347A )
	{
		LCD_WriteReg(0x02,0x00);                                                  
		LCD_WriteReg(0x03,0x00);  
		                
		LCD_WriteReg(0x04,0x00);                           
		LCD_WriteReg(0x05,0xEF);  
		                 
		LCD_WriteReg(0x06,0x00);                           
		LCD_WriteReg(0x07,0x00);    
		               
		LCD_WriteReg(0x08,0x01);                           
		LCD_WriteReg(0x09,0x3F);     
	}
	else
	{	
		LCD_SetCursor(0,0); 
	}	

	LCD_WriteIndex(0x0022);
	for( index = 0; index < MAX_X * MAX_Y; index++ )
	{
		LCD_WriteData(Color);
	}

}

static void LCD_SetCursor( uint16_t Xpos, uint16_t Ypos )
{				   
    #if  ( DISP_ORIENTATION == 90 ) || ( DISP_ORIENTATION == 270 )

		uint16_t temp;
		Ypos = ( MAX_Y - 1 ) - Ypos;
		temp = Ypos;
		Ypos = Xpos;
		Xpos = temp; 

	#elif  ( DISP_ORIENTATION == 0 ) || ( DISP_ORIENTATION == 180 )

		Ypos = ( MAX_Y - 1 ) - Ypos;
			
	#endif
		   
  switch( LCD_Code )
  {
     default:		 /* 0x9320 0x9325 0x9328 0x9331 0x5408 0x1505 0x0505 0x7783 0x4531 0x4535 */
          LCD_WriteReg(0x0020, Xpos );     
          LCD_WriteReg(0x0021, Ypos );     
	      break; 

     case SSD1298: 	 /* 0x8999 */
     case SSD1289:   /* 0x8989 */
	      LCD_WriteReg(0x004e, Xpos );      
        LCD_WriteReg(0x004f, Ypos );          
	      break;  

     case HX8347A: 	 /* 0x0047 */
     case HX8347D: 	 /* 0x0047 */
	      LCD_WriteReg(0x02, Xpos>>8 );                                                  
	      LCD_WriteReg(0x03, Xpos );  

	      LCD_WriteReg(0x06, Ypos>>8 );                           
	      LCD_WriteReg(0x07, Ypos );    
	
	      break;     
     case SSD2119:	 /* 3.5 LCD 0x9919 */
	      break; 
  }
}

/******************************************************************************
* Function Name  : LCD_BGR2RGB
* Description    : RRRRRGGGGGGBBBBB convert to BBBBBGGGGGGRRRRR
* Input          : RGB color
* Output         : None
* Return         : RGB color
* Attention		 :
*******************************************************************************/
static uint16_t LCD_BGR2RGB(uint16_t color)
{
	uint16_t  r, g, b, rgb;
	
	b = ( color>>0 )  & 0x1f;
	g = ( color>>5 )  & 0x3f;
	r = ( color>>11 ) & 0x1f;
	
	rgb =  (b<<11) + (g<<5) + (r<<0);
	
	return( rgb );
}

/******************************************************************************
* Function Name  : LCD_GetPoint
* Description    : Get color of the point
* Input          : - Xpos: Row Coordinate
*                  - Ypos: Line Coordinate 
* Output         : None
* Return         : Screen Color
* Attention		 : None
*******************************************************************************/
uint16_t LCD_GetPoint(uint16_t Xpos,uint16_t Ypos)
{
	uint16_t dummy;
	
	LCD_SetCursor(Xpos,Ypos);

	LCD_WriteIndex(0x0022);  
	
	switch( LCD_Code )
	{
		case ST7781:
		case LGDP4531:
		case LGDP4535:
		case SSD1289:
		case SSD1298:
      dummy = LCD_ReadData();
      dummy = LCD_ReadData(); 

 		  return  dummy;	      
    case HX8347A:
	  case HX8347D:
    {
      uint8_t red,green,blue;
      red = LCD_ReadData()>>3;
      green = LCD_ReadData()>>3; 
      blue = LCD_ReadData()>>2; 
      dummy = ( green << 11 ) | (blue << 5 ) | red;
		}
      return  dummy;
    default:	/* 0x9320 0x9325 0x9328 0x9331 0x5408 0x1505 0x0505 0x9919 */
      dummy = LCD_ReadData();
      dummy = LCD_ReadData(); 
      return  LCD_BGR2RGB( dummy );
	}
}

/******************************************************************************
* Function Name  : LCD_SetPoint
* Description    : 
* Input          : - Xpos: Row Coordinate
*                  - Ypos: Line Coordinate 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_SetPoint(uint16_t Xpos,uint16_t Ypos,uint16_t point)
{
	if( Xpos >= MAX_X || Ypos >= MAX_Y )
	{
		return;
	}
	LCD_SetCursor(Xpos,Ypos);
	LCD_WriteReg(0x0022,point);
}



/******************************************************************************
* Function Name  : LCD_DrawLine
* Description    : Bresenham's line algorithm
* Input          : - x0:
*                  - y0:
*       				   - x1:
*       		       - y1:
*                  - color:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/	 
void LCD_DrawLine( uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 , uint16_t color )
{
  short dx,dy;
  short temp;

  if( x0 > x1 )
  {
    temp = x1;
    x1 = x0;
    x0 = temp;   
  }
  if( y0 > y1 )
  {
    temp = y1;
    y1 = y0;
    y0 = temp;   
  }

  dx = x1-x0;
  dy = y1-y0;

  if( dx == 0 )
  {
    do
    { 
      LCD_SetPoint(x0, y0, color);
      y0++;
    }while( y1 >= y0 ); 
    return; 
  }
  if( dy == 0 )
  {
    do
    {
      LCD_SetPoint(x0, y0, color);
      x0++;
    }
    while( x1 >= x0 ); 
		return;
  }

	/* Bresenham's line algorithm  */
  if( dx > dy )
  {
    temp = 2 * dy - dx;
    while( x0 != x1 )
    {
	    LCD_SetPoint(x0,y0,color);
	    x0++;
	    if( temp > 0 )
	    {
	      y0++;
	      temp += 2 * dy - 2 * dx; 
	 	  }
      else         
      {
			  temp += 2 * dy;
			}       
    }
    LCD_SetPoint(x0,y0,color);
  }  
  else
  {
    temp = 2 * dx - dy;
    while( y0 != y1 )
    {
	 	  LCD_SetPoint(x0,y0,color);     
      y0++;                 
      if( temp > 0 )           
      {
        x0++;               
        temp+=2*dy-2*dx; 
      }
      else
			{
        temp += 2 * dy;
			}
    } 
    LCD_SetPoint(x0,y0,color);
	}
} 

/******************************************************************************
* Function Name  : PutChar
* Description    : 
* Input          : - Xpos:
*                  - Ypos:
*				           - ASCI:
*				           - charColor:
*				           - bkColor:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void PutChar(uint16_t Xpos, uint16_t Ypos, uint8_t ASCII, uint16_t charColor, uint16_t bkColor)
{
	SSD1289_PutCharFont(Xpos, Ypos, ASCII, charColor, bkColor, SYSTEM_8x16);
}

void SSD1289_PutCharFont( uint16_t Xpos, uint16_t Ypos, uint8_t ASCII, uint16_t charColor, uint16_t bkColor, uint16_t FONTx)
{
	uint16_t i, j;
    uint8_t buffer[16], tmp_char;
    uint8_t len_x, len_y;

    switch (FONTx)
    {
    	case FONT6x8:
    		len_x = 6;
    		len_y = 8;
    		break;    	
    	case FONT8x8:
    		len_x = 8;
    		len_y = 8;
    		break;
    	case MS_GOTHIC_8x16:
    	case SYSTEM_8x16:
    	default:
    		len_x = 8;
    		len_y = 16;
    		break;
    }

    GetASCIICode(buffer, ASCII, FONTx);
    for( i=0; i<len_y; i++ )
    {
        tmp_char = buffer[i];
        for( j=0; j<len_x; j++ )
        {
            if( (((tmp_char >> (7 - j))) & 0x01) == 0x01 )
            {
                LCD_SetPoint( Xpos + j, Ypos + i, charColor );
            }
            else
            {
                LCD_SetPoint( Xpos + j, Ypos + i, bkColor );
            }
        }
    }
}

void SSD1289_Text(uint16_t Xpos, uint16_t Ypos, uint8_t *str, uint16_t Color, uint16_t bkColor)
{
	SSD1289_TextFont(Xpos, Ypos, str, Color, bkColor, SYSTEM_8x16);
}

void SSD1289_TextFont(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor, uint16_t FONTx)
{
    uint8_t TempChar;
    uint8_t delta_x, delta_y;

    switch (FONTx)
    {
    	case FONT6x8:
    		delta_x = 6;
    		delta_y = 8;
    		break;    	
    	case FONT8x8:
    		delta_x = 8;
    		delta_y = 8;
    		break;
    	case MS_GOTHIC_8x16:
    	case SYSTEM_8x16:
    	default:
    		delta_x = 8;
    		delta_y = 16;
    		break;
    }

    do
    {
        TempChar = *str++;  
        SSD1289_PutCharFont( Xpos, Ypos, TempChar, Color, bkColor, FONTx );    
        if( Xpos < MAX_X - delta_x )
        {
            Xpos += delta_x;
        } 
        else if ( Ypos < MAX_Y - delta_y )
        {
            Xpos = 0;
            Ypos += delta_y;
        }   
        else
        {
            Xpos = 0;
            Ypos = 0;
        }    
    }
    while ( *str != 0 );
}

/******************************************************************************
* Function Name  : GUI_Text
* Description    : 
* Input          : - Xpos: 
*                  - Ypos: 
*				           - str:
*				           - charColor:
*				           - bkColor:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void GUI_Text(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor)
{
    uint8_t TempChar;
    do
    {
        TempChar = *str++;  
        PutChar( Xpos, Ypos, TempChar, Color, bkColor );    
        if( Xpos < MAX_X - 8 )
        {
            Xpos += 8;
        } 
        else if ( Ypos < MAX_Y - 16 )
        {
            Xpos = 0;
            Ypos += 16;
        }   
        else
        {
            Xpos = 0;
            Ypos = 0;
        }    
    }
    while ( *str != 0 );
}
///////////////////////////////
void SSD1289_CleanPutCharFont( uint16_t Xpos, uint16_t Ypos, uint8_t ASCII, uint16_t charColor, uint16_t FONTx)
{
	uint16_t i, j;
    uint8_t buffer[16], tmp_char;
    uint8_t len_x, len_y;

    switch (FONTx)
    {
    	case FONT6x8:
    		len_x = 6;
    		len_y = 8;
    		break;    	
    	case FONT8x8:
    		len_x = 8;
    		len_y = 8;
    		break;
    	case MS_GOTHIC_8x16:
    	case SYSTEM_8x16:
    	default:
    		len_x = 8;
    		len_y = 16;
    		break;
    }

    GetASCIICode(buffer, ASCII, FONTx);
    for( i=0; i<len_y; i++ )
    {
        tmp_char = buffer[i];
        for( j=0; j<len_x; j++ )
        {
            if( (tmp_char >> 7 - j) & 0x01 == 0x01 )
            {
                LCD_SetPoint( Xpos + j, Ypos + i, charColor );
            }
        }
    }	
}
void SSD1289_CleanTextFont(uint16_t Xpos, uint16_t Ypos, uint8_t *str, uint16_t Color, uint16_t FONTx)
{
    uint8_t TempChar;

    do
    {
        TempChar = *str++;  
        SSD1289_CleanPutCharFont( Xpos, Ypos, TempChar, Color, FONTx);    
        if( Xpos < MAX_X - 8 )
        {
            Xpos += 8;
        } 
        else if ( Ypos < MAX_Y - 16 )
        {
            Xpos = 0;
            Ypos += 16;
        }   
        else
        {
            Xpos = 0;
            Ypos = 0;
        }    
    }
    while ( *str != 0 );
}

void SSD1289_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	uint16_t x_index, y_index;

	if (x > MAX_X)
		x = MAX_X;
	if (y > MAX_Y)
		y = MAX_Y;

	if ((x+w) > MAX_X)
		w = MAX_X - x;

	if ((y+h) > MAX_Y)
		h = MAX_Y - y;

	for(x_index = x; x_index < x+w; x_index++)
	{
		for(y_index = y; y_index < y+h; y_index++)
		{
			LCD_SetPoint(x_index, y_index, color);
		}
	}

}

void SSD1289_FillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color)
{
	int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    int16_t i;

    for (i=y0-r; i<=y0+r; i++)
        LCD_SetPoint(x0, i, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
  
        for (i=y0-y; i<=y0+y; i++) {
            LCD_SetPoint(x0+x, i, color);
            LCD_SetPoint(x0-x, i, color);
        } 
        for (i=y0-x; i<=y0+x; i++) {
            LCD_SetPoint(x0+y, i, color);
            LCD_SetPoint(x0-y, i, color);
        }    
    }
}
void SSD1289_DrawPicture(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *pic)
{
	uint16_t x_index, y_index;
	uint32_t pic_index = 0;

    //for(y_index = y; y_index < (y+h); y_index++)
    //if wrong orientation
    for(y_index = (y+h); y_index > 0 ; y_index--)
    {
        for(x_index = x; x_index < (x+w); x_index++)
        {
			LCD_SetPoint(x_index, y_index, pic[pic_index]);
			pic_index++;
		}
	} 
}

void SSD1289_DrawPicture8bit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pic)
{
	uint16_t color;
	uint16_t x_index, y_index;
	uint32_t pic_index = 0;

    //for(y_index = y; y_index < (y+h); y_index++)
    //if wrong orientation
	for(y_index = (y+h); y_index > 0 ; y_index--)
	{
		for(x_index = x; x_index < (x+w); x_index++)
		{
			color = (pic[pic_index+1] << 8) | (pic[pic_index]);
			LCD_SetPoint(x_index, y_index, color);
			pic_index += 2;
		}
	} 
}
/*******************************************************************************
* Function Name  : LCD_WriteReg
* Description    : 
* Input          : - index:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
 void LCD_WriteIndex(uint16_t index)
{
	//GPIO_ResetBits(GPIOD , GPIO_Pin_7);		 //CS=0;
	LCD_REG	= index;
	//GPIO_SetBits(GPIOD , GPIO_Pin_7);		 //CS=1;
}

/*******************************************************************************
* Function Name  : LCD_WriteReg
* Description    : 
* Input          : - index:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
 void LCD_WriteData(uint16_t data)
{
	//GPIO_ResetBits(GPIOD , GPIO_Pin_7);		 //CS=0;
	LCD_RAM = data;
	//GPIO_SetBits(GPIOD , GPIO_Pin_7);		 //CS=1;
}

/*******************************************************************************
* Function Name  : LCD_ReadData
* Description    : 
* Input          : None
* Output         : None
* Return         : 
* Attention		 : None
*******************************************************************************/
 uint16_t LCD_ReadData(void)
{
	//uint32_t tmp;
	//GPIO_ResetBits(GPIOD , GPIO_Pin_7);		 //CS=0;
	//tmp = LCD_RAM;
	//GPIO_SetBits(GPIOD , GPIO_Pin_7);		 //CS=1;
	return LCD_RAM;
}


/*******************************************************************************
* Function Name  : LCD_WriteReg
* Description    : Writes to the selected LCD register.
* Input          : - LCD_Reg: address of the selected register.
*                  - LCD_RegValue: value to write to the selected register.
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
 void LCD_WriteReg(uint16_t LCD_Reg,uint16_t LCD_RegValue)
{ 
	/* Write 16-bit Index, then Write Reg */  
	LCD_WriteIndex(LCD_Reg);         
	/* Write 16-bit Reg */
	LCD_WriteData(LCD_RegValue);  
}

/*******************************************************************************
* Function Name  : LCD_WriteReg
* Description    : Reads the selected LCD Register.
* Input          : None
* Output         : None
* Return         : LCD Register Value.
* Attention		 : None
*******************************************************************************/
 uint16_t LCD_ReadReg(uint16_t LCD_Reg)
{
	/* Write 16-bit Index (then Read Reg) */
	LCD_WriteIndex(LCD_Reg);

	/* Read 16-bit Reg */
	return LCD_ReadData();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t  ili9320_BRG2RGB(uint32_t Color)
{
  uint16_t  r, g, b, rgb;

  b = (Color>>0)  & 0x1f;
  g = (Color>>5)  & 0x3f;
  r = (Color>>11) & 0x1f;
  
  rgb =  (b<<11) + (g<<5) + (r<<0);

  return( rgb );
} 


void ili9320PixelDraw(void *pvDisplayData, long x, long y,unsigned long point)
{
  if ( (x>320)||(y>240) ) return;
  LCD_SetCursor(x,y);
  //Clr_Cs;
  LCD_WriteIndex(0x0022);
  LCD_WriteData(point);
  //Set_Cs;
}
// void LCD_SetWindows(u16 StartX,u16 StartY,u16 EndX,u16 EndY)
//{
//     LCD_SetCursor(StartX,StartY);
//     LCD_WriteReg(0x44,StartY|(EndY<<8));
//     LCD_WriteReg(0x45,StartX);
//     LCD_WriteReg(0x46,EndX);
//}

 void LCD_SetWindows(u16 StartX,u16 StartY,u16 EndX,u16 EndY)
{
     LCD_SetCursor(StartX,StartY);
     LCD_WriteReg(0x50,StartX);
		 LCD_WriteReg(0x51,EndX);
     LCD_WriteReg(0x52,StartY);
     LCD_WriteReg(0x53,EndY);
}

 void ili9320DrawMultiple(void *pvDisplayData, long lX,
                                           long lY, long lX0, long lCount,
                                           long lBPP,
                                           const unsigned char *pucData,
                                           const unsigned char *pucPalette)
{    unsigned int ulByte;
	LCD_WriteReg(0x03,0x1038); ////////////mac dinh:0x1030
	   LCD_SetCursor(lX,lY);
		 //Clr_Cs;
     LCD_WriteIndex(0x0022);
		switch(lBPP)
    {
        //
        // The pixel data is in 1 bit per pixel format.
        //
        case 1:
        {
            //
            // Loop while there are more pixels to draw.
            //
            while(lCount)
            {
                //
                // Get the next byte of image data.
                //
                ulByte = *pucData++;

                //
                // Loop through the pixels in this byte of image data.
                //
                for(; (lX0 < 8) && lCount; lX0++, lCount--)
                {
                    //
                    // Draw this pixel in the appropriate color.
                    //
                    LCD_WriteData(((unsigned long *)pucPalette)[(ulByte >>
                                                             (7 - lX0)) & 1]);
                }

                //
                // Start at the beginning of the next byte of image data.
                //
                lX0 = 0;
            }

            //
            // The image data has been drawn.
            //
            break;
        }

        //
        // The pixel data is in 4 bit per pixel format.
        //
        case 4:
        {
            //
            // Loop while there are more pixels to draw.  "Duff's device" is
            // used to jump into the middle of the loop if the first nibble of
            // the pixel data should not be used.  Duff's device makes use of
            // the fact that a case statement is legal anywhere within a
            // sub-block of a switch statement.  See
            // http://en.wikipedia.org/wiki/Duff's_device for detailed
            // information about Duff's device.
            //
            switch(lX0 & 1)
            {
                case 0:
                    while(lCount)
                    {
                        //
                        // Get the upper nibble of the next byte of pixel data
                        // and extract the corresponding entry from the
                        // palette.
                        //
                        ulByte = (*pucData >> 4) * 3;
                        ulByte = (*(unsigned long *)(pucPalette + ulByte) &
                                  0x00ffffff);

                        //
                        // Translate this palette entry and write it to the
                        // screen.
                        //
                         LCD_WriteData(DPYCOLORTRANSLATE(ulByte));

                        //
                        // Decrement the count of pixels to draw.
                        //
                        lCount--;

                        //
                        // See if there is another pixel to draw.
                        //
                        if(lCount)
                        {
                case 1:
                            //
                            // Get the lower nibble of the next byte of pixel
                            // data and extract the corresponding entry from
                            // the palette.
                            //
                            ulByte = (*pucData++ & 15) * 3;
                            ulByte = (*(unsigned long *)(pucPalette + ulByte) &
                                      0x00ffffff);

                            //
                            // Translate this palette entry and write it to the
                            // screen.
                            //
                             LCD_WriteData(DPYCOLORTRANSLATE(ulByte));

                            //
                            // Decrement the count of pixels to draw.
                            //
                            lCount--;
                        }
                    }
            }

            //
            // The image data has been drawn.
            //
            break;
        }

        //
        // The pixel data is in 8 bit per pixel format.
        //
        case 8:
        {
            //
            // Loop while there are more pixels to draw.
            //
            while(lCount--)
            {
                //
                // Get the next byte of pixel data and extract the
                // corresponding entry from the palette.
                //
                ulByte = *pucData++ * 3;
                ulByte = *(unsigned long *)(pucPalette + ulByte) & 0x00ffffff;

                //
                // Translate this palette entry and write it to the screen.
                //
                 LCD_WriteData(DPYCOLORTRANSLATE(ulByte));
            }

            //
            // The image data has been drawn.
            //
            break;
        }

        //
        // We are being passed data in the display's native format.  Merely
        // write it directly to the display.  This is a special case which is
        // not used by the graphics library but which is helpful to
        // applications which may want to handle, for example, JPEG images.
        //
        case 16:
        {
            unsigned short usByte;

            //
            // Loop while there are more pixels to draw.
            //
            while(lCount--)
            {
                //
                // Get the next byte of pixel data and extract the
                // corresponding entry from the palette.
                //
                usByte = *((unsigned short *)pucData);
                pucData += 2;

                //
                // Translate this palette entry and write it to the screen.
                //
                 LCD_WriteData(usByte);
            }
        }
    }
   //Set_Cs; 
		//LCD_SetWindows(0,0,319,239);
}

 void ili9320RectFill(void *pvDisplayData, const tRectangle *pRect,unsigned long ulValue)
{ 
    long lCount;
    for(lCount = pRect->sYMin ; lCount <= pRect->sYMax; lCount++)ili9320LineDrawH(&pvDisplayData, pRect->sXMin, pRect->sXMax,lCount,ulValue);		
}

 void ili9320LineDrawH(void *pvDisplayData, long lX1, long lX2,
                                   long lY, unsigned long ulValue)
{    LCD_WriteReg(0x03,0x1038); //
	   LCD_SetCursor(lX1,lY);
     //Clr_Cs;
     LCD_WriteIndex(0x0022);
     while(lX1++ <= lX2)LCD_WriteData(ulValue);
     //Set_Cs;
	   //LCD_WriteReg(0x03,0x1030); //reset ve mac dinh
     //LCD_WriteIndex(0x0022);
}

 void ili9320LineDrawV(void *pvDisplayData, long lX, long lY1,
                                   long lY2, unsigned long ulValue)
{ LCD_WriteReg(0x03,0x1018); //
  LCD_SetCursor(lX,lY1);
  //Clr_Cs;
  LCD_WriteIndex(0x0022);
  while(lY1++ <= lY2)LCD_WriteData(ulValue);
  //Set_Cs;
	//LCD_WriteReg(0x03,0x1030); //reset ve mac dinh
  //LCD_WriteIndex(0x0022);
}

void ili9320Flush(void *pvDisplayData){}

void GrContextInit(tContext *pContext, const tDisplay *pDisplay)
{
    pContext->lSize = sizeof(tContext);
    pContext->pDisplay = pDisplay;
    pContext->sClipRegion.sXMin = 0;
    pContext->sClipRegion.sYMin = 0;
    pContext->sClipRegion.sXMax = pDisplay->usWidth - 1;
    pContext->sClipRegion.sYMax = pDisplay->usHeight - 1;
    pContext->ulForeground = 0;
    pContext->ulBackground = 0;
    pContext->pFont = 0;
}
void GrContextClipRegionSet(tContext *pContext, tRectangle *pRect)
{
    unsigned long ulW, ulH;
    ulW = DpyWidthGet(pContext->pDisplay);
    ulH = DpyHeightGet(pContext->pDisplay);
    pContext->sClipRegion.sXMin = ((pRect->sXMin < 0) ? 0 :
                                   ((pRect->sXMin >= ulW) ? (ulW - 1) :
                                    pRect->sXMin));
    pContext->sClipRegion.sYMin = ((pRect->sYMin < 0) ? 0 :
                                   ((pRect->sYMin >= ulH) ? (ulH - 1) :
                                    pRect->sYMin));
    pContext->sClipRegion.sXMax = ((pRect->sXMax < 0) ? 0 :
                                   ((pRect->sXMax >= ulW) ? (ulW - 1) :
                                    pRect->sXMax));
    pContext->sClipRegion.sYMax = ((pRect->sYMax < 0) ? 0 :
                                   ((pRect->sYMax >= ulH) ? (ulH - 1) :
                                    pRect->sYMax));
}
void GrLineDrawH(const tContext *pContext, long lX1, long lX2, long lY)
{
    long lTemp;
    if((lY < pContext->sClipRegion.sYMin) ||
       (lY > pContext->sClipRegion.sYMax))
    {
        return;
    }
    if(lX1 > lX2)
    {
        lTemp = lX1;
        lX1 = lX2;
        lX2 = lTemp;
    }
    if((lX1 > pContext->sClipRegion.sXMax) ||
       (lX2 < pContext->sClipRegion.sXMin))
    {
        return;
    }
    if(lX1 < pContext->sClipRegion.sXMin)
    {
        lX1 = pContext->sClipRegion.sXMin;
    }
    if(lX2 > pContext->sClipRegion.sXMax)
    {
        lX2 = pContext->sClipRegion.sXMax;
    }
    DpyLineDrawH(pContext->pDisplay, lX1, lX2, lY, pContext->ulForeground);
}

void
GrLineDrawV(const tContext *pContext, long lX, long lY1, long lY2)
{
    long lTemp;
    if((lX < pContext->sClipRegion.sXMin) ||
       (lX > pContext->sClipRegion.sXMax))
    {
        return;
    }
    if(lY1 > lY2)
    {
        lTemp = lY1;
        lY1 = lY2;
        lY2 = lTemp;
    }
    if((lY1 > pContext->sClipRegion.sYMax) ||
       (lY2 < pContext->sClipRegion.sYMin))
    {
        return;
    }
    if(lY1 < pContext->sClipRegion.sYMin)
    {
        lY1 = pContext->sClipRegion.sYMin;
    }
    if(lY2 > pContext->sClipRegion.sYMax)
    {
        lY2 = pContext->sClipRegion.sYMax;
    }
    DpyLineDrawV(pContext->pDisplay, lX, lY1, lY2, pContext->ulForeground);
}
static long
GrClipCodeGet(const tContext *pContext, long lX, long lY)
{
    long lCode;
    lCode = 0;
    if(lY < pContext->sClipRegion.sYMin)
    {
        lCode |= 1;
    }
    if(lY > pContext->sClipRegion.sYMax)
    {
        lCode |= 2;
    }
    if(lX < pContext->sClipRegion.sXMin)
    {
        lCode |= 4;
    }
    if(lX > pContext->sClipRegion.sXMax)
    {
        lCode |= 8;
    }
    return(lCode);
}
static long
GrLineClip(const tContext *pContext, long *plX1, long *plY1, long *plX2,
           long *plY2)
{
    long lCode, lCode1, lCode2, lX, lY;
    lCode1 = GrClipCodeGet(pContext, *plX1, *plY1);
    lCode2 = GrClipCodeGet(pContext, *plX2, *plY2);
    while(1)
    {
        if((lCode1 == 0) && (lCode2 == 0))
        {
            return(1);
        }
        if((lCode1 & lCode2) != 0)
        {
            return(0);
        }
        if(lCode1)
        {
            lCode = lCode1;
        }
        else
        {
            lCode = lCode2;
        }
        if(lCode & 1)
        {
            lX = (*plX1 + (((*plX2 - *plX1) *
                            (pContext->sClipRegion.sYMin - *plY1)) /
                           (*plY2 - *plY1)));
            lY = pContext->sClipRegion.sYMin;
        }
        else if(lCode & 2)
        {
            lX = (*plX1 + (((*plX2 - *plX1) *
                            (pContext->sClipRegion.sYMax - *plY1)) /
                           (*plY2 - *plY1)));
            lY = pContext->sClipRegion.sYMax;
        }
        else if(lCode & 4)
        {
            lX = pContext->sClipRegion.sXMin;
            lY = (*plY1 + (((*plY2 - *plY1) *
                            (pContext->sClipRegion.sXMin - *plX1)) /
                           (*plX2 - *plX1)));
        }
        else
        {
            lX = pContext->sClipRegion.sXMax;
            lY = (*plY1 + (((*plY2 - *plY1) *
                            (pContext->sClipRegion.sXMax - *plX1)) /
                           (*plX2 - *plX1)));
        }
        if(lCode1)
        {
            *plX1 = lX;
            *plY1 = lY;
            lCode1 = GrClipCodeGet(pContext, lX, lY);
        }
        else
        {
            *plX2 = lX;
            *plY2 = lY;

            lCode2 = GrClipCodeGet(pContext, lX, lY);
        }
    }
}

void
GrLineDraw(const tContext *pContext, long lX1, long lY1, long lX2, long lY2)
{
    long deltax,deltay,heso,i,tam;
    if(lX1 == lX2)
    {
        GrLineDrawV(pContext, lX1, lY1, lY2);
        return;
    }

    if(lY1 == lY2)
    {
      
        GrLineDrawH(pContext, lX1, lX2, lY1);

    
        return;
    }
    if(GrLineClip(pContext, &lX1, &lY1, &lX2, &lY2) == 0)
    {
        return;
    }
		deltax= lX2-lX1; deltay =lY2-lY1;
		heso=lX2*lY1 -lX1*lY2;
    if(((lY2 > lY1) ? (lY2 - lY1) : (lY1 - lY2)) >
       ((lX2 > lX1) ? (lX2 - lX1) : (lX1 - lX2)))
    {
     if(lY1>lY2){i=lY1;lY1=lY2;lY2=i;}
     for(i=lY1;i<=lY2;i++)
     {   tam = (deltax*i - heso)/deltay;
         DpyPixelDraw(pContext->pDisplay, tam, i, pContext->ulForeground);
     }
				
    }
    else
    {
      if(lX1>lX2){i=lX1;lX1=lX2;lX2=i;}
      for(i=lX1;i<=lX2;i++)
      {   tam = (deltay*i + heso)/deltax;
          DpyPixelDraw(pContext->pDisplay, i, tam, pContext->ulForeground);
      }
    }
   
}
void LineBMP(unsigned char hang, unsigned int x0,unsigned int d0, unsigned char *mang )
{  
	 u16 *bitmap_ptr = (u16*)mang,i;
	 LCD_SetCursor(x0,hang);
	 LCD_WriteReg(0x03,0x1018); 
   //Clr_Cs;
   LCD_WriteIndex(0x0022);
   for(i=0;i<=d0;i++)LCD_WriteData(*bitmap_ptr++);
   //Set_Cs; 
}
void GrTriangleDrawFill(const tContext *pContext,u16 x1,u16 y1, u16 x2, u16 y2, u16 x3, u16 y3)
{
	   signed short x[3],y[3],n,yt1,yt2,yt,dx1,dx2,dx3,dy1,dy2,dy3; 
		 signed long sh1,sh2,sh3;
		 x[0]=x1; x[1]=x2; x[2]=x3;y[0]=y1; y[1]=y2; y[2]=y3;
		 if(x[0]>x[1]){yt=x[0];x[0]=x[1];x[1]=yt;yt=y[0];y[0]=y[1];y[1]=yt;}
		 if(x[0]>x[2]){yt=x[0];x[0]=x[2];x[2]=yt;yt=y[0];y[0]=y[2];y[2]=yt;}
		 if(x[1]>x[2]){yt=x[1];x[1]=x[2];x[2]=yt;yt=y[1];y[1]=y[2];y[2]=yt;}
     dx1= x[2]-x[0]; dx2= x[1]-x[0];dx3=x[2]-x[1];
		 dy1= y[2]-y[0]; dy2= y[1]-y[0];dy3=y[2]-y[1];
		 sh1=x[2]*y[0] - x[0]*y[2]; sh2=x[1]*y[0] - x[0]*y[1]; sh3=x[2]*y[1] - x[1]*y[2];
		 
     for(n=x[0] ; n<x[1]; n++)
		 {
			  yt1= (dy1*n + sh1)/dx1;
				yt2= (dy2*n + sh2)/dx2;
			  if(yt2<yt1){yt= yt1; yt1=yt2; yt2=yt;}
		    GrLineDrawV(pContext, n, yt1, yt2);
		 }
		 for(n=x[1] ; n<x[2]; n++)
		 {
			  yt1= (dy1*n + sh1)/dx1;
				yt2= (dy3*n + sh3)/dx3;
			  if(yt2<yt1){yt= yt1; yt1=yt2; yt2=yt;}
		    GrLineDrawV(pContext, n, yt1, yt2);
		 }	 
}

void GrBar(const tContext *pContext,unsigned short x1, unsigned short y1,unsigned short x2, unsigned short y2, unsigned char width)
{
   unsigned char        half_width;
   signed long  addx=1, addy=1, j,P, diff, c1, c2,dy, dx;
   unsigned short i=0;
   if(x2>x1)dx = x2 - x1; else dx = x1 - x2;
   if(y2>y1)dy = y2 - y1; else dy = y1 - y2;

   half_width = width/2;
   c1 = -(dx*x1 + dy*y1);
   c2 = -(dx*x2 + dy*y2);

   if(x1 > x2)
   {
      signed short temp;
      temp = c1;
      c1 = c2;
      c2 = temp;
      addx = -1;
   }
   if(y1 > y2)
   {
      signed short temp;
      temp = c1;
      c1 = c2;
      c2 = temp;
      addy = -1;
   }

   if(dx >= dy)
   {
      P = 2*dy - dx;
      diff = P - dx;

      for(i=0; i<=dx; ++i)
      {
         for(j=-half_width; j<half_width+width%2; ++j)
         {
							 DpyPixelDraw(pContext->pDisplay, x1, y1+j, pContext->ulForeground);
         }
         if(P < 0)
         {
            P  += 2*dy;
            x1 += addx;
         }
         else
         {
            P  += diff;
            x1 += addx;
            y1 += addy;
         }
      }
   }
   else
   {
      P = 2*dx - dy;
      diff = P - dy;

      for(i=0; i<=dy; ++i)
      {
         if(P < 0)
         {
            P  += 2*dx;
            y1 += addy;
         }
         else
         {
            P  += diff;
            x1 += addx;
            y1 += addy;
         }
         for(j=-half_width; j<half_width+width%2; ++j)
         {
							 DpyPixelDraw(pContext->pDisplay, x1+j, y1, pContext->ulForeground);
         }
      }
   }
}

const tDisplay ili9320 =
{
    sizeof(tDisplay),
    0,
    320,
    240,
    ili9320PixelDraw,
    ili9320DrawMultiple,
    ili9320LineDrawH,
    ili9320LineDrawV,
    ili9320RectFill,
    ili9320ColorTranslate,
    ili9320Flush
};




/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
