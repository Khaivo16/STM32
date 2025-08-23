/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

#include "Arduino.h"


/* USER CODE BEGIN Includes */
#include "P5.h"
/* USER CODE END Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>  
#include <stdint.h>
#include <stdbool.h>
/* Private variables ---------------------------------------------------------*/

//-----------dinh nghia cua mode 0 -----------//
#define NUM1_X  2
#define NUM2_X  17
#define NUM3_X  33
#define NUM4_X  49
#define Dau2c_X 28

//-----------dinh nghia cua mode 4 = COUNT4 -----------//
#define N11_X  0
#define N22_X  2
#define N33_X  18
#define N44_X  33
#define N55_X  49
//#define Dau2c_X 28
//-----------dinh nghia cua mode 4 = COUNT5 -----------//
#define N1_X  0
#define N2_X  12
#define N3_X  25
#define N4_X  38
#define N5_X  51


TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

extern unsigned char display[3][32][64];

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define DELETE 9999
char ef_time=0;
char gio_auto;
char AUTO_EF;

int tt_DH=0;
uint8_t	hieu_ung=0;

uint16_t String_read(int add);
void String_SaveData(void);
void manhinhchinh(int mode,char inphut);
void mode0_inTime(int x,char loai);

unsigned char mauchuChay[3]={0,31,0};

unsigned char font_number=0,font_number2; // ADD=40
char mode_hienthi=0;         // ADD=41
struct
{
	unsigned char Hours[3];
	unsigned char Minute[3];
	unsigned char Seconds[3];
	unsigned char Day[3];
	unsigned char Date[3];
	unsigned char Month[3];
	unsigned char Year[3];
	unsigned char ND[3];
}CoLor;

uint16_t rpm=0;
void manhinhchinh(int mode,char inphut)
{
		
	uint8_t aa=0,bb=0,cc=0,dd=0;
	 //man hinh 1
	 if(mode==0)
	 {
		 
		 
		 int dich1=0;
		 if (rpm/100==2)
				dich1=1;
		 Set_color(25,0,0);  //in gio
		 P5_sendnumber_font1((NUM1_X+dich1),1,(rpm/100)%10,font_number);
		 int dich2=0;
		 if ((rpm/10)%10==2)
				dich2=1;
		 Set_color(25,0,0);  //in gio
		 P5_sendnumber_font1((NUM2_X+dich2),1,(rpm/10)%10,font_number);
		 
		 Set_color(20,20,0);       // in phut
		 P5_sendnumber_font1(30,1,11,0); 
		 
		 Set_color(25,0,0);       // in phut
		 int dich3=0;
		 if (rpm%10==2)
				dich3=1;
		 P5_sendnumber_font1((NUM3_X+dich3) ,1,rpm%10,0);
		 
		 Set_color(0,0,25);       // in phut
		 P5_sendnumber_font1(46+1,1,12,0); 
		 Set_color(25,25,0);    
		 P5_sendnumber_font1(NUM4_X+1,1,10,0); 
		 //mode0_inTime(0,1);
		 
		 
		 P5_chonvitri(32,24);
		 P5_sendStringFontAS57_wColor((unsigned char *)"70",20,20,20) ;
	   P5_chonvitri(52,24);
		 P5_sendStringFontAS57_wColor((unsigned char *)"%",0,16,0) ;

	 }
	
}

#include "HardwareSerial.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
#include "Wire.h" 
//            SDA  SCL
//TwoWire Wire2(PB3, PB10);

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
/////////////////////////////////////////////////////////////////////////////
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
                                    
//void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/////////////////////////////////////////////////////////////////////////////////////
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
//	{
//		if(htim->Instance==TIM1) for(int i=0;i<64;i++)data_M[i]--;////////////////TIM? BI THIEU////////////////// 
//		else 
//		if(htim->Instance==TIM4)ngatquetled();					
//		 //else if(htim->Instance==TIM3)IR_ngatT();
//	}
	
void P5_ngatquetled(HardwareTimer *htim)
{
  UNUSED(htim);
	ngatquetled();
	
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //ngat ngoai exti
{
//	   if(GPIO_Pin == IR_PIN) //neu tin hieu den tu chan IR 
//          IR_ngatPin(); 
}
////////////////////////////////////////////////////////////////////////////////////

int main(void)
{



  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
	pinMode(PC13,OUTPUT);digitalWrite(PC13, HIGH);
	//////////////////////////////////////////////////////////////////////

	 /* Initialize all configured peripherals */
  	

//		Serial.begin(115200);
//		//printf( "LVD_IC \n" );
//		//Serial.println("Please ");
//		Serial.println(123);
		
				
///////////////////////////////////////////////////////////////////
//		Wire.setSDA(PB7); // using pin name PY_n
//    Wire.setSCL(PB6); // using pin number PYn
//Wire.begin();//B9, B8		//Wire.begin(true); or Wire.begin(0x70,true);
//		delay(500);
		
/////////////////////////////////////////////////////////////////
	SET_dosang(100);
	MX_GPIO_Init();
  //MX_TIM2_Init();

	pwmMode( PB11,150000, PWM_8_BIT);
	MX_TIM4_Init();

	
	
	//		   HAL_TIM_Base_Start_IT(&htim4);
	//		   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,490);//GET_dosang()4
				 
	//time1=time2=time3=time4=time5=time6=time_sp=HAL_GetTick();
  //set_px(35, 12, 31,0,0);
	
//	P5_chonvitri(0,0);
//	Set_color(31,31,31);
//	P5_sendString((unsigned char *)"CN 123456789");

	P5_chonvitri(0,0);
	P5_sendString_wColor((unsigned char *)"1234",31,31,31);
  while (1)
  {
//		manhinhchinh(0,1);
//		delay(100);		
//		rpm=rpm+1;	
	////////////////////////////////////////////////////
	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/

	/**
  * @brief  System clock configuration:
  *             System clock source = PLL (HSE)
  *             SYSCLK(Hz)          = 72000000
  *             HCLK(Hz)            = 72000000
  *             AHB prescaler       = 1
  *             APB1 prescaler      = 2
  *             APB2 prescaler      = 1
  *             HSE frequency(Hz)   = 8000000
  *             HSE PREDIV1         = 1
  *             PLLMUL              = 9
  *             Flash latency(WS)   = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
    

}


/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/





/* TIM2 init function */
static void MX_TIM2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
	__HAL_RCC_TIM2_CLK_ENABLE();
	//HAL_TIM_MspPostInit(&htim2);
	/**TIM2 GPIO Configuration    
    PB11     ------> TIM2_CH4 
    */
	__HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_TIM2_PARTIAL_2();
	 	
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 400;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
}



/* TIM4 init function */
static void MX_TIM4_Init(void)
{

//  TIM_ClockConfigTypeDef sClockSourceConfig;
//  TIM_MasterConfigTypeDef sMasterConfig;
//	
//	__HAL_RCC_TIM4_CLK_ENABLE();
//    /* TIM4 interrupt Init */
//    HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
//    HAL_NVIC_EnableIRQ(TIM4_IRQn);

//  htim4.Instance = TIM4;
//  htim4.Init.Prescaler = 24000;
//  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim4.Init.Period = 38;
//  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  //htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }

			/* Configure HardwareTimer */
  HardwareTimer *p5Tim = new HardwareTimer(TIM4);
  p5Tim->setMode(1, TIMER_OUTPUT_COMPARE,0);

  /* Timer set to 10khz */
  p5Tim->setOverflow(10000, HERTZ_FORMAT);
  p5Tim->attachInterrupt(P5_ngatquetled);
  p5Tim->resume();

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

   GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11 |GPIO_PIN_12|GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 PA6 PA7 
                           PA8 PA9 PA10 PA11 
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB12 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11 |GPIO_PIN_12|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */
/**
* @brief This function handles EXTI line3 interrupt.
*/
//void EXTI0_IRQHandler(void)
//{

//  //HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
//	/* EXTI line interrupt detected */
//   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
//   {
//      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
//      HAL_GPIO_EXTI_Callback(GPIO_PIN_0);
//			
//		 //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0));
////		 	dem++; time_sp=HAL_GetTick();	
////			if(dem>9999) dem=0;
//			
//    }


//}
/* USER CODE END 4 */

//was "__weak void HAL_SYSTICK_Callback(void)" in stm32f4xx_hal_cortex.c
void HAL_SYSTICK_Callback(void)
{
	
	
}

/* USER CODE BEGIN 4 */

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	
//		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
//				
//		HAL_UART_DMA_Tx_Stop(&huartdma);//HAL_UART_DMAStop
//			
//}	

////void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
////    if (huart==&uartdma) serial_available_1 = 1;
////    //if (huart==&huart2) serial_available_2 = 1;
////}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{

//		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//			
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
