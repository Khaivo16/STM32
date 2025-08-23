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


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
/////////////////////////////////////////////////////////////////////////////
void myfunc(void *data);
void myfunc1(void *data);
void myfunc2(void *data);
	 /*define the objects of the Timer */
void myfunc(void *data)
{
    Serial.println("TIMER 1S" );
}

void myfunc2(void *data)
{
  
      static int i;
      i++;
      Serial.println("myfunc2" );    
      if (i == 2)//2x3000=6s
      {

				digitalWrite( PD13, LOW );
				i=0;
      }
}
void myfunc1(void *data)
{
     static int i;
     i++;
	Serial.println("myfunc1" );
	if (i == 3) //3x3000=9s
     {         

			 digitalWrite( PD13, HIGH );
			 i=0;
      }
		 
     
}
/////////////////////////////////////////////////////////////////////////////
uint8_t toggle=0;
void test_callback(HardwareTimer *htim)
{
  togglePin(PB12);
}
//void Update_IT_callback(HardwareTimer *htim)
//{ // Update event correspond to Rising edge of PWM when configured in PWM1 mode
//  digitalWrite(PD14, LOW); // pin2 will be complementary to pin
//}

//void Compare_IT_callback(HardwareTimer *htim)
//{ // Compare match event correspond to falling edge of PWM when configured in PWM1 mode
//  digitalWrite(PD14, HIGH);
//}

/* USER CODE END PFP */
	


int main(void)
{
	

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */

	//////////////////////////////////////////////////////////////////////

	 /* Initialize all configured peripherals */
  //MX_GPIO_Init();
	//pinMode(PB6,OUTPUT);
	pinMode(PB12,OUTPUT);
	
	delay(100);
	
		Serial.begin(9600);
		//printf( "LVD_IC \n" );
		Serial.println("Hello!");
		//Serial.println(123);
		
				
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////	
		/* Configure HardwareTimer */
//  HardwareTimer *EthTim = new HardwareTimer(TIM4);
//  EthTim->setMode(1, TIMER_OUTPUT_COMPARE,0);

//  /* Timer set to 1ms */
//  EthTim->setOverflow(5000000, MICROSEC_FORMAT);
//  EthTim->attachInterrupt(test_callback);
//  EthTim->resume();
	
	/////////////////////////
	
//	// Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup function is finished.
//  HardwareTimer *MyTim = new HardwareTimer(TIM2);

//  MyTim->setMode(4, TIMER_OUTPUT_COMPARE_PWM2,PA2);
//  // MyTim->setPrescaleFactor(8); // Due to setOverflow with MICROSEC_FORMAT, prescaler will be computed automatically based on timer input clock
//  MyTim->setOverflow(100000, MICROSEC_FORMAT); // 100000 microseconds = 100 milliseconds
//  MyTim->setCaptureCompare(4,10,HERTZ_COMPARE_FORMAT); // 50%
////  MyTim->attachInterrupt(Update_IT_callback);
////  MyTim->attachInterrupt(4, Compare_IT_callback);
//  MyTim->resume();
//////////////////////////////
//	uint32_t channel = 0;
//	switch (g_APinDescription[pin].ulTimerChannel) {
//    case TIM_CHANNEL_1:
//      channel = 1;
//      break;
//    case TIM_CHANNEL_2:
//      channel = 2;
//      break;
//    case TIM_CHANNEL_3:
//      channel = 3;
//      break;
//    case TIM_CHANNEL_4:
//      channel = 4;
//      break;
//    default:
//      channel = -1;
//  }  
HardwareTimer *MyTim = new HardwareTimer(TIM2);
// Configure and start PWM
//	MyTim->setPWM(TIM_CHANNEL_2, PA2, 5, 10, NULL, NULL); // No callback required, we can simplify the function call
//  MyTim->setPWM(1, PA2, 5, 50); // 5 Hertz, 10% dutycycle
	
	pwmStart(PA2, 5000, 10, HERTZ_COMPARE_FORMAT);//// 5 KHertz, 10% dutycycle
//	pwmStart(PB6, 5000, 0, RESOLUTION_8B_COMPARE_FORMAT);//// 5 KHertz, value=8bit... 0-255...
//	pwmStop(PB6);
/////////////////////////////////////////////////////////////////
		
  while (1)
  {
		
//		togglePin(PB12);
//		delay_ms(50);	
		
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
