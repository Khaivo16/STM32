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

#include "sys.h"
//#include "spi.h"
#include "usart.h"

/* USER CODE BEGIN Includes */


///* USER CODE END Includes */

///* Private variables ---------------------------------------------------------*/


///* Private variables ---------------------------------------------------------*/


SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

///* USER CODE BEGIN PV */
///* Private variables ---------------------------------------------------------*/
void GPIO_Init(void) {
     // B?t clock cho SPI1
			/* Configure SPI1 pins: SCK, MISO and MOSI ---------------------------------*/
			/* Confugure SCK and MOSI pins as Alternate Function Push Pull */
			GPIO_Set(GPIOA,GPIO_PIN_5 | GPIO_PIN_7,GPIO_MODE_AF_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);
			/* Confugure MISO pin as Input Floating  */
			GPIO_Set(GPIOA,GPIO_PIN_6,GPIO_MODE_INPUT,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);
	
				/* SPI2 Periph clock enable */
			 // B?t clock cho SPI1
			/* Configure SPI2 pins: SCK, MISO and MOSI ---------------------------------*/
			/* Confugure SCK and MOSI pins as Alternate Function Push Pull */
			GPIO_Set(GPIOB,GPIO_PIN_13 | GPIO_PIN_15,GPIO_MODE_INPUT,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);
			/* Confugure MISO pin as Input Floating  */
			GPIO_Set(GPIOB,GPIO_PIN_14,GPIO_MODE_AF_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);
	
}

void SPI1_Master_Init(void) {
	__HAL_RCC_SPI1_CLK_ENABLE();
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
  hspi1.Init.CRCPolynomial = 7;
	HAL_SPI_Init(&hspi1);
}

void SPI2_Slave_Init(void) {
	__HAL_RCC_SPI2_CLK_ENABLE();
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
	HAL_SPI_Init(&hspi2);
}

void SPI1_Send_Data(uint8_t data) {
    HAL_SPI_Transmit(&hspi2, &data, 8, HAL_MAX_DELAY);
    // X? lý d? li?u nh?n du?c (n?u c?n)
		while (HAL_SPI_Transmit(&hspi2, &data, 8, HAL_MAX_DELAY)!=HAL_OK);
		printf("data = %d\n",SPI2->DR);
		HAL_Delay(50);
		
}

void SPI2_Receive_Data(void) {
		uint8_t received_data;
		HAL_SPI_Receive(&hspi2, &received_data, 1, HAL_MAX_DELAY);
		printf("data_recei = %d\n",received_data);
		HAL_Delay(50);
}


///* USER CODE END PV */

///* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
///////////////////////////////////////////////////////////////////////////////
uint8_t buffer[100];

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
		GPIO_Init();
		SPI1_Master_Init();
    SPI2_Slave_Init();
		USART_Init(USART1,Pins_PA9PA10,9600);

	 /* Initialize all configured peripherals */
  //MX_GPIO_Init();
	//pinMode(PC13,OUTPUT);
	//pinMode(PD12,OUTPUT);
	//GPIO_Set(GPIOB,GPIO_PIN_12,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);	
//	SPI_init(&hspi1,SPI1,SPI_MODE_MASTER,SPI_BAUDRATEPRESCALER_8);
//	SPI_init(&hspi2,SPI2,SPI_MODE_SLAVE,SPI_BAUDRATEPRESCALER_8);

  while (1)
  {
		
//		s_data_spi(&hspi1,&data);
//		HAL_Delay(50);
//		r_data_spi(&hspi2,&a);
			SPI1_Send_Data(0x55);
			//SPI2_Receive_Data();
//		HAL_SPI_TransmitReceive(&hspi1,&data,&received_data,1,HAL_MAX_DELAY);


		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct ;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

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



void HAL_SYSTICK_Callback(void)
{

	
	
}


/* USER CODE BEGIN 4 */


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
