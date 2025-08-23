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


#include "HardwareSerial.h"
#include <SPI.h>
#include "RF24.h" 
#include "nRF24L01.h" 
#include "stdbool.h"

#define CE_PIN PB11
#define CSN_PIN PB12
// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
#define INTERVAL_MS_TRANSMISSION 250  


const byte diachi[6] = "12345"; 
//NRF24L01 buffer limit is 32 bytes (max struct size) 

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
/////////////////////////////////////////////////////////////////////////////
//payload payload; 
char data[2];
unsigned long lastSignalMillis = 0; 

int main(void)
{
	

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
	//////////////////////////////////////////////////////////////////////
	/* see Pin Tx-RX sys.h
		//Serial=Serial1
			Serial2
			Serial3
			Serial4
			Serial15
			Serial16
		*/
//	SPI.begin();
	Serial.begin(9600); 
	if (!radio.begin()) 
  {
    Serial.println("Module kh�ng kh?i d?ng du?c...!!");
    while (1) {}
  } 
	radio.openWritingPipe(diachi);
  //L?nh openWritingPipe m?c d?nh l� du?ng truy?n 0
  //m? 1 k�nh c� d?a ch? 12345 tr�n du?ng truy?n 0
  // k�nh n�y ghi data l�n d?a ch? 12345  
  radio.setPALevel(RF24_PA_LOW); 
  //C�i b? khuy?t d?a c�ng su?t ? m?c MIN, MAX, HIGH, LOW
  radio.setChannel(80); // 125 k�nh t? 0-124; TX v� RX ph?i c�ng k�nh
                        // 2.4GHz ~ 2400Mhz, bu?c k�nh l� 1MHz
                        // setchannel(1) => 2401Mhz
                        // Cao nh?t l� 2525MHz, T?c l� 2.525GHz
                        
  radio.setDataRate(RF24_1MBPS); //T?c d? truy?n d? li?u trong kh�ng kh�
                                   //250kbps, 1Mbps ho?c 2Mbps
                                   //250 th?p nh?t nhung truy?n xa, 1Mb v� 2Mb m?nh nhung truy?n kh�ng xa                                   
  /*                                   
   * T?c d? truy?n d? li?u kh�ng kh� 2Mbps, bang th�ng 2MHz b? chi?m d?ng nhi?u t?n s? k�nh 
   * r?ng hon d? ph�n gi?i c?a c�i d?t t?n s? k�nh RF
   * V� v?y, d? d?m b?o c�c k�nh kh�ng ch?ng ch�o v� gi?m k?t n?i ch�o ? ch? d? 2Mbps
   * b?n c?n gi? kho?ng c�ch 2MHz gi?a hai k�nh.                                    
   * 
   * A: Xe TX-RX k�nh 80, t?c d? truy?n l� 2Mb 80, 81, 82
   * B: M�y bay TX-RX k�nh 83, t?c d? truy?n l� 250Kb                                    
   */
  radio.stopListening(); //C�i d?t module l� TX
  
  if (!radio.available())
  {
    Serial.println("Chua k?t n?i du?c v?i RX...!!");
    Serial.println("CH? K?T N?I.......");
  }  

 
 
		
	/////////////////////////////////////////////////////////////////
		
  while (1)
  {
	char text = 'a';//M?ng ch?a chu?i k� t?
  radio.write(&text, sizeof(text));
  // &: Tr? l?i d?a ch? c?a m?t bi?n.
  // sizeof: tr? v? s? byte b? nh? c?a m?t bi?n 
  //ho?c l� tr? v? t?ng s? byte b? nh? c?a m?t m?ng
  Serial.println("d� g?i");
  delay(1000);
  }
  /* USER CODE END 3 */

}

 


/** System Clock Configuration
*/
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                 RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001) {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }

  /* Ensure CCM RAM clock is enabled */
  __HAL_RCC_CCMDATARAMEN_CLK_ENABLE();
	

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
    

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
//  led3.runToggle();
//  led4.runToggle();
//  led5.runToggle();
//  led6.runToggle();
	
	
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

//		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
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
