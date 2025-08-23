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
#define Conveyor PB10
#define Plasma PB1
#define Dir_Con PB0
#define Pulse_Z PA5
#define Dir_Z PA12

int test = 0 ;

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
uint8_t mode = 0;
uint8_t count = 0;
uint8_t solan_dao = 0;
uint8_t time = 100;
uint8_t pos_Z = 0;

/* Private variables ---------------------------------------------------------*/
uint8_t data_receive[12];
HardwareTimer *EthTim = new HardwareTimer(TIM4);
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
/////////////////////////////////////////////////////////////////////////////
void acti_step(uint8_t count);
void update(uint8_t* data);
void Dir_Conveyor();
void ON_system();
void OFF_system();
/* USER CODE END PFP */
void test_callback(HardwareTimer *htim)
{
  Dir_Conveyor();
}


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
	
	Serial.begin(9600);
	pinMode(PC13,OUTPUT);
	digitalWrite(PC13,1);
	pinMode(Conveyor,OUTPUT);
  pinMode(Plasma,OUTPUT);
  pinMode(Dir_Con,OUTPUT);
  pinMode(Pulse_Z,OUTPUT);
  pinMode(Dir_Z,OUTPUT);			
///////////////////////////////////////////////////////////////////
/* Configure HardwareTimer */
  EthTim->setMode(1, TIMER_OUTPUT_COMPARE,0);
  EthTim->attachInterrupt(test_callback);  // G?n callback m?i
//  /* Timer set to 1ms */
		
  while (1)
  {
//		digitalWrite( Dir_Z, LOW );	
//		delay(500);
//		digitalWrite( Dir_Z, HIGH );
//		delay(500);

		while (mode == 0) {
				if (Serial.available()>=12){
					for (uint8_t i=0; i<=11; i++) {
							data_receive[i] = Serial.read();
					} 
					if (data_receive[0] == 0x01) {
						update(data_receive);
						mode = 1;
						digitalWrite(PC13,0);	
						break;
					}
					else {
					 // ON/OFF Bang tai 
					 if (data_receive[0] == 0x21) {
							digitalWrite(Conveyor,1);
						}
						else if (data_receive[0] == 0x20) digitalWrite(Conveyor,0);
						// ON/OFF Plasma
						if (data_receive[0] == 0x41) {
							digitalWrite(Plasma,1);
						}
						else if (data_receive[0] == 0x40) digitalWrite(Plasma,0);
						// Dao chieu
						if (data_receive[0] == 0x03) {
							digitalWrite(Dir_Con, !digitalRead(Dir_Con));
						}
						// Set Home Z-Axis
						if (data_receive[0] == 0x05) {
							
						}
						// Set Truc Z-Axis
						if (data_receive[0] == 0x51) {
							test = 1;
							digitalWrite(Dir_Z,0);
							delay(10);
							for (int i=0; i < 200; i++){
								digitalWrite(Pulse_Z,1);
								delay(1);
								digitalWrite(Pulse_Z,0);
								delay(1);
							}
						}
						if (data_receive[0] == 0x15) {
							digitalWrite(Dir_Z,1);
							delay(10);
							for (int i=0; i < 200; i++){
								digitalWrite(Pulse_Z,1);
								delay(1);
								digitalWrite(Pulse_Z,0);
								delay(1);
							}
						}
					}
				}
			}
		while (mode == 1) {
				if (count > 0) {
					ON_system(); 
				}
				else {
					mode = 0;
					OFF_system();
					break;
				}
		// STOP
				if (Serial.available()>=12) {
					for (uint8_t i=0; i<= 11; i++) {
							data_receive[i] = Serial.read();
					} 
					if (data_receive[0] == 0x00) {
						OFF_system();
						mode = 0;
						solan_dao = 1;
						digitalWrite(PC13,1);
						break;
					}
				}
			}
		
  }
  /* USER CODE END 3 */

}

void acti_step(uint8_t count) {
  for (uint8_t i = 0; i< count; i++) {
    digitalWrite(Pulse_Z, 0);
    delay(1);
    digitalWrite(Pulse_Z, 1);
    delay(1);
  }
}

void update(uint8_t* data) {
  count = data[4];
  time = data[8];
  pos_Z = data[9];
	EthTim->setOverflow((uint32_t)time * 1000000, MICROSEC_FORMAT);
	EthTim->setCount(0);
  acti_step(count);  // delay vài tram ms
}


void Dir_Conveyor(){
  if (solan_dao > count-1) {
		EthTim->pause();
		EthTim->setCount(0);
    digitalWrite(Conveyor,0);
    digitalWrite(Plasma,0);
		mode = 0;
		digitalWrite(Dir_Con, !digitalRead(Dir_Con));
		solan_dao =1;
  }
	else { 
		solan_dao +=1;
		digitalWrite(Dir_Con, !digitalRead(Dir_Con));
	}
}


void ON_system() {
  digitalWrite(Conveyor,1);
  digitalWrite(Plasma,1);
  EthTim->resume();
}

void OFF_system() {
  digitalWrite(Conveyor,0);
  digitalWrite(Plasma,0);
  EthTim->pause();
	EthTim->setCount(0);
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
