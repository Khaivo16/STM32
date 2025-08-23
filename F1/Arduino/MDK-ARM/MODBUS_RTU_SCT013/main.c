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

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
#include "EmonLib.h"
#include <modbusRTU.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>
modbusDevice regBank;
modbusSlave slave;
// define pin modbus
#define RS485TxEnablePin PA8
#define RS485Baud 9600
#define RS485Format USART_WORDLENGTH_8B	//SERIAL_8N1 // SERIAL_8E1
//define pin ADC-SCT013
#define pinSCT1 PA0
#define pinSCT2 PA1
#define pinSCT3 PA2
#define pinSCT4 PA3
#define pinSCT5 PA4
#define pinSCT6 PA5
#define pinSCT7 PA6
#define pinSCT8 PA7
#define pinSCT9 PA15
#define pinSCT10 PB3
#define pinSCT11 PB4
#define pinSCT12 PB5
#define pinSCT13 PB6
#define pinSCT14 PB7
#define pinSCT15 PB8
#define pinSCT16 PB9

//create object
EnergyMonitor SCT013_1;
EnergyMonitor SCT013_2;
EnergyMonitor SCT013_3;
EnergyMonitor SCT013_4;
EnergyMonitor SCT013_5;
EnergyMonitor SCT013_6;
EnergyMonitor SCT013_7;
EnergyMonitor SCT013_8;
EnergyMonitor SCT013_9;
EnergyMonitor SCT013_10;
EnergyMonitor SCT013_11;
EnergyMonitor SCT013_12;
EnergyMonitor SCT013_13;
EnergyMonitor SCT013_14;
EnergyMonitor SCT013_15;
EnergyMonitor SCT013_16;

EnergyMonitor* SCT013_list[16] = {
  &SCT013_1, &SCT013_2, &SCT013_3, &SCT013_4,
  &SCT013_5, &SCT013_6, &SCT013_7, &SCT013_8,
  &SCT013_9, &SCT013_10, &SCT013_11, &SCT013_12,
  &SCT013_13, &SCT013_14, &SCT013_15, &SCT013_16
};
/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// variable of I
double Irm[20];
uint16_t data[20];
uint8_t t=0;
unsigned long lastReadTime = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
/////////////////////////////////////////////////////////////////////////////
int demo;
float current;
int main(void)
{
	

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
	pinMode(pinSCT1,INPUT);
	pinMode(pinSCT2,INPUT);
	pinMode(pinSCT3,INPUT);
	pinMode(pinSCT4,INPUT);
	pinMode(pinSCT5,INPUT);
	pinMode(pinSCT6,INPUT);
	pinMode(pinSCT7,INPUT);
	pinMode(pinSCT8,INPUT);
	pinMode(pinSCT9,INPUT);
	pinMode(pinSCT10,INPUT);
	pinMode(pinSCT11,INPUT);
	pinMode(pinSCT12,INPUT);
	pinMode(pinSCT13,INPUT);
	pinMode(pinSCT14,INPUT);
	pinMode(pinSCT15,INPUT);
	pinMode(pinSCT16,INPUT);
	//////////////////////////////////////////////////////////////////////
	/* see Pin Tx-RX sys.h
		//Serial=Serial1
			Serial2
			Serial3
			Serial4
			Serial15
			Serial16
		*/

//Assign the modbus device ID.
regBank.setId(3);

// Set value for SCT013
SCT013_1.current(PA0, 4.6);
SCT013_2.current(PA1, 4.6);
SCT013_3.current(PA2, 4.6);
SCT013_4.current(PA3, 4.6);
SCT013_5.current(PA4, 4.6);
SCT013_6.current(PA5, 4.6);
SCT013_7.current(PA6, 4.6);
SCT013_8.current(PA7, 4.6);
SCT013_9.current(PA15, 4.6);
SCT013_10.current(PB3, 4.6);
SCT013_11.current(PB4, 4.6);
SCT013_12.current(PB5, 4.6);
SCT013_13.current(PB6, 4.6);
SCT013_14.current(PB7, 4.6);
SCT013_15.current(PB8, 4.6);
SCT013_16.current(PB9, 4.6);
/*
modbus registers follow the following format
00001-09999 Digital Outputs, A master device can read and write to these registers       //01
10001-19999 Digital Inputs, A master device can only read the values from these registers    //02
30001-39999 Analog Inputs, A master device can only read the values from these registers       //04
40001-49999 Analog Outputs, A master device can read and write to these registers            //03

Analog values are 16 bit unsigned words stored with a range of 0-32767
Digital values are stored as bytes, a zero value is OFF and any nonzer value is ON

*/
for (int i = 1;i<=16;i++){
	regBank.add(40200+i);
}
slave._device = &regBank;
Serial.setTxRx(PA9,PA10);
slave.setBaud(&Serial,RS485Baud,RS485Format,RS485TxEnablePin);
//slave.setBaud(&Serial,RS232Baud,RS232Format,RS232Enable);

//slave.setBaud(9600);
		
	/////////////////////////////////////////////////////////////////
		
  while (1)
  {		
//		slave.run();
////    if (millis() - lastReadTime >= 100) { // C? m?i 500ms do l?i dòng
//        for (int i = 0; i < 16; i++) {
//            Irm[i] = SCT013_list[i]->calcIrms(2500); // gi?m m?u xu?ng
//            regBank.set(40201 + i, (word)(Irm[i] * 100)); // c?p nh?t Modbus reg
//						slave.run();
//        }
////				lastReadTime = millis();
////    }
//		demo = adcRead(pinSCT14,ADC_12_BIT);
		current = SCT013_14.calcIrms(1500);
		delay(100);
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
