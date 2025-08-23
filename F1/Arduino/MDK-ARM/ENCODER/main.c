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

#include <stdio.h>
 
/* Include ARM math */
//#include "arm_math.h"


/********************************************************
 * Basic Example
 * 
 ********************************************************/

#include <STM32Encoder.h>
/*
For Maple Mini: 

Timer 1, channels are at 26 and 27. 
The example below was tested
*/


long revolutions = 0;//revolution counter

//Encoder stuff...
STM32Encoder enc(TIM3, TIM_ENCODERMODE_TI12, 1, 100); //100xung/vong=360*

/*interrupt handler. 
  this will called every time the counter goes through zero 
  and increment/decrement the variable depending on the direction of the encoder. 
*/

void func() {  
  if (enc.getDirection() == POSITIVE) revolutions++;
  if (enc.getDirection() == NEGATIVE) revolutions--;
}

//encoder simulation stuff
unsigned char mode = 0; //clock on both pins. 
unsigned char dir = 'F';
unsigned int freq = 100;
unsigned long time = 0;
unsigned char states[4];

unsigned long interval = 0;
/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
/////////////////////////////////////////////////////////////////////////////



int main(void)
{
	

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
		pinMode(PD13,OUTPUT);
	//////////////////////////////////////////////////////////////////////
	/* see Pin Tx-RX sys.h
		//Serial=Serial1
			Serial2
			Serial3
			Serial4
			Serial15
			Serial16
		*/

		Serial.begin(115200);
		//printf( "LVD_IC \n" );
		Serial.println("Hello!");
		
	
	//configure inputs for the encoder
  //comment the timer not used. 
  //pinMode(PD6, INPUT);  //TIMER 1 channel 1
  //pinMode(PD7, INPUT);  //TIMER 1 channel 2
//  pinMode(D2, INPUT);  //TIMER 2 channel 1
//  pinMode(D3, INPUT);  //TIMER 2 channel 2
  pinMode(PA6, INPUT);  //TIMER 3 channel 1
  pinMode(PA7, INPUT);  //TIMER 3 channel 2
//  pinMode(D5, INPUT);  //TIMER 4 channel 1
//  pinMode(D9, INPUT);  //TIMER 4 channel 2

/* Configure PWM GPIO pins */
//        GPIO_InitTypeDef GPIO_InitStructure;
//				 GPIO_InitStructure.Pin = GPIO_PIN_4|GPIO_PIN_5;
//				 GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
//				 GPIO_InitStructure.Pull =  GPIO_NOPULL;
//				 GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
//				 GPIO_InitStructure.Alternate = GPIO_AF2_TIM3;//GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
//				HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);//////////////
//				
	
  //attach interrupt
  enc.attachInterrupt(func);//attach interrupt
  
  
//setup encoder simulator  
  pinMode(PA5, OUTPUT);
  pinMode(PA4, OUTPUT);

  digitalWrite(PA5, HIGH); 
  digitalWrite(PA4, LOW);
  states[0] = 0; //00
  states[1] = 1; //01
  states[2] = 3; //11
  states[3] = 2; //10
	
	
	/////////////////////////////////////////////////////////////////
		
  while (1)
  {
	//--------------------------------+ENCODER TEST CODE+-------------------------------    
    //encoder code
  if (millis() - interval >= 500) { 
		 Serial.println("");
     Serial.print(enc.value()); 
     Serial.println(" counts");
     Serial.print("direction ");
     Serial.println(enc.getDirection());
     Serial.println(enc.getAngle(DEGREES));
     Serial.println(revolutions);
		 Serial.println("");
		 
     interval = millis();
   }
  
//--------------------------------+ENCODER SIGNAL SIMULATION+-------------------------------  
//simulate encoder signals.   
//encoder simulation code  
  if (millis() - time >= freq) { 
    time = millis(); //prepare next
    
    if (dir == 'F')  mode++;
    else mode --;
    
    digitalWrite(PA4, (states[mode%4] & 0x01));
    digitalWrite(PA5, (states[mode%4] >> 1));

//--------------------------------+SERIAL COMMUNICATION+-------------------------------  
//take care of comms...
  if (Serial.available() > 0)
    switch(Serial.read()) {
      case '0': {
        freq = 100;
        break;
      }
      case '1': {
        freq = 500;
        break;
      }
      case '2': {
        freq = 1000;
        break;
      }  
      case '3': {
        freq = 5;
        break;
      }
      case 'F': {
        dir = 'F';
        break;
      }
      case 'B': {
        dir = 'B';
        break;
      }
      case 'I': {
        enc.Polarity(INVERTED);
        break;
      }
      case 'N': {
        enc.Polarity(NORMAL);
        break;
      }
      case 'R': {
        enc.reset();
        break;
      }
    }//end switch

  }       

//		togglePin(PD13);
//		delay_ms(100);		
		
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
/* USER CODE END 4 */

////////////////////////////////////////////////////////////
 /* Printf handler */
 int std::fputc(int ch, FILE* fil) {//std::
	/* Send over USART */
	//USART_Putc(USART1, ch);
	Serial.write(ch);
	
	/* Return character */
	return ch;
}
 /////////////////////////////////////////////////////////////////////
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
