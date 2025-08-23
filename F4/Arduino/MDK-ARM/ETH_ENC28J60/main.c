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
/*
=========================================================
// PIN Connections (Using STM32F103):
//
// ENC28J60 -  STM32F103
//   VCC    -    3.3V
//   GND    -    GND
//   SCK    -    Pin (PA5) PB13
//   SO     -    Pin (PA6) PB14
//   SI     -    Pin (PA7) PB15
//   CS     -    Pin PA8
 =========================================================
 */
/* USER CODE BEGIN Includes */


#include "HardwareSerial.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

////////////////////////////////////////////////////////////////////////////////////////////


//#include <UIPEthernet.h>

//EthernetClient client;
//signed long next;

//////////////////////////////////////////////////////////////////////
#include <EtherCard.h>
#include <SPI.h>

//// ethernet interface mac address, must be unique on the LAN
static byte mymac[] = { 0x74,0x69,0x69,0x2D,0x30,0x31 };
static byte myip[] = { 192,168,1,203 };



/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
/////////////////////////////////////////////////////////////////////////////


//byte Ethernet::buffer[700];
//static uint32_t timer;

//const char website[] PROGMEM = "www.google.com";
//// called when the client request is complete
//static void my_callback (byte status, u16 off, u16 len) {
//  Serial.println(">>>");
//  Ethernet::buffer[off+300] = 0;
//  Serial.print((const char*) Ethernet::buffer + off);
//  Serial.println("...");
//}

byte Ethernet::buffer[200];///////////////
BufferFiller bfill;

static word homePage() {
  long t = millis() / 1000;
  word h = t / 3600;
  byte m = (t / 60) % 60;
  byte s = t % 60;
  bfill = ether.tcpOffset();
  bfill.emit_p(PSTR(
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: text/html\r\n"
    "Pragma: no-cache\r\n"
    "\r\n"
    "<meta http-equiv='refresh' content='1'/>"
    "<title>RBBB server</title>" 
    "<h1>$D$D:$D$D:$D$D</h1>"),
      h/10, h%10, m/10, m%10, s/10, s%10);
  return bfill.position();
}


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

		// Open serial communications and wait for port to open:
	Serial.begin(115200);
	delay(100);

//	Serial.println(("*************************************************************"));
//	Serial.println(("***** Web server example *****"));
//	Serial.println(("*************************************************************"));

	
// if (ether.begin(sizeof Ethernet::buffer, mymac) == 0) 
//    Serial.println(("Failed to access Ethernet controller"));
//  if (!ether.dhcpSetup())
//    Serial.println(("DHCP failed"));

//  ether.printIp("IP:  ", ether.myip);
//  ether.printIp("GW:  ", ether.gwip);  
//  ether.printIp("DNS: ", ether.dnsip);  

//  if (!ether.dnsLookup(website))
//    Serial.println("DNS failed");
//    
//  ether.printIp("SRV: ", ether.hisip);
///////////////////////////////////////////////////////////////////////////////////
  Serial.begin(115200);
  delay(10);printf("%d \n",micros());
  if (ether.begin(sizeof Ethernet::buffer, mymac) == 0)
    Serial.println(("Failed to access Ethernet controller"));
		ether.staticSetup(myip);//,gwip,NULL,maskip
		
/////////////////////////////////////////////////////////////////
  
///////////////////////////////////////////////////////////////
  while (1)
  {
		
///////////////////////////////////////////////////////////////////////////////////////////////////		
		if (ether.packetLoop(ether.packetReceive())){   // check if valid tcp data is received
    ether.httpServerReply(homePage()); // send web page data
    }
////////////////////////////////////////////////////////////////////////////////////////////////////
//		 //Call once inside loop() - all magic here
//  ether.packetLoop(ether.packetReceive());
//  
//  if (millis() > timer) {
//    timer = millis() + 5000;
//    Serial.println();
//    Serial.print("<<< REQ ");
//    ether.browseUrl("/foo/", "bar", website, my_callback);
//  }

		togglePin(PD13);
		delay_ms(500);		
		
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


/* Printf handler */
 int std::fputc(int ch, FILE* fil) {//std::
	/* Send over USART */
	//USART_Putc(USART1, ch);
	Serial.write(ch);
	
	/* Return character */
	return ch;
}

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
