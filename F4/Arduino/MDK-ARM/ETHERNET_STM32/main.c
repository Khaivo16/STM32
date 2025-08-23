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


//#include "HardwareSerial.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/


#include <LwIP.h>
#include <STM32Ethernet.h>

// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress server(74,125,232,128);  // numeric IP for Google (no DNS)
//char server[] = "api.thingspeak.com";    // name address for Google (using DNS)

// Set the static IP address to use if the DHCP fails to assign
IPAddress ip(192, 168, 0, 133);

// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
//EthernetClient client;
EthernetServer server(80);


void printIPAddress(void)
{
  Serial.print("My IP address: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    if (thisByte < 3) Serial.print(".");
  }

  Serial.println();
}


//#include <Modbus.h>
//#include <ModbusIP.h>

////Modbus Registers Offsets (0-9999)
//const int SENSOR_IREG = 0;
////Used Pins
//const int sensorPin = 4;

////ModbusIP object
//ModbusIP mb;

//long ts;

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
/////////////////////////////////////////////////////////////////////////////
//uint8_t toggle=0;
//void test_callback(HardwareTimer *htim)
//{
//  UNUSED(htim);
//  toggle ^= 1;
//    digitalWrite(PD12, toggle);
//}
//void Update_IT_callback(HardwareTimer *htim)
//{ // Update event correspond to Rising edge of PWM when configured in PWM1 mode
//  digitalWrite(PD14, LOW); // pin2 will be complementary to pin
//}

//void Compare_IT_callback(HardwareTimer *htim)
//{ // Compare match event correspond to falling edge of PWM when configured in PWM1 mode
//  digitalWrite(PD14, HIGH);
//}


int main(void)
{
	
	char USART_Buffer[100] = "Hello via USART with TX DMA\n";
	char Buffer[100];
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
		pinMode(PD12,OUTPUT);
		pinMode(PE4,OUTPUT);
		pinMode(PD14,OUTPUT);
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
		//Serial.println("Hello!");
		Serial.println(SystemCoreClock);
		Serial.println(123,HEX);//DEC
		
//		Serial1.DMA_Init(DMA2,DMA2_Stream7,DMA2_Stream5);		
//		Serial1.dmaRecive((uint8_t*)Buffer,10,1);
///////////////////////////////////////////////////////////////////////////////////	
//		/* Configure HardwareTimer */
//  HardwareTimer *EthTim = new HardwareTimer(TIM14);
//  EthTim->setMode(1, TIMER_OUTPUT_COMPARE,0);

//  /* Timer set to 1ms */
//  EthTim->setOverflow(100000, MICROSEC_FORMAT);
//  EthTim->attachInterrupt(test_callback);
//  EthTim->resume();
	
	/////////////////////////
	
//	// Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup function is finished.
//  HardwareTimer *MyTim = new HardwareTimer(TIM4);

//  MyTim->setMode(4, TIMER_OUTPUT_COMPARE_PWM1,PD15);
//  // MyTim->setPrescaleFactor(8); // Due to setOverflow with MICROSEC_FORMAT, prescaler will be computed automatically based on timer input clock
//  MyTim->setOverflow(100000, MICROSEC_FORMAT); // 100000 microseconds = 100 milliseconds
//  MyTim->setCaptureCompare(4, 50, PERCENT_COMPARE_FORMAT); // 50%
//  MyTim->attachInterrupt(Update_IT_callback);
//  MyTim->attachInterrupt(4, Compare_IT_callback);
//  MyTim->resume();
	////////////////////////////
// Configure and start PWM
  // MyTim->setPWM(channel, pin, 5, 10, NULL, NULL); // No callback required, we can simplify the function call
  //MyTim->setPWM(4, PD15, 5, 10); // 5 Hertz, 10% dutycycle
//////////////////////DHCP/////////////////////////////////////////////
	// start the Ethernet connection:
//  if (Ethernet.begin() == 0) {
//    Serial.println("Failed to configure Ethernet using DHCP");
//    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(ip);
//  }
  // give the Ethernet shield a second to initialize:
  delay(1000);
	printIPAddress();
  Serial.println("connecting..."); 
	////////////////////////////////////
//	// start the Ethernet connection and the server:
//	byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
//  Ethernet.begin(mac,ip);//Ethernet.begin(ip);
//	delay(1000);
//	///////////////////////////
	server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
	Serial.println(Ethernet.subnetMask());
	Serial.println(Ethernet.gatewayIP());
///////////////////////////////////////////////////////	
//	// The media access control (ethernet hardware) address for the shield
//    byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
//    // The IP address for the shield
//    byte ip[] = { 192, 168, 1, 120 };
//    //Config Modbus IP
//    mb.config(mac, ip);
//		
//    // Add SENSOR_IREG register - Use addIreg() for analog Inputs
//    mb.addIreg(SENSOR_IREG);

//    ts = millis();
  while (1)
  {
		
//		if (Serial.available()) {   
//    
//    Serial.print( (char)Serial.read());
//			
//		}
//		for(int i=0; i<100;i++){
//			if(Buffer[i]=='\n') Serial1.dmaSend(Buffer,10,0);//Serial1.println(Buffer);
//		}
		
		//////////////////////////////////////////////////////////////////////
////Call once inside loop() - all magic here
//   mb.task();

//   //Read each two seconds
//   if (millis() > ts + 2000) {
//       ts = millis();
//       //Setting raw value (0-1024)
//       mb.Ireg(SENSOR_IREG, analogRead(sensorPin));
//   }
////////////////////////////////////////////////////////////////////////	 
		// listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    bool currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          // output the value of each analog input pin3-4
          for(int channel=3;channel<5;channel++){
            int sensorReading = channel;//analogRead(channel);
            client.print("analog input ");
            client.print(channel);
            client.print(" is ");
            client.print(sensorReading);
            client.println("<br />");
          }
          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
		
		togglePin(PE4);
		delay_ms(100);		
		
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
  RCC_OscInitStruct.PLL.PLLM = 25;
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
