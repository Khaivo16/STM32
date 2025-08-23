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
#include <dht11.h>
 
dht11 DHT11;
#define DHT11PIN PA8
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

#include "Wire.h" 
////            SDA  SCL
//TwoWire Wire2(PB3, PB10);//SDA  SCL

////////////////////////////////////////////////////////////////////////////
#include <LiquidCrystal_I2C.h>     // if you don´t have I2C version of the display, use LiquidCrystal.h library instead

LiquidCrystal_I2C lcd(0x27, 16, 2);    // set the LCD address to 0x27 for a 16 chars and 2 line display
//LiquidCrystal_I2C lcd(0x3f,16,2);    // set the LCD address to 0x3f for a 16 chars and 2 line display
// if you don´t know the I2C address of the display, use I2C scanner first (https://playground.arduino.cc/Main/I2cScanner/)


/* Private variables ---------------------------------------------------------*/
#include <MFRC522.h>
#include <SPI.h>


//MOSI  	PB15
//MISO  	PB14
//CLK  	  PB13
#define SAD PB12//sad    SPI chip select pin (CS/SS/SSEL)
#define RST 0  // RST(STM32)

MFRC522 nfc(SAD, RST);
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
/////////////////////////////////////////////////////////////////////////////
int check=0,check2=0,card1=0,card2=0;
char Key_Card[5]  = {0x23, 0x5c, 0xa5, 0xb9, 0x63};//Your card's number are: 23, 5c, a5, b9, 63 
char Key_Card2[5] = {0xc4, 0x24, 0x99, 0x22, 0x5b};
/////////////////////////////////////////
const byte BH1750 = 0x23;
#define MOD_REG             (0x10)

float read_BH1750(void)
{
    uint8_t msb, lsb;

    Wire.beginTransmission(BH1750);
    
    Wire.write(MOD_REG);
    
    Wire.endTransmission();
		delay(200);

    Wire.requestFrom((int)BH1750, 2);

    while(!Wire.available()) {};
			
    msb = Wire.read();
    lsb = Wire.read();
    

    return ((((short)msb << 8) | (short)lsb)) / 1.2f;
}

int main(void)
{
	
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
	pinMode(PC13,OUTPUT);
	
	delay(100);
///////////////////////////////////////////////////////////////
//		Wire.setSDA(PB7); // using pin name PY_n
//    Wire.setSCL(PB6); // using pin number PYn
	Wire.begin();//B9, B8		//Wire.begin(true); or Wire.begin(0x70,true);
//////////	
	lcd.init();                       // initialize the 16x2 lcd module
  lcd.backlight();                  // enable backlight for the LCD module  
/////////////////////////////////////////////////////////////////////////
	Serial.begin(115200);
	//Serial.println("OK ");
		//Serial.println(123);
	
	SPI.begin();
	//SPI.setClock(9000000);//9MHZ
  nfc.begin();

  // Get the firmware version of the RFID chip
  byte version = nfc.getFirmwareVersion();
  if (! version) {
    Serial.print("Didn't find MFRC522 board.");
    while(1); //halt
  }
	
	lcd.clear();
		
				
  while (1)
  {
		
		
	float Lux = read_BH1750();delay(200);
	int chk = DHT11.read(DHT11PIN);
  
//  Serial.print("Temp: ");
//  Serial.print((float)DHT11.temperature, 2);
//  Serial.print(" C ");		
	lcd.setCursor(0,0); //sets cursor to column 0, row 0
  lcd.print("Temp: "); //prints label
  lcd.print((float)DHT11.temperature, 2); 
	lcd.print(" C "); 
  

//  Serial.print(" RelF: ");
//  Serial.print((float)DHT11.humidity, 2);
//  Serial.print(" %");
	lcd.setCursor(0, 1);//sets cursor to column 0, row 1
	lcd.print("RelF: ");
	lcd.print((float)DHT11.humidity, 2);		
	lcd.print(" %");
	
	
	/////////////////////////
	byte status;
  byte data[MAX_LEN];
  byte num[6];

  // Send a general request out into the aether. If there is a tag in
  // the area it will respond and the status will be MI_OK.
  status = nfc.requestTag(MF1_REQIDL, data);

  if (status == MI_OK) {
		//Serial.printf("Find out a card: %x, %x\r\n",data[0],data[1]);

    // calculate the anti-collision value for the currently detected
    // tag and write the serial into the data array.
    status = nfc.antiCollision(data);
    memcpy(num, data, 5);
		
		if (status == MI_OK)
		{
			//Serial.printf("Your card's number are: %x, %x, %x, %x, %x \r\n",num[0], num[1], num[2],num[3],num[4]);
			for(int i=0;i<5;i++)
			{
				if(Key_Card[i]!=num[i]) check = 0;
				else check = 1;
				if(Key_Card2[i]!=num[i]) check2 = 0;
				else check2 = 1;
			}
			
			if(check == 1||check2 ==1)
			{
				card1=check;card2=check2;
				check = 0;
				check2 = 0;
				//Serial.printf("Your card's number are: %x, %x, %x, %x, %x \r\n",num[0], num[1], num[2],num[3],num[4]);
				//Serial.printf("The Card's number is Ok!\r\n");
				delay(500);
			}
		}
   
    // Stop the tag and get ready for reading a new tag.
    nfc.haltTag();
  }
  	
	Serial.printf("%2.2f,%2.2f,%2.2f,%d,%d\n",(float)DHT11.temperature,(float)DHT11.humidity,Lux,card1,card2);
	togglePin(PC13);  
		
		
			
		
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
