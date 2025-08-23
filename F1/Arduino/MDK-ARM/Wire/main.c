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


#include "usart.h"
#include <TM1637Display.h>
#include "HardwareSerial.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

//#include "Wire.h" 
#include "SoftWire.h"

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
/////////////////////////////////////////////////////////////////////////////

// DS3231 date
typedef struct {

	uint8_t seconds;

	uint8_t minutes;

	uint8_t hours;

	uint8_t wday;//day_of_week;

	uint8_t day;

	uint8_t month;

	uint8_t year;

} DS3231_date_TypeDef;	

static DS3231_date_TypeDef DS3231;

int bcd2dec(byte num);
int dec2bcd(byte num);

void setTime(byte hr, byte min, byte sec, byte wd, byte d, byte mth, byte yr);
void readDS1307(void);
float readTemperature(void);
#define readDS3231()	readDS1307()

/* Ð?a ch? c?a DS1307 */
const byte DS1307 = 0x68;
#define DS3231_REG_TIME             (0x00)
#define DS3231_REG_ALARM_1          (0x07)
#define DS3231_REG_ALARM_2          (0x0B)
#define DS3231_REG_CONTROL          (0x0E)
#define DS3231_REG_STATUS           (0x0F)
#define DS3231_REG_TEMPERATURE      (0x11)
/* S? byte d? li?u s? d?c t? DS1307 */
const byte NumberOfFields = 7;//7 byte

extern DS3231_date_TypeDef DS3231;

/* Chuy?n t? format BCD (Binary-Coded Decimal) sang Decimal */
int bcd2dec(byte num)
{
        return ((num/16 * 10) + (num % 16));
}
/* Chuy?n t? Decimal sang BCD */
int dec2bcd(byte num)
{
        return ((num/10 * 16) + (num % 10));
}
/* cài d?t th?i gian cho DS1307 */
void setTime(byte hr, byte min, byte sec, byte wd, byte d, byte mth, byte yr)
{
        sWire.beginTransmission(DS1307);
        sWire.write(0x00); // d?t l?i pointe
        sWire.write(dec2bcd(sec));
        sWire.write(dec2bcd(min));
        sWire.write(dec2bcd(hr));
        sWire.write(dec2bcd(wd)); // day of week: Sunday = 1, Saturday = 
        sWire.write(dec2bcd(d));
        sWire.write(dec2bcd(mth));
        sWire.write(dec2bcd(yr));
        sWire.endTransmission();
}
void readDS1307(void)
{
        sWire.beginTransmission(DS1307);
        sWire.write((byte)0x00);
        sWire.endTransmission();
        sWire.requestFrom(DS1307, NumberOfFields);
        
        DS3231.seconds = bcd2dec(sWire.read() & 0x7f);
        DS3231.minutes = bcd2dec(sWire.read() );
        DS3231.hours   = bcd2dec(sWire.read() & 0x3f); // ch? d? 24h
        DS3231.wday   = bcd2dec(sWire.read() );
        DS3231.day    = bcd2dec(sWire.read() );
        DS3231.month  = bcd2dec(sWire.read() );
        DS3231.year   = bcd2dec(sWire.read() );
          
}
float readTemperature(void)
{
    uint8_t msb, lsb;

    sWire.beginTransmission(DS1307);
    
     sWire.write(DS3231_REG_TEMPERATURE);
    
    sWire.endTransmission();

    sWire.requestFrom(DS1307, 2);

    while(!sWire.available()) {};
			
    msb = sWire.read();
    lsb = sWire.read();
    

    return ((((short)msb << 8) | (short)lsb) >> 6) / 4.0f;
}
#define CLK PD3
#define DIO PD4

// The amount of time (in milliseconds) between tests
#define TEST_DELAY   2000


TM1637Display display(CLK, DIO);


int main(void)
{
	char a[20]; float T=0;	char c=0;
	uint32_t time=0; //float dis=0;
	uint8_t sec=0,mm=0;
	


  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
	pinMode(PD13,OUTPUT);
	//////////////////////////////////////////////////////////////////////

	 /* Initialize all configured peripherals */
  	
	delay(100);
	
 		

		Serial.begin(115200);
		//printf( "LVD_IC \n" );
		//Serial.println("Please ");
		Serial.println(123);
		
				
///////////////////////////////////////////////////////////////////
sWire.begin();//B9, B8
		delay(500);
		//setTime(13, 40, 0, 4, 1, 1, 20); // 12:30:45 CN 08-02-2015
		delay(200);
/////////////////////////////////////////////////////////////////
		
 
  while (1)
  {
		
		
		readDS1307();
		T=readTemperature();//DS3231
		/* Format time */
		 delay(500);
		if(sec!=DS3231.seconds){
		sprintf(a, "\nTime: %02d.%02d.%02d \nDate: %02d/%02d/%04d \nT*C: %2.2f\n", DS3231.hours, DS3231.minutes, DS3231.seconds, DS3231.day, DS3231.month, DS3231.year+2000, T );
		Serial.println(a);
		sec =	DS3231.seconds;	
		togglePin(PD13);
			
		if(mm!=DS3231.minutes){
			//display.clear();mm=DS3231.minutes;
			//display.showNumberDecEx(((DS3231.hours*100)+DS3231.minutes), B01000000, true, 4, 0);//xem file .h
			}
		
		}

//////////////////////////////////////////////////////
//		byte error, address;
//  int nDevices;
// 
//  Serial.println("Scanning...");
// 
//  nDevices = 0;
//  for(address = 1; address < 127; address++ )
//  {
//    // The i2c_scanner uses the return value of
//    // the Write.endTransmisstion to see if
//    // a device did acknowledge to the address.
//    sWire.beginTransmission(address);
//    error = sWire.endTransmission();
// 
//    if (error == 0)
//    {
//      Serial.print("I2C device found at address 0x");
//      if (address<16)
//        Serial.print("0");
//      Serial.print(address,HEX);
//      Serial.println("  !");
// 
//      nDevices++;
//    }
//    else if (error==4)
//    {
//      Serial.print("Unknown error at address 0x");
//      if (address<16)
//        Serial.print("0");
//      Serial.println(address,HEX);
//    }    
//  }
//  if (nDevices == 0)
//    Serial.println("No I2C devices found\n");
//  else
//    Serial.println("done\n");
// 
//  delay(5000);           // wait 5 seconds for n
			/* USER CODE END WHILE */
		
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
