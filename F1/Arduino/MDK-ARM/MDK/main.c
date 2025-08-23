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

#include "Wire.h" 
#include <TM1637Display.h>
#include "HardwareSerial.h"
#include <dht11.h>
 
dht11 DHT11;
#define DHT11PIN PA8
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

char rx[6];

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
    //Serial.println("TIMER 1S" );
}

void myfunc2(void *data)
{
  
    
}
void myfunc1(void *data)
{
  
		 
     
}


/* USER CODE END PFP */
	
/* USER CODE BEGIN 0 */
//CLed led3(GPIOD,GPIO_PIN_13,500);
//CLed led4(GPIOD,GPIO_PIN_12,1000);
//CLed led5(GPIOD,GPIO_PIN_14,2000);
//CLed led6(GPIOD,GPIO_PIN_15,4000);
/* USER CODE END 0 */
////////////////////////////
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
        Wire.beginTransmission(DS1307);
        Wire.write(0x00); // d?t l?i pointe
        Wire.write(dec2bcd(sec));
        Wire.write(dec2bcd(min));
        Wire.write(dec2bcd(hr));
        Wire.write(dec2bcd(wd)); // day of week: Sunday = 1, Saturday = 
        Wire.write(dec2bcd(d));
        Wire.write(dec2bcd(mth));
        Wire.write(dec2bcd(yr));
        Wire.endTransmission();
}
void readDS1307(void)
{
        Wire.beginTransmission(DS1307);
        Wire.write((byte)0x00);
        Wire.endTransmission();
        Wire.requestFrom(DS1307, NumberOfFields);
        
        DS3231.seconds = bcd2dec(Wire.read() & 0x7f);
        DS3231.minutes = bcd2dec(Wire.read() );
        DS3231.hours   = bcd2dec(Wire.read() & 0x3f); // ch? d? 24h
        DS3231.wday   = bcd2dec(Wire.read() );
        DS3231.day    = bcd2dec(Wire.read() );
        DS3231.month  = bcd2dec(Wire.read() );
        DS3231.year   = bcd2dec(Wire.read() );
          
}
float readTemperature(void)
{
    uint8_t msb, lsb;

    Wire.beginTransmission(DS1307);
    
     Wire.write(DS3231_REG_TEMPERATURE);
    
    Wire.endTransmission();

    Wire.requestFrom((int)DS1307, 2);

    while(!Wire.available()) {};
			
    msb = Wire.read();
    lsb = Wire.read();
    

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
	

	int k;
  uint8_t data[] = { 0xff, 0xff, 0xff, 0xff };
  uint8_t blank[] = { 0x00, 0x00, 0x00, 0x00 };
	



  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */

	//////////////////////////////////////////////////////////////////////
//  display.setBrightness(0x0f);	
//	//  // All segments on
//  display.setSegments(data);
//  delay(TEST_DELAY);

//  // Selectively set different digits
//  data[0] = display.encodeDigit(0);
//  data[1] = display.encodeDigit(1);
//  data[2] = display.encodeDigit(2);
//  data[3] = display.encodeDigit(3);
//  display.setSegments(data);
//  delay(TEST_DELAY);
//	 display.clear();
//  display.setSegments(data+2, 2, 2);
//  delay(TEST_DELAY);
//	
//  display.clear();
//  display.setSegments(data+2, 2, 1);
//  delay(TEST_DELAY);

//  display.clear();
//  display.setSegments(data+1, 3, 1);
//  delay(TEST_DELAY);
//	display.clear();
//  display.showNumberDecEx(102, B01000000, true, 4, 0);//xem file .h
//  delay(TEST_DELAY);
//	display.clear();
//  display.display_number(-10,0);//////
//  delay(TEST_DELAY);

	 /* Initialize all configured peripherals */
  //MX_GPIO_Init();
	pinMode(PD13,OUTPUT);
	pinMode(PD12,OUTPUT);
	
	delay(100);
	
  /* USER CODE BEGIN 2 */
		//USART_Init(USART2,Pins_PA2PA3,115200);	 
//// 
		//USART_Puts(	USART2,(char*)"LVD_IC \n" );
		

		Serial.begin(115200);
		//printf( "LVD_IC \n" );
		//Serial.println("Please ");
		Serial.println(123);
		
		//DMA_USARTx_Init(USART2);//huartdma
		//DMA_USARTx_Init(USART3);//uartdma
		//HAL_UART_Receive_DMA(&uartdma,(uint8_t*)rx,100);
		
///////////////////////////////////////////////////////////////////
//		
//	Timer(1000, myfunc, t_period, NULL );//timer0
//  Timer(3000, myfunc1, t_period, NULL );//timer1
	/////////////////////////////////////////////////////////////////
		
  /* USER CODE END 2 */
		Wire.begin();//B9, B8
		delay(500);
		setTime(13, 40, 0, 4, 1, 1, 20); // 12:30:45 CN 08-02-2015
		delay(200);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  pwmMode( PD12, 5000, PWM_8_BIT);//PWM_12_BIT
	pwmWrite(PD12, 255);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		if (Serial.available()) {   
    
    Serial.print( (char)Serial.read());
			
		}


//	int chk = DHT11.read(DHT11PIN);
//  
//  Serial.print("Temp: ");
//  Serial.print((float)DHT11.temperature, 2);
//  Serial.print(" C ");  
//  
//  Serial.print(" RelF: ");
//  Serial.print((float)DHT11.humidity, 2);
//  Serial.print(" %");
//  
//  delay(2000);
		
////////////////////DMA//////////////////////		
//   delay(1000);
//		HAL_UART_Receive_DMA(&huartdma, (uint8_t *)rx, 6);
//    delay(10);
//		
//		if(rx[0] == '1'&&rx[1] == '2'&&rx[2] == '3')
//	  {
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//			HAL_UART_Transmit_DMA(&huartdma, (uint8_t *)rx,5);
//		}
/////////////////////////////////////////////////
		
//		time = pulseIn(PE7, HIGH, 1000000);
//		/* Convert us to cm */
//		//dis = (float)time * HCSR04_NUMBER;		
//		//printf("t=%d \n",time);
//		
//  /* USER CODE BEGIN 3 */
////		HAL_Delay(1000);  //delay 1s

//		
//		sprintf(a,"TEMP=%2.2f\n",readTemperature());////micros()
//		USART_Puts(	USART1,(char*)a );
//		readDS3231();
//		T=readTemperature();//DS3231
//		/* Format time */
//		if(sec!=DS3231.seconds){
//		sprintf(a, "\nTime: %02d.%02d.%02d \nDate: %02d/%02d/%04d \nT*C: %2.2f\n", DS3231.hours, DS3231.minutes, DS3231.seconds, DS3231.day, DS3231.month, DS3231.year+2000, T );
//		Serial.println(a);
//		sec =	DS3231.seconds;	
//			
//		if(mm!=DS3231.minutes){
//			display.clear();mm=DS3231.minutes;
//			display.showNumberDecEx(((DS3231.hours*100)+DS3231.minutes), B01000000, true, 4, 0);//xem file .h
//			}
//		
//		}

//		//analogWrite(D15, 255);
//				// fade in from min to max in increments of 5 points:          
    for ( int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5) {            
                    
        analogWrite(PD14, fadeValue); analogWrite(PD15, fadeValue);        // sets the value (range from 0 to 255):                 
        delay(30);          // wait for 30 milliseconds to see the dimming effect        
    }  

    // fade out from max to min in increments of 5 points:         
    for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5) {                         
        analogWrite(PD14, fadeValue);  analogWrite(PD15, fadeValue);         // sets the value (range from 0 to 255):        
        delay(30);                              // wait for 30 milliseconds to see the dimming effect         
    }
		//printf("A=%d \n",analogRead(PA5));
//	  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
//		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
//		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
//		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);

		togglePin(PD13);
		delay_ms(500);		//led6.delay(500);
		
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
//////////////////////HSI   
//	RCC_OscInitTypeDef RCC_OscInitStruct = {};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
//  RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
//  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
//    Error_Handler();
//  }
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
//                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
//    Error_Handler();
//  }
//  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
//  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
//  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
//    Error_Handler();
//  }
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
