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

//#include "Wire.h" 
//#include <TM1637Display.h>
//#include "HardwareSerial.h"
//#include <dht11.h>
 
#include <TCS3200.h>

uint8_t RGBvalue[3];

TCS3200 colSens;

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
#include <Array.h> //M?ng
//Ð?nh nghia l?p màu
class Color
{
  public:
  String name; //Tên màu
  int r,g,b; //Giá tr? R, G, B
  Color()
  {
    
  }
  Color(int red, int green, int blue, String _name)  //Constructor kh?i t?o
    { 
        r = red;
        g = green;
        b = blue;
        name = _name;
    }   
    
};

uint8_t red = 0;
uint8_t green = 0;
uint8_t blue = 0;

const int nColor = 9;
Array<Color*,nColor> knownColor;
/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
/////////////////////////////////////////////////////////////////////////////

//uint8_t R_t=0;uint8_t color = 0;
// Custom Function - readColor()
//uint8_t Color() {
//  

//	uint8_t R = RGBvalue[0];
//	uint8_t G = RGBvalue[1];
//	uint8_t B = RGBvalue[2];
//	
//	
//	
//	if(R<205 & R>140 & G>B & R>B){
//    if(abs(R-R_t)<1)color = 4; // Yellow
//		R_t=R;
//  }
//		
//	else if(R<220&B-G>20&R>150&B>150){
//    if(abs(R-R_t)<1)color = 8; // purple
//		R_t=R;
//  }
//	else if(R<245&B-G>20&R>B&B<144&G>90){
//    if(abs(R-R_t)<1)color = 2; // pink
//		R_t=R;
//  }
//	else if(R<250&R>220&abs(G-B)<5){
//    if(abs(R-R_t)<1)color = 5; // Orange
//		R_t=R;
//  }

//	else if(R<200 & G<R & B>80){
//		if(abs(R-R_t)<1){			
//    if(R+B>290) color = 7; // black
//		else color = 0; // white
//		}
//		R_t=R;
//  }
//	
////  if(R<56 & R>46 & G<65 & G>55){
////    color = 5; // Brown
////  }
//	
/////////////////////////////////////////////////////////
//  else if((R>245&G<115)|(R>230&R>G&R>B&G<120)){
//    if(abs(R-R_t)<1)color = 1; // Red
//		R_t=R;
//  }

//  else if(G>R&G>B){
//    if(abs(R-R_t)<1)color = 3; // Green
//		R_t=R;
//  }
//  

//  else if (B>R&B>G){
//    if(abs(R-R_t)<1)color = 6; // Blue
//		R_t=R;
//  }
//  return color;  
//}

double computeDistance(int pixR, int pixG, int pixB, int r, int g,int b) 
 {
      return (double) ((abs(pixR - r)  + abs(pixG - g) + abs(pixB - b)));
 }    
    
String getColorNameFromRgb(int r, int g, int b) {
    int idx = 0;    
    double minDistance = 99999;
    int mse;

  Serial.print("| Distance:  ");
    for(int i =0 ;i < nColor ; i++)
    {
      mse = computeDistance(r, g, b,knownColor[i]->r,knownColor[i]->g,knownColor[i]->b);
      
       Serial.print(mse);
       Serial.print(" ");
       if (mse < minDistance)
       {        
        minDistance = mse;
        idx = i;
      }
    }
    Serial.print(idx);
    return knownColor[idx]->name;
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
// INPUT 
// OUTPUT 
// OUTPUT_OD 
// INPUT_PULLUP 
// INPUT_PULLDOWN 
// AF_OD 
// AF_PP 
	
//	pinMode(PD15,OUTPUT);
	pinMode(PC13,OUTPUT);
//	pinMode(PD12,OUTPUT);
//	digitalWrite( PD12, LOW );
	digitalWrite( PC13, LOW );
//	digitalWrite( PD15, LOW );
	
	delay(100);
	
	///////////////////////////////////////////////////////////////////	

		Serial.begin(115200);
//		//printf( "LVD_IC \n" );
		Serial.println("Hello!");
//		Serial.println(123);
		

	/////////////////////////////////////////////////////////////////		
		colSens.begin();
  /* Infinite loop */
  knownColor.push_back(new Color(255,77,96,"Red"));
	knownColor.push_back(new Color(222, 55, 130,"Pink"));
  knownColor.push_back(new Color(110,155,130,"Green"));
  knownColor.push_back(new Color(35,163,237,"Blue"));
  knownColor.push_back(new Color(210,158,18,"Yellow"));
  knownColor.push_back(new Color(244,91,91,"Orange"));
  knownColor.push_back(new Color(140, 98, 190,"Indigo"));
  knownColor.push_back(new Color(82,96,108,"Black"));
  knownColor.push_back(new Color(81,102,114,"White"));
	
  while (1)
  {
		
	
		colSens.loop();
		colSens.getRGB (RGBvalue);
		//colSens.getRGBtoMaxCorrection (RGBvalue);
		
//		red = map(RGBvalue[0], 90,255,0,255);
//		green = map(RGBvalue[1], 85,180,0,255);
//		blue = map(RGBvalue[2], 105,190,0,255);
		
		red = RGBvalue[0];
		green = RGBvalue[1];
		blue = RGBvalue[2];
		
				Serial.print("Color:\t");
				Serial.print("\tR:\t"); Serial.print(red);//RGBvalue[0]
				Serial.print("\tG:\t"); Serial.print(green);//RGBvalue[1]
				Serial.print("\tB:\t"); Serial.print(blue);//RGBvalue[2]
				Serial.print("\t");
				Serial.println();  
		
		if(getColorNameFromRgb(red,green,blue) == "Red")
    {
     
     Serial.println("RED");

   
    } else  if(getColorNameFromRgb(red,green,blue) == "Pink")
    {
     
     Serial.println("PINK"); 
   
    } else  if(getColorNameFromRgb(red,green,blue) == "Green")
    {
     
     Serial.println("GREEN"); 
   
    }else  if(getColorNameFromRgb(red,green,blue) == "Blue")
    {
      
     Serial.println("BLUE");
   
    }else  if(getColorNameFromRgb(red,green,blue) == "Black")
    {
      
     Serial.println("Black");
   
    }else  if(getColorNameFromRgb(red,green,blue) == "White")
    {
      
     Serial.println("White");
   
    }else  if(getColorNameFromRgb(red,green,blue) == "Yellow")
    {
     
     Serial.println("Yellow"); 
   
    }else  if(getColorNameFromRgb(red,green,blue) == "Orange")
    {
     
     Serial.println("Orange"); 
   
    }else  if(getColorNameFromRgb(red,green,blue) == "Indigo")
    {
      
     Serial.println("Indigo");
   
    }
//		switch (Color())
//		{
//			case 1:
//				Serial.println("RED");
//				break;
//			case 2:
//				Serial.println("PINK");
//				break;
//			case 3:
//				Serial.println("GREEN");
//				break;
//			case 4:
//				Serial.println("YELLOW!");
//				break;
//			case 5:
//				Serial.println("ORANGE");
//				break;
//			case 6:
//				Serial.println("BLUE");
//				break;
//			case 7:
//				Serial.println("BLACK");
//				break;
//			case 8:
//				Serial.println("PURPLE");
//				break;
//			case 0:	
//				Serial.println("WHITE");
//				break;
//		}
		
		togglePin(PC13);delay(50);
		
			
		
  }
  

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
