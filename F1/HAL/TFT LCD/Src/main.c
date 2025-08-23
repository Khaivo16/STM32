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

#include "sys.h"
#include "tft.h"


/* USER CODE BEGIN Includes */


///* USER CODE END Includes */

///* Private variables ---------------------------------------------------------*/


///* Private variables ---------------------------------------------------------*/

///* USER CODE BEGIN PV */
///* Private variables ---------------------------------------------------------*/



///* USER CODE END PV */

///* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
///////////////////////////////////////////////////////////////////////////////
uint16_t ID = 0,wid=0,ht =0;

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  //__HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA5 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4 
                           PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/********************************* TESTS  *********************************************/
#define min(a, b) (((a) < (b)) ? (a) : (b))
static uint16_t color565_to_555(uint16_t color) {
    return (color & 0xFFC0) | ((color & 0x1F) << 1) | ((color & 0x01));  //lose Green LSB, extend Blue LSB
}
static uint16_t color555_to_565(uint16_t color) {
    return (color & 0xFFC0) | ((color & 0x0400) >> 5) | ((color & 0x3F) >> 1); //extend Green LSB
}
static uint8_t color565_to_r(uint16_t color) {
    return ((color & 0xF800) >> 8);  // transform to rrrrrxxx
}
static uint8_t color565_to_g(uint16_t color) {
    return ((color & 0x07E0) >> 3);  // transform to ggggggxx
}
static uint8_t color565_to_b(uint16_t color) {
    return ((color & 0x001F) << 3);  // transform to bbbbbxxx
}

uint16_t color565(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3); }
	
	
///////////////////////////////////////////////
void testFillScreen()
{
    TFT_fillScreen(BLACK);
    TFT_fillScreen(RED);
    TFT_fillScreen(GREEN);
    TFT_fillScreen(BLUE);
    TFT_fillScreen(BLACK);
}

void testLines(uint16_t color)
{
    int           x1, y1, x2, y2,
                  w = width(),
                  h = height();

    TFT_fillScreen(BLACK);

    x1 = y1 = 0;
    y2    = h - 1;
    for (x2 = 0; x2 < w; x2 += 6) TFT_drawLine(x1, y1, x2, y2, color);
    x2    = w - 1;
    for (y2 = 0; y2 < h; y2 += 6) TFT_drawLine(x1, y1, x2, y2, color);

    TFT_fillScreen(BLACK);

    x1    = w - 1;
    y1    = 0;
    y2    = h - 1;
    for (x2 = 0; x2 < w; x2 += 6) TFT_drawLine(x1, y1, x2, y2, color);
    x2    = 0;
    for (y2 = 0; y2 < h; y2 += 6) TFT_drawLine(x1, y1, x2, y2, color);

    TFT_fillScreen(BLACK);

    x1    = 0;
    y1    = h - 1;
    y2    = 0;
    for (x2 = 0; x2 < w; x2 += 6) TFT_drawLine(x1, y1, x2, y2, color);
    x2    = w - 1;
    for (y2 = 0; y2 < h; y2 += 6) TFT_drawLine(x1, y1, x2, y2, color);

    TFT_fillScreen(BLACK);

    x1    = w - 1;
    y1    = h - 1;
    y2    = 0;
    for (x2 = 0; x2 < w; x2 += 6) TFT_drawLine(x1, y1, x2, y2, color);
    x2    = 0;
    for (y2 = 0; y2 < h; y2 += 6) TFT_drawLine(x1, y1, x2, y2, color);

}

void testFastLines(uint16_t color1, uint16_t color2)
{
    int           x, y, w = width(), h = height();

    TFT_fillScreen(BLACK);
    for (y = 0; y < h; y += 5) TFT_drawFastHLine(0, y, w, color1);
    for (x = 0; x < w; x += 5) TFT_drawFastVLine(x, 0, h, color2);
}

void testRects(uint16_t color) {
    int           n, i, i2,
                  cx = width()  / 2,
                  cy = height() / 2;

    TFT_fillScreen(BLACK);
    n     = min(width(), height());
    for (i = 2; i < n; i += 6) {
        i2 = i / 2;
        TFT_drawRect(cx - i2, cy - i2, i, i, color);
    }

}

void testFilledRects(uint16_t color1, uint16_t color2)
{
    int           n, i, i2,
                  cx = width()  / 2 - 1,
                  cy = height() / 2 - 1;

    TFT_fillScreen(BLACK);
    n = min(width(), height());
    for (i = n; i > 0; i -= 6) {
        i2    = i / 2;

        TFT_fillRect(cx - i2, cy - i2, i, i, color1);

        TFT_drawRect(cx - i2, cy - i2, i, i, color2);
    }
}

void testFilledCircles(uint8_t radius, uint16_t color)
{
    int x, y, w = width(), h = height(), r2 = radius * 2;

    TFT_fillScreen(BLACK);
    for (x = radius; x < w; x += r2) {
        for (y = radius; y < h; y += r2) {
            TFT_fillCircle(x, y, radius, color);
        }
    }

}

void testCircles(uint8_t radius, uint16_t color)
{
    int           x, y, r2 = radius * 2,
                        w = width()  + radius,
                        h = height() + radius;

    // Screen is not cleared for this one -- this is
    // intentional and does not affect the reported time.
    for (x = 0; x < w; x += r2) {
        for (y = 0; y < h; y += r2) {
            TFT_drawCircle(x, y, radius, color);
        }
    }

}

void testTriangles() {
    int           n, i, cx = width()  / 2 - 1,
                        cy = height() / 2 - 1;

    TFT_fillScreen(BLACK);
    n     = min(cx, cy);
    for (i = 0; i < n; i += 5) {
        TFT_drawTriangle(
            cx    , cy - i, // peak
            cx - i, cy + i, // bottom left
            cx + i, cy + i, // bottom right
            color565(0, 0, i));
    }

}

void testFilledTriangles() {
    int           i, cx = width()  / 2 - 1,
                     cy = height() / 2 - 1;

    TFT_fillScreen(BLACK);
    for (i = min(cx, cy); i > 10; i -= 5) {
        TFT_fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                         color565(0, i, i));
        TFT_drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                         color565(i, i, 0));
    }
}

void testRoundRects() {
    int           w, i, i2, red, step,
                  cx = width()  / 2 - 1,
                  cy = height() / 2 - 1;

    TFT_fillScreen(BLACK);
    w     = min(width(), height());
    red = 0;
    step = (256 * 6) / w;
    for (i = 0; i < w; i += 6) {
        i2 = i / 2;
        red += step;
        TFT_drawRoundRect(cx - i2, cy - i2, i, i, i / 8, color565(red, 0, 0));
    }

}

void testFilledRoundRects() {
    int           i, i2, green, step,
                  cx = width()  / 2 - 1,
                  cy = height() / 2 - 1;

    TFT_fillScreen(BLACK);
    green = 256;
    step = (256 * 6) / min(width(), height());
    for (i = min(width(), height()); i > 20; i -= 6) {
        i2 = i / 2;
        green -= step;
        TFT_fillRoundRect(cx - i2, cy - i2, i, i, i / 8, color565(0, green, 0));
    }

}






int main(void)
{




  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */



	 /* Initialize all configured peripherals */
  //MX_GPIO_Init();
	pinMode(PC13,OUTPUT);
	//pinMode(PD12,OUTPUT);

	
	STM32_TFT_8bit();
	
	ID = TFT_readID();
	delay(500);
	//Serial.print("Device ID: 0x"); Serial.println(ID, HEX);
  TFT_begin(ID);
  wid = width();
  ht = height();	

	 if (wid < ht) {
    TFT_setRotation(0);
  } else {
    TFT_setRotation(1);
  }
  
	TFT_fillScreen(BLACK);
	TFT_drawPixel(0, 0, RED);
	
	testFillScreen();
  testLines(CYAN);
  testFastLines(RED, BLUE);
  testFilledCircles(10, MAGENTA);
  testCircles(10, WHITE);

  TFT_fillScreen(BLACK);
//	 TFT_setTextBgColor (BLACK);	
//	 TFT_setTextColor(GREEN);
//   TFT_setTextSize(5);
//	 TFT_write('a');


	TFT_setTextBgColor (BLACK);
  TFT_printnewtstr(100, RED, &mono12x7bold, 3, "HELLO WORLD!");
//	
//	TFT_setCursor(40, 200);TFT_setTextColor (RED);
//	TFT_setFont(&mono12x7bold);
//	TFT_printstr ("abc");
	

   TFT_scrollup(100);

   //TFT_invertDisplay(1);


  while (1)
  {

//		delay(500);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct ;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

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



void HAL_SYSTICK_Callback(void)
{

	
	
}


/* USER CODE BEGIN 4 */


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
