/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "exti.h"
#include "usart.h"

#include "adc.h"
/////////////////////////////////////////////////////////////////////////////////////

/* Private variables ---------------------------------------------------------*/
uint8_t serial_available_1 = 0;
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


/* Private function prototypes -----------------------------------------------*/
//void MX_GPIO_Init(void);
//	

char buffer[100];
uint8_t rx_buff[100];
int i=0;

void Enable_RDP_Level1(void) {
    FLASH_OBProgramInitTypeDef OBInit;

    // 1. M? khóa b? nh? Flash
    HAL_FLASH_Unlock();

    // 2. M? khóa Option Bytes
    HAL_FLASH_OB_Unlock();

    // 3. L?y c?u hình hi?n t?i c?a Option Bytes
    HAL_FLASHEx_OBGetConfig(&OBInit);

    // 4. C?u hình RDP Level 1
    OBInit.OptionType = OPTIONBYTE_RDP;  // Ch? c?u hình RDP
    OBInit.RDPLevel = OB_RDP_LEVEL_1;   // Thi?t l?p RDP Level 1
    HAL_FLASHEx_OBProgram(&OBInit);

    // 5. Kích ho?t Option Bytes
    HAL_FLASH_OB_Launch();

    // 6. Khóa l?i b? nh? Flash và Option Bytes
    HAL_FLASH_OB_Lock();

    HAL_FLASH_Lock();

    // Reset vi di?u khi?n d? áp d?ng thay d?i
//    HAL_NVIC_SystemReset();
}
#define OB_RDP_Level_0   ((uint8_t)0xA5)
#define OB_RDP_Level_1   ((uint8_t)0x00)
//#define OB_RDP_Level_2   ((uint8_t)0xCC)*/ /* Warning: When enabling read protection level 2 
                            
#define FLASH_TIMEOUT_VALUE              50000U /* 50 s */

 uint8_t __attribute__((__section__(".RamFunc1"))) FLASH_OB_RDP_LevelConfigx(uint8_t ReadProtectLevel)
{
  uint8_t status = 0;

    /* If the previous operation is completed, proceed to erase the option bytes */
    SET_BIT(FLASH->CR, FLASH_CR_OPTER);
    SET_BIT(FLASH->CR, FLASH_CR_STRT);

    /* Wait for last operation to be completed */
    FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

    /* If the erase operation is completed, disable the OPTER Bit */
    CLEAR_BIT(FLASH->CR, FLASH_CR_OPTER);

//    if(status == HAL_OK)
//    {
      /* Enable the Option Bytes Programming operation */
      SET_BIT(FLASH->CR, FLASH_CR_OPTPG);

      WRITE_REG(OB->RDP, ReadProtectLevel);

      /* Wait for last operation to be completed */
      FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

      /* if the program operation is completed, disable the OPTPG Bit */
      CLEAR_BIT(FLASH->CR, FLASH_CR_OPTPG);
//    }


  return status;
}
int main(void)
{
	HAL_Init();
	SystemClock_Config(); 

	
	pinMode(PC13,OUTPUT);
	WRITE_REG(FLASH->KEYR, FLASH_KEY1);
    WRITE_REG(FLASH->KEYR, FLASH_KEY2);
    WRITE_REG(FLASH->OPTKEYR, FLASH_KEY1);
    WRITE_REG(FLASH->OPTKEYR, FLASH_KEY2);

    while(FLASH_OB_RDP_LevelConfigx(OB_RDP_Level_1) != 0) // Write the option byte until it succeeds

      /* Force OB Load */
    NVIC_SystemReset(); // Reset the system
while (1)
{
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		delay(500);
}
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct ;
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
    //Error_Handler();
  }
  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    //Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    //Error_Handler();
  }

	
}


/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
//void MX_GPIO_Init(void)
//{

//  GPIO_InitTypeDef GPIO_InitStruct;

//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();

//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

//  /*Configure GPIO pins : PC13 */
//  GPIO_InitStruct.Pin = GPIO_PIN_13;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart==&uartdma) serial_available_1 = 1;
    //if (huart==&huart2) serial_available_2 = 1;
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
 

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
