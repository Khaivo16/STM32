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
 * PID Basic Example
 ********************************************************/

#include <PID_v1.h>

int enc[]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};  // Quadrature Encoder machine state array
int state0=0;
int state1=0;
double pos0,pos1,t0,t1,Out0,Out1;         // PID Input Signal, Output command and Setting speed for each wheel 
double kp0=40,ki0=10,kd0=1.0;             // motors PID constants. Can be modiffied while running with:
double kp1=40,ki1=10,kd1=1.0;             // s nnn (setting point), p nnn (kp), i nnn (ki), d nnn (kd). nnn is divided by 10 to get decimals nnn -> nn.n
int period=1;                             // PID sample timer period in ms
int debug=0;
int toggle=0;

//PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
//PID PID0(&In0, &Out0, &Set0, kp0, ki0, kd0, DIRECT);
//PID PID1(&In1, &Out1, &Set1, kp1, ki1, kd1, DIRECT);
PID PID0(&pos0, &Out0, &t0, kp0, ki0, kd0, DIRECT);
PID PID1(&pos1, &Out1, &t1, kp1, ki1, kd1, DIRECT);

static inline int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

 
/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


void motion(int motor, float vel){
  if(motor==1){
    if(vel>0){
      analogWrite(PA0,vel);
      analogWrite(PA1,0);
    }
    if(vel<0){
      analogWrite(PA0,0);
      analogWrite(PA1,-vel);
    }
    if(vel==0){
      analogWrite(PA0,0);
      analogWrite(PA1,0);
    }
  }
  if(motor==0){
    if(vel>0){
      analogWrite(PA2,vel);
      analogWrite(PA3,0);
    }
    if(vel<0){
      analogWrite(PA2,0);
      analogWrite(PA3,-vel);
    }
    if(vel==0){
      analogWrite(PA2,0);
      analogWrite(PA3,0);
    }
  }
}

void dump(void){
  Serial1.print("kp0:");
  Serial1.print(kp0);
  Serial1.print(" ki0:");
  Serial1.print(ki0);
  Serial1.print(" kd0:");
  Serial1.print(kd0);
  Serial1.print(" t0:");
  Serial1.print(t0);
  Serial1.print(" t1:");
  Serial1.print(t1);
  Serial1.print(" pos:");
  Serial1.print(pos0);
  Serial1.print(" pos1:");
  Serial1.print(pos1);    
  Serial1.print(" Out0:");
  Serial1.print(Out0);    
  Serial1.print(" Out1:");
  Serial1.println(Out1);    
}

void tick(HardwareTimer *HT){
	UNUSED(HT);
  PID0.Compute();
  PID1.Compute();
  motion(0,Out0);
  motion(1,Out1);
}

void encoder1(void){
  state0 = (state0<<2) | (GPIOB->IDR & B11);
  pos0 += enc[state0];
  state0 &= B11;
} 
void encoder2(void){
  state1 = (state1<<2) | ((GPIOB->IDR & B11000)>>3);
  pos1 += enc[state1];
  state1 &= B11;
} 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
/////////////////////////////////////////////////////////////////////////////

char c=0;             // input char from keys

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

	Serial1.begin(115200);
  Serial1.println("Serial1 START");
  pinMode(PB0, INPUT);  // Enc A Motor 1  
  pinMode(PB1, INPUT);  // Enc B Motor 1
  pinMode(PB3, INPUT);  // Enc A Motor 2
  pinMode(PB4, INPUT);  // Enc B Motor 1
//  pinMode(PA0, PWM);
//  pinMode(PA1, PWM);
//  pinMode(PA2, PWM);
//  pinMode(PA3, PWM);
  attachInterrupt(PB0,encoder1,CHANGE);
  attachInterrupt(PB1,encoder1,CHANGE);
  attachInterrupt(PB3,encoder2,CHANGE);
  attachInterrupt(PB4,encoder2,CHANGE);
  state0 = GPIOB->IDR & B11;          //save gpio PB1,PB0 in binary 
  state1 = (GPIOB->IDR & B11000)>>3;  //save gpio PB3,PB4 in binary 
	
	static HardwareTimer Timer3(TIM3);


//  Timer3.setChannel1Mode(TIMER_OUTPUTCOMPARE);
//  Timer3.setPeriod(period*1000); // in microseconds
//  Timer3.setCompare1(1);         // overflow might be small
//  Timer3.attachCompare1Interrupt(tick);	   
	
  Timer3.setOverflow(period*1000,MICROSEC_FORMAT); // thanks to prescaler Tick = microsec
  Timer3.attachInterrupt(tick);
  Timer3.resume();
	
  motion(0,0);
  motion(1,0);
  PID0.SetSampleTime(period);
  PID1.SetSampleTime(period);
  PID0.SetOutputLimits(-255,255);
  PID1.SetOutputLimits(-255,255);
  PID0.SetMode(AUTOMATIC);
  PID1.SetMode(AUTOMATIC); 
	
	
	
	/////////////////////////////////////////////////////////////////
		
  while (1)
  {
		// watch for input
  if(c==0){
    if (Serial1.available() != 0) {
      c = Serial1.read();
      Serial1.println(c);
    }
  } else {
    if (Serial1.available() != 0) {
      int gv = Serial1.parseInt();
      if(c=='0'){
        t0=gv/10.;
      } else if(c=='1'){
        t1=gv/10.;
      } else if(c=='p'){
        kp0=gv/10.;
        kp1=gv/10.;
        PID0.SetTunings(kp0, ki0, kd0);
        PID1.SetTunings(kp1, ki1, kd1);
      } else if(c=='i'){
        ki0=gv/10.;
        ki1=gv/10.;
        PID0.SetTunings(kp0, ki0, kd0);
        PID1.SetTunings(kp1, ki1, kd1);
      } else if(c=='d'){
        kd0=gv/10.;
        kd1=gv/10.;
        PID0.SetTunings(kp0, ki0, kd0);
        PID1.SetTunings(kp1, ki1, kd1);
      } else if(c=='s'){
        dump();
      } else if(c=='b'){
        debug^=1;
      } else {
        Serial1.println("Tunning command not recognized. Use 0 NNN, 1 NNN, p NNN, i NNN, d NNN with NNN=n*10, s to dump variables, b to toggle debug");
      }
      while (Serial1.available()) Serial1.read();
      c=0;
    }
  }
  if(debug){
    dump();
  }
  

		togglePin(PD13);
		delay_ms(100);		
		
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
