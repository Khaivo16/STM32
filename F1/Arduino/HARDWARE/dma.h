/**
  ******************************************************************************
  * @file    dma.h
  * @author  xC0000005
  * @version V1.0.0
  * @date    12-July-2019
  * @brief   provide dma callbacks for dma
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_H
#define __DMA_H

#include <Arduino.h>

/**
  * @brief  This function will store the DMA handle in the appropriate slot
  * @param  dma_handle : dma handle
  * @retval None
  */
void prepare_dma(DMA_HandleTypeDef *dma_handle);

/**
  * @brief  This function will remove the DMA handle from the appropriate slot
  * @param  dma_handle : dma handle
  * @retval None
  */
void end_dma(DMA_HandleTypeDef *dma_handle);
/**
 * @brief Initialize a DMA device.
 * @param dev Device to initialize.
 */
 
// typedef enum {
//    DMA_CH0 = 0,                /**< Channel 0 */
//    DMA_CH1 = 1,                /**< Channel 1 */
//    DMA_CH2 = 2,                /**< Channel 2 */
//    DMA_CH3 = 3,                /**< Channel 3 */
//    DMA_CH4 = 4,                /**< Channel 4 */
//    DMA_CH5 = 5,                /**< Channel 5 */
//    DMA_CH6 = 6,                /**< Channel 6 */
//    DMA_CH7 = 7,                /**< Channel 7 */
//} dma_channel;
typedef enum dma_xfer_size {
    DMA_SIZE_8BITS  = ( DMA_PDATAALIGN_BYTE|DMA_MDATAALIGN_BYTE ),  // 8-bit transfers
    DMA_SIZE_16BITS = (DMA_PDATAALIGN_HALFWORD|DMA_MDATAALIGN_HALFWORD),  // 16-bit transfers
    DMA_SIZE_32BITS = (DMA_PDATAALIGN_WORD|DMA_MDATAALIGN_WORD)   // 32-bit transfers
} dma_xfer_size;

static inline void dma_init(const DMA_TypeDef  *dev, DMA_Channel_TypeDef *channel)
{
	IRQn_Type IRQn_dma;
	if(dev==DMA1)__HAL_RCC_DMA1_CLK_ENABLE();
	#ifdef DMA2
	else __HAL_RCC_DMA2_CLK_ENABLE();
	#endif
			 if(channel==DMA1_Channel1) IRQn_dma =DMA1_Channel1_IRQn;
	else if(channel==DMA1_Channel2) IRQn_dma =DMA1_Channel2_IRQn;
	else if(channel==DMA1_Channel3) IRQn_dma =DMA1_Channel3_IRQn;
	else if(channel==DMA1_Channel4) IRQn_dma =DMA1_Channel4_IRQn;
	else if(channel==DMA1_Channel5) IRQn_dma =DMA1_Channel5_IRQn;
	else if(channel==DMA1_Channel6) IRQn_dma =DMA1_Channel6_IRQn;
	else if(channel==DMA1_Channel7) IRQn_dma =DMA1_Channel7_IRQn;
	
	#ifdef DMA2
	else if(channel==DMA2_Channel1) IRQn_dma =DMA2_Channel1_IRQn;
	else if(channel==DMA2_Channel2) IRQn_dma =DMA2_Channel2_IRQn;
	else if(channel==DMA2_Channel3) IRQn_dma =DMA2_Channel3_IRQn;
	else if(channel==DMA2_Channel4) IRQn_dma =DMA2_Channel4_5_IRQn;
	else if(channel==DMA2_Channel5) IRQn_dma =DMA2_Channel4_5_IRQn;
	#endif
	/* DMA1_Channelx_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(IRQn_dma, 0, 0);
	HAL_NVIC_EnableIRQ(IRQn_dma);
	
}

static inline void dma_setup_transfer(const DMA_TypeDef  *dev,
                                      DMA_Channel_TypeDef *    channel,
                                      //dma_channel   channel,
                                      dma_xfer_size trx_size,
                                      __IO void     *peripheral_address,//SPI, UART, I2C,....
                                      const void    *memory_address,//BUFFER
                                      //const void    *memory_address1,//NULL
                                      uint32_t        flags)
{
	
    channel->CCR &=  ~DMA_CCR_EN;//stream->CR &=  ~DMA_SxCR_EN; // disable		
		while( (channel->CCR)&DMA_CCR_EN ); // wait till enable bit is cleared
    channel->CPAR = (uint32_t)peripheral_address;
    channel->CMAR = (uint32_t)memory_address;
    //channel->CM1AR = (uint32_t)memory_address1;
    channel->CCR = (uint32_t)((flags|trx_size) ); // mask out reserved and enable
}

static inline void dma_set_num_transfers(const DMA_TypeDef  *dev, DMA_Channel_TypeDef *channel, uint16_t num_transfers)
{
    channel->CNDTR = (uint32_t)num_transfers;
}

static inline void dma_enable(const DMA_TypeDef  *dev, DMA_Channel_TypeDef *channel)
{
    channel->CCR |= (uint32_t)DMA_CCR_EN;
}

static inline void dma_disable(const DMA_TypeDef  *dev, DMA_Channel_TypeDef *channel)
{
  channel->CCR &= (uint32_t)(~DMA_CCR_EN);
	while (channel->CCR & DMA_CCR_EN); // wait till EN bit is reset, see AN4031, chapter 4.1
}

//-----------------------------------------------------------------------------


#endif
