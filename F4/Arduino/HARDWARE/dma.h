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
 
 typedef enum {
    DMA_CH0 = DMA_CHANNEL_0,                /**< Channel 0 */
    DMA_CH1 = DMA_CHANNEL_1,                /**< Channel 1 */
    DMA_CH2 = DMA_CHANNEL_2,                /**< Channel 2 */
    DMA_CH3 = DMA_CHANNEL_3,                /**< Channel 3 */
    DMA_CH4 = DMA_CHANNEL_4,                /**< Channel 4 */
    DMA_CH5 = DMA_CHANNEL_5,                /**< Channel 5 */
    DMA_CH6 = DMA_CHANNEL_6,                /**< Channel 6 */
    DMA_CH7 = DMA_CHANNEL_7,                /**< Channel 7 */
} dma_channel;
typedef enum dma_xfer_size {
    DMA_SIZE_8BITS  = ( DMA_PDATAALIGN_BYTE|DMA_MDATAALIGN_BYTE ),  // 8-bit transfers
    DMA_SIZE_16BITS = (DMA_PDATAALIGN_HALFWORD|DMA_MDATAALIGN_HALFWORD),  // 16-bit transfers
    DMA_SIZE_32BITS = (DMA_PDATAALIGN_WORD|DMA_MDATAALIGN_WORD)   // 32-bit transfers
} dma_xfer_size;

static inline void dma_init(const DMA_TypeDef  *dev, DMA_Stream_TypeDef *stream)
{
	IRQn_Type IRQn_dma;
	if(dev==DMA1)__HAL_RCC_DMA1_CLK_ENABLE();
	else __HAL_RCC_DMA2_CLK_ENABLE();

	if(stream==DMA1_Stream0) IRQn_dma =DMA1_Stream0_IRQn;
	else if(stream==DMA1_Stream1) IRQn_dma =DMA1_Stream1_IRQn;
	else if(stream==DMA1_Stream2) IRQn_dma =DMA1_Stream2_IRQn;
	else if(stream==DMA1_Stream3) IRQn_dma =DMA1_Stream3_IRQn;
	else if(stream==DMA1_Stream4) IRQn_dma =DMA1_Stream4_IRQn;
	else if(stream==DMA1_Stream5) IRQn_dma =DMA1_Stream5_IRQn;
	else if(stream==DMA1_Stream6) IRQn_dma =DMA1_Stream6_IRQn;
	else if(stream==DMA1_Stream7) IRQn_dma =DMA1_Stream7_IRQn;
	
	else if(stream==DMA2_Stream0) IRQn_dma =DMA2_Stream0_IRQn;
	else if(stream==DMA2_Stream1) IRQn_dma =DMA2_Stream1_IRQn;
	else if(stream==DMA2_Stream2) IRQn_dma =DMA2_Stream2_IRQn;
	else if(stream==DMA2_Stream3) IRQn_dma =DMA2_Stream3_IRQn;
	else if(stream==DMA2_Stream4) IRQn_dma =DMA2_Stream4_IRQn;
	else if(stream==DMA2_Stream5) IRQn_dma =DMA2_Stream5_IRQn;
	else if(stream==DMA2_Stream6) IRQn_dma =DMA2_Stream6_IRQn;
	else if(stream==DMA2_Stream7) IRQn_dma =DMA2_Stream7_IRQn;
	
	/* DMA1_Streamx_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(IRQn_dma, 0, 0);
	HAL_NVIC_EnableIRQ(IRQn_dma);
	
}

static inline void dma_setup_transfer(const DMA_TypeDef  *dev,
                                      DMA_Stream_TypeDef *    stream,
                                      dma_channel   channel,
                                      dma_xfer_size trx_size,
                                      __IO void     *peripheral_address,//SPI, UART, I2C,....
                                      const void    *memory_address0,//BUFFER
                                      const void    *memory_address1,//NULL
                                      uint32_t        flags)
{
	
    stream->CR &=  ~DMA_SxCR_EN; // disable
		while( (stream->CR)&DMA_SxCR_EN ); // wait till enable bit is cleared
    stream->PAR = (uint32_t)peripheral_address;
    stream->M0AR = (uint32_t)memory_address0;
    stream->M1AR = (uint32_t)memory_address1;
    stream->CR = (uint32_t)((flags|channel|trx_size) & 0x0feffffe); // mask out reserved and enable
}

static inline void dma_set_num_transfers(const DMA_TypeDef  *dev, DMA_Stream_TypeDef * stream, uint16_t num_transfers)
{
    stream->NDTR = (uint32_t)num_transfers;
}

static inline void dma_enable(const DMA_TypeDef  *dev, DMA_Stream_TypeDef * stream)
{
    stream->CR |= (uint32_t)DMA_SxCR_EN;
}

static inline void dma_disable(const DMA_TypeDef  *dev, DMA_Stream_TypeDef * stream)
{
  stream->CR &= (uint32_t)(~DMA_SxCR_EN);
	while (stream->CR & DMA_SxCR_EN); // wait till EN bit is reset, see AN4031, chapter 4.1
}

//-----------------------------------------------------------------------------


#endif
