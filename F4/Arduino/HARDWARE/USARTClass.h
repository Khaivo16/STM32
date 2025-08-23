#ifndef _USART_CLASS_
#define _USART_CLASS_


#include <inttypes.h>
#include "Stream.h"

#include "RingBuffer.h"

// Includes Atmel CMSIS
//#include <chip.h>
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal.h"


#define id_serial1 1
#define id_serial2 2
#define id_serial3 3
#define id_serial4 4
#define id_serial5 5
#define id_serial6 6


class USARTClass : public Stream
{
  protected:
    RingBuffer *_rx_buffer ;

  protected:
    USART_TypeDef* _pUsart ;
    USART_InitTypeDef USART_InitStructure ;
    IRQn_Type _dwIrq ;
    uint32_t _dwId ;
		uint8_t pTx;
		uint8_t pRx;

  public:
    //USARTClass( Usart* pUsart, IRQn_Type dwIrq, uint32_t dwId, RingBuffer* pRx_buffer ) ;
    USARTClass( USART_TypeDef* pUsart, IRQn_Type dwIrq, uint32_t dwId, RingBuffer* pRx_buffer ) ;
		
		void setTxRx(uint8_t tx , uint8_t rx)
    {
      pTx = tx;
			pRx = rx;
    };
		
    void begin( const uint32_t dwBaudRate ) ;
    void end( void ) ;
    int available( void ) ;
    int peek( void ) ;
    int read( void ) ;
    void flush( void ) ;
    size_t write( const uint8_t c ) ;
		
		void DMA_Init(const DMA_TypeDef  *dev, DMA_Stream_TypeDef *streamTx, DMA_Stream_TypeDef *streamRx);
		void dmaSend(const void * txBuf, uint16_t length, uint16_t flags);
		void dmaRecive( uint8_t* rxBuf, uint16_t length, uint16_t flags);

    void IrqHandler( void ) ;

		//using Print::write ; // pull in write(str) and write(buf, size) from Print
    operator bool() { return true; }; // USART always active
};

#endif // _USART_CLASS_