
#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED
#ifdef __cplusplus
 extern "C" {
#endif 
	 
#include "sys.h"


#define SPI_MODE0 0x02
#define SPI_MODE1 0x00
#define SPI_MODE2 0x03
#define SPI_MODE3 0x01

#define SPI_CLOCK_DIV2     SPI_BAUDRATEPRESCALER_2  
#define SPI_CLOCK_DIV4     SPI_BAUDRATEPRESCALER_4  
#define SPI_CLOCK_DIV8     SPI_BAUDRATEPRESCALER_8  
#define SPI_CLOCK_DIV16    SPI_BAUDRATEPRESCALER_16 
#define SPI_CLOCK_DIV32    SPI_BAUDRATEPRESCALER_32 
#define SPI_CLOCK_DIV64    SPI_BAUDRATEPRESCALER_64 
#define SPI_CLOCK_DIV128   SPI_BAUDRATEPRESCALER_128
#define SPI_CLOCK_DIV256   SPI_BAUDRATEPRESCALER_256
	 
//	 class SPISettings {
//			public:
//				SPISettings(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) {
//					
//						init_MightInline(clock, bitOrder, dataMode);
//					
//				}
//				SPISettings(uint32_t clock, BitOrder bitOrder, uint8_t dataMode, uint32_t dataSize) {
//					
//						init_MightInline(clock, bitOrder, dataMode);
//					
//				}
//				SPISettings(uint32_t clock) {
//					
//						init_MightInline(clock, MSBFIRST, SPI_MODE0);
//					
//				}
//				SPISettings() { init_AlwaysInline(4000000, MSBFIRST, SPI_MODE0); }
//			private:
//				void init_MightInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) {
//					init_AlwaysInline(clock, bitOrder, dataMode);
//				}
//				void init_AlwaysInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) __attribute__((__always_inline__)) {
//					this->clock = clock;
//					this->bitOrder = bitOrder;
//					this->dataMode = dataMode;
//				}
//				uint32_t clock;
//				uint32_t clockDivider;
//				BitOrder bitOrder;
//				uint8_t dataMode;
//				uint8_t _SSPin;
//				
//				friend class SPIClass;
//};
/////////////////////////////////////////////////////////////////////////////////////////

typedef void (*Funct)(void);

	 
class SPIClass {
  public:
	SPIClass(SPI_TypeDef *_spi, void(*_initCb)(void));

	//byte transfer(uint8_t _data, SPITransferMode _mode = SPI_LAST) { return transfer(BOARD_SPI_DEFAULT_SS, _data, _mode); }
	//byte transfer(byte _channel, uint8_t _data, SPITransferMode _mode = SPI_LAST);
    uint8_t transfer(uint8_t _data);
		uint16_t  transfer16( uint16_t _data);
		void write(uint8_t data);
		void send16(uint16_t data);// 8 bit mode
		void write(const uint16_t data, uint32_t n);
		void write16(const uint16_t data);// 16 bit mode
	/////////////////////////////////////////////////////////////
		void dmaSend(const void * txBuf, uint16_t length, uint16_t flags);
		void DMA_Init(const DMA_TypeDef  *dev, DMA_Channel_TypeDef *channel);
				
	// SPI Configuration methods

	void attachInterrupt(void);
	void detachInterrupt(void);

	void begin(void);
	void end(void);

	// Attach/Detach pin to/from SPI controller
	//void begin(uint8_t _pin);
	//void end(uint8_t _pin);

	// These methods sets a parameter on a single pin
	void setBitOrder( uint16_t );
	void setdataSize( uint16_t );
	void setDataMode( uint8_t);
	void setClockDivider( uint8_t);
	void setClock( uint32_t freq );

	protected:
	DMA_Channel_TypeDef *channelTx1 ,  *channelRx1;
	DMA_Channel_TypeDef *channelTx2 ,  *channelRx2;
	DMA_Channel_TypeDef *channelTx3 ,  *channelRx3;
		
  private:
	void init();

	SPI_TypeDef *spi;
	//uint32_t id;
	uint16_t dataSize;
	uint16_t bitOrder;
	uint32_t divider;
	uint32_t mode;
	Funct initCb;//////////////////////////typedef void (*Funct)(void);
	bool initialized;
  SPI_InitTypeDef SPI_InitStructure;
};


extern SPIClass SPI;

extern SPIClass SPIx;

extern SPIClass SPI_3;


#ifdef __cplusplus
}
#endif

#endif

