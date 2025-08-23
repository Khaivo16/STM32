#ifndef __DELAY_H
#define __DELAY_H 	
#ifdef __cplusplus
 extern "C" {
#endif 
#include <sys.h>	  
//////////////////////////////////////////////////////////////////////////////////  
uint32_t millis(void);
uint32_t micros(void);
////////////////////////////////////////////////////////////////////////////////// 	 
void delay_init(void);
void delay_ms(uint32_t nms);
void delay_us(uint32_t nus);
void delay( uint32_t ms );
#ifdef __cplusplus
}
#endif
#endif





























