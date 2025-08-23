#ifndef __SYS_H
#define __SYS_H	

#ifdef __cplusplus
 extern "C" {
#endif 
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_conf.h"
#include "string.h" 
#include "stdio.h" 
#include "math.h"
	 
void yield(void);
 
extern void Error_Handler(void);
#define _Error_Handler() Error_Handler()
//////////////////////////////////////////////////////////////////////////////////	 
//STM32VN.TK
//********************************************************************************
//////////////////////////////////////////////////////////////////////////////////
	 /* Define attribute */
#if defined   ( __CC_ARM   ) /* Keil uVision 4 */
    #define WEAK (__attribute__ ((weak)))
#elif defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */
    #define WEAK __weak
#elif defined (  __GNUC__  ) /* GCC CS */
    #define WEAK __attribute__ ((weak))
#endif
/////////pgmspace.h/////
#include <inttypes.h>

#define PROGMEM
#define PGM_P  const char *
#define PSTR(str) (str)

#define _SFR_BYTE(n) (n)

typedef void prog_void;
typedef char prog_char;
typedef unsigned char prog_uchar;
typedef int8_t prog_int8_t;
typedef uint8_t prog_uint8_t;
typedef int16_t prog_int16_t;
typedef uint16_t prog_uint16_t;
typedef int32_t prog_int32_t;
typedef uint32_t prog_uint32_t;

#define memcpy_P(dest, src, num) memcpy((dest), (src), (num))
#define strcpy_P(dest, src) strcpy((dest), (src))
#define strcat_P(dest, src) strcat((dest), (src))
#define strcmp_P(a, b) strcmp((a), (b))
#define strstr_P(a, b) strstr((a), (b))
#define strlen_P(a) strlen((a))
#define sprintf_P(s, f, ...) sprintf((s), (f), __VA_ARGS__)

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned char *)(addr))
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#define pgm_read_float(addr) (*(const float *)(addr))

#define pgm_read_byte_near(addr) pgm_read_byte(addr)
#define pgm_read_word_near(addr) pgm_read_word(addr)
#define pgm_read_dword_near(addr) pgm_read_dword(addr)
#define pgm_read_float_near(addr) pgm_read_float(addr)
#define pgm_read_byte_far(addr) pgm_read_byte(addr)
#define pgm_read_word_far(addr) pgm_read_word(addr)
#define pgm_read_dword_far(addr) pgm_read_dword(addr)
#define pgm_read_float_far(addr) pgm_read_float(addr)

#define word(h, l) ((h << 8) | l )

//itoa///////////////////////////////////////////////
#if 0

extern void itoa( int n, char s[] ) ;

#else

extern char* itoa( int value, char *string, int radix ) ;
extern char* ltoa( long value, char *string, int radix ) ;
extern char* utoa( unsigned long value, char *string, int radix ) ;
extern char* ultoa( unsigned long value, char *string, int radix ) ;
#endif /* 0 */
////////////////////////////////////////////////////////////////////////////////// 

//#define FTIR                                   1                  
//#define RTIR                                   2                  


#define GPIO_MODE_IN            0                
#define GPIO_MODE_OUT                1                
#define GPIO_MODE_AF                2                
#define GPIO_MODE_AIN                3                

#define GPIO_SPEED_2M                0                
#define GPIO_SPEED_25M                1                
#define GPIO_SPEED_50M                2                
#define GPIO_SPEED_100M                3                

#define GPIO_PUPD_NONE                0                
#define GPIO_PUPD_PU                1                
#define GPIO_PUPD_PD                2                
#define GPIO_PUPD_RES                3                 

#define GPIO_OTYPE_PP                0                
#define GPIO_OTYPE_OD                1                

//PINx
//#define PIN(n)                              1<<n
#define PIN0                                1<<0
#define PIN1                                1<<1
#define PIN2                                1<<2
#define PIN3                                1<<3
#define PIN4                                1<<4
#define PIN5                                1<<5
#define PIN6                                1<<6
#define PIN7                                1<<7
#define PIN8                                1<<8
#define PIN9                                1<<9
#define PIN10                                1<<10
#define PIN11                                1<<11
#define PIN12                                1<<12
#define PIN13                                1<<13
#define PIN14                                1<<14
#define PIN15                                1<<15 
#define PIN_All														((uint16_t)0xFFFF)
////////////////////////////////////////////////////////////////////////////////// 
																	    
	 
//thiết lập địa chỉ IO trên cổng
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//địa chỉ cổng
//#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
//#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
//#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
//#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
//#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
//#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
//#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
//#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
//#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

//#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
//#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
//#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
//#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
//#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
//#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
//#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
//#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
//#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 

#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO hoạt động , chỉ có một IO duy nhất!
//giá trị n bé hơn 16!
//PAout(0); //PA0 ngõ ra
//PAin(2) ; // PA2 ngõ vào
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  // ngõ ra
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  // ngõ vào

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)   
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)   

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)   
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)   
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)   

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)   
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)   
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n) 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //ngõ ra 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //ngõ vào

///////////////////////////////////////////////////////////////////////////
#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define OUTPUT_OD 0x7
#define INPUT_PULLUP 0x2
#define INPUT_PULLDOWN 0x3
#define AF_OD 0x4
#define AF_PP 0x5
//#define AF_USART  0x9
#define AN_INPUT 0x6
#define TIMER_PWM  0x8
//      LOW 0
//      HIGH 1
#define CHANGE 12
#define FALLING 13
#define RISING 14



/*PWM modes select*/
#define  PWM_8_BIT 			    0
#define  PWM_12_BIT 			1

/*ADC modes selcet*/
#define  ADC_8_BIT              0
#define  ADC_10_BIT             1
#define  ADC_12_BIT             2


#define true 0x1
#define false 0x0

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define SERIAL  0x0
#define DISPLAY 0x1

enum BitOrder {
	LSBFIRST = 1,
	MSBFIRST = 0
};



#define DEFAULT 1
#define EXTERNAL 0

//// undefine stdlib's abs if encountered
//#ifdef abs
//#undef abs
//#endif // abs
//#define abs(x) ((x)>0?(x):-(x))

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif // min

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif // max

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

//extern long random( long ) ;
extern long random( long, long ) ;
extern void randomSeed( uint32_t dwSeed ) ;

extern uint16_t makeWord( uint16_t w ) ;
//extern uint16_t makeWord( uint8_t h, uint8_t l ) ;

//interrupts functions are in WInterrupts.c file
#define interrupts() __enable_irq()
#define noInterrupts() __disable_irq()

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

typedef unsigned int word;

#define bit(b) (1UL << (b))

// TODO: to be checked
typedef uint8_t boolean ;
typedef uint8_t byte ;
//////////////////////////////
/* Types used for the tables below */
typedef struct _PinDescription
{
  GPIO_TypeDef* pPort ;
  uint32_t ulPin ;
  uint32_t ulPeripheral ;
  /*
  EPioType ulPinType ;
  uint32_t ulPinConfiguration ;
  uint32_t ulPinAttribute ;
  */
  /*
  EAnalogChannel ulAnalogChannel ; *//* Analog pin in the Arduino context (label on the board) */
  uint8_t ulADCChannelNumber ; /* ADC Channel number in the SAM device */
  TIM_TypeDef* ulTimerPeripheral;
  uint16_t ulTimerChannel;
  /*
  EPWMChannel ulPWMChannel ;
  ETCChannel ulTCChannel ;
  */
	I2C_TypeDef* ulI2C;
} PinDescription ;

/* Pins table to be instanciated into variant.cpp */
extern const PinDescription g_APinDescription[] ;

#define RCC_CLK_GPIOA           (1u)
#define RCC_CLK_GPIOB           (2u)
#define RCC_CLK_GPIOC           (3u)
#define RCC_CLK_GPIOD           (4u)
#define RCC_CLK_GPIOE           (5u)
#define RCC_CLK_GPIOF           (6u)
#define RCC_CLK_GPIOG           (7u)
#define RCC_CLK_GPIOH           (8u)
#define RCC_CLK_GPIOI           (9u)


#define PA0           (1u)
#define PA1           (2u)
#define PA2           (3u)
#define PA3           (4u)
#define PA4           (5u)
#define PA5           (6u)
#define PA6           (7u)
#define PA7           (8u)
#define PA8           (9u)
#define PA9           (10u)
#define PA10          (11u)
#define PA11          (12u)
#define PA12           (13u)
#define PA13           (14u)
#define PA14           (15u)
#define PA15           (16u)
#define PB0           (17u)
#define PB1           (18u)
#define PB2           (19u)
#define PB3           (20u)
#define PB4           (21u)
#define PB5           (22u)
#define PB6           (23u)
#define PB7           (24u)
#define PB8           (25u)
#define PB9           (26u)
#define PB10          (27u)
#define PB11          (28u)
#define PB12           (29u)
#define PB13           (30u)
#define PB14           (31u)
#define PB15           (32u)

#define PC0           (33u)
#define PC1           (34u)
#define PC2           (35u)
#define PC3           (36u)
#define PC4           (37u)
#define PC5           (38u)
#define PC6           (39u)
#define PC7           (40u)
#define PC8           (41u)
#define PC9           (42u)
#define PC10          (43u)
#define PC11          (44u)
#define PC12           (45u)
#define PC13           (46u)
#define PC14           (47u)
#define PC15           (48u)

#define PD0           (49u)
#define PD1           (50u)
#define PD2           (51u)
#define PD3           (52u)
#define PD4           (53u)
#define PD5           (54u)
#define PD6           (55u)
#define PD7           (56u)
#define PD8           (57u)
#define PD9           (58u)
#define PD10          (59u)
#define PD11          (60u)
#define PD12           (61u)
#define PD13           (62u)
#define PD14           (63u)
#define PD15           (64u)

#define PE0           (65u)
#define PE1           (66u)
#define PE2           (67u)
#define PE3           (68u)
#define PE4           (69u)
#define PE5           (70u)
#define PE6           (71u)
#define PE7           (72u)
#define PE8           (73u)
#define PE9           (74u)
#define PE10          (75u)
#define PE11          (76u)
#define PE12           (77u)
#define PE13           (78u)
#define PE14           (79u)
#define PE15           (80u)

#define PF0           (81u)
#define PF1           (82u)
#define PF2           (83u)
#define PF3           (84u)
#define PF4           (85u)
#define PF5           (86u)
#define PF6           (87u)
#define PF7           (88u)
#define PF8           (89u)
#define PF9           (90u)
#define PF10          (91u)
#define PF11          (92u)
#define PF12           (93u)
#define PF13           (94u)
#define PF14           (95u)
#define PF15           (96u)

//#define PD15           (62u)

///////////////////////////
#define digitalPinToPort(P)        ( g_APinDescription[P].pPort )
#define digitalPinToBitMask(P)     ( g_APinDescription[P].ulPin )

#define digitalPinToTimer(P)       (  )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->ODR) )
#define portInputRegister(port)    ( &(port->IDR) )
//#define portModeRegister(P)        (  )
#define portSetRegister(P)		( &(g_APinDescription[P].pPort->BSRR) )
#define portClearRegister(P)		( &(g_APinDescription[P].pPort->BRR) )

#define portConfigRegister(P)		( &(g_APinDescription[P].pPort->CRL) )
//////////////////////////////////////////////////////////////////////////////////////////
// Serial=Serial1 (usart1)
#define RX1             (PA10) //PA10
#define TX1             (PA9) //PA9

// Serial2(usart2)
#define RX2             (PA3) //PA3
#define TX2             (PA2) //PA2

// Serial3 (usart3)
#define RX3              (PB11) //PB11
#define TX3              (PB10) //PB10


// Serial4 (uart4)
#define RX4              (PC11) //PC11
#define TX4              (PC10) //PC10

// Serial5 (uartl5)
#define RX5              (PD2) //PD2
#define TX5              (PC12) //PC12

// Serial6 (usart6)
#define RX6              (PC7) //PD2
#define TX6              (PC6) //PC12
///////////////////////////////////////////////////////////////////////////////////////////////

#define PIN_SPI_MOSI1         (PA7) //PB5
#define PIN_SPI_MISO1         (PA6)	//PB4
#define PIN_SPI_SCK1          (PA5)	//PB3


#define PIN_SPI_MOSI2        (PB15) 
#define PIN_SPI_MISO2        (PB14) ///////////////////SPI
#define PIN_SPI_SCK2         (PB13) 

#define PIN_SPI_MOSI3         (0u)
#define PIN_SPI_MISO3         (0u)
#define PIN_SPI_SCK3          (0u)

/////////////////////////////////////////////

typedef  void (*Func)(void *);

typedef enum { t_period = 0, t_single}tKind_t;  // the variable types of the timer class . 


typedef struct{
	
	    Func tFunc;
		uint32_t start;
		uint32_t msec;
		void *data;
		tKind_t type;
        uint8_t num;          //timer num;
		uint8_t flag;
}Timer_t;

typedef struct Node{

		struct Node *prev;
		struct Node *next;
		Timer_t *Timer;

}Node_t;

typedef struct {
	Node_t Head;
	uint8_t TotalNum;

}TimerManager_t;

void timer_handler(void);
uint8_t Timer_Add(uint32_t ms, Func tfunc, tKind_t type, void *data);
void Timer_delete(uint8_t Num);
uint32_t get_resitime(uint8_t num);
void change_cbFunc(uint8_t num, Func ttFunc);
void change_ttype(uint8_t num, tKind_t type);

static  TimerManager_t TManager;

void Timer(uint32_t ms, Func tfunc, tKind_t type, void *data);	  
	 void Timer_config(uint32_t ms, Func tfunc, tKind_t type, void *data);
   uint8_t Timer_get_tNum(void);
	 uint32_t Timer_get_resTime(void);
	 void Timer_change_callbackFunc(Func ttFunc);
	 void Timer_change_type(tKind_t mode);
	 
/////////////////////////////////////////////////////////

typedef struct {
	uint32_t curPin;     //
	int16_t presc;      //
	uint16_t arr;        //8-16bit=255、4096.
	
}PWM_Manager_t;

static PWM_Manager_t PWM_Manager;


#define PWM_FREQUENCY		1000
#define PWM_MAX_DUTY_CYCLE	255
#define PWM_MIN_DUTY_CYCLE	0
#define PWM_RESOLUTION		8
/////////////
#define ADC_RESOLUTION		12
////////////////////////////////

#define clockCyclesPerMicrosecond() ( SystemCoreClock / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (SystemCoreClock / 1000L) )
#define microsecondsToClockCycles(a) ( (a) * (SystemCoreClock / 1000000L) )

		
//void yield(void);
// 
//  void Servo(void);
//  uint8_t attach(int pin);           // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
//  uint8_t attachx(int pin, int min, int max); // as above but also sets min and max values for writes. 
//  void detach(void);
//  void write(int value);             // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds 
//  void writeMicroseconds(int value); // Write pulse width in microseconds 
//  int read(void);                        // returns current pulse width as an angle between 0 and 180 degrees
//  int readMicroseconds(void);            // returns current pulse width in microseconds for this servo (was read_us() in first release)
//  uint8_t attached(void);                   // return true if this servo is attached, otherwise false 
//#define MIN_PULSE_WIDTH       544     // the shortest pulse sent to a servo  
//#define MAX_PULSE_WIDTH      2400     // the longest pulse sent to a servo 
//#define DEFAULT_PULSE_WIDTH  1500     // default pulse width when servo is attached
//#define REFRESH_INTERVAL    20000     // minumim time to refresh servos in microseconds 

//#define SERVOS_PER_TIMER       12     // the maximum number of servos controlled by one timer 
//#define MAX_SERVOS   (_Nbr_16timers  * SERVOS_PER_TIMER)

//#define INVALID_SERVO         255     // flag indicating an invalid servo index

//typedef struct  {
//  uint8_t nbr        :6 ;             // a pin number from 0 to 63
//  uint8_t isActive   :1 ;             // true if this channel is enabled, pin not pulsed if false 
//} ServoPin_t   ;  

//typedef struct {
//  ServoPin_t Pin;
//  volatile unsigned int ticks;
//} servo_t;


int digitalRead( uint32_t ulPin );
void digitalWrite( uint32_t ulPin, uint8_t ulVal );
void pinMode(uint32_t PINx,uint32_t MODE);
void pinModeEX(uint32_t PINx,uint32_t MODE,uint32_t Alternate);
void GPIO_Set(GPIO_TypeDef* GPIOx,uint32_t BITx,uint32_t MODE,uint32_t OTYPE,uint32_t OSPEED,uint32_t PUPD);

void togglePin(uint8_t pin) ;
uint8_t isButtonPressed(uint8_t pin, uint32_t pressedLevel);
uint8_t waitForButtonPress(uint8_t pin, uint32_t pressedLevel,uint32_t timeout);

unsigned long pulseIn( uint32_t PINx, uint8_t state, unsigned long timeout);

long map(long x, long in_min, long in_max, long out_min, long out_max);
uint32_t shiftIn( uint32_t ulDataPin, uint32_t ulClockPin, uint32_t ulBitOrder );
void shiftOut(uint32_t ulDataPin, uint32_t ulClockPin, uint32_t ulBitOrder, uint32_t ulVal );
char *dtostrf (double val, signed char width, unsigned char prec, char *sout) ;	

void pwmMode( uint32_t ulPin, uint32_t pwmFre, uint32_t pwmMode);
void pwmWrite(uint32_t ulPin, uint32_t ulValue);
void adcMode(uint32_t ulPin, uint8_t Mode);
uint32_t analogRead(uint32_t ulPin);
void analogWrite(uint32_t ulPin, uint32_t ulValue);

void ADC_DMA_Init(uint32_t Pin);
uint16_t ADC_DMA_GetValue(uint8_t ADC_Channel);
uint16_t analogRead_DMA(uint8_t Pin);

////////////////////////////////////
#define USART_DMA 
extern  UART_HandleTypeDef uartdma;
extern UART_HandleTypeDef huartdma;
////////////////////////////////////
extern "C" void DMA_USARTx_Init(USART_TypeDef* USARTx);


///////////////////////////////////////////////////
////một số chức năng
//void WFI_SET(void);		//WFI
//void INTX_DISABLE(void);//Đóng tất cả các ngắt
//void INTX_ENABLE(void);	//Mở tất cả các ngắt
//void MSR_MSP(uint32_t addr);	//Đặt địa chỉ ngăn xếp

//////////////////////////////////////////////////////////////////////////////////  
uint32_t millis(void);
uint32_t micros(void);
////////////////////////////////////////////////////////////////////////////////// 	 
void delay_init(void);
void delay_ms(uint32_t nms);
void delay_us(uint32_t nus);
void delay( uint32_t ms );
void delayMicroseconds(uint32_t us);
/////////////////////////////////////////////////////////////
#define B0 0
#define B00 0
#define B000 0
#define B0000 0
#define B00000 0
#define B000000 0
#define B0000000 0
#define B00000000 0
#define B1 1
#define B01 1
#define B001 1
#define B0001 1
#define B00001 1
#define B000001 1
#define B0000001 1
#define B00000001 1
#define B10 2
#define B010 2
#define B0010 2
#define B00010 2
#define B000010 2
#define B0000010 2
#define B00000010 2
#define B11 3
#define B011 3
#define B0011 3
#define B00011 3
#define B000011 3
#define B0000011 3
#define B00000011 3
#define B100 4
#define B0100 4
#define B00100 4
#define B000100 4
#define B0000100 4
#define B00000100 4
#define B101 5
#define B0101 5
#define B00101 5
#define B000101 5
#define B0000101 5
#define B00000101 5
#define B110 6
#define B0110 6
#define B00110 6
#define B000110 6
#define B0000110 6
#define B00000110 6
#define B111 7
#define B0111 7
#define B00111 7
#define B000111 7
#define B0000111 7
#define B00000111 7
#define B1000 8
#define B01000 8
#define B001000 8
#define B0001000 8
#define B00001000 8
#define B1001 9
#define B01001 9
#define B001001 9
#define B0001001 9
#define B00001001 9
#define B1010 10
#define B01010 10
#define B001010 10
#define B0001010 10
#define B00001010 10
#define B1011 11
#define B01011 11
#define B001011 11
#define B0001011 11
#define B00001011 11
#define B1100 12
#define B01100 12
#define B001100 12
#define B0001100 12
#define B00001100 12
#define B1101 13
#define B01101 13
#define B001101 13
#define B0001101 13
#define B00001101 13
#define B1110 14
#define B01110 14
#define B001110 14
#define B0001110 14
#define B00001110 14
#define B1111 15
#define B01111 15
#define B001111 15
#define B0001111 15
#define B00001111 15
#define B10000 16
#define B010000 16
#define B0010000 16
#define B00010000 16
#define B10001 17
#define B010001 17
#define B0010001 17
#define B00010001 17
#define B10010 18
#define B010010 18
#define B0010010 18
#define B00010010 18
#define B10011 19
#define B010011 19
#define B0010011 19
#define B00010011 19
#define B10100 20
#define B010100 20
#define B0010100 20
#define B00010100 20
#define B10101 21
#define B010101 21
#define B0010101 21
#define B00010101 21
#define B10110 22
#define B010110 22
#define B0010110 22
#define B00010110 22
#define B10111 23
#define B010111 23
#define B0010111 23
#define B00010111 23
#define B11000 24
#define B011000 24
#define B0011000 24
#define B00011000 24
#define B11001 25
#define B011001 25
#define B0011001 25
#define B00011001 25
#define B11010 26
#define B011010 26
#define B0011010 26
#define B00011010 26
#define B11011 27
#define B011011 27
#define B0011011 27
#define B00011011 27
#define B11100 28
#define B011100 28
#define B0011100 28
#define B00011100 28
#define B11101 29
#define B011101 29
#define B0011101 29
#define B00011101 29
#define B11110 30
#define B011110 30
#define B0011110 30
#define B00011110 30
#define B11111 31
#define B011111 31
#define B0011111 31
#define B00011111 31
#define B100000 32
#define B0100000 32
#define B00100000 32
#define B100001 33
#define B0100001 33
#define B00100001 33
#define B100010 34
#define B0100010 34
#define B00100010 34
#define B100011 35
#define B0100011 35
#define B00100011 35
#define B100100 36
#define B0100100 36
#define B00100100 36
#define B100101 37
#define B0100101 37
#define B00100101 37
#define B100110 38
#define B0100110 38
#define B00100110 38
#define B100111 39
#define B0100111 39
#define B00100111 39
#define B101000 40
#define B0101000 40
#define B00101000 40
#define B101001 41
#define B0101001 41
#define B00101001 41
#define B101010 42
#define B0101010 42
#define B00101010 42
#define B101011 43
#define B0101011 43
#define B00101011 43
#define B101100 44
#define B0101100 44
#define B00101100 44
#define B101101 45
#define B0101101 45
#define B00101101 45
#define B101110 46
#define B0101110 46
#define B00101110 46
#define B101111 47
#define B0101111 47
#define B00101111 47
#define B110000 48
#define B0110000 48
#define B00110000 48
#define B110001 49
#define B0110001 49
#define B00110001 49
#define B110010 50
#define B0110010 50
#define B00110010 50
#define B110011 51
#define B0110011 51
#define B00110011 51
#define B110100 52
#define B0110100 52
#define B00110100 52
#define B110101 53
#define B0110101 53
#define B00110101 53
#define B110110 54
#define B0110110 54
#define B00110110 54
#define B110111 55
#define B0110111 55
#define B00110111 55
#define B111000 56
#define B0111000 56
#define B00111000 56
#define B111001 57
#define B0111001 57
#define B00111001 57
#define B111010 58
#define B0111010 58
#define B00111010 58
#define B111011 59
#define B0111011 59
#define B00111011 59
#define B111100 60
#define B0111100 60
#define B00111100 60
#define B111101 61
#define B0111101 61
#define B00111101 61
#define B111110 62
#define B0111110 62
#define B00111110 62
#define B111111 63
#define B0111111 63
#define B00111111 63
#define B1000000 64
#define B01000000 64
#define B1000001 65
#define B01000001 65
#define B1000010 66
#define B01000010 66
#define B1000011 67
#define B01000011 67
#define B1000100 68
#define B01000100 68
#define B1000101 69
#define B01000101 69
#define B1000110 70
#define B01000110 70
#define B1000111 71
#define B01000111 71
#define B1001000 72
#define B01001000 72
#define B1001001 73
#define B01001001 73
#define B1001010 74
#define B01001010 74
#define B1001011 75
#define B01001011 75
#define B1001100 76
#define B01001100 76
#define B1001101 77
#define B01001101 77
#define B1001110 78
#define B01001110 78
#define B1001111 79
#define B01001111 79
#define B1010000 80
#define B01010000 80
#define B1010001 81
#define B01010001 81
#define B1010010 82
#define B01010010 82
#define B1010011 83
#define B01010011 83
#define B1010100 84
#define B01010100 84
#define B1010101 85
#define B01010101 85
#define B1010110 86
#define B01010110 86
#define B1010111 87
#define B01010111 87
#define B1011000 88
#define B01011000 88
#define B1011001 89
#define B01011001 89
#define B1011010 90
#define B01011010 90
#define B1011011 91
#define B01011011 91
#define B1011100 92
#define B01011100 92
#define B1011101 93
#define B01011101 93
#define B1011110 94
#define B01011110 94
#define B1011111 95
#define B01011111 95
#define B1100000 96
#define B01100000 96
#define B1100001 97
#define B01100001 97
#define B1100010 98
#define B01100010 98
#define B1100011 99
#define B01100011 99
#define B1100100 100
#define B01100100 100
#define B1100101 101
#define B01100101 101
#define B1100110 102
#define B01100110 102
#define B1100111 103
#define B01100111 103
#define B1101000 104
#define B01101000 104
#define B1101001 105
#define B01101001 105
#define B1101010 106
#define B01101010 106
#define B1101011 107
#define B01101011 107
#define B1101100 108
#define B01101100 108
#define B1101101 109
#define B01101101 109
#define B1101110 110
#define B01101110 110
#define B1101111 111
#define B01101111 111
#define B1110000 112
#define B01110000 112
#define B1110001 113
#define B01110001 113
#define B1110010 114
#define B01110010 114
#define B1110011 115
#define B01110011 115
#define B1110100 116
#define B01110100 116
#define B1110101 117
#define B01110101 117
#define B1110110 118
#define B01110110 118
#define B1110111 119
#define B01110111 119
#define B1111000 120
#define B01111000 120
#define B1111001 121
#define B01111001 121
#define B1111010 122
#define B01111010 122
#define B1111011 123
#define B01111011 123
#define B1111100 124
#define B01111100 124
#define B1111101 125
#define B01111101 125
#define B1111110 126
#define B01111110 126
#define B1111111 127
#define B01111111 127
#define B10000000 128
#define B10000001 129
#define B10000010 130
#define B10000011 131
#define B10000100 132
#define B10000101 133
#define B10000110 134
#define B10000111 135
#define B10001000 136
#define B10001001 137
#define B10001010 138
#define B10001011 139
#define B10001100 140
#define B10001101 141
#define B10001110 142
#define B10001111 143
#define B10010000 144
#define B10010001 145
#define B10010010 146
#define B10010011 147
#define B10010100 148
#define B10010101 149
#define B10010110 150
#define B10010111 151
#define B10011000 152
#define B10011001 153
#define B10011010 154
#define B10011011 155
#define B10011100 156
#define B10011101 157
#define B10011110 158
#define B10011111 159
#define B10100000 160
#define B10100001 161
#define B10100010 162
#define B10100011 163
#define B10100100 164
#define B10100101 165
#define B10100110 166
#define B10100111 167
#define B10101000 168
#define B10101001 169
#define B10101010 170
#define B10101011 171
#define B10101100 172
#define B10101101 173
#define B10101110 174
#define B10101111 175
#define B10110000 176
#define B10110001 177
#define B10110010 178
#define B10110011 179
#define B10110100 180
#define B10110101 181
#define B10110110 182
#define B10110111 183
#define B10111000 184
#define B10111001 185
#define B10111010 186
#define B10111011 187
#define B10111100 188
#define B10111101 189
#define B10111110 190
#define B10111111 191
#define B11000000 192
#define B11000001 193
#define B11000010 194
#define B11000011 195
#define B11000100 196
#define B11000101 197
#define B11000110 198
#define B11000111 199
#define B11001000 200
#define B11001001 201
#define B11001010 202
#define B11001011 203
#define B11001100 204
#define B11001101 205
#define B11001110 206
#define B11001111 207
#define B11010000 208
#define B11010001 209
#define B11010010 210
#define B11010011 211
#define B11010100 212
#define B11010101 213
#define B11010110 214
#define B11010111 215
#define B11011000 216
#define B11011001 217
#define B11011010 218
#define B11011011 219
#define B11011100 220
#define B11011101 221
#define B11011110 222
#define B11011111 223
#define B11100000 224
#define B11100001 225
#define B11100010 226
#define B11100011 227
#define B11100100 228
#define B11100101 229
#define B11100110 230
#define B11100111 231
#define B11101000 232
#define B11101001 233
#define B11101010 234
#define B11101011 235
#define B11101100 236
#define B11101101 237
#define B11101110 238
#define B11101111 239
#define B11110000 240
#define B11110001 241
#define B11110010 242
#define B11110011 243
#define B11110100 244
#define B11110101 245
#define B11110110 246
#define B11110111 247
#define B11111000 248
#define B11111001 249
#define B11111010 250
#define B11111011 251
#define B11111100 252
#define B11111101 253
#define B11111110 254
#define B11111111 255
////////////////////////////////
#define BIT0 (1 << 0)
#define BIT1 (1 << 1)
#define BIT2 (1 << 2)
#define BIT3 (1 << 3)
#define BIT4 (1 << 4)
#define BIT5 (1 << 5)
#define BIT6 (1 << 6)
#define BIT7 (1 << 7)
#define BIT8 (1 << 8)
#define BIT9 (1 << 9)
#define BIT10 (1 << 10)
#define BIT11 (1 << 11)
#define BIT12 (1 << 12)
#define BIT13 (1 << 13)
#define BIT14 (1 << 14)
#define BIT15 (1 << 15)
#define BIT16 (1 << 16)
#define BIT17 (1 << 17)
#define BIT18 (1 << 18)
#define BIT19 (1 << 19)
#define BIT20 (1 << 20)
#define BIT21 (1 << 21)
#define BIT22 (1 << 22)
#define BIT23 (1 << 23)
#define BIT24 (1 << 24)
#define BIT25 (1 << 25)
#define BIT26 (1 << 26)
#define BIT27 (1 << 27)
#define BIT28 (1 << 28)
#define BIT29 (1 << 29)
#define BIT30 (1 << 30)
#define BIT31 (1 << 31)
//////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif

void attachInterrupt(uint32_t pin, void (*callback)(void), uint32_t mode);
void detachInterrupt(uint32_t pin);
//void interrupts(void);
//void noInterrupts(void);

#endif











