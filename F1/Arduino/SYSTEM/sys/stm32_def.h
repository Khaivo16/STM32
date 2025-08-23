#ifndef _STM32_DEF_
#define _STM32_DEF_

#include "sys.h"

/**
 * @brief STM32 core version number
 */
#define STM32_CORE_VERSION_MAJOR    (0x01U) /*!< [31:24] major version */
#define STM32_CORE_VERSION_MINOR    (0x09U) /*!< [23:16] minor version */
#define STM32_CORE_VERSION_PATCH    (0x00U) /*!< [15:8]  patch version */
/*
 * Extra label for development:
 * 0: official release
 * [1-9]: release candidate
 * F[0-9]: development
 */
#define STM32_CORE_VERSION_EXTRA    (0xF0U) /*!< [7:0]  extra version */
#define STM32_CORE_VERSION          ((STM32_CORE_VERSION_MAJOR << 24U)\
                                        |(STM32_CORE_VERSION_MINOR << 16U)\
                                        |(STM32_CORE_VERSION_PATCH << 8U )\
                                        |(STM32_CORE_VERSION_EXTRA))

//#define USE_HAL_DRIVER



#ifndef F_CPU
#define F_CPU SystemCoreClock
#endif

// Here define some compatibility
//#ifndef CAN1
//#define CAN1 CAN
//#endif

/* SYS_WKUP */
enum WKUP_pin {
#ifdef PWR_WAKEUP_PIN1
  SYS_WKUP1 = PA0,
#endif
#ifdef PWR_WAKEUP_PIN2
  SYS_WKUP2 = 0,
#endif
#ifdef PWR_WAKEUP_PIN3
  SYS_WKUP3 = 0,
#endif
#ifdef PWR_WAKEUP_PIN4
  SYS_WKUP4 = 0,
#endif
#ifdef PWR_WAKEUP_PIN5
  SYS_WKUP5 = 0,
#endif
#ifdef PWR_WAKEUP_PIN6
  SYS_WKUP6 = 0,
#endif
#ifdef PWR_WAKEUP_PIN7
  SYS_WKUP7 = 0,
#endif
#ifdef PWR_WAKEUP_PIN8
  SYS_WKUP8 = 0,
#endif
};

/* USB */
#ifdef USBCON
enum USB_pin {	
  USB_OTG_FS_SOF = PA8,//CDC
  USB_OTG_FS_VBUS = PA9,
  USB_OTG_FS_ID = PA10,
  USB_OTG_FS_DM = PA11,
  USB_OTG_FS_DP = PA12,
	
  USB_OTG_HS_ULPI_D0 = PA3,  
  USB_OTG_HS_ULPI_CK = PA5,
  USB_OTG_HS_ULPI_D1 = PB0,
  USB_OTG_HS_ULPI_D2 = PB1,
  USB_OTG_HS_ULPI_D7 = PB5,
  USB_OTG_HS_ULPI_D3 = PB10,
  USB_OTG_HS_ULPI_D4 = PB11,  
  USB_OTG_HS_ULPI_D5 = PB12,
  USB_OTG_HS_ULPI_D6 = PB13,
	USB_OTG_HS_ULPI_STP = PC0,
  USB_OTG_HS_ULPI_DIR = PC2,
  USB_OTG_HS_ULPI_NXT = PC3,
	
	USB_OTG_HS_SOF = PA4,
	USB_OTG_HS_ID = PB12,
  USB_OTG_HS_VBUS = PB13,
  USB_OTG_HS_DM = PB14,
  USB_OTG_HS_DP = PB15,
  
};
#endif

#define NUM_DIGITAL_PINS        97

#ifdef SDIO

enum SDIO_pin {	
	
  SDMMC1_D0 = PC8,  
  SDMMC1_D1 = PC9,
  SDMMC1_D2 = PC10,  
  SDMMC1_D3 = PC11,
  SDMMC1_D4 = PB8,  
  SDMMC1_D5 = PB9,
  SDMMC1_D6 = PC6,
	SDMMC1_D7 = PC7,
	SDMMC1_CK = PC12,
  SDMMC1_CMD = PD2,

  
};
#endif

enum I2C_pin {	
	
  SDA = PB9,  
  SCL = PB8,

};


#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// weaked functions declaration
extern void SystemClock_Config(void);

//void _Error_Handler(void);


#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif //_STM32_DEF_
