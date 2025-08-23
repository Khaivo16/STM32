#ifndef __CAN_H
#define __CAN_H	 

#include "stm32f1xx_hal.h"


typedef enum {
	Pins_PA11PA12,	//0
	Pins_PB8PB9,	////1:CAN_RX mapped to PB8, CAN_TX mapped to PB9 (not available on 36-pin package)
	Pins_PD0PD1,	//2CAN_RX mapped to PD0, CAN_TX mapped to PD1 (available on 100-pin and 144-pin package)
} CAN_PinsPack;
 
typedef enum  {CAN_50KBPS, CAN_100KBPS, CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1MBPS} BITRATE;


int CANx_Init(CAN_HandleTypeDef* CANx, CAN_PinsPack pinmap, BITRATE bitrate);

#endif