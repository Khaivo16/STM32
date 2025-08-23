/**
  ******************************************************************************
  * @file    ethernetif.c
  * @author  MCD Application Team & Wi6Labs
  * @version V1.5.0
  * @date    20-june-2017
  * @brief   This file implements Ethernet network interface drivers for lwIP
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Arduino.h"
#include "stm32_def.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include <string.h>
//#include "PeripheralPins.h"
#include "lwip/igmp.h"
#include "stm32_eth.h"
#include "dp83848.h"
#include "lan8742.h"
#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  <= 0x01050000)
#include "Arduino.h"
#endif

#ifdef __cplusplus
 extern "C" {
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Network interface name */
#define IFNAME0 's'
#define IFNAME1 't'

#define ETH_RX_BUFFER_SIZE                     (1536UL)

#define ETH_DMA_TRANSMIT_TIMEOUT                (20U)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 
@Note: This interface is implemented to operate in zero-copy mode only:
        - Rx buffers are allocated statically and passed directly to the LwIP stack
          they will return back to DMA after been processed by the stack.
        - Tx Buffers will be allocated from LwIP stack memory heap, 
          then passed to ETH HAL driver.

@Notes: 
  1.a. ETH DMA Rx descriptors must be contiguous, the default count is 4, 
       to customize it please redefine ETH_RX_DESC_CNT in stm32xxxx_hal_conf.h
  1.b. ETH DMA Tx descriptors must be contiguous, the default count is 4, 
       to customize it please redefine ETH_TX_DESC_CNT in stm32xxxx_hal_conf.h

  2.a. Rx Buffers number must be between ETH_RX_DESC_CNT and 2*ETH_RX_DESC_CNT
  2.b. Rx Buffers must have the same size: ETH_RX_BUFFER_SIZE, this value must
       passed to ETH DMA in the init field (EthHandle.Init.RxBuffLen)
*/

#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

//__attribute__((section(".RxDecripSection"))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
//__attribute__((section(".TxDecripSection"))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
//__attribute__((section(".RxArraySection"))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE]; /* Ethernet Receive Buffer */
__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE]; /* Ethernet Receive Buffer */


#elif defined ( __GNUC__ ) /* GNU Compiler */ 

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_HandleTypeDef EthHandle;
ETH_TxPacketConfig TxConfig; 

lan8742_Object_t LAN8742;

/* Private function prototypes -----------------------------------------------*/
u32_t sys_now(void);
void pbuf_free_custom(struct pbuf *p);

int32_t ETH_PHY_IO_Init(void);
int32_t ETH_PHY_IO_DeInit (void);
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
int32_t ETH_PHY_IO_GetTick(void);


lan8742_IOCtx_t  LAN8742_IOCtx = {ETH_PHY_IO_Init,
                               ETH_PHY_IO_DeInit,
                               ETH_PHY_IO_WriteReg,
                               ETH_PHY_IO_ReadReg,
                               ETH_PHY_IO_GetTick};

LWIP_MEMPOOL_DECLARE(RX_POOL, 10, sizeof(struct pbuf_custom), "Zero-copy RX PBUF pool");

/* USER CODE BEGIN 3 */

DP83848_Object_t DP83848;
DP83848_IOCtx_t  DP83848_IOCtx = {ETH_PHY_IO_Init,
                                  ETH_PHY_IO_DeInit,
                                  ETH_PHY_IO_WriteReg,
                                  ETH_PHY_IO_ReadReg,
                                  ETH_PHY_IO_GetTick};

/* Private variables ---------------------------------------------------------*/
//#if defined ( __ICCARM__ ) /*!< IAR Compiler */
//  #pragma data_alignment=4
//#endif
//__ALIGN_BEGIN ETH_DMADescTypeDef  DMARxDscrTab[ETH_RXBUFNB] __ALIGN_END;/* Ethernet Rx MA Descriptor */

//#if defined ( __ICCARM__ ) /*!< IAR Compiler */
//  #pragma data_alignment=4
//#endif
//__ALIGN_BEGIN ETH_DMADescTypeDef  DMATxDscrTab[ETH_TXBUFNB] __ALIGN_END;/* Ethernet Tx DMA Descriptor */

//#if defined ( __ICCARM__ ) /*!< IAR Compiler */
//  #pragma data_alignment=4
//#endif
//__ALIGN_BEGIN uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __ALIGN_END; /* Ethernet Receive Buffer */

//#if defined ( __ICCARM__ ) /*!< IAR Compiler */
//  #pragma data_alignment=4
//#endif
//__ALIGN_BEGIN uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __ALIGN_END; /* Ethernet Transmit Buffer */

//static ETH_HandleTypeDef EthHandle;

static uint8_t macaddress[6]= { MAC_ADDR0, MAC_ADDR1, MAC_ADDR2, MAC_ADDR3, MAC_ADDR4, MAC_ADDR5 };

#if LWIP_IGMP
uint32_t ETH_HashTableHigh=0x0;
uint32_t ETH_HashTableLow=0x0;
#endif




/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
                       Ethernet MSP Routines
*******************************************************************************/
/**
  * @brief  Initializes the ETH MSP.
  * @param  heth: ETH handle
  * @retval None
  */
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{


/* Ethernet pins configuration ************************************************/
   /*
        ETH_MII_CRS ----------------------> PA0/PH2
        ETH_MII_RX_CLK/ETH_RMII_REF_CLK---> PA1				
				ETH_MDIO -------------------------> PA2
				ETH_MII_COL ----------------------> PA3/PH3
				ETH_MII_RX_DV/ETH_RMII_CRS_DV ----> PA7			
				
				ETH_MII_RXD2 ---------------------> PB0/PH6
				ETH_MII_RXD3 ---------------------> PB1/PH7
				ETH_PPS_OUT ----------------------> PB5
				ETH_MII_TXD3 ---------------------> PB8
				
        ETH_MDC --------------------------> PC1
				ETH_MII_TXD2 ---------------------> PC2
				ETH_MII_TX_CLK -------------------> PC3
				ETH_MII_RXD0/ETH_RMII_RXD0 -------> PC4
        ETH_MII_RXD1/ETH_RMII_RXD1 -------> PC5
				
				ETH_MII_RX_ER --------------------> PB10/PI10   
        ETH_MII_TX_EN/ETH_RMII_TX_EN -----> PB11/PG11
        ETH_MII_TXD0/ETH_RMII_TXD0 -------> PB12/PG13
        ETH_MII_TXD1/ETH_RMII_TXD1 -------> PB13/PG14
                                                  */

   GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Ethernett MSP init: RMII Mode */
  
  /* Enable GPIOs clocks */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
 // __HAL_RCC_GPIOG_CLK_ENABLE();
	/* Configure PA1, PA2 and PA7 */
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL; 
  GPIO_InitStructure.Alternate = GPIO_AF11_ETH;
  GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Configure PB13 */
  GPIO_InitStructure.Pin = GPIO_PIN_13| GPIO_PIN_11| GPIO_PIN_12;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Configure PC1, PC4 and PC5 */
  GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PG2, PG11, PG13 and PG14 */
 // GPIO_InitStructure.Pin =  GPIO_PIN_2 | GPIO_PIN_11 | GPIO_PIN_13;
 // HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);	

	#ifdef ETH_INPUT_USE_IT
		/* Enable the Ethernet global Interrupt */
		HAL_NVIC_SetPriority(ETH_IRQn, 0x7, 0);
		HAL_NVIC_EnableIRQ(ETH_IRQn);
	#endif /* ETH_INPUT_USE_IT */
  

	  /* Enable Ethernet clocks */
  __HAL_RCC_ETH1MAC_CLK_ENABLE();
  __HAL_RCC_ETH1TX_CLK_ENABLE();
  __HAL_RCC_ETH1RX_CLK_ENABLE();

	
}

/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH)
*******************************************************************************/
/**
  * @brief In this function, the hardware should be initialized.
  * Called from ethernetif_init().
  *
  * @param netif the already initialized lwip network interface structure
  *        for this ethernetif
  */
//static void low_level_init(struct netif *netif)
//{
//  uint32_t regvalue;

//  EthHandle.Instance = ETH;
//  EthHandle.Init.MACAddr = macaddress;
//  EthHandle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
//  EthHandle.Init.Speed = ETH_SPEED_100M;
//  EthHandle.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
//#ifdef ETHERNET_RMII_MODE_CONFIGURATION
//  EthHandle.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;
//#else
//  EthHandle.Init.MediaInterface = ETH_MEDIA_INTERFACE_MII;
//#endif /* ETHERNET_RMII_MODE_CONFIGURATION */
//#ifdef ETH_INPUT_USE_IT
//  EthHandle.Init.RxMode = ETH_RXINTERRUPT_MODE;
//#else
//  EthHandle.Init.RxMode = ETH_RXPOLLING_MODE;
//#endif /* ETH_INPUT_USE_IT */
//  EthHandle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
//	
//#if LAN87xx
//  EthHandle.Init.PhyAddress = LAN8742A_PHY_ADDRESS;//DP83848_PHY_ADDRESS ;//LAN8742A_PHY_ADDRESS;//////////////////////////
//#else 
//EthHandle.Init.PhyAddress = DP83848_PHY_ADDRESS ;
//#endif
//  /* configure ethernet peripheral (GPIOs, clocks, MAC, DMA) */
//  if (HAL_ETH_Init(&EthHandle) == HAL_OK)
//  {
//    /* Set netif link flag */
//    netif->flags |= NETIF_FLAG_LINK_UP;//Serial1.println("OK");
//  }

//  /* Initialize Tx Descriptors list: Chain Mode */
//  HAL_ETH_DMATxDescListInit(&EthHandle, DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);

//  /* Initialize Rx Descriptors list: Chain Mode  */
//  HAL_ETH_DMARxDescListInit(&EthHandle, DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);

//  /* set MAC hardware address length */
//  netif->hwaddr_len = ETH_HWADDR_LEN;

//  /* set MAC hardware address */
//  netif->hwaddr[0] =  macaddress[0];
//  netif->hwaddr[1] =  macaddress[1];
//  netif->hwaddr[2] =  macaddress[2];
//  netif->hwaddr[3] =  macaddress[3];
//  netif->hwaddr[4] =  macaddress[4];
//  netif->hwaddr[5] =  macaddress[5];

//  /* maximum transfer unit */
//  netif->mtu = 1500;

//  /* device capabilities */
//	
//	/* Accept broadcast address and ARP traffic */
////		/* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
////		#if LWIP_ARP
////		netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
////		# else 
////		netif->flags |= NETIF_FLAG_BROADCAST;
////		#endif /* LWIP_ARP */

//  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
//  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

//  /* Enable MAC and DMA transmission and reception */
//  HAL_ETH_Start(&EthHandle);
//	
//	
/////////////////////////////////////////////////////////////////////////		
//#if LWIP_IGMP
//  netif_set_igmp_mac_filter(netif, igmp_mac_filter);
//#endif

//#if LAN87xx
////////////////////////////////////////////LAN87xx//////////////////////////////////////////	
//  /**** Configure PHY to generate an interrupt when Eth Link state changes ****/
//  /* Read Register Configuration */
//  HAL_ETH_ReadPHYRegister(&EthHandle, PHY_IMR, &regvalue);

//  regvalue |= PHY_ISFR_INT4;

//  /* Enable Interrupt on change of link status */
//  HAL_ETH_WritePHYRegister(&EthHandle, PHY_IMR, regvalue );
//#else
///////////////////////////////////DP83848//////////////////////////////////////////////////////////
//    /**** Configure PHY to generate an interrupt when Eth Link state changes ****/
//    /* Read Register Configuration */
//    HAL_ETH_ReadPHYRegister(&EthHandle, PHY_MICR, &regvalue);

//    regvalue |= (PHY_MICR_INT_EN | PHY_MICR_INT_OE);

//    /* Enable Interrupts */
//    HAL_ETH_WritePHYRegister(&EthHandle, PHY_MICR, regvalue);

//    /* Read Register Configuration */
//    HAL_ETH_ReadPHYRegister(&EthHandle, PHY_MISR, &regvalue);

//    regvalue |= PHY_MISR_LINK_INT_EN;

//    /* Enable Interrupt on change of link status */
//    HAL_ETH_WritePHYRegister(&EthHandle, PHY_MISR, regvalue);
//#endif
////////////////////////////////////////////////////////////////////////////
//#if LWIP_IGMP
//  ETH_HashTableHigh=EthHandle.Instance->MACHTHR;
//  ETH_HashTableLow=EthHandle.Instance->MACHTLR;
//#endif

//}


static uint8_t eth_init_status = 0;
static void low_level_init(struct netif *netif)
{ 
   HAL_StatusTypeDef hal_eth_init_status; 
  uint32_t idx = 0;
	eth_init_status = 0;
  /* Start ETH HAL Init */

//   uint8_t MACAddr[6] ;
  EthHandle.Instance = ETH;
//  MACAddr[0] = 0x00;
//  MACAddr[1] = 0x80;
//  MACAddr[2] = 0xE1;
//  MACAddr[3] = 0x00;
//  MACAddr[4] = 0x00;
//  MACAddr[5] = 0x00;
  EthHandle.Init.MACAddr = macaddress;//&MACAddr[0]
//  EthHandle.Init.MediaInterface = HAL_ETH_RMII_MODE;
#ifdef ETHERNET_RMII_MODE_CONFIGURATION
  EthHandle.Init.MediaInterface = HAL_ETH_RMII_MODE;
#else
  EthHandle.Init.MediaInterface = HAL_ETH_MII_MODE;
#endif /* ETHERNET_RMII_MODE_CONFIGURATION */

  EthHandle.Init.TxDesc = DMATxDscrTab;
  EthHandle.Init.RxDesc = DMARxDscrTab;
  EthHandle.Init.RxBuffLen = ETH_RX_BUFFER_SIZE;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  hal_eth_init_status = HAL_ETH_Init(&EthHandle);


  /* set MAC hardware address length */
  netif->hwaddr_len = ETH_HWADDR_LEN;
  
  /* set MAC hardware address */
//  netif->hwaddr[0] =  EthHandle.Init.MACAddr[0];
//  netif->hwaddr[1] =  EthHandle.Init.MACAddr[1];
//  netif->hwaddr[2] =  EthHandle.Init.MACAddr[2];
//  netif->hwaddr[3] =  EthHandle.Init.MACAddr[3];
//  netif->hwaddr[4] =  EthHandle.Init.MACAddr[4];
//  netif->hwaddr[5] =  EthHandle.Init.MACAddr[5];
	/* set MAC hardware address */
		netif->hwaddr[0] =  macaddress[0];
		netif->hwaddr[1] =  macaddress[1];
		netif->hwaddr[2] =  macaddress[2];
		netif->hwaddr[3] =  macaddress[3];
		netif->hwaddr[4] =  macaddress[4];
		netif->hwaddr[5] =  macaddress[5];
  
  /* maximum transfer unit */
  netif->mtu = 1500;
  
  /* Accept broadcast address and ARP traffic */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  #if LWIP_ARP
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
  #else 
    netif->flags |= NETIF_FLAG_BROADCAST;
  #endif /* LWIP_ARP */

  for(idx = 0; idx < ETH_RX_DESC_CNT; idx ++)
  {
    HAL_ETH_DescAssignMemory(&EthHandle, idx, Rx_Buff[idx], NULL);
  } 
  
	
	/* Initialize the RX POOL */
  LWIP_MEMPOOL_INIT(RX_POOL);
	
	memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

  /* End ETH HAL Init */
  
  
	
/* USER CODE BEGIN PHY_PRE_CONFIG */ 
#if LAN87xx
//////////////////////////////////////////LAN87xx//////////////////////////////////////////	
 /* USER CODE END PHY_PRE_CONFIG */
		/* Set PHY IO functions */
		LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx);
		
		/* Initialize the LAN8742 ETH PHY */
		LAN8742_Init(&LAN8742);

		if (hal_eth_init_status == HAL_OK)
		{
		/* Get link state */
		ethernet_link_check_state(netif);
		eth_init_status = 1;
		}
		else 
		{
			Error_Handler();
		}   
#else
/////////////////////////////////DP83848//////////////////////////////////////////////////////////
// USE DP83848 instead of built-in LAN8742

    /* Set PHY IO functions */
    DP83848_RegisterBusIO(&DP83848, &DP83848_IOCtx);

    /* Initialize the LAN8742 ETH PHY */
    DP83848_Init(&DP83848);

    if (hal_eth_init_status == HAL_OK)
    {
      ethernet_link_check_state(netif);
    }
    else
    {
      Error_Handler();
    }	

#endif /*ETH PHY*/
///////////////////////////////////////////////////////////////////////////////////////////
/* USER CODE BEGIN LOW_LEVEL_INIT */ 

/* USER CODE END LOW_LEVEL_INIT */

}

/**
  * @brief This function should do the actual transmission of the packet. The packet is
  * contained in the pbuf that is passed to the function. This pbuf
  * might be chained.
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
  * @return ERR_OK if the packet could be sent
  *         an err_t value if the packet couldn't be sent
  *
  * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
  *       strange results. You might consider waiting for space in the DMA queue
  *       to become availale since the stack doesn't retry to send a packet
  *       dropped because of memory failure (except for the TCP timers).
  */
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  uint32_t i=0, framelen = 0;
  struct pbuf *q;
  err_t errval = ERR_OK;
  ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT];
  
  memset(Txbuffer, 0 , ETH_TX_DESC_CNT*sizeof(ETH_BufferTypeDef));
  
  for(q = p; q != NULL; q = q->next)
  {
    if(i >= ETH_TX_DESC_CNT)	
      return ERR_IF;
    
    Txbuffer[i].buffer = (u8_t*)q->payload;
    Txbuffer[i].len = q->len;
    framelen += q->len;
    
    if(i>0)
    {
      Txbuffer[i-1].next = &Txbuffer[i];
    }
    
    if(q->next == NULL)
    {
      Txbuffer[i].next = NULL;
    }
    
    i++;
  }

  TxConfig.Length = framelen;
  TxConfig.TxBuffer = Txbuffer;
  
  HAL_ETH_Transmit(&EthHandle, &TxConfig, ETH_DMA_TRANSMIT_TIMEOUT);
  
  return errval;

}

/**
  * @brief Should allocate a pbuf and transfer the bytes of the incoming
  * packet from the interface into the pbuf.
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @return a pbuf filled with the received packet (including MAC header)
  *         NULL on memory error
  */
static struct pbuf * low_level_input(struct netif *netif)
{
  struct pbuf *p = NULL;
  ETH_BufferTypeDef RxBuff;
  uint32_t framelength = 0;
  struct pbuf_custom* custom_pbuf;
  
  if (HAL_ETH_IsRxDataAvailable(&EthHandle))
  {
    HAL_ETH_GetRxDataBuffer(&EthHandle, &RxBuff);
    HAL_ETH_GetRxDataLength(&EthHandle, &framelength);
    
    /* Build Rx descriptor to be ready for next data reception */
	HAL_ETH_BuildRxDescriptors(&EthHandle);

    /* Invalidate data cache for ETH Rx Buffers */
    SCB_InvalidateDCache_by_Addr((uint32_t *)RxBuff.buffer, framelength);
    
    custom_pbuf  = (struct pbuf_custom*)LWIP_MEMPOOL_ALLOC(RX_POOL);
    custom_pbuf->custom_free_function = pbuf_free_custom;

    p = pbuf_alloced_custom(PBUF_RAW, framelength, PBUF_REF, custom_pbuf, RxBuff.buffer, ETH_RX_BUFFER_SIZE);
    
    return p;
  }
  else
  {
    return NULL;
  }

}

/**
  * @brief This function should be called when a packet is ready to be read
  * from the interface. It uses the function low_level_input() that
  * should handle the actual reception of bytes from the network
  * interface. Then the type of the received packet is determined and
  * the appropriate input function is called.
  *
  * @param netif the lwip network interface structure for this ethernetif
  */
void ethernetif_input(struct netif *netif)
{
 err_t err;
  struct pbuf *p;
  
  /* move received packet into a new pbuf */
  p = low_level_input(netif);
    
  /* no packet could be read, silently ignore this */
  if (p == NULL) return;
    
  /* entry point to the LwIP stack */
  err = netif->input(p, netif);
    
  if (err != ERR_OK)
  {
    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
    pbuf_free(p);
    p = NULL;
  }
}

/**
  * @brief Returns the current state
  *
  * @param None
  * @return 0 not initialized else 1
  */
uint8_t ethernetif_is_init(void)
{
  return (eth_init_status);
}

#if !LWIP_ARP
/**
 * This function has to be completed by user in case of ARP OFF.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if ...
 */
static err_t low_level_output_arp_off(struct netif *netif, struct pbuf *q, const ip4_addr_t *ipaddr)
{  
  err_t errval;
  errval = ERR_OK;
    
/* USER CODE BEGIN 5 */ 
    
/* USER CODE END 5 */  
    
  return errval;
  
}
#endif /* LWIP_ARP */ 

/**
  * @brief  Custom Rx pbuf free callback
  * @param  pbuf: pbuf to be freed
  * @retval None
  */
void pbuf_free_custom(struct pbuf *p)
{
   struct pbuf_custom* custom_pbuf = (struct pbuf_custom*)p;
  /* Invalidate data cache: lwIP and/or application may have written into buffer */
  SCB_InvalidateDCache_by_Addr((uint32_t *)p->payload, p->tot_len);
  LWIP_MEMPOOL_FREE(RX_POOL, custom_pbuf);
}


/**
  * @brief Should be called at the beginning of the program to set up the
  * network interface. It calls the function low_level_init() to do the
  * actual setup of the hardware.
  *
  * This function should be passed as a parameter to netif_add().
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @return ERR_OK if the loopif is initialized
  *         ERR_MEM if private data couldn't be allocated
  *         any other err_t on error
  */
err_t ethernetif_init(struct netif *netif)
{
   LWIP_ASSERT("netif != NULL", (netif != NULL));
  
#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */

#if LWIP_IPV4
#if LWIP_ARP || LWIP_ETHERNET
#if LWIP_ARP
  netif->output = etharp_output;
#else
  /* The user should write ist own code in low_level_output_arp_off function */
  netif->output = low_level_output_arp_off;
#endif /* LWIP_ARP */
#endif /* LWIP_ARP || LWIP_ETHERNET */
#endif /* LWIP_IPV4 */
 
#if LWIP_IPV6
  netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */

  netif->linkoutput = low_level_output;

  /* initialize the hardware */
  low_level_init(netif);

  return ERR_OK;

}

/**
  * @brief  Returns the current time in milliseconds
  *         when LWIP_TIMERS == 1 and NO_SYS == 1
  * @param  None
  * @retval Current Time value
  */
u32_t sys_now(void)
{
  return HAL_GetTick();
}

///**
//  * @brief  This function sets the netif link status.
//  * @param  netif: the network interface
//  * @retval None
//  */
//void ethernetif_set_link(struct netif *netif)
//{
//  uint32_t regvalue = 0;
//	
//#if LAN87xx
////////////////////////////////LAN87xx/////////////////////////////////////
//  /* Read PHY_MISR*/
//  HAL_ETH_ReadPHYRegister(&EthHandle, PHY_ISFR, &regvalue);

//  /* Check whether the link interrupt has occurred or not */
//  if((regvalue & PHY_ISFR_INT4) != (uint16_t)RESET)
//  {
//    netif_set_link_down(netif);
//  }

//  HAL_ETH_ReadPHYRegister(&EthHandle, PHY_BSR, &regvalue);

//  if((regvalue & PHY_LINKED_STATUS) != (uint16_t)RESET) {
//		#if LWIP_IGMP
//				if (!(netif->flags & NETIF_FLAG_IGMP)) {
//					 netif->flags |= NETIF_FLAG_IGMP;
//					 igmp_init();
//					 igmp_start(netif);
//				}
//		#endif
//    netif_set_link_up(netif);
//  }
//# else /////////////////////////////////DP83848////////////////////////////////////
//	/* Read PHY_MISR*/
//  HAL_ETH_ReadPHYRegister(&EthHandle, PHY_MISR, &regvalue);//Serial1.println(regvalue);
//	/* Check whether the link interrupt has occurred or not */
//  if((regvalue & PHY_LINK_INTERRUPT) != (uint16_t)RESET)
//  {
//    /* Read PHY_SR*/
//    HAL_ETH_ReadPHYRegister(&EthHandle, PHY_SR, &regvalue);
//    
//    /* Check whether the link is up or down*/
//    if((regvalue & PHY_LINK_STATUS)!= (uint16_t)RESET)
//    {
//			#if LWIP_IGMP
//			if (!(netif->flags & NETIF_FLAG_IGMP)) {
//				 netif->flags |= NETIF_FLAG_IGMP;
//				 igmp_init();
//				 igmp_start(netif);
//			}
//			#endif
//      netif_set_link_up(netif);
//    }
//    else
//    {
//      netif_set_link_down(netif);
//    }
//  }
//	
//#endif /* ETHERNET CHIP */	
//////////////////////////////////////////////////////////////////
//}

///**
//  * @brief  Link callback function, this function is called on change of link status
//  *         to update low level driver configuration.
//  * @param  netif: The network interface
//  * @retval None
//  */
//void ethernetif_update_config(struct netif *netif)
//{
//  __IO uint32_t tickstart = 0;
//  uint32_t regvalue = 0;

//  if(netif_is_link_up(netif))
//  {
//    /* Restart the auto-negotiation */
//    if(EthHandle.Init.AutoNegotiation != ETH_AUTONEGOTIATION_DISABLE)
//    {
//      /* Enable Auto-Negotiation */
//      HAL_ETH_WritePHYRegister(&EthHandle, PHY_BCR, PHY_AUTONEGOTIATION);

//      /* Get tick */
//      tickstart = HAL_GetTick();

//      /* Wait until the auto-negotiation will be completed */
//      do
//      {
//        HAL_ETH_ReadPHYRegister(&EthHandle, PHY_BSR, &regvalue);

//        /* Check for the Timeout ( 1s ) */
//        if((HAL_GetTick() - tickstart ) > 1000)
//        {
//          /* In case of timeout */
//          goto error;
//        }

//      } while (((regvalue & PHY_AUTONEGO_COMPLETE) != PHY_AUTONEGO_COMPLETE));

//      /* Read the result of the auto-negotiation */
//      HAL_ETH_ReadPHYRegister(&EthHandle, PHY_SR, &regvalue);

//      /* Configure the MAC with the Duplex Mode fixed by the auto-negotiation process */
//      if((regvalue & PHY_DUPLEX_STATUS) != (uint32_t)RESET)
//      {
//        /* Set Ethernet duplex mode to Full-duplex following the auto-negotiation */
//        EthHandle.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
//      }
//      else
//      {
//        /* Set Ethernet duplex mode to Half-duplex following the auto-negotiation */
//        EthHandle.Init.DuplexMode = ETH_MODE_HALFDUPLEX;
//      }
//      /* Configure the MAC with the speed fixed by the auto-negotiation process */
//      if(regvalue & PHY_SPEED_STATUS)
//      {
//        /* Set Ethernet speed to 10M following the auto-negotiation */
//        EthHandle.Init.Speed = ETH_SPEED_10M;
//      }
//      else
//      {
//        /* Set Ethernet speed to 100M following the auto-negotiation */
//        EthHandle.Init.Speed = ETH_SPEED_100M;
//      }
//    }
//    else /* AutoNegotiation Disable */
//    {
//    error :
//      /* Check parameters */
//      assert_param(IS_ETH_SPEED(EthHandle.Init.Speed));
//      assert_param(IS_ETH_DUPLEX_MODE(EthHandle.Init.DuplexMode));

//      /* Set MAC Speed and Duplex Mode to PHY */
//      HAL_ETH_WritePHYRegister(&EthHandle, PHY_BCR, ((uint16_t)(EthHandle.Init.DuplexMode >> 3) |
//                                                     (uint16_t)(EthHandle.Init.Speed >> 1)));
//    }

//    /* ETHERNET MAC Re-Configuration */
//    HAL_ETH_ConfigMAC(&EthHandle, (ETH_MACInitTypeDef *) NULL);

//    /* Restart MAC interface */
//    HAL_ETH_Start(&EthHandle);
//  }
//  else
//  {
//    /* Stop MAC interface */
//    HAL_ETH_Stop(&EthHandle);
//  }

//  ethernetif_notify_conn_changed(netif);
//}

///**
//  * @brief  This function notify user about link status changement.
//  * @param  netif: the network interface
//  * @retval None
//  */
__weak void ethernetif_notify_conn_changed(struct netif *netif)
{
  /* NOTE : This is function clould be implemented in user file
            when the callback is needed,
  */
  UNUSED(netif);
}
static struct pbuf * low_level_input_own(struct netif *netif)
{
  struct pbuf *p = NULL;
  ETH_BufferTypeDef RxBuff;
  uint32_t framelength = 0;
  struct pbuf_custom* custom_pbuf;


  if (HAL_ETH_IsRxDataAvailable(&EthHandle))
  {
	/* Clean and Invalidate data cache */
	  SCB_CleanInvalidateDCache();

    HAL_ETH_GetRxDataBuffer(&EthHandle, &RxBuff);
    HAL_ETH_GetRxDataLength(&EthHandle, &framelength);

    /* Build Rx descriptor to be ready for next data reception */
    HAL_ETH_BuildRxDescriptors(&EthHandle);


    /* Invalidate data cache for ETH Rx Buffers */
    SCB_InvalidateDCache_by_Addr((uint32_t *)RxBuff.buffer, framelength);


    custom_pbuf  = (struct pbuf_custom*)LWIP_MEMPOOL_ALLOC(RX_POOL);
    custom_pbuf->custom_free_function = pbuf_free_custom;

    p = pbuf_alloced_custom(PBUF_RAW, framelength, PBUF_REF, custom_pbuf, RxBuff.buffer, ETH_RX_BUFFER_SIZE);

    return p;
  }
  else
  {
    return NULL;
  }
}

err_t low_level_output_own(struct netif *netif, struct pbuf *p)
{
  uint32_t i=0, framelen = 0;
  struct pbuf *q;
  err_t errval = ERR_OK;
  ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT];

  memset(Txbuffer, 0 , ETH_TX_DESC_CNT*sizeof(ETH_BufferTypeDef));

  for(q = p; q != NULL; q = q->next)
  {
    if(i >= ETH_TX_DESC_CNT)
      return ERR_IF;

    Txbuffer[i].buffer =(uint8_t*) q->payload;
    Txbuffer[i].len = q->len;
    framelen += q->len;

    if(i>0)
    {
      Txbuffer[i-1].next = &Txbuffer[i];
    }

    if(q->next == NULL)
    {
      Txbuffer[i].next = NULL;
    }

    i++;
  }

  TxConfig.Length = framelen;
  TxConfig.TxBuffer = Txbuffer;

  /* Clean and Invalidate data cache */
  SCB_CleanInvalidateDCache();

  HAL_ETH_Transmit(&EthHandle, &TxConfig, ETH_DMA_TRANSMIT_TIMEOUT);

  return errval;
}

void ethernetif_input_own(struct netif *netif)
{
  err_t err;
  struct pbuf *p;

  /* move received packet into a new pbuf */
  p = low_level_input_own(netif);

  /* no packet could be read, silently ignore this */
  if (p == NULL) return;

  /* entry point to the LwIP stack */
  err = netif->input(p, netif);

  if (err != ERR_OK)
  {
    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
    pbuf_free(p);
    p = NULL;
  }

}


/**
* @brief  Returns the current time in milliseconds
*         when LWIP_TIMERS == 1 and NO_SYS == 1
* @param  None
* @retval Current Time value
*/
u32_t sys_jiffies(void)
{
  return HAL_GetTick();
}



// ethernet_link_check_state function is going to be overdefined here
// because STM32CubeMX automatically regenerates its own function  on
// this name but we are going to use our own function later

//#define ethernet_link_check_state	ethernet_link_check_state_orig

/* USER CODE END 6 */

/*******************************************************************************
                       PHI IO Functions
*******************************************************************************/
/**
  * @brief  Initializes the MDIO interface GPIO and clocks.
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_Init(void)
{  
  /* We assume that MDIO GPIO configuration is already done
     in the ETH_MspInit() else it should be done here 
  */
  
  /* Configure the MDIO Clock */
  HAL_ETH_SetMDIOClockRange(&EthHandle);
  
  return 0;
}

/**
  * @brief  De-Initializes the MDIO interface .
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_DeInit (void)
{
  return 0;
}

/**
  * @brief  Read a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  pRegVal: pointer to hold the register value 
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{
  if(HAL_ETH_ReadPHYRegister(&EthHandle, DevAddr, RegAddr, pRegVal) != HAL_OK)
  {
    return -1;
  }
  
  return 0;
}

/**
  * @brief  Write a value to a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  RegVal: Value to be written 
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{
  if(HAL_ETH_WritePHYRegister(&EthHandle, DevAddr, RegAddr, RegVal) != HAL_OK)
  {
    return -1;
  }
  
  return 0;
}

/**
  * @brief  Get the time in millisecons used for internal PHY driver process.
  * @retval Time value
  */
int32_t ETH_PHY_IO_GetTick(void)
{
  return HAL_GetTick();
}

/**
  * @brief  
  * @retval None
  */
#if LAN87xx
//////////////////////////////////////////LAN87xx//////////////////////////////////////////	
void ethernet_link_check_state(struct netif *netif)
{
  ETH_MACConfigTypeDef MACConf;
  uint32_t PHYLinkState;
  uint32_t linkchanged = 0, speed = 0, duplex =0;
  
  PHYLinkState = LAN8742_GetLinkState(&LAN8742);
  
  if(netif_is_link_up(netif) && (PHYLinkState <= LAN8742_STATUS_LINK_DOWN))
  {
    HAL_ETH_Stop(&EthHandle);
    netif_set_down(netif);
    netif_set_link_down(netif);
  }
  else if(!netif_is_link_up(netif) && (PHYLinkState > LAN8742_STATUS_LINK_DOWN))
  {
    switch (PHYLinkState)
    {
    case LAN8742_STATUS_100MBITS_FULLDUPLEX:
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      linkchanged = 1;
      break;
    case LAN8742_STATUS_100MBITS_HALFDUPLEX:
      duplex = ETH_HALFDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      linkchanged = 1;
      break;
    case LAN8742_STATUS_10MBITS_FULLDUPLEX:
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_10M;
      linkchanged = 1;
      break;
    case LAN8742_STATUS_10MBITS_HALFDUPLEX:
      duplex = ETH_HALFDUPLEX_MODE;
      speed = ETH_SPEED_10M;
      linkchanged = 1;
      break;
    default:
      break;      
    }
    
    if(linkchanged)
    {
      /* Get MAC Config MAC */
      HAL_ETH_GetMACConfig(&EthHandle, &MACConf); 
      MACConf.DuplexMode = duplex;
      MACConf.Speed = speed;
      HAL_ETH_SetMACConfig(&EthHandle, &MACConf);
      HAL_ETH_Start(&EthHandle);
      netif_set_up(netif);
      netif_set_link_up(netif);
    }
  }
}
#else
/////////////////////////////////DP83848//////////////////////////////////////////////////////////
void ethernet_link_check_state(struct netif *netif)
{
  ETH_MACConfigTypeDef MACConf;
  uint32_t PHYLinkState;
  uint32_t linkchanged = 0, speed = 0, duplex =0;
	//digitalWrite(PE4,HIGH);
  PHYLinkState = DP83848_GetLinkState(&DP83848);

  if(netif_is_link_up(netif) && (PHYLinkState <= DP83848_STATUS_LINK_DOWN))
  {
    HAL_ETH_Stop(&EthHandle);
    netif_set_down(netif);
    netif_set_link_down(netif);
  }
  else if(!netif_is_link_up(netif) && (PHYLinkState > DP83848_STATUS_LINK_DOWN))
  {
    switch (PHYLinkState)
    {
    case DP83848_STATUS_100MBITS_FULLDUPLEX:
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      linkchanged = 1;
      break;
    case DP83848_STATUS_100MBITS_HALFDUPLEX:
      duplex = ETH_HALFDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      linkchanged = 1;
      break;
    case DP83848_STATUS_10MBITS_FULLDUPLEX:
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_10M;
      linkchanged = 1;
      break;
    case DP83848_STATUS_10MBITS_HALFDUPLEX:
      duplex = ETH_HALFDUPLEX_MODE;
      speed = ETH_SPEED_10M;
      linkchanged = 1;
      break;
    default:
      break;
    }

    if(linkchanged)
    {
      /* Get MAC Config MAC */
      HAL_ETH_GetMACConfig(&EthHandle, &MACConf);
      MACConf.DuplexMode = duplex;
      MACConf.Speed = speed;
      HAL_ETH_SetMACConfig(&EthHandle, &MACConf);

      HAL_ETH_Start(&EthHandle);
      netif_set_up(netif);
      netif_set_link_up(netif);
    }
  }

		
}

#endif
///////////////////////////////////////////////////////////////////////////////////////////////////
/* USER CODE BEGIN ETH link code for User BSP */ 
    
/* USER CODE END ETH link code for User BSP */


/* USER CODE BEGIN 8 */

//// The ethernek_link_chk_state function runs intead of ethernet_link_check_state
//// because the function generated by CubeMX currently force to put
//// the interface to "up" state even when the cable is unplugged.
//// Additionally, we use DP83848 instead of LAN8742.
//#undef ethernet_link_check_state
//#define ethernet_link_chk_state	ethernet_link_check_state

//void ethernet_link_chk_state(struct netif *netif)
//{
//  ETH_MACConfigTypeDef MACConf;
//  uint32_t PHYLinkState;
//  uint32_t linkchanged = 0, speed = 0, duplex =0;
//digitalWrite(PE4,HIGH);
//  PHYLinkState = DP83848_GetLinkState(&DP83848);

//  if(netif_is_link_up(netif) && (PHYLinkState <= DP83848_STATUS_LINK_DOWN))
//  {
//    HAL_ETH_Stop(&EthHandle);
//    netif_set_down(netif);
//    netif_set_link_down(netif);
//  }
//  else if(!netif_is_link_up(netif) && (PHYLinkState > DP83848_STATUS_LINK_DOWN))
//  {
//    switch (PHYLinkState)
//    {
//    case DP83848_STATUS_100MBITS_FULLDUPLEX:
//      duplex = ETH_FULLDUPLEX_MODE;
//      speed = ETH_SPEED_100M;
//      linkchanged = 1;
//      break;
//    case DP83848_STATUS_100MBITS_HALFDUPLEX:
//      duplex = ETH_HALFDUPLEX_MODE;
//      speed = ETH_SPEED_100M;
//      linkchanged = 1;
//      break;
//    case DP83848_STATUS_10MBITS_FULLDUPLEX:
//      duplex = ETH_FULLDUPLEX_MODE;
//      speed = ETH_SPEED_10M;
//      linkchanged = 1;
//      break;
//    case DP83848_STATUS_10MBITS_HALFDUPLEX:
//      duplex = ETH_HALFDUPLEX_MODE;
//      speed = ETH_SPEED_10M;
//      linkchanged = 1;
//      break;
//    default:
//      break;
//    }

//    if(linkchanged)
//    {
//      /* Get MAC Config MAC */
//      HAL_ETH_GetMACConfig(&EthHandle, &MACConf);
//      MACConf.DuplexMode = duplex;
//      MACConf.Speed = speed;
//      HAL_ETH_SetMACConfig(&EthHandle, &MACConf);

//      HAL_ETH_Start(&EthHandle);
//      netif_set_up(netif);
//      netif_set_link_up(netif);
//    }
//  }

//		ethernetif_notify_conn_changed(netif);
//}

/**
  * @brief  This function set a custom MAC address. This function must be called
  *         before ethernetif_init().
  * @param  mac: mac address
  * @retval None
  */
void ethernetif_set_mac_addr(const uint8_t *mac) {
  if(mac != NULL) {
    memcpy(macaddress,mac,6);
  }
}

#if LWIP_IGMP
err_t igmp_mac_filter( struct netif *netif, const ip4_addr_t *ip4_addr, netif_mac_filter_action action )
{
  uint8_t mac[6];
  const uint8_t *p = (const uint8_t *)ip4_addr;

  mac[0] = 0x01;
  mac[1] = 0x00;
  mac[2] = 0x5E;
  mac[3] = *(p+1) & 0x7F;
  mac[4] = *(p+2);
  mac[5] = *(p+3);

  register_multicast_address(mac);

  return 0;
}

#ifndef HASH_BITS
#define HASH_BITS 6 /* #bits in hash */
#endif

uint32_t ethcrc(const uint8_t *data, size_t length)
{
  uint32_t crc = 0xffffffff;
  size_t i;
  int j;

  for (i = 0; i < length; i++) {
    for (j = 0; j < 8; j++) {
      if (((crc >> 31) ^ (data[i] >> j)) & 0x01) {
        /* x^26+x^23+x^22+x^16+x^12+x^11+x^10+x^8+x^7+x^5+x^4+x^2+x+1 */
        crc = (crc << 1) ^ 0x04C11DB7;
      } else {
        crc = crc << 1;
      }
    } 
  }
  return ~crc;
}

void register_multicast_address(const uint8_t *mac)
{
  uint32_t crc;
  uint8_t hash;

  /* Calculate crc32 value of mac address */
  crc = ethcrc(mac, HASH_BITS);

  /* 
   * Only upper HASH_BITS are used
   * which point to specific bit in the hash registers
   */
  hash = (crc >> 26) & 0x3F;

	if (hash > 31) {
    ETH_HashTableHigh |= 1 << (hash - 32);
    EthHandle.Instance->MACHTHR = ETH_HashTableHigh;
  }	else {
    ETH_HashTableLow |= 1 << hash;
    EthHandle.Instance->MACHTLR =ETH_HashTableLow;
  }
}
#endif /* LWIP_IGMP */


#ifdef ETH_INPUT_USE_IT
/**
  * @brief  Ethernet Rx Transfer completed callback
  * @param  heth: ETH handle
  * @retval None
  */
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
  ethernetif_input(&gnetif);
}

/**
  * @brief  This function handles Ethernet interrupt request.
  * @param  None
  * @retval None
  */
void ETH_IRQHandler(void)
{
  HAL_ETH_IRQHandler(&EthHandle);
}
#endif /* ETH_INPUT_USE_IT */

#ifdef __cplusplus
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
