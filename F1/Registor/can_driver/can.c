#include "gpio.h"


uint8_t CANx_Init(CAN_TypeDef  *CANx , uint8_t psc, uint8_t sjw, uint8_t ts1, uint8_t ts2){
	
	uint16_t i=0;
	IRQn_Type IRQn;
	
	RCC->APB2ENR |=  1<<0;//AFIO
	if(CANx==CAN1) {IRQn=USB_LP_CAN1_RX0_IRQn; RCC->APB1ENR |= 1<<25;}// CAN1
	
	//if()
	GPIOx_Init( GPIOA,11  , INPUT_FLOATING, NOPULL, 0);// RX
	GPIOx_Init( GPIOA,12  , OUTPUT_AF_PP, NOPULL, MODE_OUTPUT_50MHZ);//TX
	
	CAN1->MCR=0x0000;		//(xoa 0)
	CAN1->MCR|=1<<0;		// yeu cau khoi tao CAN
	while((CAN1->MSR&1<<0)==0)
	{
		i++;
		if(i>100)return 2;	//khoi tao loi
	}
	CAN1->MCR|=0<<7;		//CAN_TTCM=DISABLE;	-TimeTriggeredMode
	CAN1->MCR|=0<<6;		//CAN_ABOM=ENABLE;-bus-off 
	CAN1->MCR|=0<<5;		//CAN_AWUM=DISABLE;- sleep/wake-up mode
	CAN1->MCR|=0<<4;		//CAN_NART=ENABLE;-retransmission-gui cho den khi thanh cong thi thoi
	CAN1->MCR|=0<<3;		//CAN_RFLM=DISABLE;	-ReceiveFifoLocked 
	CAN1->MCR|=0<<2;		//CAN_TXFP=DISABLE;	-Transmit FIFO priority
	
	CAN1->BTR=0x00000000;	//xoa
	CAN1->BTR|=(0<<31)|(0<<30);	// CAN normal,...;
	//Fpclk1/((Tbs1+Tbs2+1)*Fdiv)=>speed=36M/((SJW+BS1+BS2)*CAN_Prescaler)---500kbps=36M/((8+9+1)*4)

	CAN1->BTR|=(sjw-1)<<24; 	// tsjw+1
	CAN1->BTR|=(ts2-1)<<20; 	//Tbs2=tbs2+1
	CAN1->BTR|=(ts1-1)<<16;	//Tbs1=tbs1+1
	CAN1->BTR|=(psc-1)<<0;  	//(Fdiv)=brp+1
							
	CAN1->MCR&=~(1<<0);		//yeu cau tat che do khoi tao
	i = 0;
	while((CAN1->MSR&1<<0)==1)
	{
		i++;
		if(i>100)return 3;//loi
	}
	//filter
	
	CANx->FMR |= (1<<0);//Initialization filter 
	CANx->FA1R &=~(1<<0);
	
	CANx->FM1R |= 0<<0;  //32bit Identifier Mask mode
	CANx->FS1R |= 1<<0;   //CAN_FILTERSCALE_32BIT
	CANx->FFA1R =0; //CAN_RX_FIFO0
	CANx->sFilterRegister[0].FR1 = 0x00000000;// ID = CAN1 :0-13, CAN2: 14-28
	CANx->sFilterRegister[0].FR2 = 0x00000000;//MASk
	
	CANx->FA1R |=1<<0;	//EN Filter
	CANx->FMR &= ~(1<<0);//Initialization filter 
	
	CANx->IER |= 1<<1;//: FIFO message pending interrupt enable
	NVICx_Init(IRQn, 1, 1);

	return 0;
}

void CAN_Receive(CAN_TypeDef* CANx,uint32_t *idx, uint8_t *ide, uint8_t *rtr, uint8_t *dlc,  uint8_t FIFONumber, uint8_t * Data)
{
  /* Get the Id */
  *ide = (uint8_t)0x04 & CANx->sFIFOMailBox[FIFONumber].RIR;
  if (*ide == 0)//std
  {
    *idx = (uint32_t)0x000007FF & (CANx->sFIFOMailBox[FIFONumber].RIR >> 21);
  }
  else
  {
    *idx = (uint32_t)0x1FFFFFFF & (CANx->sFIFOMailBox[FIFONumber].RIR >> 3);
  }
  
  *rtr = (uint8_t)0x02 & CANx->sFIFOMailBox[FIFONumber].RIR;
  /* Get the DLC */
  *dlc = (uint8_t)0x0F & CANx->sFIFOMailBox[FIFONumber].RDTR;
  /* Get the FMI */
  //RxMessage->FMI = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONumber].RDTR >> 8);
  /* Get the data field */
  Data[0] = (uint8_t)0xFF & CANx->sFIFOMailBox[FIFONumber].RDLR;
  Data[1] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONumber].RDLR >> 8);
  Data[2] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONumber].RDLR >> 16);
  Data[3] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONumber].RDLR >> 24);
  Data[4] = (uint8_t)0xFF & CANx->sFIFOMailBox[FIFONumber].RDHR;
  Data[5] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONumber].RDHR >> 8);
  Data[6] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONumber].RDHR >> 16);
  Data[7] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONumber].RDHR >> 24);
  /* Release the FIFO */
  /* Release FIFO0 */
  if (FIFONumber == 0)
  {
    CANx->RF0R |= CAN_RF0R_RFOM0;
  }
  /* Release FIFO1 */
  else /* FIFONumber == CAN_FIFO1 */
  {
    CANx->RF1R |= CAN_RF1R_RFOM1;
  }
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
	uint32_t id;
	uint8_t ide=0;
	uint8_t rtr=0;
	uint8_t dlc=0;
	uint8_t rxbuf[8];
 // CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);
	CAN_Receive(CAN1,&id, &ide, &rtr, &dlc, 0 , rxbuf);
	
	  printf("id:%d\r\n",id);
    printf("ide:%d\r\n",ide);
    printf("rtr:%d\r\n",rtr);
    printf("len:%d\r\n",dlc);
    printf("rxbuf[0]:%d\r\n",rxbuf[0]);
    printf("rxbuf[1]:%d\r\n",rxbuf[1]);
		printf("rxbuf[2]:%d\r\n",rxbuf[2]);
    printf("rxbuf[3]:%d\r\n",rxbuf[3]);
    printf("rxbuf[4]:%d\r\n",rxbuf[4]);
    printf("rxbuf[5]:%d\r\n",rxbuf[5]);
    printf("rxbuf[6]:%d\r\n",rxbuf[6]);
    printf("rxbuf[7]:%d\r\n",rxbuf[7]);

 CAN1->RF0R &= ~(3<<0);

}

//uint32_t id
// uint8_t  ide khung truyen
// uint8_t rtr   khung data/dieu khien

uint8_t CANx_Send(CAN_TypeDef  *CANx ,uint32_t id, uint8_t  ide,  uint8_t rtr ,uint8_t dlc, uint8_t * Data){

	uint8_t mailbox=0;	// Id_type=0;
/* Select one empty transmit mailbox */
  if ((CANx->TSR&(1<<26)))  mailbox = 0;

  else if ((CANx->TSR&(1<<27))) mailbox = 1;
 
  else if ((CANx->TSR&(1<<28)))mailbox = 2;
 
  else return 0xFF;
	
	if(mailbox != 0xff){
	/* Set up the Id */
    CANx->sTxMailBox[mailbox].TIR &= 0x00000001;
	
    if (ide == 0)// std
    {
       id &=0x7ff;// 11bit
			
      CANx->sTxMailBox[mailbox].TIR |= ((id << 21) | (rtr<<1));//id(std)-Data frame
    }
    else
    {
			id &=0x1fffffff;// 29bit
      CANx->sTxMailBox[mailbox].TIR |= ((id << 3) |(ide<<2)| (rtr<<1));  //id-Extended identifier- Data frame                                             TxMessage->RTR);
    }
		
		 /* Set up the DLC */
    dlc &= (uint8_t)0x0000000F;
    CANx->sTxMailBox[mailbox].TDTR &= (uint32_t)0xFFFFFFF0;
    CANx->sTxMailBox[mailbox].TDTR |= dlc;

    /* Set up the data field */
		
		 CANx->sTxMailBox[mailbox].TDHR = (((uint32_t)Data[7] << 24) | 
                                             ((uint32_t)Data[6] << 16) |
                                             ((uint32_t)Data[5] << 8) |
                                             ((uint32_t)Data[4]));
		
    CANx->sTxMailBox[mailbox].TDLR = (((uint32_t)Data[3] << 24) | 
                                             ((uint32_t)Data[2] << 16) |
                                             ((uint32_t)Data[1] << 8) | 
                                             ((uint32_t)Data[0]));
   
    /* Request transmission */
    CANx->sTxMailBox[mailbox].TIR |= 0x00000001;
		
	 }
	return mailbox;
}

//0X05,cho xu ly/thatbai; 0X07,thanh cong.
uint8_t CAN_Tx_Staus(uint8_t mbox)
{	
	uint8_t sta=0;					    
	switch (mbox)
	{
		case 0: 
			sta |= CAN1->TSR&(1<<0);		//RQCP0
			sta |= CAN1->TSR&(1<<1);		//TXOK0
			sta |=((CAN1->TSR&(1<<26))>>24);//TME0
			break;
		case 1: 
			sta |= CAN1->TSR&(1<<8)>>8;		//RQCP1
			sta |= CAN1->TSR&(1<<9)>>8;		//TXOK1
			sta |=((CAN1->TSR&(1<<27))>>25);//TME1	   
			break;
		case 2: 
			sta |= CAN1->TSR&(1<<16)>>16;	//RQCP2
			sta |= CAN1->TSR&(1<<17)>>16;	//TXOK2
			sta |=((CAN1->TSR&(1<<28))>>26);//TME2
			break;
		default:
			sta=0X05;//
		break;
	}
	return sta;
} 

uint8_t CAN_Transmit(CAN_TypeDef* CANx, uint32_t id, uint8_t* TxMessage,uint8_t len){
uint8_t  mbox=0;
uint16_t i=0;	
	mbox=CANx_Send(CANx ,id, 0, 0 ,len, TxMessage);
	while((CAN_Tx_Staus(mbox)!=0x07)& (i<0xffff )) {i++;  if(i>=0xffff) return 1;}//loi
	return 0;//ok
}

















