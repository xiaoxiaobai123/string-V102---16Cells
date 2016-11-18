/**
  ******************************************************************************
  * @file        can.c
  * @author      Nystrom Engineering
  * @version     2.0.0
  * @date        Jan 27, 2014
  * @copyright   Powin Energy
  * @brief       This file provides all the CAN-Bus functions.
  * @page        can_page Controller Automation Network (CAN) Functions
  * @section     can_intro Introduction
  * @par
  *  The CAN functions are used to interface with the system Controller Automation Network.  \n
  *
  ******************************************************************************
  */
/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
#include <stdio.h> 
#include <stdlib.h>
#include "BESS_3-0_AHM_device_lib_V_1-0.h"
#include "BESS_3-0_AHM_can_V_1-0.h"
#include "BESS_3-0_AHM_main_V_1-0.h"
#include "BESS_3-0_AHM_systick_V_1-0.h"
//#include "BESS_3-0_AHM_i2c_eeprom_V_1-0.h"
#include "BESS_3-0_AHM_ad7327_V_1-0.h"
#include "BESS_3-0_AHM_canmsg_V_1-0.h"
#include "BESS_3-0_AHM_config_V_1-0.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "adc.h"
#include "contactor.h"
#include "led.h"
#include "messageid.h"
#include "i2c_ee.h"

#include "canupgrade.h"
#include "cancmd.h"

#include "NewStringADC.h"
#include "math.h"
#include "DataProcess.h"
#define DEBUG_RX_COMMAND 1

#ifdef STM32F10X_CL  //for WaveShare F107
    //#define USE_CAN_REMAP  1
#else
    //#define USE_CAN_REMAP  0
#endif

#ifdef STM32F10X_CL
    #define __CAN2_USED__       1
    #define CAN_SUPPORT_NUMBER  2
#else
    #define __CAN2_USED__       0
    #define CAN_SUPPORT_NUMBER  1

#endif

u8 OverChargeCount = 0;
u8 OVerDischargeCount = 0;
u8 OverChargeWarningCount = 0;
u8 OVerDischargeWarningCount = 0;

#define AUTO_SEND_EX_NUMBER 1

#define AUTO_SEND_TIMES 0xff

 

#define DEAFULT_TIMER   1  
 
uint16_t totalSendCounter = 0;
uint16_t totalSendCounter2 = 0;

uint16_t rxOverFolwCounter = 0;
uint16_t rxOverFolwCounter2 = 0;

uint8_t raw_req;      // can sets this to request raw sample

uint32_t CANTimer;
 
static volatile BUFFER can_rx_buffer[CAN_RX_BUFFER_LEN], can_rx_buffer2[CAN_RX_BUFFER_LEN]; /**< CAN Receive fifo data space*/

volatile CanFifo can_rx_fifo, can_rx_fifo2;  /*can_rx_fifo 为 bp-string的 can的接收fifo

											   can_rx_fifo2 为 string-array的 can 的接收FIFO*/

static volatile BUFFER can_tx_buffer[CAN_TX_BUFFER_LEN], can_tx_buffer2[CAN_TX_BUFFER_LEN]; /**< CAN Transmit Data Buffer */
volatile CanFifo can_tx_fifo, can_tx_fifo2;  /*can_tx_fifo 为 bp-string的 can的发送fifo
                                                   
											   can_tx_fifo2 为 string-array的 can 的发送FIFO*/


//static uint32_t Serial_Number;
static uint8_t device_id;
uint8_t suppress_can_tx = 0;
uint32_t can_tx_block_timer;

uint8_t Self_ID;
uint8_t BPCounter = 0;


volatile StringData mStringData;

MESSAGE AlarmMessage;
MESSAGE WarningMessage;
MESSAGE ErrorMessage;   

_StringStateFlag ToArray;
u8 MsgGroupID;                //add by jason，2015年8月24日12:40:27

/*add by jason，2016年2月24日13:03:23 for BMC_OFFLine */

u8 BMC_OFFLINE_AlarmSet_Flag[BP_NUMBER][CELL_NUMBER];     
u8 BMC_OFFLINE_AlarmClr_Flag[BP_NUMBER][CELL_NUMBER];

u8 BMC_OFFLINE_WarningSet_Flag[BP_NUMBER][CELL_NUMBER];    
u8 BMC_OFFLINE_WarningClr_Flag[BP_NUMBER][CELL_NUMBER];

u8 BMC_OFFLINE_AlarmSetAck_Flag[BP_NUMBER][CELL_NUMBER];     
u8 BMC_OFFLINE_AlarmClrAck_Flag[BP_NUMBER][CELL_NUMBER];

u8 BMC_OFFLINE_WarningSetAck_Flag[BP_NUMBER][CELL_NUMBER];    
u8 BMC_OFFLINE_WarningClrAck_Flag[BP_NUMBER][CELL_NUMBER];

u8 BMC_OFFLINE_AlarmReason[BP_NUMBER][CELL_NUMBER]; 
u8 BMC_OFFLINE_WarningReason[BP_NUMBER][CELL_NUMBER];

u8 BMC_OFFLINE_ErrorSet_Flag[BP_NUMBER][CELL_NUMBER];
u8 BMC_OFFLINE_ErrorClr_Flag[BP_NUMBER][CELL_NUMBER];

u8 BMC_OFFLINE_ErrorSetAck_Flag[BP_NUMBER][CELL_NUMBER];
u8 BMC_OFFLINE_ErrorClrAck_Flag[BP_NUMBER][CELL_NUMBER];

u8 BMC_OFFLINE_ErrorReason[BP_NUMBER][CELL_NUMBER];
 

u8 BMC_OFFLINE_AlarmCount;
u8 BMC_OFFLINE_WarningCount;

u8 MaintenanceModeFlag = 0;
u8 GetMaintencanceModeFlagFromSys = 0;

_BP BPbooterErrorbit;

uint32_t TargetValueCounter = 0;
u8 TargetValueCountTimeOutFlag = 0;
//------
static void cpMemoryData(void)  
{

    
}
//----

/*-----------------------------------------------------------------------------*/



 
uint8_t getSelfID(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint8_t deviceid = 0x00;
 
    /* Enable GPIOC clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
#if 1 
    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOC, &GPIO_InitStructure); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

#endif    
    
    
    deviceid = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)|
                (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)<<1)|
                (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)<<2)|
                (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)<<3);
    deviceid = ~deviceid;
    deviceid &= 0x0F;
    return deviceid;

}

/**
 * @fn       void make_filter(uint8_t fltnum, uint8_t dvcid)
 * @brief    Create a CAN filter based on device ID (bits 15:8 of the 29 bit ID)
 * @param    fltnum    hardware filter number, 0..13 in this device
 * @param    dvcid     device ID, bits 8..15 of 29-bit ID to filter on
 * @retval   None
 */

void make_filter(uint8_t fltnum, uint8_t dvctype)
{
    CAN_FilterInitTypeDef f_init;
    uint32_t tmp;

    // fill in the fields which don't change among all the filters we'll set:
    f_init.CAN_FilterMode = CAN_FilterMode_IdMask;
    f_init.CAN_FilterScale = CAN_FilterScale_32bit;
    f_init.CAN_FilterFIFOAssignment = 0;
    // we always filter on the same fields. We specify them using the macros
    // which build up the 29-bit ID field and putting '1' in the bit locations
    // we wish to filter. In other words, only the bits which contain '1' will be
    // looked at by the filter - where there are 0, any data is accepted.
    // We shift the value left 3 bits to align the resulting 29 bit ID in the left
    // of the field which the filter needs.
	if(dvctype == RELAY_TYPE)
	{
		//            rsvd      stringid    msgtype  dvctype  dvcid   msgid
		tmp = MAKE_ID(0x0,     0,    0,       0x0f,    0,      0xFF   ) << 3;
	}
	else
	{
		tmp = MAKE_ID(0x0,     0,    0,       0x0f,    0,      0   ) << 3;
	}
//    tmp = 0;        // don't give a fuck about filtering, just want everything right now
    f_init.CAN_FilterMaskIdHigh = tmp >> 16;
    // We OR in CAN_TI0R_IDE to filter out 11-bit frames.  We OR in CAN_T10R_RTR
    // to filter out any but data frames.
    f_init.CAN_FilterMaskIdLow = (tmp & 0xffff) | CAN_TI0R_IDE | CAN_TI0R_RTR;

    f_init.CAN_FilterNumber = fltnum;

    // build up the 'do care' fields with specific values to filter on
	if(dvctype == RELAY_TYPE)
	{
		tmp = MAKE_ID(ReserveID, 0, 0, dvctype, 0, 0x10) << 3;
    }
	else
	{
		tmp = MAKE_ID(ReserveID, 0, 0, dvctype, 0, 0) << 3;
	}

    f_init.CAN_FilterIdHigh = tmp >> 16;  // set the values of the upper 'do care' bits
    f_init.CAN_FilterIdLow = (tmp & 0xffff) | CAN_TI0R_IDE; // set the values of the lower 'do care' bits,
    // and insist upon 29-bit frames
    f_init.CAN_FilterActivation=ENABLE; // Turn on this filter, please
    CAN_FilterInit(&f_init);
}

/**
 * @fn        void CAN_Device_Init(void)
 * @brief    Initialize the CAN device hardware.
 * @param    None
 * @retval    None
 */
void CAN_Device_Init(RCC_ClocksTypeDef *clk)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_ClocksTypeDef clks;
    uint8_t m[4] = {0};
    //I2C_EE_BufferRead(m, 32, 4);
   // I2C_EE_BufferRead(m, 31, 1);
    device_id = m[0];
    

    //srand(getInitSeed());
	Self_ID = getSelfID();
  
    device_id = Self_ID ;
 
    RCC_GetClocksFreq(&clks);

    memset((void *)&mStringData, 0x0, sizeof(StringData));

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    /* Initialize the ResetClockController to enable the CAN clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); /**< CAN Peripheral clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);/**< AFIO Peripheral clock enable*/
    
#if(__CAN2_USED__) 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE); /**< CAN Peripheral clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
#endif  /* __CAN2_USED__ */
    

    /* Configure CAN pin: RX */

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure CAN pin: TX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

 //   GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);


#if(__CAN2_USED__)    
    /* Configure CAN pin: RX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
  
    /* Configure CAN pin: TX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
  
//    GPIO_PinRemapConfig(GPIO_Remap_CAN2 , ENABLE);
#endif
 
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   
 
 
#ifndef STM32F10X_CL
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    #if(__CAN2_USED__) 
        /* CAN2 is not implemented in the device */
        #error "CAN2 is implemented only in Connectivity line devices"
    #endif /*__CAN1_USED__*/
#else
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    #if(__CAN2_USED__) 
    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    #endif /*__CAN1_USED__*/

#endif /* STM32F10X_CL*/
    
  

    /* De-Initialize to default CAN settings */
    CAN_DeInit(CAN1);
    #if(__CAN2_USED__) 
    CAN_DeInit(CAN2);
    #endif
    
    CAN_StructInit(&CAN_InitStructure);
    #if(1)
    /* CAN cell init */
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;//DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = ENABLE;//DISABLE;
    CAN_InitStructure.CAN_RFLM = ENABLE;//DISABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE;//DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    #else
    CAN_InitStructure.CAN_TTCM=DISABLE; // disable time-triggered communication mode.
    CAN_InitStructure.CAN_ABOM=ENABLE;  // enable automatic bus-off
    CAN_InitStructure.CAN_AWUM=DISABLE; // disable automatic wake-up mode.
    CAN_InitStructure.CAN_NART=DISABLE; // disable the no-automatic retransmission mode (enable auto retransmission)
    CAN_InitStructure.CAN_RFLM=ENABLE;  // enable Receive FIFO Locked mode.
    CAN_InitStructure.CAN_TXFP=DISABLE; // disable transmit FIFO priority.
#ifdef CAN_DEBUG_LOOPBACK
    CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
#else
    CAN_InitStructure.CAN_Mode=CAN_Mode_Normal; /*!< Specifies the CAN operating mode. This parameter can be a value of @ref CAN_operating_mode */
#endif
    #endif


#if(0)//by sam
    /* CAN Baudrate = 1MBps*/
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
    CAN_InitStructure.CAN_Prescaler = 4;
#else
    CAN_InitStructure.CAN_SJW=CAN_SJW_4tq;  // limit sync adjustment to 1/4 total bit period
    CAN_InitStructure.CAN_BS1=CAN_BS1_10tq; // 10 bits in segment 1
    CAN_InitStructure.CAN_BS2=CAN_BS2_5tq;  // 5 bits in segment 2
    // calculate prescalar based on present clock values, number of quanta in the
    // bit, and desired CAN baud rate. Note this value gets decremented by the init
    // call, so don't subract 1 here.
    CAN_InitStructure.CAN_Prescaler = clk->PCLK1_Frequency / (16 * CAN_BUS_SPEED); // 16 is # quanta in one bit, set just above here
    
    printf("<<  clk->PCLK1_Frequency = %d, CAN_InitStructure.CAN_Prescaler = %d  >>\r\n", clk->PCLK1_Frequency, CAN_InitStructure.CAN_Prescaler);
    //#define CAN_BAUDRATE 125000
    //CAN_InitStructure.CAN_Prescaler = clks.PCLK1_Frequency / (16 * CAN_BAUDRATE); // 16 is # quanta in one bit, set just above here.  Formula double checked. It's right ABY 7/3/2015
#endif


    CAN_Init(CAN1, &CAN_InitStructure);
#if(__CAN2_USED__) 
    CAN_Init(CAN2, &CAN_InitStructure);
#endif


    #if(1)
	
    make_filter(0, BP_TYPE);           /*0-13为CAN1的filter */
    make_filter(14, ARRAY_TYPE);      /*14往上的filter为CAN2的*/
	make_filter(15, RELAY_TYPE);
    #else
    {
        CAN_FilterInitTypeDef  CAN_FilterInitStructure;
        CAN_FilterInitStructure.CAN_FilterNumber=0;

        CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
        CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
        CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
        CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
        CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
        CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
        CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
        CAN_FilterInit(&CAN_FilterInitStructure);
        
        #if(__CAN2_USED__) 
            /* CAN2 filter init */
        CAN_FilterInitStructure.CAN_FilterNumber = 14;
        CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
        CAN_FilterInit(&CAN_FilterInitStructure);
        #endif
    }
    #endif

    // turn on rx interrupts from FIFO 0
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    #if(__CAN2_USED__) 
    CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
    #endif
}

//******************************************************************************
// Fifo functions.
// Both rx and tx fifos are handled by the same structure, CanFifo.
// All operations on that structure are defined here.
//******************************************************************************
void fifo_init(volatile CanFifo *f, volatile BUFFER *buffer, uint16_t buffer_len)
{
    memset((void *)f, 0, sizeof(*f));                     // set head, tail, count, sz, and data to 0
    f->sz = buffer_len;                           // set length of buffer into fifo
    f->data = buffer;                            // point fifo structure at item storage
}

// return oldest item in queue, adjusting tail and count to remove the item.
// return 0 on empty queue.
BUFFER volatile *fifo_get(volatile CanFifo *f)
{
    BUFFER volatile *ret;

    if (f->count) {
        ret = f->data + f->tail++;   // &(f->data[f->tail]); tail += 1;
        if (f->tail >= f->sz)
            f->tail = 0;
        f->count--;
        return ret;
    }
    return 0;
}

// allocate a new item in the queue, adjust head and count to insert it,
// and return a pointer to the allocated item.
// Return 0 on full queue.
BUFFER volatile *fifo_alloc(volatile CanFifo *f)
{
    BUFFER volatile *ret;

    if (f->count >= f->sz)
        return 0;
    ret = f->data + f->head++;		// &(f->data[f->head]); head += 1;
    if (f->head >= f->sz)
        f->head = 0;
    f->count++;
    return ret;
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn        void CAN_Data_Init(void)
 * @brief    Initialize CAN data, including BUFFER and MESSAGE.
 * @param    None
 * @retval    None
 */
void CAN_Data_Init(void)
{
    fifo_init(&can_rx_fifo, can_rx_buffer, ASIZE(can_rx_buffer));
    fifo_init(&can_tx_fifo, can_tx_buffer, ASIZE(can_tx_buffer));
#if(__CAN2_USED__) 
    fifo_init(&can_rx_fifo2, can_rx_buffer2, ASIZE(can_rx_buffer2));
    fifo_init(&can_tx_fifo2, can_tx_buffer2, ASIZE(can_tx_buffer2));
#endif
}
/*-----------------------------------------------------------------------------*/
/**
 * @fn        void CAN_Initialize(void)
 * @brief    Initialize CAN device and data structures.
 * @param    None
 * @retval    None
 */
void CAN_Initialize(RCC_ClocksTypeDef *clk)
{
    CAN_Device_Init(clk);
    CAN_Data_Init();
    //prepareDataBuffer();
}

// Returns next available can buffer index or 0xff for none available
uint8_t can_buf_avail(void)
{
    uint32_t tsr = CAN1->TSR;

    if (tsr & CAN_TSR_TME0)
        return 0;
    if (tsr & CAN_TSR_TME1)
        return 1;
    if (tsr & CAN_TSR_TME2)
        return 2;
    return 0xff;               // no available buffer found
}
uint8_t can_buf_avail2(void)
{
    uint32_t tsr = CAN2->TSR;

    if (tsr & CAN_TSR_TME0)
        return 0;
    if (tsr & CAN_TSR_TME1)
        return 1;
    if (tsr & CAN_TSR_TME2)
        return 2;
    return 0xff;               // no available buffer found
}
uint16_t measureStringVoltage(void) /*获取 测量到的所有包的总电压*/
{
    #if(1)
    uint16_t tempVoltage = ((float)adc_getresult(ADC_CHANNEL_VAMP) /(float)3726) * 1000;
    #else
    uint16_t tempVoltage = adc_getresult(ADC_CHANNEL_VAMP);
    #endif
    printf("<<  -------=====  measureStringVoltage (%d) =====-------  >>\r\n", tempVoltage);
    return tempVoltage;
}
uint16_t StringCurrentADValue = 0;
int16_t getStringCurrent(void)  /*获取总线电流*/
{
//    #if(1)
//    int16_t currentTemp = 0;
//    uint16_t tempAdc = adc_getresult(ADC_CHANNEL_IAMP1);

//    //int16_t shiftAdc = tempAdc - 1861;
//    //currentTemp = shiftAdc *((float)30000/(float)1861);    
//    int16_t shiftAdc = tempAdc - 2048;
//		StringCurrentADValue = tempAdc;
//    //currentTemp = shiftAdc*((float)30000/(float)2048);  
////    currentTemp = (float)shiftAdc/(float)2048 * 30000 * 4;  
///*Add by jason ,2015年9月16日10:42:54*/	
////	currentTemp = ((float)(200000 / 3726) * tempAdc - 100000) / 4;
///*Add by jason ,2015年9月22日13:19:56，by 3V power*/	
//    currentTemp = (float)(200000 / 4096) * tempAdc - 100000 ;
//	if((currentTemp < 2000) && (currentTemp > -2000))
//        currentTemp = 0;
//    #else
//    int16_t currentTemp = adc_getresult(ADC_CHANNEL_IAMP1);
//    #endif
//    //printf("<<  -------=====  getStringCurrent (%d) =====-------  >>\r\n", currentTemp);
//    return currentTemp;
	return 0;
}

 
uint8_t getPostitiveContactor(void)
{
    //printf("<<  -------=====  getPostitiveContactor  =====-------  >>\r\n");
    //return 88;
    if(is_contactor_1_closed())
    {
        printf("<<  -------=====  getPostitiveContactor (close:0xff) =====-------  >>\r\n");
        return 0xff;
    }
    else
    {
        printf("<<  -------=====  getPostitiveContactor (open:0x0x00) =====-------  >>\r\n");
        return 0x0;
    }
} 
 uint8_t getPostitiveContactor2(void)
{
    //printf("<<  -------=====  getPostitiveContactor  =====-------  >>\r\n");
    //return 88;
    if(is_contactor_2_closed())
    {
        printf("<<  -------=====  getPostitiveContactor2 (close:0xff) =====-------  >>\r\n");
        return 0xff;
    }
    else
    {
        printf("<<  -------=====  getPostitiveContactor2 (open:0x0x00) =====-------  >>\r\n");
        return 0x0;
    }
}    
 
static void getStringHLACellVoltage(void)/*获取所有电池的最大 最小 平均 电压*/
{
    int i;
    uint32_t tmpValue;
    uint32_t utmpValue;
    mStringData.highCellVolt = 0;
    for(i = 0; i < BP_NUMBER; i++)
    {
		if(mStringData.bp[i].highCellVolt != 0)
		{
			if( BpLossCommunicationWarningFlag[i] == 0) /*bp not lose communication*/
			{
				if(mStringData.bp[i].highCellVolt > mStringData.highCellVolt)
				{
					mStringData.highCellVolt = mStringData.bp[i].highCellVolt;					
				}
			}
		}
    }
	
 
    
    mStringData.lowCellVolt = mStringData.bp[0].lowCellVolt;
    for(i = 1; i<BP_NUMBER; i++)
    {
        if(mStringData.bp[i].lowCellVolt != 10000)
		{
			if(BpLossCommunicationWarningFlag[i] == 0)
			{
				if(mStringData.bp[i].lowCellVolt < mStringData.lowCellVolt)
				{
					mStringData.lowCellVolt = mStringData.bp[i].lowCellVolt;
				}
			}
		}
    }
    
 
    //
    
#if 0   
    tmpValue = 0;
    for(i = 0; i<BP_NUMBER; i++)
    {
        tmpValue = tmpValue + mStringData.bp[i].averageCellVolt;
    }       
    mStringData.averageCellVolt = tmpValue / BP_NUMBER;
#endif
    //check
    {
        int m, n;
        tmpValue = 0;
        for(m = 0; m<BP_NUMBER; m++)
        {
            for(n = 0; n<CELL_NUMBER; n++)
            {
                tmpValue = tmpValue + mStringData.bp[m].cell[n].volt;//mStringData.bp[m].cell[n].volt;
            }                
        }
        tmpValue = tmpValue /(BP_NUMBER * CELL_NUMBER);
        printf("<< check >>  get String A Cell Voltage: %d vs %d \r\n", mStringData.averageCellVolt, tmpValue);
    }
    //
    
    mStringData.calculateStringVoltage = 0;
    utmpValue = 0;
    for(i = 0; i<BP_NUMBER; i++)
    {
        utmpValue = utmpValue + (uint32_t)(mStringData.bp[i].averageCellVolt)*(uint32_t)CELL_NUMBER;        
        //printf("<<%d>>  : %d vs %d \r\n", i, mStringData.calculateStringVoltage, utmpValue);
    }
    mStringData.calculateStringVoltage = utmpValue/*mStringData.calculateStringVoltage*/ /100;     
    
    //check
    {
        int m, n;
        utmpValue = 0;
        for(m = 0; m<BP_NUMBER; m++)
        {
            for(n = 0; n<CELL_NUMBER; n++)
            {
                utmpValue = utmpValue + mStringData.bp[m].cell[n].volt;//mStringData.bp[m].cell[n].volt;
                //printf("<<%d, %d>>  : %d vs %d \r\n", m, n, utmpValue, mStringData.bp[m].cell[n].volt);
            }                
        }
        utmpValue = utmpValue / 100;
        printf("<< check >>  get calculateStringVoltage: %d vs %d \r\n", mStringData.calculateStringVoltage, utmpValue);
    }
    //
    //mStringData.calculateStringVoltage = mStringData.calculateStringVoltage /100;     
    //#warning -> need re get measureStringVoltage...
 //   mStringData.measureStringVoltage = GetStringMeasuredVoltage();//mStringData.calculateStringVoltage;
	
#if 0
/*the initial value ,not very accuracy*/	
    {
		static u8 SimpleAHOnce = 0;
		if(SimpleAHOnce <= 20 && mStringData.averageCellVolt != 0)
		{
#if 0
			SimpleAHOnce++;
//			mStringData.ah = 7 * mStringData.highCellVolt - 16590;
// 		 	mStringData.kWh =  mStringData.ah * mStringData.measureStringVoltage * 10;
////			 mStringData.kWh = 33 * mStringData.highCellVolt - 81818;
//			mStringData.soc = mStringData.ah / 73;
//			mStringData.averageCellVolt
			if(mStringData.averageCellVolt < 2500)
			{
				mStringData.soc = 0;
				mStringData.ah = mStringData.soc * TotalAH / 100;
				mStringData.kWh = mStringData.ah *mStringData.averageCellVolt/ 1000 * BP_NUMBER * 12 / 100;
			}
			else if(mStringData.averageCellVolt >= 2500 && mStringData.averageCellVolt <2800)
			{
				mStringData.soc = 10;
				mStringData.ah = mStringData.soc * TotalAH / 100;
				mStringData.kWh = mStringData.ah *mStringData.averageCellVolt/ 1000 * BP_NUMBER * 12 / 100;				
				
			}
			else if(mStringData.averageCellVolt >= 2800 && mStringData.averageCellVolt <3100)
			{
				mStringData.soc = 30;
				mStringData.ah = mStringData.soc * TotalAH / 100;
				mStringData.kWh = mStringData.ah *mStringData.averageCellVolt/ 1000 * BP_NUMBER * 12 / 100;			
			}
			else if(mStringData.averageCellVolt >= 3100 && mStringData.averageCellVolt <3200)
			{
				mStringData.soc = 40;
				mStringData.ah = mStringData.soc * TotalAH / 100;
				mStringData.kWh = mStringData.ah *mStringData.averageCellVolt/ 1000 * BP_NUMBER * 12 / 100;				
			}
			if(mStringData.averageCellVolt >= 3200 && mStringData.averageCellVolt< 3250)
			{
				mStringData.soc = 60;
				mStringData.ah = mStringData.soc * TotalAH / 100;
				mStringData.kWh = mStringData.ah *mStringData.averageCellVolt/ 1000 * BP_NUMBER * 12 / 100;
			}
			else if(mStringData.averageCellVolt > 3250 && mStringData.averageCellVolt<= 3300)
			{
				mStringData.soc = 70;
				mStringData.ah = mStringData.soc * TotalAH / 100;
				mStringData.kWh = mStringData.ah *mStringData.averageCellVolt/1000 * BP_NUMBER * 12 / 100;
			}
			else if(mStringData.averageCellVolt > 3300 && mStringData.averageCellVolt<= 3400)
			{
				mStringData.soc = 80;
				mStringData.ah = mStringData.soc * TotalAH / 100;
				mStringData.kWh = mStringData.ah *mStringData.averageCellVolt/1000 * BP_NUMBER * 12 / 100;				
			}
			else if(mStringData.averageCellVolt > 3400 && mStringData.averageCellVolt<= 3600)
			{
				mStringData.soc = 85;
				mStringData.ah = mStringData.soc * TotalAH / 100;
				mStringData.kWh = mStringData.ah *mStringData.averageCellVolt/1000 * BP_NUMBER * 12 / 100;					
			}
			else if(mStringData.averageCellVolt > 3600)
			{
				mStringData.soc = 100;
				mStringData.ah = mStringData.soc * TotalAH / 100;
				mStringData.kWh = mStringData.ah *mStringData.averageCellVolt/1000 * BP_NUMBER * 12 / 100;					
			}
#endif			
		}

				mStringData.soc = 100;
				mStringData.ah =  7300;
				mStringData.kWh =  34000;				
			CopyAHKWHToEEPROMTemp(); 
			StringEEProm_Write(AHKWHEEPageNumber);			
		
	}  
#endif	
}

//static void getStringHLACellTemp(void)
//{
//    int i;
//    int32_t tmpValue;
//    mStringData.highCellTemp = 0;
//    for(i = 0; i<BP_NUMBER; i++)
//    {
//        if(mStringData.bp[i].highCellTemp > mStringData.highCellTemp)
//        {
//            mStringData.highCellTemp = mStringData.bp[i].highCellTemp;
//        }
//    }
// 
//    
//    mStringData.lowCellTemp = mStringData.bp[0].lowCellTemp;
//    for(i = 0; i<BP_NUMBER; i++)
//    {
//        if(mStringData.bp[i].lowCellTemp < mStringData.lowCellTemp)
//        {
//            mStringData.lowCellTemp = mStringData.bp[i].lowCellTemp;
//        }
//    }
//    
//    //check
// 
//    //
//    mStringData.averageCellTemp = 0;
//    for(i = 0; i<BP_NUMBER; i++)
//    {
//        tmpValue = tmpValue + mStringData.bp[i].averageCellTemp;
//    }
//    mStringData.averageCellTemp = tmpValue / BP_NUMBER;   
//    
// 
//    
//    //
//    mStringData.maxCellTempRise = 0;
//    for(i = 0; i<BP_NUMBER; i++)
//    {
//        if(mStringData.bp[i].maxCellTempRise > mStringData.maxCellTempRise)
//        {
//            mStringData.maxCellTempRise = mStringData.bp[i].maxCellTempRise;
//        }
//    }
//    
//}

#define CHECK_CURRENT_RANGE_ADC 1850
#define CHECK_CURRENT_RANGE 30000  //30A

 
/*-----------------------------------------------------------------------------*/
void checkCurrent(void)
{
    
    int16_t currentTmp = ReadStringCurrent(); /*1 路*/
//	int16_t currentTmp1 = GetDCBusCurrent();
 
    mStringData.stringCurrent = currentTmp + 50;  /*500 ma为修正值*/
 
 
 
      
}
/*String sent to bp*/
void send_autodata_Ex_2_BP(uint8_t idx, uint8_t bpIndex) // MsgID,BPID
{
    uint8_t buf[8] = {0};
    uint8_t msg_id = 0x0;
    uint8_t dataLen = 0;     
//    uint8_t bpFlag = 0xff;
    memset(buf, 0x0, 8) ;
    
	buf[0] = 0;	

    msg_id = idx;
    dataLen = 0;
/*------------add by jason,2015年9月1---------------------*/
	if(idx == MsgID_BpRequestArrayStringID)  /*0x22,BP来询问 string 和 array 的ID*/
	{
		Self_ID = device_id;
		buf[0] = ArrayStringID(mStringData.ArrayID,Self_ID);
//		buf[0] = 0x11;
		dataLen = 1;
		//bpIndex = 0x00;
	}
	if(idx == 0x20)                      /*BP  来询问string的相关信息，主要是电流，通过电流判断是否报警*/
	{
		dataLen = 8;
		buf[0] = 0x00;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = mStringData.stringCurrent >> 8;
		buf[5] = mStringData.stringCurrent;
		buf[6] = mStringData.ah >> 8;                                 /*应该是 校准AH*/
		buf[7] = mStringData.ah;		
	}
	if(idx == MsgID_SystemBPSelect)  /*发送选中BPID的消息给BP*/
	{
		buf[0] = 0x11;                 /*选中的ArrayID*/
		buf[1] = 0x00;                 /*选中的StringID*/
		buf[2] = 0x00;                 /*选择的BPID*/
		dataLen = 1;
	}
 
	MsgGroupID = ID_Data;
/*----------------------------------------------------------*/
    CAN_make_send_bypass_2_BP( buf, dataLen, msg_id, bpIndex);

}

 void send_autodata_Ex(uint8_t idx)  /*string 发给 array*/
{
    uint8_t buf[8] = {0};
    uint8_t msg_id = 0x0;
    uint8_t dataLen = 0;
//    int16_t s16Value;//, shiftValue;
//    uint16_t u16Value;
//    uint32_t u32Value;
     
    uint8_t bpFlag = 0xff;

    //printf("\r\n<< send_autodata_Ex >> idx = %d\n", idx);
    memset(buf, 0x0, 8);
    switch (idx) 
    {
      
        case 2://String Cell Voltage Message
            
            getStringHLACellVoltage();  
            
            buf[0] = (mStringData.highCellVolt>>8)&0xff;			
            buf[1] = mStringData.highCellVolt&0xff;	
			if(mStringData.lowCellVolt == 10000)
			{
				buf[2] = 0;					
				buf[3] = 0;				
			}
			else
			{
				
				buf[2] = (mStringData.lowCellVolt>>8)&0xff;					
				buf[3] = mStringData.lowCellVolt&0xff;
			}

            buf[4] = (mStringData.averageCellVolt>>8)&0xff;					
            buf[5] = mStringData.averageCellVolt&0xff;
        
            buf[6] = (mStringData.measureStringVoltage>>8)&0xff;  					
            buf[7] =  mStringData.measureStringVoltage&0xff;     
        
            msg_id = STRING_CELL_VOLTAGE_CMD_ID;  //0x52
            dataLen = 8;
            break;
        case 3://String Cell Temp Message
         //   getStringHLACellTemp(); 
       
            buf[0] = (mStringData.highCellTemp>>8)&0xff;			
            buf[1] = mStringData.highCellTemp&0xff;
			
			if(mStringData.lowCellTemp == 2500)
			{
				buf[2] = 0;
				buf[3] = 0;
			}
			else
			{
				
				buf[2] = (mStringData.lowCellTemp>>8)&0xff;			
				buf[3] = mStringData.lowCellTemp&0xff;
			}

            buf[4] = (mStringData.averageCellTemp>>8)&0xff;			
            buf[5] = mStringData.averageCellTemp&0xff;
        
            buf[6] = (mStringData.maxCellTempRise>>8)&0xff;			
            buf[7] = mStringData.maxCellTempRise&0xff;
        
        
            msg_id = STRING_CELL_TEMP_CMD_ID;  //0x53
            dataLen = 8;
            break;
        case 4: //pos/net contactor, String current, AH, StringSoc
        {
              
            mStringData.positiveContactor = getPostitiveContactor();
            mStringData.negativeContactor = getPostitiveContactor2();//mStringData.positiveContactor;
		
		    if(fabs(mStringData.stringCurrent) < 100 )            /*1A 以下的电流 算为0认为是干扰*/
			{
				buf[0] = 0;
				buf[1] = 0;
			}
			else
			{
				buf[0] = (mStringData.stringCurrent >>8) & 0xff;			
				buf[1] = mStringData.stringCurrent & 0xff;
			}
            buf[2] = (mStringData.ah >> 8) & 0xff;			
            buf[3] = mStringData.ah & 0xff;
        
            buf[4] = (mStringData.soc>>8)&0xff;		
            buf[5] = mStringData.soc& 0xff;	
        
        	buf[6] = mStringData.positiveContactor&0xff;			
            buf[7] = mStringData.negativeContactor&0xff;
            
			msg_id = 0x51;
            dataLen = 8;
        
            break;
        }

        
        case 5:  //kW,kWH
//            mStringData.kW = getDCPower();        
//            mStringData.kWh = getWaltH();   

			
            buf[0] = ((mStringData.kW / 1000) >> 8) & 0xff;			
            buf[1] = (mStringData.kW / 1000) & 0xff;

		
			buf[2] = ((mStringData.kWh/ 1000) >> 8) & 0xff;			
            buf[3] = (mStringData.kWh / 1000) & 0xff;
        
			buf[4] = ContactorClosedPermissionFlag;   //add by jason， 返回 contactor的实际permission 状态给array
		
			buf[5] = (mStringData.maxCellDeltaT >> 8) & 0xff;
			buf[6] = mStringData.maxCellDeltaT;
            msg_id = 0x54;  
            dataLen = 7;                  /*change by jason, use 2 bytes of kwh ，2016年6月1日9:40:31*/
            break;
        
        case 6:  //均衡时间
 
          
            buf[0] = (mStringData.averageBalanceCharger>>24)&0xff;			
            buf[1] = (mStringData.averageBalanceCharger>>16)&0xff;

            buf[2] = (mStringData.averageBalanceCharger>>8)&0xff;		
            buf[3] = mStringData.averageBalanceCharger&0xff;
        
           
        
            buf[4] = (mStringData.groundLeakge>>8)&0xff;		
            buf[5] = (mStringData.groundLeakge)&0xff;
        
            msg_id = 0x55;
            dataLen = 6;
            break;
        
        case 7:  /*Battery String Usage History*/
          
    
          
            buf[0] = (mStringData.totalChargekWh>>8)&0xff;			
            buf[1] = mStringData.totalChargekWh&0xff;

            buf[2] = (mStringData.totalDischargekWh>>8)&0xff;			
            buf[3] = mStringData.totalDischargekWh&0xff;        
            msg_id = 0x56;
            dataLen = 4;
            break;        

        case 8:
            

            buf[0] = (mStringData.bp[BPCounter].balanceChargerOnTime>>24)&0xff;			
            buf[1] = (mStringData.bp[BPCounter].balanceChargerOnTime>>16)&0xff;
        
            buf[2] = (mStringData.bp[BPCounter].balanceChargerOnTime>>8)&0xff;		
            buf[3] = mStringData.bp[BPCounter].balanceChargerOnTime&0xff;
        
           
        
            msg_id = 0x45;
            dataLen = 4;
            bpFlag = BPCounter;
            break; 
        ////////////////////////////////////////////////
        case 9://String BP Voltage Message
           
            
            //shiftVoltageValue(&mStringData.bp[BPCounter].highCellVolt);
            //shiftVoltageValue(&mStringData.bp[BPCounter].lowCellVolt);
            //shiftVoltageValue(&mStringData.bp[BPCounter].averageCellVolt);
            //shiftBPVoltageValue(&stringBPVolt[BPCounter]);
        
            buf[0] = (mStringData.bp[BPCounter].highCellVolt>>8)&0xff;					
            buf[1] = mStringData.bp[BPCounter].highCellVolt&0xff;	        
            
            buf[2] = (mStringData.bp[BPCounter].lowCellVolt>>8)&0xff;					
            buf[3] = mStringData.bp[BPCounter].lowCellVolt&0xff;
           
            buf[4] = (mStringData.bp[BPCounter].averageCellVolt>>8)&0xff;					
            buf[5] = mStringData.bp[BPCounter].averageCellVolt&0xff;
        
            buf[6] = (mStringData.bp[BPCounter].measureBPVoltage>>8)&0xff;				
            buf[7] = mStringData.bp[BPCounter].measureBPVoltage&0xff;	
        
            msg_id = 0x42; 
            dataLen = 8;
            bpFlag = BPCounter;
            break;
        
        case 10://String BP Temp Message
       

            //shiftTempValue(&mStringData.bp[BPCounter].highCellTemp);
            //shiftTempValue(&mStringData.bp[BPCounter].lowCellTemp);
            //shiftTempValue(&mStringData.bp[BPCounter].averageCellTemp);
            //shiftTempValue(&stringBPTemp[BPCounter]);
        
        
            buf[0] = (mStringData.bp[BPCounter].highCellTemp>>8)&0xff;			
            buf[1] = mStringData.bp[BPCounter].highCellTemp&0xff;
        
            buf[2] = (mStringData.bp[BPCounter].lowCellTemp>>8)&0xff;			
            buf[3] = mStringData.bp[BPCounter].lowCellTemp&0xff;

            buf[4] = (mStringData.bp[BPCounter].averageCellTemp>>8)&0xff;		
            buf[5] = mStringData.bp[BPCounter].averageCellTemp&0xff;	
        
            buf[6] = (mStringData.bp[BPCounter].maxCellTempRise>>8)&0xff;			
            buf[7] = mStringData.bp[BPCounter].maxCellTempRise&0xff;
        
        
            msg_id = 0x43; 
            dataLen = 8;
            bpFlag = BPCounter;
            break;
        case 11://resistor
            /*
                Byte 1 (Cells 9-12) Shunt Resistors ?On(1)/Off(0)
                Byte 2 (Cells 1-8) Shunt Resistors ?On(1)/Off(0)
                Byte 3: Balance Charger - On(FF)/Off(00)
                Bytes 4-7: Balancing Charger On Time

                */
            buf[0] =    ((mStringData.bp[BPCounter].cell[8].resistor)&0x01)        | 
                        ((mStringData.bp[BPCounter].cell[9].resistor)&0x01)<<1     |
                        ((mStringData.bp[BPCounter].cell[10].resistor)&0x01)<<2    |
                        ((mStringData.bp[BPCounter].cell[11].resistor)&0x01)<<3    |
						((mStringData.bp[BPCounter].cell[12].resistor)&0x01)<<4	   |
						((mStringData.bp[BPCounter].cell[13].resistor)&0x01)<<5    |
						((mStringData.bp[BPCounter].cell[14].resistor)&0x01)<<6    |
						((mStringData.bp[BPCounter].cell[15].resistor)&0x01)<<7;	
										;			
            buf[1] =    ((mStringData.bp[BPCounter].cell[0].resistor)&0x01)       | 
                        ((mStringData.bp[BPCounter].cell[1].resistor)&0x01)<<1    |
                        ((mStringData.bp[BPCounter].cell[2].resistor)&0x01)<<2    |
                        ((mStringData.bp[BPCounter].cell[3].resistor)&0x01)<<3    |
                        ((mStringData.bp[BPCounter].cell[4].resistor)&0x01)<<4    | 
                        ((mStringData.bp[BPCounter].cell[5].resistor)&0x01)<<5    |
                        ((mStringData.bp[BPCounter].cell[6].resistor)&0x01)<<6    |
                        ((mStringData.bp[BPCounter].cell[7].resistor)&0x01)<<7     ;                    
        
            if(mStringData.bp[BPCounter].balanceChargerStatus == 0)
            {
                buf[2] = 0x00;	
            }
            else
            {
                buf[2] = 0xff;	
            }
            		           
            buf[3] = (mStringData.bp[BPCounter].balanceChargerOnTime>>24)&0xff;			
            buf[4] = (mStringData.bp[BPCounter].balanceChargerOnTime>>16)&0xff;	
        
            buf[5] = (mStringData.bp[BPCounter].balanceChargerOnTime>>8)&0xff;	
            buf[6] = mStringData.bp[BPCounter].balanceChargerOnTime&0xff;
 
        
            msg_id = MsgID_SelectedBPBalState;//0xA1;
            dataLen = 7;
            bpFlag = BPCounter;
            
            break;         
        ////////////////////////////////////////////////////////
        case 12://Cell 1-4 Voltage
            
            buf[0] = (mStringData.bp[BPCounter].cell[0].volt>>8)&0xff;			
            buf[1] = mStringData.bp[BPCounter].cell[0].volt&0xff;
        
            buf[2] = (mStringData.bp[BPCounter].cell[1].volt>>8)&0xff;			
            buf[3] = mStringData.bp[BPCounter].cell[1].volt&0xff;

            buf[4] = (mStringData.bp[BPCounter].cell[2].volt>>8)&0xff;			
            buf[5] = mStringData.bp[BPCounter].cell[2].volt&0xff;
        
            buf[6] = (mStringData.bp[BPCounter].cell[3].volt>>8)&0xff;			
            buf[7] = mStringData.bp[BPCounter].cell[3].volt&0xff;  
        
            msg_id = MsgID_SelectedBPCellVol1To4;//0xA2;
            dataLen = 8;

            bpFlag = BPCounter;
            break;     
        case 13://Cell 5-8 Voltage
      
            buf[0] = (mStringData.bp[BPCounter].cell[4].volt>>8)&0xff;			
            buf[1] = mStringData.bp[BPCounter].cell[4].volt&0xff;
        
            buf[2] = (mStringData.bp[BPCounter].cell[5].volt>>8)&0xff;			
            buf[3] = mStringData.bp[BPCounter].cell[5].volt&0xff;

            buf[4] = (mStringData.bp[BPCounter].cell[6].volt>>8)&0xff;		
            buf[5] = mStringData.bp[BPCounter].cell[6].volt&0xff;	
        
            buf[6] = (mStringData.bp[BPCounter].cell[7].volt>>8)&0xff;			
            buf[7] = mStringData.bp[BPCounter].cell[7].volt&0xff;  
        
            msg_id = MsgID_SelectedBPCellVol5To8;//0xA3;
            dataLen = 8;

            bpFlag = BPCounter;
            break;  
        
        case 14://Cell 9-12 Voltage
            
            buf[0] = (mStringData.bp[BPCounter].cell[8].volt>>8)&0xff;			
            buf[1] = mStringData.bp[BPCounter].cell[8].volt&0xff;
        
            buf[2] = (mStringData.bp[BPCounter].cell[9].volt>>8)&0xff;			
            buf[3] = mStringData.bp[BPCounter].cell[9].volt&0xff;

            buf[4] = (mStringData.bp[BPCounter].cell[10].volt>>8)&0xff;		
            buf[5] = mStringData.bp[BPCounter].cell[10].volt&0xff;	
        
            buf[6] = (mStringData.bp[BPCounter].cell[11].volt>>8)&0xff;			
            buf[7] = mStringData.bp[BPCounter].cell[11].volt&0xff;  
        
            msg_id = MsgID_SelectedBPCellVol9To12;//0xA4;
            dataLen = 8;

            bpFlag = BPCounter;
            break;  
        
       ///////////////////////////////////////////////////
       case 15://Cell 1-4 Temp
            
            buf[0] = (mStringData.bp[BPCounter].cell[0].temp >> 8) & 0xff;		
            buf[1] = mStringData.bp[BPCounter].cell[0].temp & 0xff;	
        
            buf[2] = (mStringData.bp[BPCounter].cell[1].temp>>8)&0xff;		
            buf[3] = mStringData.bp[BPCounter].cell[1].temp&0xff;	

            buf[4] = (mStringData.bp[BPCounter].cell[2].temp>>8)&0xff;			
            buf[5] = mStringData.bp[BPCounter].cell[2].temp&0xff;
        
            buf[6] = (mStringData.bp[BPCounter].cell[3].temp>>8)&0xff;			
            buf[7] =  mStringData.bp[BPCounter].cell[3].temp&0xff; 
        
            msg_id = MsgID_SelectedBPCellTemp1To4;//0xA6;
            dataLen = 8;

            bpFlag = BPCounter;
            break;     
        case 16://Cell 5-8 Voltage
            
            buf[0] = (mStringData.bp[BPCounter].cell[4].temp>>8)&0xff;			
            buf[1] = mStringData.bp[BPCounter].cell[4].temp&0xff;
        
            buf[2] = (mStringData.bp[BPCounter].cell[5].temp>>8)&0xff;			
            buf[3] = mStringData.bp[BPCounter].cell[5].temp&0xff;

            buf[4] = (mStringData.bp[BPCounter].cell[6].temp>>8)&0xff;			
            buf[5] = mStringData.bp[BPCounter].cell[6].temp&0xff;
        
            buf[6] = (mStringData.bp[BPCounter].cell[7].temp>>8)&0xff;			
            buf[7] = mStringData.bp[BPCounter].cell[7].temp&0xff;  
        
            msg_id = MsgID_SelectedBPCellTemp5To8;//0xA7;
            dataLen = 8;

            bpFlag = BPCounter;
            break;  
        
        case 17://Cell 9-12 Voltage
           
            buf[0] = (mStringData.bp[BPCounter].cell[8].temp>>8)&0xff;			
            buf[1] = mStringData.bp[BPCounter].cell[8].temp&0xff;
        
            buf[2] = (mStringData.bp[BPCounter].cell[9].temp>>8)&0xff;			
            buf[3] = mStringData.bp[BPCounter].cell[9].temp&0xff;

            buf[4] = (mStringData.bp[BPCounter].cell[10].temp>>8)&0xff;		
            buf[5] = mStringData.bp[BPCounter].cell[10].temp&0xff;	
        
            buf[6] = (mStringData.bp[BPCounter].cell[11].temp>>8)&0xff; 			
            buf[7] =  mStringData.bp[BPCounter].cell[11].temp&0xff;
        
            msg_id = MsgID_SelectedBPCellTemp9To12;//0xA8;
            dataLen = 8;

            bpFlag = BPCounter;

            break;  
		case 18:           //发送warrantyTracker信息
			buf[0] = (mStringData.bp[BPCounter].warrantyTracker >> 24) & 0xff;
			buf[1] = (mStringData.bp[BPCounter].warrantyTracker >> 16) & 0xff;
			buf[2] = (mStringData.bp[BPCounter].warrantyTracker >> 8) & 0xff;
			buf[3] = mStringData.bp[BPCounter].warrantyTracker & 0xff;
			buf[4] = mStringData.bp[BPCounter].ProductYear;
			buf[5] = mStringData.bp[BPCounter].ProductMonth;
			buf[6] = mStringData.bp[BPCounter].ProductDay;
			buf[7] = mStringData.bp[BPCounter].WarrantyValid;
			msg_id = MsgID_SelectedBPWarrantyTracker;
			dataLen = 8;
			bpFlag = BPCounter;
		
			break;
		case 19:            //发送 warranty最大 最小 电流值
			buf[0] = (mStringData.bp[BPCounter].maxChargeCurrent >> 8) & 0xff;
			buf[1] = mStringData.bp[BPCounter].maxChargeCurrent & 0xff;
			buf[2] = (mStringData.bp[BPCounter].maxDischargeCurrent >> 8) & 0xff;
			buf[3] = mStringData.bp[BPCounter].maxDischargeCurrent & 0xff;	
			buf[4] = (mStringData.bp[BPCounter].kwh >> 24) & 0xff;
			buf[5] = (mStringData.bp[BPCounter].kwh >> 16) & 0xff;
			buf[6]= (mStringData.bp[BPCounter].kwh >> 8) & 0xff;
			buf[7] = (mStringData.bp[BPCounter].kwh ) & 0xff;
		
			msg_id = MsgID_SelectedBPMAXChargeDisachargeCurrent;
					 
			bpFlag = BPCounter;
			dataLen = 8;
			break;
		case 20:             // MAX volt temp and MIN volt temp ,the whole life of bp
            buf[0] = (mStringData.bp[BPCounter].maxCellVolt >> 8) & 0xff;		
            buf[1] = mStringData.bp[BPCounter].maxCellVolt & 0xff;	
        
            buf[2] = (mStringData.bp[BPCounter].minCellVolt >> 8) & 0xff;			
            buf[3] = mStringData.bp[BPCounter].minCellVolt & 0xff;

            buf[4] = (mStringData.bp[BPCounter].maxCellTemp >> 8) & 0xff;			
            buf[5] = mStringData.bp[BPCounter].maxCellTemp & 0xff;
        
            buf[6] = (mStringData.bp[BPCounter].minCellTemp >> 8) & 0xff;			
            buf[7] =  mStringData.bp[BPCounter].minCellTemp & 0xff; 
        
            msg_id = MsgID_SelectedBPCellWarrantyVolTemp;//0xAC;
            dataLen = 8;

            bpFlag = BPCounter;		
			break;
		case 21:
			buf[0] = mStringData.state;
			buf[1] = mStringData.mode;
			buf[2] = Count.Alarm;
		    buf[3] = Count.Warning + EnterContactorOpenWarningFlag;
		    buf[4] = Count.Error;
			buf[5] = 0x00;         /*calibrate state*/
			buf[6] = 0x00;         /*firmware upgrade*/
			msg_id = MsgID_StringState;
			dataLen = 8;
			
			break;
		case 22:
			buf[0] =  getSelfID();             /*StringID*/
			buf[1] = mStringData.ArrayID;                   /*ArrayID*/
 
			msg_id = MsgID_StringHeartBeat;
			dataLen = 2;
			break;
		
		case 23:     /*发送OverVoltage参数*/
			buf[0] = (u8)(SetClrValue.SetCellOverVoltageAlarm >> 8);
			buf[1] = (u8)(SetClrValue.SetCellOverVoltageAlarm);
			buf[2] = (u8)(SetClrValue.ClrCellOverVoltageAlarm >> 8);
		    buf[3] = (u8)(SetClrValue.ClrCellOverVoltageAlarm);
		    buf[4] = (u8)(SetClrValue.SetCellOverVoltageWarning >> 8);
			buf[5] = (u8)(SetClrValue.SetCellOverVoltageWarning);          
			buf[6] = (u8)(SetClrValue.ClrCellOverVoltageWarning >> 8);         
			buf[7] = (u8)(SetClrValue.ClrCellOverVoltageWarning);
			msg_id = CAN_MSG_OVER_VOLTAGE_SETTING;
			dataLen = 8;			
			break;
		case 24:     /*发送UnderVoltage参数*/
			buf[0] = (u8)(SetClrValue.SetCellUnderVoltageAlarm >> 8);
			buf[1] = (u8)(SetClrValue.SetCellUnderVoltageAlarm);
			buf[2] = (u8)(SetClrValue.ClrCellUnderVoltageAlarm >> 8);
		    buf[3] = (u8)(SetClrValue.ClrCellUnderVoltageAlarm);
		    buf[4] = (u8)(SetClrValue.SetCellUnderVoltageWarning >> 8);
			buf[5] = (u8)(SetClrValue.SetCellUnderVoltageWarning);          
			buf[6] = (u8)(SetClrValue.ClrCellUnderVoltageWarning >> 8);         
			buf[7] = (u8)(SetClrValue.ClrCellUnderVoltageWarning);
			msg_id = CAN_MSG_UNDER_VOLTAGE_SETTING;
			dataLen = 8;			
			break;
		case 25:   /*发送OverTemp参数*/
			buf[0] = (u8)(SetClrValue.SetCellOverTempAlarm >> 8);
			buf[1] = (u8)(SetClrValue.SetCellOverTempAlarm);
			buf[2] = (u8)(SetClrValue.ClrCellOverTempAlarm >> 8);
		    buf[3] = (u8)(SetClrValue.ClrCellOverTempAlarm);
		    buf[4] = (u8)(SetClrValue.SetCellOverTempWarning >> 8);
			buf[5] = (u8)(SetClrValue.SetCellOverTempWarning);          
			buf[6] = (u8)(SetClrValue.ClrCellOverTempWarning >> 8);         
			buf[7] = (u8)(SetClrValue.ClrCellOverTempWarning);
			msg_id = CAN_MSG_OVER_TEMP_SETTING;
			dataLen = 8;				
			break;
		case 26:  /*发送UnderTemp参数*/
			buf[0] = (u8)(SetClrValue.SetCellUnderTempAlarm >> 8);
			buf[1] = (u8)(SetClrValue.SetCellUnderTempAlarm);
			buf[2] = (u8)(SetClrValue.ClrCellUnderTempAlarm >> 8);
		    buf[3] = (u8)(SetClrValue.ClrCellUnderTempAlarm);
		    buf[4] = (u8)(SetClrValue.SetCellUnderTempWarning >> 8);
			buf[5] = (u8)(SetClrValue.SetCellUnderTempWarning);          
			buf[6] = (u8)(SetClrValue.ClrCellUnderTempWarning >> 8);         
			buf[7] = (u8)(SetClrValue.ClrCellUnderTempWarning);
			msg_id = CAN_MSG_UNDER_TEMP_SETTING;
			dataLen = 8;				
			break;
		case 27: /*发送Temp Delta Alarm/Warning 参数*/
			buf[0] = (u8)(SetClrValue.SetHighCellTempDeltaAlarm >> 8);
			buf[1] = (u8)(SetClrValue.SetHighCellTempDeltaAlarm);
			buf[2] = (u8)(SetClrValue.ClrHighCellTempDeltaAlarm >> 8);
		    buf[3] = (u8)(SetClrValue.ClrHighCellTempDeltaAlarm);
		    buf[4] = (u8)(SetClrValue.SetHighCellTempDeltaWarning >> 8);
			buf[5] = (u8)(SetClrValue.SetHighCellTempDeltaWarning);          
			buf[6] = (u8)(SetClrValue.ClrHighCellTempDeltaWarning >> 8);         
			buf[7] = (u8)(SetClrValue.ClrHighCellTempDeltaWarning);
			msg_id = CAN_MSG_HCDT_SETTING;
			dataLen = 8;			
			break;
		case 28: /*发送Temp Rise Alarm/Warning 参数*/
			buf[0] = (u8)(SetClrValue.SetHighCellTempRiseAlarm >> 8);
			buf[1] = (u8)(SetClrValue.SetHighCellTempRiseAlarm);
			buf[2] = (u8)(SetClrValue.ClrHighCellTempRiseAlarm >> 8);
		    buf[3] = (u8)(SetClrValue.ClrHighCellTempRiseAlarm);
		    buf[4] = (u8)(SetClrValue.SetHighCellTempRiseWarning >> 8);
			buf[5] = (u8)(SetClrValue.SetHighCellTempRiseWarning);          
			buf[6] = (u8)(SetClrValue.ClrHighCellTempRiseWarning >> 8);         
			buf[7] = (u8)(SetClrValue.ClrHighCellTempRiseWarning);
			msg_id = CAN_MSG_HCDT_SETTING;
			dataLen = 8;			
			break;
		case 29:/*发送 ChargeRate Alarm/Warning 参数*/
			buf[0] = (u8)(SetClrValue.SetHighChargeRateAlarm >> 8);
			buf[1] = (u8)(SetClrValue.SetHighChargeRateAlarm);
			buf[2] = (u8)(SetClrValue.ClrHighChargeRateAlarm >> 8);
		    buf[3] = (u8)(SetClrValue.ClrHighChargeRateAlarm);
		    buf[4] = (u8)(SetClrValue.SetHighChargeRateWarning >> 8);
			buf[5] = (u8)(SetClrValue.SetHighChargeRateWarning);          
			buf[6] = (u8)(SetClrValue.ClrHighChargeRateWarning >> 8);         
			buf[7] = (u8)(SetClrValue.ClrHighChargeRateWarning);
			msg_id = CAN_MSG_HCR_TEMP_SETTING;
			dataLen = 8;			
			break;
		case 30:/*发送 DisChargeRate Alarm/Warning 参数*/
			buf[0] = (u8)(SetClrValue.SetHighDisChargeRateAlarm >> 8);
			buf[1] = (u8)(SetClrValue.SetHighDisChargeRateAlarm);
			buf[2] = (u8)(SetClrValue.ClrHighDisChargeRateAlarm >> 8);
		    buf[3] = (u8)(SetClrValue.ClrHighDisChargeRateAlarm);
		    buf[4] = (u8)(SetClrValue.SetHighDisChargeRateWarning >> 8);
			buf[5] = (u8)(SetClrValue.SetHighDisChargeRateWarning);          
			buf[6] = (u8)(SetClrValue.ClrHighDisChargeRateWarning >> 8);         
			buf[7] = (u8)(SetClrValue.ClrHighDisChargeRateWarning);
			msg_id = CAN_MSG_HDR_TEMP_SETTING;
			dataLen = 8;			
			break;
		
		case 31:
			
			buf[0] =  (u8)(PortADC1 >> 8);
			buf[1] =  (u8)(PortADC1);
			buf[2] =  (u8)(PortADC2 >> 8); 
		    buf[3] =  (u8)(PortADC2); 
		    buf[4] =  GetGroundFaultState();  /*0 表示 有 接地*/
			buf[5] =  (u8)(testDCVoltage >> 8);          
			buf[6] =  (u8)(testDCVoltage);         
			buf[7] =  0x00;
			msg_id = 0xBB;                /*To Test ground fault*/
			dataLen = 8;	
			break;
		
		case 32:
			buf[0] =  mStringData.bp[BPCounter].bpstatus ;
			buf[1] =  mStringData.bp[BPCounter].bpalarmcount ; 
			buf[2] =  mStringData.bp[BPCounter].bpwarningcount ; 
		    buf[3] =  mStringData.bp[BPCounter].bperrorcount ;
 		
		
			msg_id = 0xA1;
			dataLen = 4;
			bpFlag = BPCounter;
			break;
		case 33:
			buf[0] = Count.BpToStringAlarm;
			buf[1] = BPToStringHVolAlarmbit.BPbyte;
			buf[2] = BPToStringHTempAlarmbit.BPbyte;
			buf[3] = BPToStringLVolAlarmbit.BPbyte;
			buf[4] = BPToStringLTempAlarmbit.BPbyte;
			buf[5] = OverChargeCount;
		    buf[6] = OVerDischargeCount;
		
		    msg_id = 0xF7;
			dataLen = 7;
		    break;
		case 34:
			buf[0] = Count.BpToStringWarning;
			buf[1] = BPToStringHVolWarningbit.BPbyte;
			buf[2] = BPToStringHTempWarningbit.BPbyte;
			buf[3] = BPToStringLVolWarningbit.BPbyte;
			buf[4] = BPToStringLTempWarningbit.BPbyte;
			buf[5] = OverChargeCount;
		    buf[6] = OVerDischargeCount;
		
		    msg_id = 0xF8;
			dataLen = 7;
		    break;	

		case 35:
            buf[0] = (mStringData.dcbusVoltage>>8)&0xff;			
            buf[1] = mStringData.dcbusVoltage&0xff;	
			buf[2] = ArrayStringID(mStringData.ArrayID,Self_ID); 
			buf[3] = Self_ID;
			buf[4] = device_id;
			buf[5] = Self_ID;
			buf[6] = getSelfID();
            msg_id = DCBUS_Voltage;  //0x52
            dataLen = 7;		
		break;
		 case 36://Cell 13-16 Voltage
            
            buf[0] = (mStringData.bp[BPCounter].cell[12].volt>>8)&0xff;			
            buf[1] = mStringData.bp[BPCounter].cell[12].volt&0xff;
        
            buf[2] = (mStringData.bp[BPCounter].cell[13].volt>>8)&0xff;			
            buf[3] = mStringData.bp[BPCounter].cell[13].volt&0xff;

            buf[4] = (mStringData.bp[BPCounter].cell[14].volt>>8)&0xff;		
            buf[5] = mStringData.bp[BPCounter].cell[14].volt&0xff;	
        
            buf[6] = (mStringData.bp[BPCounter].cell[15].volt>>8)&0xff;			
            buf[7] = mStringData.bp[BPCounter].cell[15].volt&0xff;  
        
            msg_id = MsgID_SelectedBPCellVol13To16;//0xA5;
            dataLen = 8;

            bpFlag = BPCounter;
            break; 
        case 37://Cell 13-16 Temp
           
            buf[0] = (mStringData.bp[BPCounter].cell[12].temp>>8)&0xff;			
            buf[1] = mStringData.bp[BPCounter].cell[12].temp&0xff;
        
            buf[2] = (mStringData.bp[BPCounter].cell[13].temp>>8)&0xff;			
            buf[3] = mStringData.bp[BPCounter].cell[13].temp&0xff;

            buf[4] = (mStringData.bp[BPCounter].cell[14].temp>>8)&0xff;		
            buf[5] = mStringData.bp[BPCounter].cell[14].temp&0xff;	
        
            buf[6] = (mStringData.bp[BPCounter].cell[15].temp>>8)&0xff; 			
            buf[7] =  mStringData.bp[BPCounter].cell[15].temp&0xff;
        
            msg_id = MsgID_SelectedBPCellTemp13To16;//0xA8;
            dataLen = 8;

            bpFlag = BPCounter;

            break; 			
        default:
            printf("<*****< send_autodata_Ex not implement : 0x%02x >>\n", idx);
            //autodataEx = AUTO_SEND_EX_NUMBER;

            break;
    }
    if(dataLen == 0)
        return;
    if(bpFlag != 0xff)
    {
        MsgGroupID = ID_Data;
		CAN_make_send_from_BP(buf, dataLen, msg_id, bpFlag);
    }
    else
    {
        MsgGroupID = ID_Data;
		CAN_make_send_2_Array( buf, dataLen, msg_id);
    }
    //printf("  [send_autodata_Ex]: can_tx_fifo.count = %d ...\n\r", can_tx_fifo.count);
    

}







//送给BP
void CAN_make_send_2_BP (BUFFER volatile * rxbuf2)
{
    BUFFER volatile *txbuf;
    if (suppress_can_tx) 
        return;

    txbuf = fifo_alloc(&can_tx_fifo);
    if (txbuf == 0) 
    {
        printf("-> CAN_make_send_2_BP FIFO full\n");
        return;
    }
    memcpy((void*)txbuf, (void*)rxbuf2, sizeof(BUFFER volatile));  
    
    #if(1)
    {
        MESSAGE message;
        DisassembleID(&message, txbuf->ID);
        message.MessageNum  = txbuf->DLC;   // For incoming messages, MessageNum holds payload length.
        memcpy(message.Data, (void *)&txbuf->data.bytes, txbuf->DLC);
        printf("-> CAN_make_send_2_BP : [0x%08x: 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x : %d ]\r\n", txbuf->ID, 
                    message.StringID, message.MessageType, message.DeviceType, message.DeviceID, message.MessageID, txbuf->DLC);
        printf("           =[0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x] \r\n",
                        message.Data[0], message.Data[1], message.Data[2], message.Data[3], message.Data[4], message.Data[5], message.Data[6], message.Data[7]);
    }
    #endif
}
//送给BP
void CAN_make_send_bypass_2_BP(uint8_t *buf, uint8_t length, uint8_t msg_id, uint8_t device_id)
{
    BUFFER volatile *txbuf ;
    if (suppress_can_tx) 
        return;
                                          

    txbuf = fifo_alloc(&can_tx_fifo);
    if (txbuf == 0) 
    {
        printf("-> CAN_make_send_bypass_2_BP FIFO full\n");
        return;
    }
    memcpy((void *)txbuf->data.bytes, buf, sizeof(txbuf->data.bytes));
    txbuf->DLC = length;
    txbuf->ID = MAKE_ID(0x00, Self_ID, MsgGroupID, STRING_TYPE, device_id, msg_id);
}

/*For Efacec 2015年11月26日11:11:51 add by jason*/
void Can_Send_2_CCU(uint32_t ExtendID,uint8_t Len,uint8_t *buf)
{
   BUFFER volatile *txbuf;
    if (suppress_can_tx) 
        return;                                          

    txbuf = fifo_alloc(&can_tx_fifo2);
    if (txbuf == 0) 
    {
        printf("-> CAN_make_send_2_Array FIFO full\n");
        return;
    }
	
    memcpy((void *)txbuf->data.bytes, buf, sizeof(txbuf->data.bytes));
    txbuf->DLC = Len;
    txbuf->ID = ExtendID;		
}

//发送给ARRAY
void CAN_make_send_2_Array (uint8_t *buf, uint8_t length, uint8_t msg_id)
{
    BUFFER volatile *txbuf;
    if (suppress_can_tx) 
        return;
    #if(0)
    printf("-> CAN_make_send_2_Array(device_id = 0x%02x): msg_id = 0x%02x \n", device_id, msg_id);
    printf("            data =<0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x>\r\n"
                                            , buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
    #endif                                            

    txbuf = fifo_alloc(&can_tx_fifo2);
    if (txbuf == 0) 
    {
        printf("-> CAN_make_send_2_Array FIFO full\n");
        return;
    }
    memcpy((void *)txbuf->data.bytes, buf, sizeof(txbuf->data.bytes));
    txbuf->DLC = length;
    txbuf->ID = MAKE_ID(0x00, Self_ID, MsgGroupID, STRING_TYPE, 0x00, msg_id);
}
//送给Array，转送bp的资料
void CAN_make_send_from_BP (uint8_t *buf, uint8_t length, uint8_t msg_id, int8_t bp_id)
{
    BUFFER volatile *txbuf;
    if (suppress_can_tx) 
        return;
    
    bp_id = bp_id + 1;//index start from 1
                                          
    txbuf = fifo_alloc(&can_tx_fifo2);
    if (txbuf == 0) 
    {
        printf("\r\n !!!!!!-> CAN_make_send_from_BP buffer is full....\r\n\n");
         
        return;
    }
    memcpy((void *)txbuf->data.bytes, buf, sizeof(txbuf->data.bytes));
    txbuf->DLC = length;

    txbuf->ID = MAKE_ID(0x00, Self_ID, MsgGroupID, STRING_TYPE, bp_id, msg_id);
}

void block_can_tx()
{
    suppress_can_tx = 1;

    can_tx_block_timer = CAN_TX_BLOCK_TIMER_INIT_VALUE;
}

void unblock_can_tx()
{
    suppress_can_tx = 0;

 //   I2C_EE_BufferWrite((uint8_t *)0xff, EEADDRESS(can_tx_block), 1);
}

void suicide(void)
{
    NVIC_SETPRIMASK(); /**< disable global interrupts */
//    Bal_Switch(Pack.BalanceSwitch&0x0ffff); //Turn off the balance switch
//    Power_ON_OFF(DISABLE); // Power off
    while (1) {
        __asm(" nop");       // handy break line for debugger
    }
}
/*-----------------------------------------------------------------------------*/
/**
 * @fn        void CAN_RX_Process(void)
 * @brief    Process the received CAN bus data.
 * @param    None
 * @retval    None


 */
void CAN_RX_Process(void) // BP 发给 String
{
    MESSAGE msg;
    BUFFER volatile *rxbuf;  // pointers into FIFO for active entries during processing
    //uint8_t State;
    //uint8_t msg_id;
    //uint8_t temp;
    //uint16_t current_ca_set_new, power_watts_set_new;

    // If configured to check for timeout on CAN bus, and timeout has been exceeded,
#if 0     // remove CAN timeout for debug todo: re-install for production! pjn 2014-2-18
    if ((PackPara.CANOutTime != 0) && (Pack.CANTimer >= PackPara.CANOutTime)) {

        suicide();   // turn our own power off
    }
#endif
    // while there is a message to process
    while (can_rx_fifo.count) 
	{
        CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
        CANTimer = 0; // Empty delay timer

        rxbuf = fifo_get(&can_rx_fifo);  // decrements fifo.count

        // split received ID into bit fields
        DisassembleID(&msg, rxbuf->ID);
        msg.MessageNum  = rxbuf->DLC;   // For incoming messages, MessageNum holds payload length.
        memcpy(msg.Data, (void *)&rxbuf->data.bytes, rxbuf->DLC);

        //printf("\r\n[CAN_Receive]: ID = 0x%08x, DLC = 0x%08x, DataH = 0x%08x, DataL = 0x%08x,\n", rxbuf->ID, rxbuf->DLC, rxbuf->data.longwords.DataH, rxbuf->data.longwords.DataL);
        //return;
        //void DisassembleID(MESSAGE* message, uint32_t ID)
        {
            MESSAGE message;
            DisassembleID(&message, rxbuf->ID);
            //printf("     StrinfID = 0x%02x, MessageType = 0x%02x, DeviceType = 0x%02x,\r\n     DeviceID = 0x%02x,  MessageID = 0x%02x\n\r\n", 
            //                        message.StringID, message.MessageType, message.DeviceType, message.DeviceID, message.MessageID);
            #if(0)
            printf("=<CAN_RX_1>=[0x%08x: 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x : %d (total send:%d, rx overfolw:%d)]\r\n", rxbuf->ID, 
                        message.StringID, message.MessageType, message.DeviceType, message.DeviceID, message.MessageID, 
                        rxbuf->DLC, totalSendCounter, rxOverFolwCounter);
            #endif
        }
        
        if(msg.DeviceType != BP_TYPE)
        {
            printf("RX: wrong device type %d....\r\n", msg.DeviceType);
			CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
            return;
        }
        if( ((msg.DeviceID-1) < 0) || ((msg.DeviceID-1) >= BP_NUMBER))
        {
            printf("RX: wrong device id, save to %d [ get id %d]....\r\n", msg.DeviceID - 1, msg.DeviceID);
            CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
			return;
        }
       
        switch(msg.MessageID) 
        {            
            
            case CMD_ID_SERVER_UPGRADE_WRITE_MEMORY:
                {                   
                    MsgGroupID = ID_Upgrade;
					CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID-1);
                    //#if(DEBUG_RX_COMMAND)                   
                                    
                    printf("@@@-0x78-@@@ Firmware Message -- String id: %d, BP id: %d \r\n ", msg.StringID, msg.DeviceID);
                    //#endif
                }
                break;
            case CMD_ID_SERVER_FIRMWARE_INFO:
                {                   
                    MsgGroupID = ID_Upgrade;
					CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID-1);
 
						if(msg.Data[2] >= 200)  /*booter error message*/
						{
							MESSAGE bootererror;
							Count.OldBpBootError = CalculateCountOfState(BPbooterErrorbit.BPbyte);
							BPbooterErrorbit.BPbyte |= 1 << (msg.DeviceID - 1);
							StringAWEHappenedFlag.BPBootError = 1;
							if(CalculateCountOfState(BPbooterErrorbit.BPbyte) <= 1)
							{
								SysStateAckFlag[msg.DeviceID - 1].BPBootError = 1;
								bootererror.MessageID = ID_Error;
								MsgGroupID = ID_Error;
								bootererror.Data[0] = Error_BpBooter;
								bootererror.Data[1] = Update; 
								bootererror.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
								bootererror.Data[3] = 0x00;
								bootererror.Data[4] = BPbooterErrorbit.Byte1;
								bootererror.Data[5] = BPbooterErrorbit.Byte2;
								bootererror.Data[6] = BPbooterErrorbit.Byte3;
								bootererror.Data[7] = BPbooterErrorbit.Byte4;
								bootererror.DeviceID = -1;
								CAN_make_send_from_BP(bootererror.Data,8,MsgID_Error,bootererror.DeviceID); 
							}
						}
						else
						{							
							Count.OldBpBootError = CalculateCountOfState(BPbooterErrorbit.BPbyte);
							BPbooterErrorbit.BPbyte &= ~(1 << (msg.DeviceID - 1));							
						}
				  
					 
					  
						
                    //#if(DEBUG_RX_COMMAND)                   
                                    
                    printf("@@@-0x78-@@@ Firmware Message -- String id: %d, BP id: %d \r\n ", msg.StringID, msg.DeviceID);
                    //#endif
                }
                break;
            case CMD_ID_SERVER_COMMAND_ACTION:
                {                   
                    MsgGroupID = ID_Upgrade;
					CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID-1);
                    //#if(DEBUG_RX_COMMAND)                   
                                    
                    printf("@@@-0x78-@@@ Firmware Message -- String id: %d, BP id: %d \r\n ", msg.StringID, msg.DeviceID);
                    //#endif
                }
                break;
 
			
			/*add by jason @ 2016年7月21日10:43:07
			  新增warrantyTracker消息，转发给array controller
				
			*/
			case MsgID_AllWarrantyTracker:
			{
				BPCounter = msg.DeviceID - 1;
				MsgGroupID = ID_Warranty;
				
				CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1);
					
			}
			break;
			/*add by jason,2016年11月8日9:29:23*/
			case MsgID_BlueBoxSN:
			{
				BPCounter = msg.DeviceID - 1;
				MsgGroupID = ID_Configure;
				
				CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1);				
				
			}
			break;
			case MsgID_BPSN:
			{
				BPCounter = msg.DeviceID - 1;
				MsgGroupID = ID_Configure;
				
				CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1);					
			}
			break;
			case MsgID_BPUID:
			{
				BPCounter = msg.DeviceID - 1;
				MsgGroupID = ID_Configure;
				
				CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1);					
				
			}
			break;
			
            case MsgID_BpRequestArrayStringID:            /*Bp request stringID and arrayID*/
			{
				BPCounter = msg.DeviceID - 1;
				
 				send_autodata_Ex_2_BP(MsgID_BpRequestArrayStringID, msg.DeviceID);  /*string  send arrayid and stringid to bp*/
				
				
			}
			break;
			case MsgIDToBP_HeartBeat:                        /*清除失联计数器*/
			{
				Count.BpLoseHeatBeat[msg.Data[0] - 1] = 0;  /*数组下标从0开始*/
				BpLossCommunicationWarningFlag[msg.Data[0] - 1] = 0;  /*失联标志清理0，未失联*/
//	 			 ReCheckLossCommunication();
			}
			break;
			case 0x20:                              /*bp query the target*/
				 
//			    DecodeFour16BitData(msg.Data, 
//                                        &(mStringData.balancingTargetVoltage),
//                                        &(mStringData.balancingTargetSOC),
//										&(mStringData.stringCurrent),
//										&(mStringData.ah));
				BPCounter = msg.DeviceID - 1;
				send_autodata_Ex_2_BP(0x20, msg.DeviceID);   /*string  send arrayid and stringid to bp*/
			   
				break;
			 
			case 0x42://0x27:  /*获取 BP相关电压*/
                {
                    
					mStringData.bp[msg.DeviceID-1].highCellVolt = ((msg.Data[1])&0xff)  | ((msg.Data[0]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].lowCellVolt = ((msg.Data[3])&0xff)  | ((msg.Data[2]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].averageCellVolt = ((msg.Data[5])&0xff)  | ((msg.Data[4]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].measureBPVoltage = ((msg.Data[7])&0xff)  | ((msg.Data[6]&0xff) << 8);
					

                    BPCounter = msg.DeviceID - 1;
                    //send_autodata_Ex(9);  
                }
                break;
            case 0x43://0x29: /*获取BP相关温度*/
                {
                    mStringData.bp[msg.DeviceID-1].highCellTemp = ((msg.Data[1])&0xff)  | ((msg.Data[0]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].lowCellTemp = ((msg.Data[3])&0xff)  | ((msg.Data[2]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].averageCellTemp = ((msg.Data[5])&0xff)  | ((msg.Data[4]&0xff) << 8);
                    mStringData.bp[ msg.DeviceID-1].maxCellTempRise = ((msg.Data[7])&0xff)  | ((msg.Data[6]&0xff) << 8);
                    
                    BPCounter = msg.DeviceID-1;
                    //send_autodata_Ex(10); 
                }
                break;
            //case 0x28:
            case 0x48://0x0c: /*电池1-4 电压*/
                {
                    mStringData.bp[msg.DeviceID-1].cell[0].volt = ((msg.Data[1])&0xff)  | ((msg.Data[0]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[1].volt = ((msg.Data[3])&0xff)  | ((msg.Data[2]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[2].volt = ((msg.Data[5])&0xff)  | ((msg.Data[4]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[3].volt = ((msg.Data[7])&0xff)  | ((msg.Data[6]&0xff) << 8);
                    
                    BPCounter = msg.DeviceID-1;
                    send_autodata_Ex(12);
                }
                break;
            case 0x49://0x0d:/* 电池5- 8 电压*/
                {
                    mStringData.bp[msg.DeviceID-1].cell[4].volt = ((msg.Data[1])&0xff)  | ((msg.Data[0]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[5].volt = ((msg.Data[3])&0xff)  | ((msg.Data[2]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[6].volt = ((msg.Data[5])&0xff)  | ((msg.Data[4]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[7].volt = ((msg.Data[7])&0xff)  | ((msg.Data[6]&0xff) << 8);
                  
                    BPCounter = msg.DeviceID-1;
                    send_autodata_Ex(13);
                }
                break;
            case 0x4a://0x0e: /*电池 9- 12 电压*/
                {
                    mStringData.bp[msg.DeviceID-1].cell[8].volt =  ((msg.Data[1])&0xff)  | ((msg.Data[0]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[9].volt =  ((msg.Data[3])&0xff)  | ((msg.Data[2]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[10].volt = ((msg.Data[5])&0xff)  | ((msg.Data[4]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[11].volt = ((msg.Data[7])&0xff)  | ((msg.Data[6]&0xff) << 8);
                   
                    BPCounter = msg.DeviceID-1;
                    send_autodata_Ex(14);
                }
                break;
			    case 0x4b://0x0e: /*电池 13- 16 电压*/
                {
                    mStringData.bp[msg.DeviceID-1].cell[12].volt =  ((msg.Data[1])&0xff)  | ((msg.Data[0]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[13].volt =  ((msg.Data[3])&0xff)  | ((msg.Data[2]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[14].volt = ((msg.Data[5])&0xff)  | ((msg.Data[4]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[15].volt = ((msg.Data[7])&0xff)  | ((msg.Data[6]&0xff) << 8);
                   
                    BPCounter = msg.DeviceID-1;
                    send_autodata_Ex(36);
                }
                break;
				
            case 0x4c://0x10: /*电池1-4 温度*/
                {
                    mStringData.bp[msg.DeviceID-1].cell[0].temp = ((msg.Data[1])&0xff)  | ((msg.Data[0]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[1].temp = ((msg.Data[3])&0xff)  | ((msg.Data[2]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[2].temp = ((msg.Data[5])&0xff)  | ((msg.Data[4]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[3].temp = ((msg.Data[7])&0xff)  | ((msg.Data[6]&0xff) << 8);
                    
                    BPCounter = msg.DeviceID-1;
                    send_autodata_Ex(15);
                }
                break;
            case 0x4d://0x11: /*电池5-8 温度*/
                {
                    mStringData.bp[msg.DeviceID-1].cell[4].temp = ((msg.Data[1])&0xff)  | ((msg.Data[0]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[5].temp = ((msg.Data[3])&0xff)  | ((msg.Data[2]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[6].temp = ((msg.Data[5])&0xff)  | ((msg.Data[4]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[7].temp = ((msg.Data[7])&0xff)  | ((msg.Data[6]&0xff) << 8);
 
                    BPCounter = msg.DeviceID-1;
                    send_autodata_Ex(16);
                }
                break;
            case 0x4e://0x12:/*电池9-12 温度*/
                {
                    mStringData.bp[msg.DeviceID-1].cell[8].temp =  ((msg.Data[1])&0xff)  | ((msg.Data[0]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[9].temp =  ((msg.Data[3])&0xff)  | ((msg.Data[2]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[10].temp = ((msg.Data[5])&0xff)  | ((msg.Data[4]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[11].temp = ((msg.Data[7])&0xff)  | ((msg.Data[6]&0xff) << 8);
 
                    BPCounter = msg.DeviceID-1;
                    send_autodata_Ex(17);
                }
                break;
                case 0x4f://0x12:/*电池13-16 温度*/
                {
                    mStringData.bp[msg.DeviceID-1].cell[12].temp =  ((msg.Data[1])&0xff)  | ((msg.Data[0]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[13].temp =  ((msg.Data[3])&0xff)  | ((msg.Data[2]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[14].temp = ((msg.Data[5])&0xff)  | ((msg.Data[4]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].cell[15].temp = ((msg.Data[7])&0xff)  | ((msg.Data[6]&0xff) << 8);
 
                    BPCounter = msg.DeviceID-1;
                    send_autodata_Ex(37);
                }
                break;                
            case 0x44://0x12: /* 读取均衡电阻和均衡电源状态  */
                {
                    mStringData.bp[msg.DeviceID-1].cell[0].resistor = ((msg.Data[1])>>0x0)&0x1;
                    mStringData.bp[msg.DeviceID-1].cell[1].resistor = ((msg.Data[1])>>0x1)&0x1;
                    mStringData.bp[msg.DeviceID-1].cell[2].resistor = ((msg.Data[1])>>0x2)&0x1;
                    mStringData.bp[msg.DeviceID-1].cell[3].resistor = ((msg.Data[1])>>0x3)&0x1;
                    mStringData.bp[msg.DeviceID-1].cell[4].resistor = ((msg.Data[1])>>0x4)&0x1;
                    mStringData.bp[msg.DeviceID-1].cell[5].resistor = ((msg.Data[1])>>0x5)&0x1;
                    mStringData.bp[msg.DeviceID-1].cell[6].resistor = ((msg.Data[1])>>0x6)&0x1;
                    mStringData.bp[msg.DeviceID-1].cell[7].resistor = ((msg.Data[1])>>0x7)&0x1;
                    
                    mStringData.bp[msg.DeviceID-1].cell[8].resistor = ((msg.Data[0])>>0x0)&0x1;
                    mStringData.bp[msg.DeviceID-1].cell[9].resistor = ((msg.Data[0])>>0x1)&0x1;
                    mStringData.bp[msg.DeviceID-1].cell[10].resistor = ((msg.Data[0])>>0x2)&0x1;
                    mStringData.bp[msg.DeviceID-1].cell[11].resistor = ((msg.Data[0])>>0x3)&0x1;
                    mStringData.bp[msg.DeviceID-1].cell[12].resistor = ((msg.Data[0])>>0x4)&0x1;
                    mStringData.bp[msg.DeviceID-1].cell[13].resistor = ((msg.Data[0])>>0x5)&0x1;
                    mStringData.bp[msg.DeviceID-1].cell[14].resistor = ((msg.Data[0])>>0x6)&0x1;
                    mStringData.bp[msg.DeviceID-1].cell[15].resistor = ((msg.Data[0])>>0x7)&0x1;                   
					
                    mStringData.bp[msg.DeviceID-1].balanceChargerStatus = msg.Data[2];
                    
                    
                    mStringData.bp[msg.DeviceID-1].balanceChargerOnTime = ((msg.Data[4])&0xff)  | ((msg.Data[3]&0xff) << 8)  | 
                                                                        ((msg.Data[6]&0xff) << 16)  | ((msg.Data[5]&0xff) << 24);

                    BPCounter = msg.DeviceID-1;
                    send_autodata_Ex(11);
                }
                break;
            case 0x45://0x12: /* 读取均衡时间*/
                {
                    mStringData.bp[msg.DeviceID-1].balanceChargerOnTime = ((msg.Data[1])&0xff)  | ((msg.Data[0]&0xff) << 8)  | 
                                                                        ((msg.Data[3]&0xff) << 16)  | ((msg.Data[2]&0xff) << 24);
					mStringData.bp[msg.DeviceID-1].calculateBPVoltage = ((msg.Data[5])&0xff)  | ((msg.Data[4]&0xff) << 8);
					
					mStringData.bp[msg.DeviceID-1].cellDeltaT = ((msg.Data[7])&0xff)  | ((msg.Data[6]&0xff) << 8);
					
 
                }
                //????
                break;
				
			case 0x41:
				{
					mStringData.bp[msg.DeviceID-1].bpstatus = msg.Data[0];
					mStringData.bp[msg.DeviceID-1].bpalarmcount = msg.Data[1];
					mStringData.bp[msg.DeviceID-1].bpwarningcount = msg.Data[2];
					mStringData.bp[msg.DeviceID-1].bperrorcount = msg.Data[3];
				    
					BPCounter = msg.DeviceID-1;
					send_autodata_Ex(32);
					
				}
			
				break;
            case  MsgIDToBP_SelectBPWarrantyTracker:    // 0xDF ,读取年月日 追踪时间
				{
					mStringData.bp[msg.DeviceID-1].warrantyTracker = (msg.Data[0] << 24) | (msg.Data[1] << 16) | (msg.Data[2] << 8) | msg.Data[3];
					mStringData.bp[msg.DeviceID-1].ProductYear = msg.Data[4];   // year
					mStringData.bp[msg.DeviceID-1].ProductMonth = msg.Data[5];   // month
					mStringData.bp[msg.DeviceID-1].ProductDay = msg.Data[6];   // day
					mStringData.bp[msg.DeviceID-1].WarrantyValid = msg.Data[7];   // WarrantyValid
					BPCounter = msg.DeviceID-1;
					send_autodata_Ex(18);
				}	
				break;
			case MsgIDToBP_MaxChargeDischargeCurrent: //0xDE
				{
					mStringData.bp[msg.DeviceID-1].maxChargeCurrent = (msg.Data[0] << 8) | msg.Data[1];
					mStringData.bp[msg.DeviceID-1].maxDischargeCurrent = (msg.Data[2] << 8) | msg.Data[3];
					mStringData.bp[msg.DeviceID-1].kwh = (msg.Data[4] << 24) | (msg.Data[5] << 16) | (msg.Data[6] << 8) | msg.Data[7];
					BPCounter = msg.DeviceID-1;
					send_autodata_Ex(19) ;
				}
				break;
            case MsgID_Alarm: /* BP 发上里的alarm消息，断开两个接触器，并Clear标志位后将数据返还给BP*/
                {

					
					if(msg.Data[1] == AlarmSet)
					{

						uint8_t ByteTemp = msg.Data[1];
						
			
						MsgGroupID = ID_Alarm;
						
						msg.Data[1] = AlarmSetAck;
						
						CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID);     // 返回
						
						msg.Data[1] = ByteTemp;						
										
						
						switch (msg.Data[0])
						{
							case Alarm_CellOverVoltage:
							{
								
								BPToStringHVolAlarmbit.BPbyte |= 1 << (msg.Data[3]-1) ;                             /*ACK 标志位为1，继续发，同时改变 5byte-8byte*/
								
								if(CalculateCountOfState(BPToStringHVolAlarmbit.BPbyte) <= 1)
								{
									BpToStringFlag[msg.DeviceID - 1].HVolAlarmFlag = 1;                                 /*bp 判断到的alarm ack标志，如果array有回ack，则设置为 0*/
									BPHVolAlarmFlag[msg.Data[3]-1] = 1;                                                /*暂时未用到*/
	
									
									msg.Data[1] = Update;
									msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
									msg.Data[3] = 0x01;     
									msg.Data[4] = BPToStringHVolAlarmbit.Byte1;
									msg.Data[5] = BPToStringHVolAlarmbit.Byte2;
									msg.Data[6] = BPToStringHVolAlarmbit.Byte3;
									msg.Data[7] = BPToStringHVolAlarmbit.Byte4;
									msg.MessageNum = 8;	
									MsgGroupID = ID_Alarm;			
									CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID - 1   );
								}
								
							}
								break;
							case Alarm_CellUnderVoltage:
							{
								BPToStringLVolAlarmbit.BPbyte |= 1 << (msg.Data[3]-1);
								
								if(CalculateCountOfState(BPToStringLVolAlarmbit.BPbyte) <= 1)
								{
									BpToStringFlag[msg.DeviceID - 1].LVolAlarmFlag = 1;
									BPLVolAlarmFlag[msg.Data[3]-1] = 1;
									
									msg.Data[1] = Update;
									msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
									msg.Data[3] = 0x01;     
									msg.Data[4] = BPToStringLVolAlarmbit.Byte1;
									msg.Data[5] = BPToStringLVolAlarmbit.Byte2;
									msg.Data[6] = BPToStringLVolAlarmbit.Byte3;
									msg.Data[7] = BPToStringLVolAlarmbit.Byte4;
									msg.MessageNum = 8;	
									MsgGroupID = ID_Alarm;	
																	
									CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID - 1  );	
								}
							}
								break;
							case Alarm_CellOverTemp:
							{
								BPToStringHTempAlarmbit.BPbyte |= 1 << (msg.Data[3]-1) ; 
								
								if(CalculateCountOfState(BPToStringHTempAlarmbit.BPbyte) <=1 )
								{
									BpToStringFlag[msg.DeviceID - 1].HTempAlarmFlag = 1;
									BPHTempAlarmFlag[msg.Data[3]-1] = 1;
									
									msg.Data[1] = Update;
									msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
									msg.Data[3] = 0x01;     
									msg.Data[4] = BPToStringHTempAlarmbit.Byte1;
									msg.Data[5] = BPToStringHTempAlarmbit.Byte2;
									msg.Data[6] = BPToStringHTempAlarmbit.Byte3;
									msg.Data[7] = BPToStringHTempAlarmbit.Byte4;
									msg.MessageNum = 8;	
									MsgGroupID = ID_Alarm;			
									CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID - 1 );						
								}
							}
								break;
							case Alarm_CellUnderTemp:
							{
								BPToStringLTempAlarmbit.BPbyte |= 1 << (msg.Data[3]-1); 
								
								if(CalculateCountOfState(BPToStringLTempAlarmbit.BPbyte) <= 1)
								{
									BpToStringFlag[msg.DeviceID - 1].LTempAlarmFlag = 1;
									BPLTempAlarmFlag[msg.Data[3]-1] = 1;
	
									msg.Data[1] = Update;
									msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
									msg.Data[3] = 0x01;     
									msg.Data[4] = BPToStringLTempAlarmbit.Byte1;
									msg.Data[5] = BPToStringLTempAlarmbit.Byte2;
									msg.Data[6] = BPToStringLTempAlarmbit.Byte3;
									msg.Data[7] = BPToStringLTempAlarmbit.Byte4;
									msg.MessageNum = 8;
									MsgGroupID = ID_Alarm;	
									
									CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID - 1   );
								}
								
							}
								break;
								
							case Alarm_HighChargeRate:
							{
								BPToStringChargeRateAlarmbit.BPbyte |= 1 << (msg.Data[3]-1); 
								
								if(CalculateCountOfState(BPToStringChargeRateAlarmbit.BPbyte) <= 1)
								{
								
									BpToStringChargeDisChargeFLag.ChargeStringCurrentAlarmFlag = 1;
									BpToStringFlag[msg.DeviceID - 1].ChargeStringCurrentAlarmFlag = 1;
									
									msg.Data[1] = Update;
									msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
									msg.Data[3] = 0x01;     
									msg.Data[4] = BPToStringChargeRateAlarmbit.Byte1;
									msg.Data[5] = BPToStringChargeRateAlarmbit.Byte2;
									msg.Data[6] = BPToStringChargeRateAlarmbit.Byte3;
									msg.Data[7] = BPToStringChargeRateAlarmbit.Byte4;
									msg.MessageNum = 8;
									MsgGroupID = ID_Alarm;	
									
									CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID - 1   );									
									
								}
								
								OverChargeCount = 1;
							}
								break;
							case Alarm_HighDischargeRate:
							{
								BPToStringDischargeRateAlarmbit.BPbyte |= 1 << (msg.Data[3]-1);
								
								if(CalculateCountOfState(BPToStringDischargeRateAlarmbit.BPbyte) <= 1)
								{
									BpToStringChargeDisChargeFLag.DischargeStringCurrentAlarmFlag = 1;
									BpToStringFlag[msg.DeviceID - 1].DischargeStringCurrentAlarmFlag = 1;
									
									msg.Data[1] = Update;
									msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
									msg.Data[3] = 0x01;     
									msg.Data[4] = BPToStringDischargeRateAlarmbit.Byte1;
									msg.Data[5] = BPToStringDischargeRateAlarmbit.Byte2;
									msg.Data[6] = BPToStringDischargeRateAlarmbit.Byte3;
									msg.Data[7] = BPToStringDischargeRateAlarmbit.Byte4;
									msg.MessageNum = 8;
									MsgGroupID = ID_Alarm;	
									
									CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID - 1   );										
								}
									
								OVerDischargeCount = 1;
							}
								break;
							
							
						}
					}
					if(msg.Data[1] == AlarmClr)
					{
						uint8_t ByteTemp = msg.Data[1];
						
						MsgGroupID = ID_Alarm;
						
						msg.Data[1] = AlarmClrAck;
						
  						CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID );     //  回复 alarmclrack
						
						msg.Data[1] = ByteTemp;						
						
						switch (msg.Data[0])
						{
							case Alarm_CellOverVoltage:
							{								
								BPAWEHappenedFlag.HVoltageAlarm = 1;
								BPToStringHVolAlarmbit.BPbyte &= ~(1 << (msg.Data[3]-1)) ;                             /*ACK 标志位为1，继续发，同时改变 5byte-8byte*/
								 
								{
								
									BpToStringFlag[msg.DeviceID - 1].HVolAlarmFlag = 0;
									BPHVolAlarmFlag[msg.Data[3]-1] = 0;
//									if(BPHVolAlarmFlag[msg.Data[3] -1] == 0 && StringHVolAlarmFlag[msg.Data[3]-1] == 0)
//										BPHVolAlarmbit->BPbyte &= ~(1 << (msg.DeviceID - 1));         /*clr the bit*/
									
									/*转发 clear给 Array */
								#if 0	
									MsgGroupID = ID_Alarm;
									msg.Data[1] = AlarmClr;
									msg.Data[3] = 1;
									msg.Data[4] = BPToStringHVolAlarmbit.Byte1;
									msg.Data[5] = BPToStringHVolAlarmbit.Byte2;
									msg.Data[6] = BPToStringHVolAlarmbit.Byte3;
									msg.Data[7] = BPToStringHVolAlarmbit.Byte4;
									msg.MessageNum = 8;	
	
									WaitClrAckFlag[msg.DeviceID - 1].HVolAlarmFlag = 1;
								
									CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1);			/*转发Clr给Array*/	
								#endif
								}
							}
							break;
							case Alarm_CellUnderVoltage:
							{
								 BPAWEHappenedFlag.LVoltageAlarm = 1;
								BPToStringLVolAlarmbit.BPbyte &= ~(1 << (msg.Data[3]-1)) ;
								{
								
									BpToStringFlag[msg.DeviceID - 1].LVolAlarmFlag = 0;
									
									BPLVolAlarmFlag[msg.Data[3]-1] = 0;
//									if(BPLVolAlarmFlag[msg.Data[3]-1] == 0 && StringLVolAlarmFlag[msg.Data[3]-1] == 0)
//										BPLVolAlarmbit->BPbyte &= ~(1 << (msg.DeviceID - 1)); 
									/* 转发 clear 给 array*/
								#if 0	
									MsgGroupID = ID_Alarm;
									msg.Data[1] = AlarmClr;
									msg.Data[3] = 1;
									msg.Data[4] = BPToStringLVolAlarmbit.Byte1;
									msg.Data[5] = BPToStringLVolAlarmbit.Byte2;
									msg.Data[6] = BPToStringLVolAlarmbit.Byte3;
									msg.Data[7] = BPToStringLVolAlarmbit.Byte4;		
									msg.MessageNum = 8;
									WaitClrAckFlag[msg.DeviceID - 1].LVolAlarmFlag = 1;
									
									CAN_make_send_from_BP (msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1)	;			/*转发Clr给Array*/
								#endif
								}
							}
							break;
							case Alarm_CellOverTemp:
							{
								BPAWEHappenedFlag.HTempAlarm = 1;
								BPToStringHTempAlarmbit.BPbyte &= ~(1 <<(msg.Data[3]-1)) ; 
								{
									
									BpToStringFlag[msg.DeviceID - 1].HTempAlarmFlag = 0;
									BPHTempAlarmFlag[msg.Data[3]-1] = 0;
								#if 0
//									if(BPHTempAlarmFlag[msg.Data[3]-1] == 0 && StringHTempAlarmFlag[msg.Data[3]-1] == 0)								
//										BPHTempAlarmbit->BPbyte &= ~(1 << (msg.DeviceID - 1));
									MsgGroupID = ID_Alarm;
									msg.Data[1] = AlarmClr;
									msg.Data[3] = 1;
									msg.Data[4] = BPToStringHTempAlarmbit.Byte1;
									msg.Data[5] = BPToStringHTempAlarmbit.Byte2;
									msg.Data[6] = BPToStringHTempAlarmbit.Byte3;
									msg.Data[7] = BPToStringHTempAlarmbit.Byte4;	
									msg.MessageNum = 8;	
									WaitClrAckFlag[msg.DeviceID - 1].HTempAlarmFlag = 1;
									CAN_make_send_from_BP (msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1)	;			/*转发Clr给Array*/		
								#endif
								}
							}
							break;
							case Alarm_CellUnderTemp:
							{
								BPAWEHappenedFlag.LTempAlarm = 1;
								BPToStringLTempAlarmbit.BPbyte &= ~(1 << (msg.Data[3]-1)) ; 
								{
									
									BpToStringFlag[msg.DeviceID - 1].LTempAlarmFlag = 0;
									BPLTempAlarmFlag[msg.Data[3]-1] = 0;
								#if 0
//									if(BPLTempAlarmFlag[msg.Data[3]-1] == 0 && StringLTempAlarmFlag[msg.Data[3]-1] == 0)								
//										BPLTempAlarmbit->BPbyte &= ~(1 << (msg.DeviceID - 1));
									MsgGroupID = ID_Alarm;
									msg.Data[1] = AlarmClr;
									msg.Data[3] = 1;
									msg.Data[4] = BPToStringLTempAlarmbit.Byte1;
									msg.Data[5] = BPToStringLTempAlarmbit.Byte2;
									msg.Data[6] = BPToStringLTempAlarmbit.Byte3;
									msg.Data[7] = BPToStringLTempAlarmbit.Byte4;
									msg.MessageNum = 8;	
									WaitClrAckFlag[msg.DeviceID - 1].LTempAlarmFlag = 0;
									CAN_make_send_from_BP (msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1)	;			/*转发Clr给Array*/		
								#endif
								}
							}
							break;
							
							case Alarm_HighChargeRate:
							{
								BPAWEHappenedFlag.HChargeRateAlarm = 1;
								BPToStringChargeRateAlarmbit.BPbyte &= ~(1 << (msg.Data[3]-1)) ; 
								
								OverChargeCount = 0;
								BpToStringChargeDisChargeFLag.ChargeStringCurrentAlarmFlag = 0;
								BpToStringFlag[msg.DeviceID - 1].ChargeStringCurrentAlarmFlag = 0;
								#if 0
								MsgGroupID = ID_Alarm;
								msg.Data[1] = AlarmClr;
								msg.Data[3] = 1;
								msg.MessageNum = 8;	
								WaitClrAckFlag[msg.DeviceID - 1].ChargeStringCurrentAlarmFlag = 1;
								CAN_make_send_from_BP (msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1)	;			/*转发Clr给Array*/
								#endif
							}
							break;
							case Alarm_HighDischargeRate:
							{
								BPAWEHappenedFlag.HDischargeRateAlarm = 1;
								BPToStringDischargeRateAlarmbit.BPbyte &= ~(1 << (msg.Data[3]-1)) ; 
								
								OVerDischargeCount = 0;
								BpToStringChargeDisChargeFLag.DischargeStringCurrentAlarmFlag = 0;
								BpToStringFlag[msg.DeviceID - 1].DischargeStringCurrentAlarmFlag = 0;
								#if 0
								MsgGroupID = ID_Alarm;
								msg.Data[1] = AlarmClr;
								msg.Data[3] = 1;
								msg.MessageNum = 8;	
						        WaitClrAckFlag[msg.DeviceID - 1].DischargeStringCurrentAlarmFlag = 1;
								CAN_make_send_from_BP (msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1)	;			/*转发Clr给Array*/								
								#endif
							}
							break;
													
						}						
					}
				}
                 
                break;
            case MsgID_Warning: /**/
                {
					if(msg.Data[1] == WarningSet)
					{ 
						uint8_t ByteTemp = msg.Data[1];
						MsgGroupID = ID_Warning;
						msg.Data[1] = WarningSetAck;
						CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID);     // 返回
						msg.Data[1] = ByteTemp;
						switch (msg.Data[0])
						{
							case Warning_CellOverVoltage:
							{
								BPAWEHappenedFlag.HVoltageWarning = 1;
								BPToStringHVolWarningbit.BPbyte |= 1 << (msg.Data[3]-1) ;                             /*ACK 标志位为1，继续发，同时改变 5byte-8byte*/
								if(CalculateCountOfState(BPToStringHVolWarningbit.BPbyte) <= 1)
								{
									BpToStringFlag[msg.DeviceID - 1].HVolWarningFlag = 1;	
									BPHVolWarningFlag[msg.Data[3]-1] = 1;									
									msg.Data[1] = Update;
									msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
									msg.Data[3] = 0x01;     
									msg.Data[4] = BPToStringHVolWarningbit.Byte1;
									msg.Data[5] = BPToStringHVolWarningbit.Byte2;
									msg.Data[6] = BPToStringHVolWarningbit.Byte3;
									msg.Data[7] = BPToStringHVolWarningbit.Byte4;
									msg.MessageNum = 8;
									MsgGroupID = ID_Warning;										
									CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID - 1   );			
								}
							}
								break;
							case Warning_CellUnderVoltage:
							{
								BPAWEHappenedFlag.LVoltageWarning = 1;
								BPToStringLVolWarningbit.BPbyte |= 1 <<(msg.Data[3]-1) ; 
								if(CalculateCountOfState(BPToStringLVolWarningbit.BPbyte) <=1)
								{
									BpToStringFlag[msg.DeviceID - 1].LVolWarningFlag = 1;
	
									BPLVolWarningFlag[msg.Data[3]-1] = 1;
									
									msg.Data[1] = Update;
									msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
									msg.Data[3] = 0x01;     
									msg.Data[4] = BPToStringLVolWarningbit.Byte1;
									msg.Data[5] = BPToStringLVolWarningbit.Byte2;
									msg.Data[6] = BPToStringLVolWarningbit.Byte3;
									msg.Data[7] = BPToStringLVolWarningbit.Byte4;
									msg.MessageNum = 8;
									MsgGroupID = ID_Warning;	
									
									CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID - 1   );	
								}
							}
								break;
							case Warning_CellOverTemp:
							{
								BPAWEHappenedFlag.LTempWarning = 1;
								BPToStringHTempWarningbit.BPbyte |= 1 << (msg.Data[3]-1) ; 
								if(CalculateCountOfState(BPToStringHTempWarningbit.BPbyte) <= 1)
								{
									BpToStringFlag[msg.DeviceID - 1].HTempWarningFlag = 1;
	
									BPHTempWarningFlag[msg.Data[3]-1] = 1;
	
									msg.Data[1] = Update;
									msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
									msg.Data[3] = 0x01;     
									msg.Data[4] = BPToStringHTempWarningbit.Byte1;
									msg.Data[5] = BPToStringHTempWarningbit.Byte2;
									msg.Data[6] = BPToStringHTempWarningbit.Byte3;
									msg.Data[7] = BPToStringHTempWarningbit.Byte4;
									msg.MessageNum = 8;
									MsgGroupID = ID_Warning;	
									
									CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID - 1   );	
								}
							}
								break;
							case Warning_CellUnderTemp:
							{
								BPAWEHappenedFlag.LTempWarning = 1;
								BPToStringLTempWarningbit.BPbyte |= 1 << (msg.Data[3]-1) ; 
								if(CalculateCountOfState(BPToStringLTempWarningbit.BPbyte) <= 1)
								{
									BpToStringFlag[msg.DeviceID - 1].LTempWarningFlag = 1;
//									BPLTempWarningbit->BPbyte |= 1 << (msg.DeviceID - 1);
									BPLTempWarningFlag[msg.Data[3]-1] = 1;
									
									msg.Data[1] = Update;
									msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
									msg.Data[3] = 0x01;     
									msg.Data[4] = BPToStringLTempWarningbit.Byte1;
									msg.Data[5] = BPToStringLTempWarningbit.Byte2;
									msg.Data[6] = BPToStringLTempWarningbit.Byte3;
									msg.Data[7] = BPToStringLTempWarningbit.Byte4;
									msg.MessageNum = 8;
									MsgGroupID = ID_Warning;	
									
									CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID - 1   );	
								}
							}
								break;

							case Warning_HighChargeRate:
							{
								BPAWEHappenedFlag.HChargeRateWarning = 1;
								BPToStringChargeRateWarningbit.BPbyte |= 1 << (msg.Data[3]-1);
								if(CalculateCountOfState(BPToStringChargeRateWarningbit.BPbyte) <= 1)
								{
									BpToStringChargeDisChargeFLag.ChargeStringCurrentWarningFlag = 1;
									BpToStringFlag[msg.DeviceID - 1].ChargeStringCurrentWarningFlag = 1;
									
									msg.Data[1] = Update;
									msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
									msg.Data[3] = 0x01;     
									msg.Data[4] = BPToStringChargeRateWarningbit.Byte1;
									msg.Data[5] = BPToStringChargeRateWarningbit.Byte2;
									msg.Data[6] = BPToStringChargeRateWarningbit.Byte3;
									msg.Data[7] = BPToStringChargeRateWarningbit.Byte4;
									msg.MessageNum = 8;
									MsgGroupID = ID_Warning;	
									
									CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID - 1); 
	
								}
							}
								break;
							case Warning_HighDischargeRate:
							{
								BPAWEHappenedFlag.HDischargeRateWarning = 1;
								BPToStringDischargeRateWarningbit.BPbyte |= 1 << (msg.Data[3]-1);
								if(CalculateCountOfState(BPToStringDischargeRateWarningbit.BPbyte) <= 1)
								{
								
									BpToStringChargeDisChargeFLag.DischargeStringCurrentWarningFlag = 1;
									BpToStringFlag[msg.DeviceID - 1].DischargeStringCurrentWarningFlag = 1;
										
									msg.Data[1] = Update;
									msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
									msg.Data[3] = 0x01;     
									msg.Data[4] = BPToStringDischargeRateWarningbit.Byte1;
									msg.Data[5] = BPToStringDischargeRateWarningbit.Byte2;
									msg.Data[6] = BPToStringDischargeRateWarningbit.Byte3;
									msg.Data[7] = BPToStringDischargeRateWarningbit.Byte4;
									msg.MessageNum = 8;
									MsgGroupID = ID_Warning;	
									
									CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID - 1); 									
								}
							}
								break;
 						
						}
					}
					if(msg.Data[1] == WarningClr)        /*---接收到 Clr消息---*/
					{
						uint8_t ByteTemp = msg.Data[1];
						MsgGroupID = ID_Warning;
						msg.Data[1] = WarningClrAck;
						CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID );     // 返回
						msg.Data[1] = ByteTemp;
						switch (msg.Data[0])
						{
							case Warning_CellOverVoltage:
							{
								
								 
								BPToStringHVolWarningbit.BPbyte &= ~(1 << (msg.Data[3]-1)) ;                             /*ACK 标志位为1，继续发，同时改变 5byte-8byte*/
								{
								
									BpToStringFlag[msg.DeviceID - 1].HVolWarningFlag = 0;
									/*转发 clear给 Array */
								#if 0	
//									BPHVolWarningbit->BPbyte &= ~(1 << (msg.DeviceID - 1));
									MsgGroupID = ID_Warning;
									msg.Data[1] = WarningClr;
									msg.Data[4] = BPToStringHVolWarningbit.Byte1;
									msg.Data[5] = BPToStringHVolWarningbit.Byte2;
									msg.Data[6] = BPToStringHVolWarningbit.Byte3;
									msg.Data[7] = BPToStringHVolWarningbit.Byte4;
									msg.MessageNum = 8;	
									WaitClrAckFlag[msg.DeviceID - 1].HVolWarningFlag = 1;
									
									CAN_make_send_from_BP (msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1)	;			/*转发Clr给Array*/	
								#endif
								}
							}
							break;
							case Warning_CellUnderVoltage:
							{
								 
								BPToStringLVolWarningbit.BPbyte &= ~(1 << (msg.Data[3]-1)) ;
								{
								
									BpToStringFlag[msg.DeviceID - 1].LVolWarningFlag = 0;
//									BPLVolWarningbit->BPbyte &= ~(1 << (msg.DeviceID - 1));
								#if 0	
									/* 转发 clear 给 array*/
									MsgGroupID = ID_Warning;
									msg.Data[1] = WarningClr;
									msg.Data[4] = BPToStringLVolWarningbit.Byte1;
									msg.Data[5] = BPToStringLVolWarningbit.Byte2;
									msg.Data[6] = BPToStringLVolWarningbit.Byte3;
									msg.Data[7] = BPToStringLVolWarningbit.Byte4;		
									msg.MessageNum = 8;	
									
									WaitClrAckFlag[msg.DeviceID - 1].LVolWarningFlag = 1;
									CAN_make_send_from_BP (msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1)	;			/*转发Clr给Array*/	
								#endif
								}
							}
							break;
							case Warning_CellOverTemp:
							{
								 
								BPToStringHTempWarningbit.BPbyte &= ~(1 << (msg.Data[3]-1)) ; 
								{
								 	
									BpToStringFlag[msg.DeviceID - 1].HTempWarningFlag = 0;
									
//									BPHTempWarningbit->BPbyte &= ~(1 << (msg.DeviceID - 1));
								#if 0
									MsgGroupID = ID_Warning;
									msg.Data[1] = WarningClr;
									msg.Data[4] = BPToStringHTempWarningbit.Byte1;
									msg.Data[5] = BPToStringHTempWarningbit.Byte2;
									msg.Data[6] = BPToStringHTempWarningbit.Byte3;
									msg.Data[7] = BPToStringHTempWarningbit.Byte4;	
									msg.MessageNum = 8;							
									
									WaitClrAckFlag[msg.DeviceID - 1].HTempWarningFlag = 1;
									CAN_make_send_from_BP (msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1)	;			/*转发Clr给Array*/		
								#endif
								}
							}
							break;
							case Warning_CellUnderTemp:
							{
								 
								BPToStringLTempWarningbit.BPbyte &= ~(1 << (msg.Data[3]-1)) ;
								{
									
									BpToStringFlag[msg.DeviceID - 1].LTempWarningFlag = 0;
									
//									BPLTempWarningbit->BPbyte &= ~(1 << (msg.DeviceID - 1));
								#if 0	
									MsgGroupID = ID_Warning;
									msg.Data[1] = WarningClr;
									msg.Data[4] = BPToStringLTempWarningbit.Byte1;
									msg.Data[5] = BPToStringLTempWarningbit.Byte2;
									msg.Data[6] = BPToStringLTempWarningbit.Byte3;
									msg.Data[7] = BPToStringLTempWarningbit.Byte4;	
									msg.MessageNum = 8;	
									
									WaitClrAckFlag[msg.DeviceID - 1].LTempWarningFlag = 0;
									CAN_make_send_from_BP (msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1)	;			/*转发Clr给Array*/	
								#endif
								}
							}
							break;
 
							case Warning_HighChargeRate:
							{
								 
								BPToStringChargeRateWarningbit.BPbyte &= ~(1 << (msg.Data[3]-1)) ; 
								BpToStringChargeDisChargeFLag.ChargeStringCurrentWarningFlag = 0;
								BpToStringFlag[msg.DeviceID - 1].ChargeStringCurrentWarningFlag = 0;
							#if 0	
								MsgGroupID = ID_Warning;
								msg.Data[1] = WarningClr;
								msg.MessageNum = 8;	
								WaitClrAckFlag[msg.DeviceID - 1].ChargeStringCurrentWarningFlag = 1;
								CAN_make_send_from_BP (msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1)	;			/*转发Clr给Array*/
							#endif
							}
							break;
							case Warning_HighDischargeRate:
							{
								 
								BPToStringDischargeRateWarningbit.BPbyte &= ~(1 << (msg.Data[3]-1)) ; 
								BpToStringChargeDisChargeFLag.DischargeStringCurrentWarningFlag = 0;
								BpToStringFlag[msg.DeviceID - 1].DischargeStringCurrentWarningFlag = 0;
							#if 0
								MsgGroupID = ID_Warning;
								msg.Data[1] = WarningClr;
								msg.MessageNum = 8;	
								WaitClrAckFlag[msg.DeviceID - 1].DischargeStringCurrentWarningFlag = 1;
								CAN_make_send_from_BP (msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1)	;			/*转发Clr给Array*/								
							#endif
							}
							break;
 							
						}										
					
					 
                }
			}
                //???
                break;
			case MsgID_Error:
				{
						if(msg.Data[1] == ErrorSet)
						{
							uint8_t ByteTemp = msg.Data[1];
							MsgGroupID = ID_Error;
							
							msg.Data[1] = ErrorSetAck;							
														
							CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID);	
							msg.Data[1] = ByteTemp;	
							
							switch(msg.Data[0])
							{
								case Error_BMCOFFLINE:
								{
									BPToStringCellOfflineErrorbit.BPbyte |= 1 << (msg.Data[3]-1);
									BpToStringFlag[msg.DeviceID - 1].BmcOfflineErrorFlag = 1;

									msg.Data[1] = ErrorSet;
									msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
									msg.Data[3] = 0x01;     
									msg.Data[4] = BPToStringCellOfflineErrorbit.Byte1;
									msg.Data[5] = BPToStringCellOfflineErrorbit.Byte2;
									msg.Data[6] = BPToStringCellOfflineErrorbit.Byte3;
									msg.Data[7] = BPToStringCellOfflineErrorbit.Byte4;
									msg.MessageNum = 8;
									MsgGroupID = ID_Error;	
									
									CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID - 1   );										
								}
								break;
								case Error_HighCellVolDelta:
								{
									BPAWEHappenedFlag.HCellVolDeltaError = 1;
									BPToStringHighCellDeltaErrorbit.BPbyte |= 1 << (msg.Data[3]-1);
									
									if(CalculateCountOfState(BPToStringHighCellDeltaErrorbit.BPbyte) <= 1)
									{
										BpToStringFlag[msg.DeviceID - 1].HighCellDeltaErrorFlag = 1;
										
										msg.Data[1] = Update;
										msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
										msg.Data[3] = 0x01;     
										msg.Data[4] = BPToStringHighCellDeltaErrorbit.Byte1;
										msg.Data[5] = BPToStringHighCellDeltaErrorbit.Byte2;
										msg.Data[6] = BPToStringHighCellDeltaErrorbit.Byte3;
										msg.Data[7] = BPToStringHighCellDeltaErrorbit.Byte4;
										msg.MessageNum = 8;
										MsgGroupID = ID_Error;	
										
										CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID - 1   );	
									}
								}
								break;
								
							}
							
						}
						else if(msg.Data[1] == ErrorClr)
						{
							uint8_t ByteTemp = msg.Data[1];
							MsgGroupID = ID_Error;
							
							msg.Data[1] = ErrorClrAck;							
														
							CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID);	
							msg.Data[1] = ByteTemp;	
							
							switch(msg.Data[0])
							{
								case Error_BMCOFFLINE:
								{
									BPToStringCellOfflineErrorbit.BPbyte &= ~(1 << (msg.Data[3]-1));
									BpToStringFlag[msg.DeviceID - 1].BmcOfflineErrorFlag = 0;
									MsgGroupID = ID_Error;
									
									msg.Data[1] = ErrorClr;
									msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
									msg.Data[3] = 1;
									msg.Data[4] = BPToStringCellOfflineErrorbit.Byte1;
									msg.Data[5] = BPToStringCellOfflineErrorbit.Byte2;
									msg.Data[6] = BPToStringCellOfflineErrorbit.Byte3;
									msg.Data[7] = BPToStringCellOfflineErrorbit.Byte4;
									msg.MessageNum = 8;	
									
									WaitClrAckFlag[msg.DeviceID - 1].Bmc_OffLineErrorFlag = 1;
									
									CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1);
								}
								
								break;
								case Error_HighCellVolDelta:
								{
									 
									BPToStringHighCellDeltaErrorbit.BPbyte &= ~(1 << (msg.Data[3]-1));
									{
										
										BpToStringFlag[msg.DeviceID - 1].HighCellDeltaErrorFlag = 0;																				
									#if 0	
										MsgGroupID = ID_Error;
										msg.Data[1] = ErrorClr;
										msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
										msg.Data[3] = 1;
										msg.Data[4] = BPToStringHighCellDeltaErrorbit.Byte1;
										msg.Data[5] = BPToStringHighCellDeltaErrorbit.Byte2;
										msg.Data[6] = BPToStringHighCellDeltaErrorbit.Byte3;
										msg.Data[7] = BPToStringHighCellDeltaErrorbit.Byte4;									
										msg.MessageNum = 8;	
										WaitClrAckFlag[msg.DeviceID - 1].HighCellDeltaErrorFlag = 1;
										CAN_make_send_from_BP (msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID - 1)	;			/*转发Clr给Array*/	
									#endif
									}
								}
								break;
							}							
					
						}
						
				}
				break;
			case MsgIDTo_BPWarrantyVoltageTemp:  //0xC0
				{	
                    mStringData.bp[msg.DeviceID-1].maxCellVolt = ((msg.Data[1])&0xff)  | ((msg.Data[0]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].minCellVolt = ((msg.Data[3])&0xff)  | ((msg.Data[2]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].maxCellTemp = ((msg.Data[5])&0xff)  | ((msg.Data[4]&0xff) << 8);
                    mStringData.bp[msg.DeviceID-1].minCellTemp = ((msg.Data[7])&0xff)  | ((msg.Data[6]&0xff) << 8);
                  

                    BPCounter = msg.DeviceID - 1;
                    send_autodata_Ex(20);  					
				
				}
				break;
            default:
               
                break;
        }
		CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    }  // while can_rx_fifo.count
}

 
 

void CAN_RX_Process2(void) // Array 发给 String
{
    MESSAGE msg;
    BUFFER volatile *rxbuf;  // pointers into FIFO for active entries during processing
 
    // while there is a message to process
    while (can_rx_fifo2.count) 
	{

        rxbuf = fifo_get(&can_rx_fifo2);  // decrements fifo.count

        // split received ID into bit fields
        DisassembleID(&msg, rxbuf->ID);
        msg.MessageNum  = rxbuf->DLC;   // For incoming messages, MessageNum holds payload length.
        memcpy(msg.Data, (void *)&rxbuf->data.bytes, rxbuf->DLC);
        #if(1)
        {
            MESSAGE message;
            DisassembleID(&message, rxbuf->ID);
            message.MessageNum  = rxbuf->DLC;   // For incoming messages, MessageNum holds payload length.
            memcpy(message.Data, (void *)&rxbuf->data.bytes, rxbuf->DLC);
             
        }
        #endif
		
		 
		 
		 
        if( (msg.StringID == Self_ID) ||  (msg.StringID == 0x00))
        {
//            if(msg.DeviceType != ARRAY_TYPE)
//            {
//                printf("= error device type to process, msg.DeviceType = 0x%02x(Self_ID = 0x%02x) .... \r\n", msg.DeviceType, Self_ID);
//                break;
//            }
            //break;
			CanDelay(Self_ID * 11000); 
            switch(msg.MessageID)
            {
               
				CanDelay(Self_ID * 10000); 
				case RelayQueryMsgID:
				{
					MsgGroupID = ID_Command;
					{
						MESSAGE TemperatureForRelay;
						TemperatureForRelay.Data[0] = mStringData.highCellTemp >> 8;
						TemperatureForRelay.Data[1] = mStringData.highCellTemp ;
						TemperatureForRelay.Data[2] = mStringData.lowCellTemp >> 8;
						TemperatureForRelay.Data[3] = mStringData.lowCellTemp ;						
						TemperatureForRelay.Data[4] = mStringData.averageCellTemp >> 8;
						TemperatureForRelay.Data[5] = mStringData.averageCellTemp ;		
						TemperatureForRelay.Data[6] = mStringData.mode;
						 		
						
					 
						CanDelay(Self_ID * 11000);
						CAN_make_send_2_Array (TemperatureForRelay.Data, 7, RelayQueryMsgID);
					}
				}
				break;
				// 0x80 ---------------------------------------------------------------------------
                case CMD_ID_SERVER_COMMAND_ACTION :
					  CanDelay(Self_ID * 10000);
					  MsgGroupID = ID_Upgrade;
                     if((msg.Data[COMMAND_ACTION_CMD_TYPE_INDEX] == COMMAND_ACTION_CMD_TYPE_UPGRADE_BYPASS_BP) ||
                                                            (msg.Data[COMMAND_ACTION_CMD_TYPE_INDEX] == COMMAND_ACTION_CMD_TYPE_UPGRADE_JUST_STRING))
                     {
                         
						 CanServerCommandActionProcess((uint8_t*)msg.Data, msg.MessageNum);
                     }
                     else if(CanServerGetBPBypassMode())
                     {
                         
						 CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID);      
                     }
                     else
                     {
                         CanServerCommandActionProcess((uint8_t*)msg.Data, msg.MessageNum);
                     }
                    
                    
                    break;
                // 0x79 ---------------------------------------------------------------------------
                case CMD_ID_SERVER_FIRMWARE_INFO :
				 	CanDelay(Self_ID * 10000);
					MsgGroupID = ID_Upgrade;
                    if(CanServerGetBPBypassMode())
                    {
                        CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID);                        
                        
                    }
                    else
                    {
                        CanServerFirmwareInfoProcess((uint8_t*)msg.Data, msg.MessageNum);
                    }
                    break;
                // 0x78 ---------------------------------------------------------------------------
                case CMD_ID_SERVER_UPGRADE_WRITE_MEMORY :
				 	CanDelay(Self_ID * 10000);
					MsgGroupID = ID_Upgrade;
                    if(CanServerGetBPBypassMode())
                    {
                        CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID);                        
                    }
                    else
                    {
                        CanUpgradeServerProcess((uint8_t*)msg.Data, msg.MessageNum);
                    }
                    break;
 
				case MsgID_QueryAllAWE:  /*询问所有bp 的 alarm/warning情况*/
				{
					MESSAGE msg;
					MsgGroupID = ID_Command;
					msg.MessageID = 0x06;
					msg.DeviceID = 0x00;
					msg.MessageNum = 0;
					CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID);
					ReSendStringAWE();
					ReCheckContactorStatus();
					ReCheckLossCommunication();
				}
				break;
				
				case ControlBalancing:  /*add by jason, add the balancing controler function*/
				{
					
					MsgGroupID = ID_Data;

					CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID);					
					
					
				}
				break;
			/*add by jason @ 2016年7月21日10:54:13
			  新增warrantyTracker消息，转发给bp controller
				
			*/				
				case MsgID_AllWarrantyTracker: 
				{

					MsgGroupID = ID_Warranty;								
					msg.MessageNum = 0;
					CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID);	
					
				
				}
		    /*add by jason ,2016-11-8 9:34:11*/		
			case MsgID_BlueBoxSN:
			{
		
				MsgGroupID = ID_Configure;
				msg.MessageNum = 0;
				CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID);				
			
			}
			break;
			case MsgID_BPSN:
			{
				MsgGroupID = ID_Configure;
				msg.MessageNum = 0;
				CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID);					
			}
			break;
			case MsgID_BPUID:
			{
				MsgGroupID = ID_Configure;
				msg.MessageNum = 0;
				CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID);					
				
			}
			break;				 
				case MsgID_PCSState:
				{
					
					if(msg.Data[0] == 0x00)
					{
						mStringData.mode = Idle_Mode;
						if(Count.Alarm == 0 && Count.BpToStringAlarm == 0)
						{
//							contactor1_on();
//							contactor2_on();
						}
					}
					else if(msg.Data[0] == 0x0F )
						mStringData.mode = Charge_Mode;
					else if(msg.Data[0] == 0xF0)
						mStringData.mode = DisCharge_Mode;
						
				}
				break;
				
				case 0xD8:
					ContactorAllTurnOff();
				break;
				case 0xD9:
					ContactorAllTurnOn();
				break;
				
				case EnterIntoMaintenanceMode:               /*进入维护模式*/
					GetMaintencanceModeFlagFromSys = 1;
					if(msg.Data[0] == 1)
					{
						MaintenanceModeFlag = 1;
						ContactorAllTurnOff();
					}
					else
					{
						MaintenanceModeFlag = 0;
					}
					 
				break;
				
				case MsgID_ContactorClosedPermission:
 				 if(msg.Data[0] == Self_ID)	
				 {
					 ContactorClosedPermissionFlag = msg.Data[1];
					 
					send_autodata_Ex(35);
				 }
				break;
				 
				case DCBUS_Voltage:
					
					send_autodata_Ex(35);                   //发送DCBUS总线电压
				
				break;
                case MsgID_Alarm:             /* 接收到Array回复的Alarm的消息，string不需要继续发送Alarm指令，如果没有收到继续发送alarm指令给Array*/
					
/*----------old version from bp alarm msg-------------------*/
//				if(msg.Data[1] == AlarmAck)
//						ToArray.AlarmFlag = 0;
/*---------------------------------------------------------*/					
				if(msg.Data[0] == Alarm_CellOverVoltage)	
				{
					if(msg.Data[1] == Update)
					{
						int i = 0;
						for(i = 0 ;i < BP_NUMBER;i++)
						{
							SysStateAckFlag[i].HVolAlarmAckFlag = 1;             /*表示 收到 array的 ack信息  数组下标从0开始,string 不再发送*/
							BpToStringFlag[i].HVolAlarmFlag = 0;                  /*不再转发bp 发送的消息*/
						}
					}										
				}
	
				if(msg.Data[0] == Alarm_CellUnderVoltage) 
				{
					if(msg.Data[1] == Update)
					{
						int i = 0;
						for(i = 0;i < BP_NUMBER;i++)
						{
							SysStateAckFlag[i].LVolAlarmAckFlag = 1;
							BpToStringFlag[i].LVolAlarmFlag = 0;            /*不再转发bp 发送的消息*/
						}
					}    					
				}

				if(msg.Data[0] == Alarm_CellOverTemp)
				{
					if(msg.Data[1] == Update)
					{
						int i = 0;
						for(i = 0;i < BP_NUMBER;i++)
						{
							SysStateAckFlag[i].HTempAlarmAckFlag = 1;
							BpToStringFlag[i].HTempAlarmFlag = 0;         /*不再转发bp 发送的消息*/
						}
					}
//					if(msg.Data[1] == AlarmClrAck)
//					{
//						WaitClrAckFlag[msg.DeviceID - 1].HTempAlarmFlag = 0;
//					}
					
				}

				if(msg.Data[0] == Alarm_CellUnderTemp)
				{
					if(msg.Data[1] == Update)
					{
						int i = 0;
						for(i = 0;i < BP_NUMBER;i++)
						{
							SysStateAckFlag[i].LTempAlarmAckFlag = 1;
							BpToStringFlag[i].LTempAlarmFlag = 0;        /*不再转发bp 发送的消息*/
						}
					}

				}

				if(msg.Data[0] == Alarm_HighChargeRate)
				{
					if(msg.Data[1] == AlarmAck)
					{
						int i = 0;
						for(i = 0;i < BP_NUMBER;i++)
						{
							SysChargeDisChargeFlag.ChargeStringCurrentAlarmAckFlag = 1;
							BpToStringChargeDisChargeFLag.ChargeStringCurrentAlarmFlag = 0;
							BpToStringFlag[i].ChargeStringCurrentAlarmFlag = 0;
						}
					}					
				}
				if(msg.Data[0] == Alarm_HighDischargeRate)
				{
					if(msg.Data[1] == AlarmAck)
					{
						int i = 0;
						for(i = 0;i < BP_NUMBER;i++)
						{
							SysChargeDisChargeFlag.DischargeStringCurrentAlarmAckFlag= 1;
							BpToStringChargeDisChargeFLag.DischargeStringCurrentAlarmFlag = 0; /*不再转发bp发来的alarm/warning/error*/
							BpToStringFlag[i].DischargeStringCurrentAlarmFlag = 0;
						}
					}
				}
	 
				 
				
				if(msg.Data[0] == Alarm_BMCOFFLINE)
				{
					if(msg.Data[1] == AlarmAck)
					{
						BMC_OFFLINE_AlarmSetAck_Flag[msg.Data[3] - 1][msg.Data[4] - 1] = 1;
						
					}
				}
				break;		
				
				case MsgID_Warning:
					
/*-----------old version from bp Warning msg---------------*/					
					if(msg.Data[1] == WarningClr)
						ToArray.WarningFlag = 0;
/*---------------------------------------------------------*/	
					if(msg.Data[0] == Warning_CellOverVoltage)
					{
						if(msg.Data[1] == Update)
						{
							int i = 0;
							for(i = 0;i < BP_NUMBER;i++)
							{
								SysStateAckFlag[i].HVolWarningAckFlag = 1;
								BpToStringFlag[i].HVolWarningFlag = 0;
							}							 
						}
					}
 
					if(msg.Data[0] == Warning_CellUnderVoltage)
					{
						if(msg.Data[1] == Update)
						{
							int i = 0;
							for( i = 0 ;i < BP_NUMBER;i++)
							{
								SysStateAckFlag[i].LVolWarningAckFlag = 1;
								BpToStringFlag[i].LVolWarningFlag = 0; 
							}
						}					
					}	
					
					if(msg.Data[0] == Warning_CellOverTemp)
					{
						if(msg.Data[1] == Update)
						{
							int i = 0;
							for(i = 0;i < BP_NUMBER;i++)
							{
								SysStateAckFlag[i].HTempWarningAckFlag = 1;
								BpToStringFlag[i].HTempWarningFlag = 0;
							} 
						}						
					}
					
					if(msg.Data[0] == Warning_CellUnderTemp)
					{
						if(msg.Data[1] == Update)
						{
							int i = 0;
							for(i = 0;i < BP_NUMBER;i++)
							{
								SysStateAckFlag[i].LTempWarningAckFlag = 1;
								BpToStringFlag[i].LTempWarningFlag = 0; 
							}
						}					
					}	
					
					if(msg.Data[0] == Warning_HighCellTempDelta)
					{
						if(msg.Data[1] == Update)
						{
							int i = 0;
							for(i = 0;i < BP_NUMBER;i++)
							{
								SysStateAckFlag[i].HCellDeltaTempWarningAckFlag = 1;
								BpToStringFlag[i].HCellDeltaTempWarningFlag = 0;
							}
						}						
					}

					if(msg.Data[0] == Warning_HighChargeRate)
					{
						if(msg.Data[1] == WarningSetAck)
						{
							int i = 0;
							for(i = 0;i < BP_NUMBER;i++)
							{
								SysChargeDisChargeFlag.ChargeStringCurrentWarningAckFlag = 1;
								BpToStringFlag[i].ChargeStringCurrentWarningFlag = 0;
								BpToStringChargeDisChargeFLag.ChargeStringCurrentWarningFlag = 0;
							}
						}						
					}
					if(msg.Data[0] == Warning_HighDischargeRate)
					{
						if(msg.Data[1] == WarningSetAck)
						{
							int i = 0;
							for(i = 0 ;i < BP_NUMBER;i++)
							{
								SysChargeDisChargeFlag.DischargeStringCurrentWarningAckFlag= 1;
								BpToStringFlag[i].DischargeStringCurrentWarningFlag = 0;
								BpToStringChargeDisChargeFLag.DischargeStringCurrentWarningFlag = 1;
							}
						}
 						
					}	
						
					if(msg.Data[0] == Warning_ContactorOpen)
					{
						if(msg.Data[1] == WarningSetAck)
						{
							ContactorOpenWarningAck = 1;
						}
						if(msg.Data[1] == WarningClrAck)
						{
							ContactorOpenwarningClrAck = 0;           /*contactor warning clr ack = 0 不发，*/	
						}
							
						
					}
					 
/*2016年2月24日14:28:27 add by Jason*/
					if(msg.Data[0] == Warning_BMCOFFLINE)
					{
						if(msg.Data[1] == WarningSetAck)
						{
							BMC_OFFLINE_WarningSetAck_Flag[msg.Data[3] - 1][msg.Data[4] - 1] = 1;
							
						}
						if(msg.Data[1] == WarningClrAck)
						{
							BMC_OFFLINE_WarningClrAck_Flag[msg.Data[3] - 1][msg.Data[4] - 1] = 1;
							
						}
					}					
					break;
				
				case MsgID_Error:
/*-----------receive the erro ack from array controller -----------------*/						
 
/*---------------------------------------------------------*/	
					if(msg.Data[0] == Error_ContactorError)
					{
						if(msg.Data[1] == ErrorSetAck)
						{
							ContactorError_Flag = 1;
						}					
					}
					if(msg.Data[0] == Error_LossOfCommunication)
					{
						if(msg.Data[1] == Update)
						{
							int i = 0;
							for(i = 0; i < BP_NUMBER;i++)
							{
								ErrorLoseofCommunicationFlag[i] = 1;	
								SysStateAckFlag[i].BPLoseofCommunicationErrorAckFlag = 1;
							}
						}
						if(msg.Data[1] == ErrorClrAck)
						{
							WaitClrAckFlag[msg.DeviceID - 1].BPLoseofCommunicationErrorFlag = 0;
						}
							
					}
					if(msg.Data[0] == Error_BMCOFFLINE )          
					{						
						if(msg.Data[1] == ErrorSetAck)
						{
							SysStateAckFlag[msg.DeviceID - 1].BmcOfflineErrorAckFlag = 1;
							BpToStringFlag[msg.DeviceID - 1].BmcOfflineErrorFlag = 0;            /*不再转发bp 发送的消息*/
						}    
						if(msg.Data[1] == ErrorClrAck)
						{
							WaitClrAckFlag[msg.DeviceID - 1].Bmc_OffLineErrorFlag = 0;
						}
					}
					if(msg.Data[0] == Error_HighCellVolDelta)
					{
						if(msg.Data[1] == Update)
						{
							int i = 0;
							for(i = 0;i < BP_NUMBER;i++)
							{
								BpToStringFlag[i].HighCellDeltaErrorFlag = 0;
							}
						}
					}
					if(msg.Data[0] == Error_BpBooter)
					{
						if(msg.Data[1] == Update)
						{
							int i = 0;
							for(i = 0;i < BP_NUMBER;i++)
							{
								SysStateAckFlag[i].BPBootError = 1;
							}
						}
							
					}
					if(msg.Data[0] == Error_HighAveVoltaDelta)
					{
						if(msg.Data[1] == Update)
						{
							int i = 0;
							for(i = 0;i < BP_NUMBER;i++)
							{
								
								SysStateAckFlag[i].HighAveVoltaDeltaErrorAckFlag = 1;
							}
							
						}
						
					}
					break;
				
   	
				case MsgID_StringState:              /*更新string的状态和模式
					
														state：Normal/Warning/Error/Alarm
														Mode:  Idle/Charging/Discharging 
													 */
                    send_autodata_Ex(21);	    
					break;
				case MsgID_StringShuntDown:
					
					CopyAHKWHToEEPROMTemp(); 
				    StringEEProm_Write(AHKWHEEPageNumber);
					
					break;
				case MsgID_SystemBPSelect:         /*array  发给 string 选中的bp消息*/
					if(msg.Data[1] == getSelfID()) /*如果是本stringID*/
					{
					 	mStringData.ArrayID = msg.Data[0];	
						mStringData.SelectedBPID = msg.Data[2]; 
						
					}
					send_autodata_Ex_2_BP(MsgID_SystemBPSelect,mStringData.SelectedBPID);       /*String 发送选中的消息给BP // MsgID,BPID*/
					
					break;
				case MsgID_GetBPExtremeVol:  //  0x42 从bp获取电压消息
					send_autodata_Ex_2_BP(MsgIDToBP_BPExtremeVoltage, msg.DeviceID);
					break;
				case MsgID_GetBPExtremeTemp:  // 0x43 从bp获取温度消息
					send_autodata_Ex_2_BP(MsgIDToBP_BPExtremeTemp, msg.DeviceID);
					break;              
				case MsgID_StringHeartBeat:  // 0x46, 收到 array的 heartbeat 信息。
					send_autodata_Ex(22);
					break;
				
				
				case MsgID_UpdateSelectedBPMsg:// 回送所有 有关 string的消息  ；0x50
					                 
					mStringData.ArrayID = msg.Data[0];
					send_autodata_Ex(4); // 51            
					send_autodata_Ex(2); // 52
					send_autodata_Ex(3); // 53
					send_autodata_Ex(5); // 54
					send_autodata_Ex(6); // 55
					send_autodata_Ex(7); // 56
					send_autodata_Ex(35); // 56
					break;
				case MsgID_StringCurrent: //获取电流相关信息 0x51
					send_autodata_Ex(4);
				    break;
				case MsgID_StringVoltageInfo:// string电压 extreme 值
					send_autodata_Ex(2);
					break;
				case MsgID_StringTempInfo: //string温度 extreme值
                    send_autodata_Ex(3);
					break;
				case MsgID_StringPower: // kW ,kWH
                    send_autodata_Ex(5);
					break;
				case MsgID_StringBalChargingTimeLeakageCurrent:
                    send_autodata_Ex(6);
					break;
				case MsgID_StringUsageHistory:
                     send_autodata_Ex(7);
					break;	

				case MsgID_SetNumbersofBPs: // 获取bp数量

					break;
					
/*读取被选中的BP相关信息*/
				case MsgID_SelectedBPID: //  返回所有有关BP的信息 发送 
					
					send_autodata_Ex_2_BP(MsgIDToBP_bpStatus, msg.DeviceID);  // return bp status
					send_autodata_Ex_2_BP(MsgIDToBP_CellVol1To4, msg.DeviceID);//0xA2 cell1-cell12 voltage
					send_autodata_Ex_2_BP(MsgIDToBP_CellTemp1To4, msg.DeviceID); // 0xA6  cell1-cell12 temp
				    send_autodata_Ex_2_BP(MsgIDToBP_BPBalState, msg.DeviceID);  //0xAA
					send_autodata_Ex_2_BP(MsgIDToBP_SelectBPWarrantyTracker, msg.DeviceID); // 0xAB
					send_autodata_Ex_2_BP(MsgIDTo_BPWarrantyVoltageTemp,msg.DeviceID); // 0xAC
					send_autodata_Ex_2_BP(MsgIDToBP_MaxChargeDischargeCurrent, msg.DeviceID); //0xAD
				
					break;
				case MsgID_SelectedBPStatus:  //0xA1
					send_autodata_Ex_2_BP(MsgIDToBP_bpStatus, msg.DeviceID);
					break;
				case MsgID_SelectedBPCellVol1To4:              //查询选中的BP电池1-4电压 0xA2
					send_autodata_Ex_2_BP(MsgIDToBP_CellVol1To4, msg.DeviceID);//发给BP	0x48				
					break;
				case MsgID_SelectedBPCellVol5To8:              //查询选中的BP电池5-8电压       
					send_autodata_Ex_2_BP(MsgIDToBP_CellVol5To8, msg.DeviceID);//发给BP	0x49
					break;
				case MsgID_SelectedBPCellVol9To12:            //查询选中的BP电池9-12电压
					send_autodata_Ex_2_BP(MsgIDToBP_CellVol9To12, msg.DeviceID);//发给BP 0x4A
					break;
				case MsgID_SelectedBPCellVol13To16:           //保留 
					send_autodata_Ex_2_BP(MsgIDToBP_CellVol13To16, msg.DeviceID);//发给BP 0x4A
					break;
				case MsgID_SelectedBPCellTemp1To4:
					send_autodata_Ex_2_BP(MsgIDToBP_CellTemp1To4, msg.DeviceID);
					break;
				case MsgID_SelectedBPCellTemp5To8:
					send_autodata_Ex_2_BP(MsgIDToBP_CellTemp5To8, msg.DeviceID);
					break;
				case MsgID_SelectedBPCellTemp9To12:
					send_autodata_Ex_2_BP(MsgIDToBP_CellTemp9To12, msg.DeviceID);
					break;
				case MsgID_SelectedBPCellTemp13To16:         //保留
					send_autodata_Ex_2_BP(MsgIDToBP_CellTemp13To16, msg.DeviceID);
					break;	
				case MsgID_SelectedBPBalState:          // 获取selectedbp balance状态 0xAA
					send_autodata_Ex_2_BP(MsgIDToBP_BPBalState, msg.DeviceID);
					break;			
				case MsgID_SelectedBPWarrantyTracker: //0xAB,回复生产年月日等等
					send_autodata_Ex_2_BP(MsgIDToBP_SelectBPWarrantyTracker, msg.DeviceID);
					break;
				case MsgID_SelectedBPCellWarrantyVolTemp: //0xAC
					//send_autodata_Ex_2_BP(MsgIDToBP_BPExtremeVoltage, msg.DeviceID);      //0x42 从string获取 最高最低平均电压，
					//send_autodata_Ex_2_BP(MsgIDToBP_BPExtremeTemp, msg.DeviceID);       // 0x43 最高最低平均温度
					send_autodata_Ex_2_BP(MsgIDTo_BPWarrantyVoltageTemp,msg.DeviceID);
					break;
				case MsgID_SelectedBPMAXChargeDisachargeCurrent: //   0xAD 获取最大充电电流，放电电流
					send_autodata_Ex_2_BP(MsgIDToBP_MaxChargeDischargeCurrent, msg.DeviceID); //0xDE
					break;
				
 
				
/*This Instruction use to test*/
				case 0xBB:                          /*读取 PVolt 和 NVolt的AD值，接地状态*/
					send_autodata_Ex(31);
					break;
				case Debug_BPAlarmCount:
					send_autodata_Ex(33);
				
					break;
				case Debug_BPWarningCount:
					
					send_autodata_Ex(34);
					break;
				
/*-------------------------------------------------------------------------*/                
     
                
                case 0x20:      
                    //mStringData.balancingTargetVoltage = ((msg.Data[0])&0xff)  | ((msg.Data[1]&0xff) << 8);
                    //mStringData.balancingTargetSOC = ((msg.Data[2])&0xff)  | ((msg.Data[3]&0xff) << 8);
 
				
//                    DecodeFour16BitData(msg.Data, 
//                                        &(mStringData.balancingTargetVoltage),
//                                        &(mStringData.balancingTargetSOC),
//                                        (uint16_t*)&(mStringData.stringCurrent),
//                                        &(mStringData.ah));		
					DecodeTwo16BitData(msg.Data,&(mStringData.balancingTargetVoltage),&(mStringData.balancingTargetSOC));
					
					ArrayStringSendTargetVol = mStringData.balancingTargetVoltage;
					TargetValueCounter = 0;              /*add by jason,如果收到就清零*/
					//ToArray.Led1sCount = 0;
                    TargetValueCountTimeOutFlag = 0;
                    break;  
                
                case CAN_MSG_OVER_VOLTAGE_SETTING:
//                    DecodeFour16BitData(msg.Data, 
//                                        &(mConfig.alarmWarningConfig.overCellVoltageAlarmSet),
//                                        &(mConfig.alarmWarningConfig.overCellVoltageAlarmClr),
//                                        &(mConfig.alarmWarningConfig.overCellVoltageWarningSet),
//                                        &(mConfig.alarmWarningConfig.overCellVoltageWarningClr));
//    
//                    CAN_make_send_2_BP(rxbuf);
				
	                DecodeFour16BitData(msg.Data, 
                                    &(SetClrValue.SetCellOverVoltageAlarm),
                                    &(SetClrValue.ClrCellOverVoltageAlarm),
                                    &(SetClrValue.SetCellOverVoltageWarning),
                                    &(SetClrValue.ClrCellOverVoltageWarning));
					rxbuf->ID = 0x15040e0;
					CAN_make_send_2_BP(rxbuf);
					
					CopyOverVolThresholdValueToEEPROM();
					StringEEProm_Write(OverVoltagePageNumber);
					
					EEPROM_WriteFlag.OverVoltageEEPROMFlag = 1;
					 
					CopyEEPROMWriteFlag();
					StringEEProm_Write(EEPROMFlagPageNumber);
					InitCmpValueofAlarmWarning();
					send_autodata_Ex(23); // 配置成功返回
                    break;
                case CAN_MSG_UNDER_VOLTAGE_SETTING:
//                    DecodeFour16BitData(msg.Data, 
//                                        &(mConfig.alarmWarningConfig.underCellVoltageAlarmSet),
//                                        &(mConfig.alarmWarningConfig.underCellVoltageAlarmClr),
//                                        &(mConfig.alarmWarningConfig.underCellVoltageWarningSet),
//                                        &(mConfig.alarmWarningConfig.underCellVoltageWarningClr));
// 
//                    CAN_make_send_2_BP(rxbuf);
	                DecodeFour16BitData(msg.Data, 
                                    &(SetClrValue.SetCellUnderVoltageAlarm),
                                    &(SetClrValue.ClrCellUnderVoltageAlarm),
                                    &(SetClrValue.SetCellUnderVoltageWarning),
                                    &(SetClrValue.ClrCellUnderVoltageWarning));
					rxbuf->ID = 0x15040e1;
					CAN_make_send_2_BP(rxbuf);	
				
					CopyUnderVolThresholdValueToEEPROM();
					StringEEProm_Write(UnderVoltagePageNumber);	
					EEPROM_WriteFlag.UnderVoltageEEPROMFlag = 1;
					CopyEEPROMWriteFlag();
					StringEEProm_Write(EEPROMFlagPageNumber);
					InitCmpValueofAlarmWarning();
					send_autodata_Ex(24); // 配置成功返回
                    break;
                case CAN_MSG_OVER_TEMP_SETTING:
//                    DecodeFour16BitData(msg.Data, 
//                                        (uint16_t*)&(mConfig.alarmWarningConfig.overCellTempAlarmSet),
//                                        (uint16_t*)&(mConfig.alarmWarningConfig.overCellTempAlarmClr),
//                                        (uint16_t*)&(mConfig.alarmWarningConfig.overCellTempWarningSet),
//                                        (uint16_t*)&(mConfig.alarmWarningConfig.overCellTempWarningClr));
// 
//                    CAN_make_send_2_BP(rxbuf);
	                DecodeFour16BitData(msg.Data, 
                                    (uint16_t *)&(SetClrValue.SetCellOverTempAlarm),
                                    (uint16_t *)&(SetClrValue.ClrCellOverTempAlarm),
                                    (uint16_t *)&(SetClrValue.SetCellOverTempWarning),
                                    (uint16_t *)&(SetClrValue.ClrCellOverTempWarning));
					rxbuf->ID = 0x15040e2;
					CAN_make_send_2_BP(rxbuf);	
				
					CopyOverTempThresholdValueToEEPROM();
					StringEEProm_Write(OverTempPageNumber);
				
					EEPROM_WriteFlag.OverTempEEPROMFlag = 1;
					CopyEEPROMWriteFlag();
					StringEEProm_Write(EEPROMFlagPageNumber);
					InitCmpValueofAlarmWarning();
					send_autodata_Ex(25); // 配置成功返回
                    break;
                case CAN_MSG_UNDER_TEMP_SETTING:
//                    DecodeFour16BitData(msg.Data, 
//                                        (uint16_t*)&(mConfig.alarmWarningConfig.underCellTempAlarmSet),
//                                        (uint16_t*)&(mConfig.alarmWarningConfig.underCellTempAlarmClr),
//                                        (uint16_t*)&(mConfig.alarmWarningConfig.underCellTempWarningSet),
//                                        (uint16_t*)&(mConfig.alarmWarningConfig.underCellTempWarningClr));
// 
//                    CAN_make_send_2_BP(rxbuf);
	                DecodeFour16BitData(msg.Data, 
                                    (uint16_t *)&(SetClrValue.SetCellUnderTempAlarm),
                                    (uint16_t *)&(SetClrValue.ClrCellUnderTempAlarm),
                                    (uint16_t *)&(SetClrValue.SetCellUnderTempWarning),
                                    (uint16_t *)&(SetClrValue.ClrCellUnderTempWarning));
					rxbuf->ID = 0x15040e3;
					CAN_make_send_2_BP(rxbuf);	
					CopyUnderTempThresholdValueToEEPROM();
					StringEEProm_Write(UnderTempPageNumber);
					EEPROM_WriteFlag.UnderTempEEPROMFlag = 1;
					CopyEEPROMWriteFlag();				
					StringEEProm_Write(EEPROMFlagPageNumber);
					InitCmpValueofAlarmWarning();
					send_autodata_Ex(26); // 配置成功返回
                    break;
                case CAN_MSG_HCDT_SETTING://High Cell Delta Temp(Diff Temp)
//                    DecodeFour16BitData(msg.Data, 
//                                        &(mConfig.alarmWarningConfig.HCDTAlarmSet),
//                                        &(mConfig.alarmWarningConfig.HCDTAlarmClr),
//                                        &(mConfig.alarmWarningConfig.HCDTWarningSet),
//                                        &(mConfig.alarmWarningConfig.HCDTWarningClr));
// 
//                    CAN_make_send_2_BP(rxbuf);
	                DecodeFour16BitData(msg.Data, 
                                    (uint16_t *)&(SetClrValue.SetHighCellTempDeltaAlarm),
                                    (uint16_t *)&(SetClrValue.ClrHighCellTempDeltaAlarm),
                                    (uint16_t *)&(SetClrValue.SetHighCellTempDeltaWarning),
                                    (uint16_t *)&(SetClrValue.ClrHighCellTempDeltaWarning));
					rxbuf->ID = 0x15040e4;
					CAN_make_send_2_BP(rxbuf);	
					CopyHighTempDeltaThresholdValueToEEPROM();
					StringEEProm_Write(HighTempDeltaPageNumber);
					EEPROM_WriteFlag.HighDeltaTempEEPROMFlag = 1;
					CopyEEPROMWriteFlag();
					StringEEProm_Write(EEPROMFlagPageNumber);
					InitCmpValueofAlarmWarning();
					send_autodata_Ex(27); // 配置成功返回
                    break;
                case CAN_MSG_HCTR_SETTING://High Cell Temp Rise
//                    DecodeFour16BitData(msg.Data, 
//                                        &(mConfig.alarmWarningConfig.HCTRAlarmSet),
//                                        &(mConfig.alarmWarningConfig.HCTRAlarmClr),
//                                        &(mConfig.alarmWarningConfig.HCTRWarningSet),
//                                        &(mConfig.alarmWarningConfig.HCTRWarningClr));
//					CAN_make_send_2_BP(rxbuf);
	                DecodeFour16BitData(msg.Data, 
                                    (uint16_t *)&(SetClrValue.SetHighCellTempRiseAlarm),
                                    (uint16_t *)&(SetClrValue.ClrHighCellTempRiseAlarm),
                                    (uint16_t *)&(SetClrValue.SetHighCellTempRiseWarning),
                                    (uint16_t *)&(SetClrValue.ClrHighCellTempRiseWarning));
					rxbuf->ID = 0x15040e5;
					CAN_make_send_2_BP(rxbuf);	
					CopyHighTempRiseThresholdValueToEEPROM();
					StringEEProm_Write(HighTempRisePageNumber);
					EEPROM_WriteFlag.HighTempRiseEEPROMFlag = 1;
					CopyEEPROMWriteFlag();
					StringEEProm_Write(EEPROMFlagPageNumber);
					InitCmpValueofAlarmWarning();
					send_autodata_Ex(28); // 配置成功返回
                    break;
                case CAN_MSG_HCR_TEMP_SETTING://High Charge Rate
//                    DecodeFour16BitData(msg.Data, 
//                                        &(mConfig.alarmWarningConfig.HCRAlarmSet),
//                                        &(mConfig.alarmWarningConfig.HCRAlarmClr),
//                                        &(mConfig.alarmWarningConfig.HCRWarningSet),
//                                        &(mConfig.alarmWarningConfig.HCRWarningClr));
//					CAN_make_send_2_BP(rxbuf);
	                DecodeFour16BitData(msg.Data, 
                                    (uint16_t *)&(SetClrValue.SetHighChargeRateAlarm),
                                    (uint16_t *)&(SetClrValue.ClrHighChargeRateAlarm),
                                    (uint16_t *)&(SetClrValue.SetHighChargeRateWarning),
                                    (uint16_t *)&(SetClrValue.ClrHighChargeRateWarning));
					rxbuf->ID = 0x15040e6;
					CAN_make_send_2_BP(rxbuf);	
					CopyChargeCurrentThresholdValueEEPROM();
					StringEEProm_Write(HighChargeCurrentPageNumber);
					EEPROM_WriteFlag.HighChargeRateEERPOMFlag = 1;
					CopyEEPROMWriteFlag();
					StringEEProm_Write(EEPROMFlagPageNumber);
					InitCmpValueofAlarmWarning();
					send_autodata_Ex(29); // 配置成功返回
                    break;
                case CAN_MSG_HDR_TEMP_SETTING://High Discharge Rate
//                    DecodeFour16BitData(msg.Data, 
//                                        &(mConfig.alarmWarningConfig.HDRAlarmSet),
//                                        &(mConfig.alarmWarningConfig.HDRAlarmClr),
//                                        &(mConfig.alarmWarningConfig.HDRWarningSet),
//                                        &(mConfig.alarmWarningConfig.HDRWarningClr));
//					CAN_make_send_2_BP(rxbuf);
	                DecodeFour16BitData(msg.Data, 
                                    (uint16_t *)&(SetClrValue.SetHighDisChargeRateAlarm),
                                    (uint16_t *)&(SetClrValue.ClrHighDisChargeRateAlarm),
                                    (uint16_t *)&(SetClrValue.SetHighDisChargeRateWarning),
                                    (uint16_t *)&(SetClrValue.ClrHighDisChargeRateWarning));
					rxbuf->ID = 0x15040e7;
					CAN_make_send_2_BP(rxbuf);
					CopyDischargeCurrentThresholdValueEEPROM();
					StringEEProm_Write(HighDisChargeCurrentPageNumber);		
					EEPROM_WriteFlag.HighDisChargeRateEEPROMFlag = 1;
					CopyEEPROMWriteFlag();
					StringEEProm_Write(EEPROMFlagPageNumber);
					InitCmpValueofAlarmWarning();
					send_autodata_Ex(30); // 配置成功返回				
                    break;
             
                
                default:
                    printf("= not implement CAN_RX_Process2 yet, message id = 0x%02x\r\n", msg.MessageID);
                    break;
            }
        }
        else
        {
            if(msg.MessageNum == 0)                               // Array 读取EEPROM配置值
			{
				switch(msg.MessageID)
				{
					case CAN_MSG_OVER_VOLTAGE_SETTING:
						send_autodata_Ex(23);
					break;
				
					case CAN_MSG_UNDER_VOLTAGE_SETTING:
						send_autodata_Ex(24);
					break;
					
					case CAN_MSG_OVER_TEMP_SETTING:
						send_autodata_Ex(25);
					break;
					
					case CAN_MSG_UNDER_TEMP_SETTING:
						send_autodata_Ex(26);
					break;
					
					case CAN_MSG_HCDT_SETTING:
						send_autodata_Ex(27);
					break;
					
					case CAN_MSG_HCTR_SETTING:
						send_autodata_Ex(28);
					break;
					
					case CAN_MSG_HCR_TEMP_SETTING:
						send_autodata_Ex(29);
					break;
					
					case CAN_MSG_HDR_TEMP_SETTING:
						send_autodata_Ex(30);
					break;
				}
				
			}
	//		printf("= not this String to process, msg.StringID = 0x%02x(local id:0x%02x) .... \r\n", msg.StringID, Self_ID);
        }

    }  // while can_rx_fifo.count
}

//#if(0)// by sam
/*******************************************************************************
* Function Name  : AssembleID
* Description    : Combined dispersive ID into a 32-bit ID inside MESSAGE, exclude identifiers and long-distance data
* Input          : message pointer, internally include various of ID, ID is used for saving 32-bit pointer for combined ID
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-07
* Modifier         : None
* Date           : None
*******************************************************************************/
uint32_t AssembleID(MESSAGE* message)
{
#if(USE_CAN_40)
   
    return ((message->ReserveField & 0x7) << 26) |         
           ((message->MessageType & 0x0f) << 22) |
           ((message->DeviceType & 0x0f)  <<  18) |
           ((message->StringID & 0xf) << 14) |
           ((message->DeviceID & 0x3f) << 8) |
           (message->MessageID);
#else
    return ((message->ReserveField & 0x1) << 28) |
           ((message->StringID & 0xf) << 24) |
           ((message->MessageType & 0x0f) << 20) |
           ((message->DeviceType & 0x0f)  <<  16) |
           (message->DeviceID << 8) |
           (message->MessageID);
#endif
}
//#endif

/*******************************************************************************
* Function Name  : DisassembleID
* Description    : Disassemble 32-bit ID into different ID
* Input          : message pointer, include variety of ID address; ID: ID pointer wait for disassembling
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-07
* Modifier         : None
* Date           : None
*******************************************************************************/
void DisassembleID(MESSAGE* message, uint32_t ID)
{    
#if(USE_CAN_40)
    message->ReserveField =(uint8_t)(ID >> 26)& 0x7;//Reserved field 3bit
    message->MessageType = (uint8_t)(ID >>22)& 0x0f; //Message type	4bit
    message->DeviceType = (uint8_t)(ID >>18)& 0x0f;//Device type     4bit
    message->StringID =(uint8_t)(ID >> 14)& 0xf;//Reserved field 4bit
    message->DeviceID =    (uint8_t)(ID >> 8)& 0x3f;//Device ID     6bit
    message->MessageID = (uint8_t)(ID);//Message ID    8bit
#else
    message->ReserveField =(uint8_t)(ID >> 28)& 0x1;//Reserved field 1bit
    message->StringID =(uint8_t)(ID >> 24)& 0xf;//Reserved field 4bit

    message->MessageType = (uint8_t)(ID >>20)& 0x0f; //Message type	4bit
    message->DeviceType = (uint8_t)(ID >>16)& 0x0f;//Device type     4bit
    message->DeviceID =    (uint8_t)(ID >> 8);//Device ID     8bit
    message->MessageID = (uint8_t)(ID);//Message ID    8bit
#endif
}
/*******************************************************************************
* Function Name  : GET_Device_ID
* Description    : Get device according to the setting and DIP switch setting
*                  If the setting is not 0 or 0xff , use DIP switch setting
* Input          : None
* Output         : None
* Return         : None
* Author         : Sam Gao
* Date             : 2010-12-08
* Modifier         : None
* Date           : None
*******************************************************************************/

//uint8_t GET_Device_ID(void){
//    return 1;   // todo: wtf do we do for this board's ID?
//}

/*******************************************************************************
* Function Name  : Save_Data_BeforeShutDown
* Description    : Save before turn off
* Input          :
* Output         : None
* Return         : 1-byte, sucessful or not
* Author         : Sam
* Date             : 2010-12-14
* Modifier         : None
* Date           : None
* Remark         : Might write less parameter, only WH AH need be updated
*******************************************************************************/
uint8_t Save_Data_BeforeShutDown()
{
#if 0
    uint32_t CRCResult;

    if (System.ee_state == 0) {

        I2C_EE_BufferWrite((uint8_t *)(CellPara), EE_CELLPARA_ADDR, sizeof(CellPara));
        //        I2C_EE_BufferUpdate((uint8_t *)(CellPara),EE_CELLPARA_ADDR,sizeof(CellPara));
        I2C_EE_BufferWrite((uint8_t *)(&PackPara),EE_PACKPARA_ADDR,sizeof(PackPara));
        //        I2C_EE_BufferUpdate((uint8_t *)(&PackPara),EE_PACKPARA_ADDR,sizeof(PackPara));
        CRCResult = Para_CRC_Verify();//Calculate CRC value
        I2C_EE_BufferWrite((uint8_t*)(&CRCResult),EE_CRC_VERIFY_ADDR,4);

    }
    else return 1;
#endif
    return 0;
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn        void USB_LP_CAN_RX0_IRQHandler(void)
 * @brief    USB low priority interrupt or CAN FIFO0 receive interrupt handler, for CAN receiving data.
 * @param    None
 * @retval    None
 */
#ifndef STM32F10X_CL
void USB_LP_CAN1_RX0_IRQHandler(void)
#else
void CAN1_RX0_IRQHandler(void)
#endif
{
    uint8_t Msg_Num;
    BUFFER volatile *buf;
    //printf("CAN1_RX0: %d...\n", can_rx_fifo.count);


    Msg_Num=((uint8_t)(CAN1->RF0R))& 0x03;//Query the current message number
    while (Msg_Num != 0) {    // While there is another message to process
        buf = fifo_alloc(&can_rx_fifo);  // allocate a free location in the rx FIFO
        // If buf == NULL (0), there is nowhere to put the data - we have overrun.
        // todo: where to store critical error counts
        if (buf) {
            buf->ID = CAN1->sFIFOMailBox[0].RIR >> 3;
            buf->DLC = (uint8_t)(CAN1->sFIFOMailBox[0].RDTR);
            buf->data.longwords.DataL=CAN1->sFIFOMailBox[0].RDLR;
            buf->data.longwords.DataH=CAN1->sFIFOMailBox[0].RDHR;
        }
        else
        {
            //rxOverFolwCounter++;
			CAN1->RF0R |= 0x20; //  set bit of RFOM0 release FIFO 0 box
			Msg_Num = CAN1->RF0R & 0x03;  // re-query hardware for next full input mailbox
			return;
        }

        CAN1->RF0R |= 0x20; //  set bit of RFOM0 release FIFO 0 box
        Msg_Num = CAN1->RF0R & 0x03;  // re-query hardware for next full input mailbox
    }
}


void CAN2_RX0_IRQHandler(void)

{
    uint8_t Msg_Num;
    BUFFER volatile *buf;

    Msg_Num=((uint8_t)(CAN2->RF0R))& 0x03;//Query the current message number
    while (Msg_Num != 0) {    // While there is another message to process
        buf = fifo_alloc(&can_rx_fifo2);  // allocate a free location in the rx FIFO
        // If buf == NULL (0), there is nowhere to put the data - we have overrun.
        // todo: where to store critical error counts
        if (buf) {
            buf->ID = CAN2->sFIFOMailBox[0].RIR >> 3;
            buf->DLC = (uint8_t)(CAN2->sFIFOMailBox[0].RDTR);
            buf->data.longwords.DataL=CAN2->sFIFOMailBox[0].RDLR;
            buf->data.longwords.DataH=CAN2->sFIFOMailBox[0].RDHR;
        }
        else
        {
            //rxOverFolwCounter2++;
			CAN2->RF0R |= 0x20; //  set bit of RFOM0 release FIFO 0 box
			Msg_Num = CAN2->RF0R & 0x03;  // re-query hardware for next full input mailbox
			return;
        }

        CAN2->RF0R |= 0x20; //  set bit of RFOM0 release FIFO 0 box
        Msg_Num = CAN2->RF0R & 0x03;  // re-query hardware for next full input mailbox
    }
}

/**
 * @fn        uint8_t CAN_Send(BUFFER *SendData)
 * @brief    CAN send program, write message into sender. (Successful or not is not known)
 * @param    SendData, pointer to a BUFFER structure
 * @param    idx, hardware index of buffer to use, returned by can_buf_avail, 0..2
 * @retval   None
 */
void CAN_Send(BUFFER volatile *SendData, uint8_t idx)
{
    CAN_TxMailBox_TypeDef *mb;  // pointer to mailbox to use

    mb = CAN1->sTxMailBox + idx;

    mb->TIR = ((SendData->ID)<< 3)| 0x04;//loading identifiter (non-set sending request)
    /* Set up the DLC */
    mb->TDTR = (SendData->DLC)&0x0f;
    /* Set up the data field */
    mb->TDLR = SendData->data.longwords.DataL;         // load bytes 0..3
    mb->TDHR = SendData->data.longwords.DataH;         // load bytes 4..7
    /* Request transmission */
    mb->TIR |= 0x01;
    
#if(0)
    printf("\r\n[CAN_Send]: ID = 0x%08x, DLC = 0x%08x, DataH = 0x%08x, DataL = 0x%08x,\n", 
                        SendData->ID, SendData->DLC, SendData->data.longwords.DataH, SendData->data.longwords.DataL);

    
    {
        MESSAGE message;
        DisassembleID(&message, SendData->ID);


        printf("        StringID = 0x%02x, MessageType = 0x%02x, DeviceType = 0x%02x,\r\n        DeviceID = 0x%02x, MessageID = 0x%02x\n\r\n", 
                                message.StringID, message.MessageType, message.DeviceType, message.DeviceID, message.MessageID);

        printf("        data =<0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x>\r\n"
                                            , SendData->data.bytes[0], SendData->data.bytes[1], SendData->data.bytes[2], SendData->data.bytes[3], 
                                                SendData->data.bytes[4], SendData->data.bytes[5], SendData->data.bytes[6], SendData->data.bytes[7]);
        #if(0)
        {
            int i =0 ;
            for (i=0; i<SendData->DLC; i++)
            {
                printf("  [%d]: 0x%02x\n", i, SendData->data.bytes[i]);
            }
        }
        #endif
    }
#endif
    //printf("CAN_Send: ID = 0x%08x, DLC = 0x%08x, DataL = 0x%08x, DataH = 0x%08x,\n", SendData->ID, SendData->DLC, SendData->data.longwords.DataL, SendData->data.longwords.DataH);

}

void CAN_Send2(BUFFER volatile *SendData, uint8_t idx)
{
    CAN_TxMailBox_TypeDef *mb;  // pointer to mailbox to use

    mb = CAN2->sTxMailBox + idx;

    mb->TIR = ((SendData->ID)<< 3)| 0x04;//loading identifiter (non-set sending request)
    /* Set up the DLC */
    mb->TDTR = (SendData->DLC)&0x0f;
    /* Set up the data field */
    mb->TDLR = SendData->data.longwords.DataL;         // load bytes 0..3
    mb->TDHR = SendData->data.longwords.DataH;         // load bytes 4..7
    /* Request transmission */
    mb->TIR |= 0x01;
    
 
     

}
/*-----------------------------------------------------------------------------*/
/**
 * @fn        void CAN_TX(void)
 * @brief    AN_TX_buffer's sending program, invoke CAN_Send to send, and update the buffer
 * @param    None
 * @retval    None
 */
void CAN_TX(void)
{
    uint8_t hwbuf;
    BUFFER volatile *buf;

 
    
    
    // this loop terminates when there are no more messages in the tx fifo
    // or no more available hardware buffers.
    while (can_tx_fifo.count)// && (hwbuf != 0xff)) 
    {
        
        do
        {
            hwbuf = can_buf_avail();      // 0xff on none available
            if(hwbuf == 0xff)
            {
                //printf("  [CAN_TX]: hwbuf == 0xff, ignore send\n\r");
                //printf(".");
            }
        }while(hwbuf == 0xff);
        //printf("  [CAN_TX]: send (can_tx_fifo.count = %d) ...\n\r", can_tx_fifo.count);
        buf = fifo_get(&can_tx_fifo);   // decrements fifo.count
        CAN_Send(buf, hwbuf);           // makes hwbuf unavailable
        totalSendCounter++;
    }
}


void CAN_TX2(void)
{
    uint8_t hwbuf;
    BUFFER volatile *buf;
    uint16_t totalSendCounter2Tmp = totalSendCounter2;

 
    
    cpMemoryData();
    
    // this loop terminates when there are no more messages in the tx fifo
    // or no more available hardware buffers.
    while (can_tx_fifo2.count)// && (hwbuf != 0xff)) 
    {        
        do
        {
            hwbuf = can_buf_avail2();      // 0xff on none available
            if(hwbuf == 0xff)
            {
                //printf("  [CAN_TX]: hwbuf == 0xff, ignore send\n\r");
                //printf(".");
            }
        }while(hwbuf == 0xff);
        //printf("  [CAN_TX]: send (can_tx_fifo.count = %d) ...\n\r", can_tx_fifo.count);
        buf = fifo_get(&can_tx_fifo2);   // decrements fifo.count
        CAN_Send2(buf, hwbuf);           // makes hwbuf unavailable
		
        totalSendCounter2++;
		
    }
    if(totalSendCounter2Tmp != totalSendCounter2)
        printf("[==CAN_TX2==]:  totalSendCounter2 = %d ...\n\r", totalSendCounter2);
}

/**
  * Close the Doxygen can_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen can group.
  *    @}
*/

/* End of can.c */
