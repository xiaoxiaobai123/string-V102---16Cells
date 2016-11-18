/**
  ******************************************************************************
  * @file        usart.c
  * @author      Nystrom Engineering
  * @version     2.0.0
  * @date        Jan 27, 2014
  * @copyright   Powin Energy
  * @brief       This file provides all the usart functions.
  * @page        usart_page Universal Serial Asynchronous Receiver Transmitter (USART) Functions
  * @section     usart_intro Introduction
  * @par
  * The USART functions are used for the Universal Serial Asynchronous Receiver Transmitter (USART).  \n
  *
  ******************************************************************************
  */
/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
#include "BESS_3-0_AHM_device_lib_V_1-0.h"

/** @addtogroup usart usart
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Private Types                                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup usart_Private_Types
  * @{
  */

/**
  * Close the Doxygen usart__Private_Types group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private_Constants                                                           */
/*-----------------------------------------------------------------------------*/
/** @defgroup usart_Private_Constants
  * @{
  */

/**
  * Close the Doxygen usart_Private_Constants group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private Variable Declarations                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup usart_Private_Variables
  * @{
  */

/**
  * Close the Doxygen usart_Private_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private Function Declarations                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup usart_Private_Functions
  * @{
  */

/**
  * Close the Doxygen usart_Private_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup usart_Exported_Variables
  * @{
  */
/*USART receive and send buffer info is showing below, all the received interruption function will not be processed, in case of delay*/
uint8_t USART_RX_Buffer[USART_RX_Buffer_MAX][32];
//UART info received buffer, stored 10 USART info
//13 Byte, 4 for identifier, 1 for data length, 8 for data
uint8_t USART_RX_WaitLoad;
//The USART info under processing, stands for the first element of the array
//When data processing is over, this data will +1 or =0
uint8_t USART_RX_WaitProcess;
//The idle USART receiving buffer, 0-UART_RX_Buffer_MAX-1£¬stands for this array is used for store USART info
//Data +1 or =0 when this frame received
//UART_RX_Processing cannot exceed USART_RX_Empty
uint8_t USART_RX_Buffer_State[USART_RX_Buffer_MAX];
//The current receving buffer state
//00:EMPTY
//01:RECEIVEING
//02:RECEIVED
//03:PROCESSING

uint8_t USART_TX_Buffer[USART_TX_Buffer_MAX][32];
//USART info sending buffer
uint8_t USART_TX_WaitLoad;
//Current sending buffer for USART
uint8_t USART_TX_WaitSend;
//Current idle USART sending buffer
uint8_t USART_TX_Buffer_State[USART_TX_Buffer_MAX];
//Current status for sending buffer
//00:EMPTY
//01:LOADING
//02:LAODED
//03:SENDING

/**
  * Close the Doxygen usart_Exported_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Function Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup usart_Exported_Functions
  * @{
  */
/*******************************************************************************
* Function Name  : USART_Data_Init
* Description    : USART data initialization
* Input          : None
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-20
* Modifier         : None
* Date           : None
*******************************************************************************/
void USART_Data_Init(void)
{
    uint8_t i,j;
    for(i=0; i<USART_RX_Buffer_MAX; i++) {
        for(j=0; j<32; j++)
            USART_RX_Buffer[i][j]=0;

        USART_RX_WaitLoad = 0;
        USART_RX_WaitProcess =0;
        USART_RX_Buffer_State[i] = RX_WAITLOAD;
    }

    for(i=0; i<USART_TX_Buffer_MAX; i++) {
        for(j=0; j<32; j++)
            USART_TX_Buffer[i][j]=0;

        USART_TX_WaitLoad = 0;
        USART_TX_WaitSend = 0;
        USART_TX_Buffer_State[i] = TX_WAITLOAD;
    }

}

/*******************************************************************************
* Function Name  : USART_Config
* Description    : USART initialization
* Input          : None
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-20
* Modifier         : None
* Date           : None
*******************************************************************************/
void USART_config(void)
{
    USART_Device_Init();
    USART_Data_Init();
}

/*******************************************************************************
* Function Name  : USART_Device_Init
* Description    : Configuring USART1
* Input          : None
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-17
* Modifier         : None
* Date           : None
*******************************************************************************/
void USART_Device_Init(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* USART1 and USART2 configuration ------------------------------------------------------*/
    /* USART and USART2 configured as follow:
    - BaudRate = 57600 baud
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
    */


    //Configure USART1 TXD RXD
    //GPIOA_9  USART1_TXD, 2M speed,  reuse push-pull output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    //GPIOA_10  USART1_RXD   pull up open-drain input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//USART receives 0 interruption
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//Preemption priority 2 (low)
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate = 57600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* Configure USART1 */
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}
/*******************************************************************************
* Function Name  : USART_Load_Message
* Description    : Put received data into Message structure
* Input          : MESSAGE type pointer
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-20
* Modifier         : None
* Date           : None
*******************************************************************************/
void USART_Load_Message(MESSAGE *Message)
{
    uint32_t ID = 0;

    ID |= (uint32_t)(USART_RX_Buffer[USART_RX_WaitProcess][2]&0x0f)<<28;
    ID |= (uint32_t)(USART_RX_Buffer[USART_RX_WaitProcess][2]&0x0f)<<24;
    ID |= (uint32_t)(USART_RX_Buffer[USART_RX_WaitProcess][3]&0x0f)<<20;
    ID |= (uint32_t)(USART_RX_Buffer[USART_RX_WaitProcess][4]&0x0f)<<16;
    ID |= (uint32_t)(USART_RX_Buffer[USART_RX_WaitProcess][5]&0x0f)<<12;
    ID |= (uint32_t)(USART_RX_Buffer[USART_RX_WaitProcess][6]&0x0f)<<8;
    ID |= (uint32_t)(USART_RX_Buffer[USART_RX_WaitProcess][7]&0x0f)<<4;
    ID |= (uint32_t)(USART_RX_Buffer[USART_RX_WaitProcess][8]&0x0f);

    DisassembleID(Message ,ID);// Disassemble ID
    /*ID*/
    Message->MessageNum = USART_RX_Buffer[USART_RX_WaitProcess][9]&0x0f;
    //USART_RX_Message.DLC |= (USART_RX_Buffer[USART_RX_WaitProcess][10]&0xf0)<<4;
    //Cannot exceed 8, stands by 1 ASCII code

    /*Data[0]*/
    Message->Data[0] = USART_RX_Buffer[USART_RX_WaitProcess][11]&0x0f;
    Message->Data[0] |= (USART_RX_Buffer[USART_RX_WaitProcess][12]&0x0f)<<4;
    /*Data[1]*/
    Message->Data[1] = USART_RX_Buffer[USART_RX_WaitProcess][13]&0x0f;
    Message->Data[1] |= (USART_RX_Buffer[USART_RX_WaitProcess][14]&0x0f)<<4;
    /*Data[2]*/
    Message->Data[2] = USART_RX_Buffer[USART_RX_WaitProcess][15]&0x0f;
    Message->Data[2] |= (USART_RX_Buffer[USART_RX_WaitProcess][16]&0x0f)<<4;
    /*Data[3]*/
    Message->Data[3] = USART_RX_Buffer[USART_RX_WaitProcess][17]&0x0f;
    Message->Data[3] |= (USART_RX_Buffer[USART_RX_WaitProcess][18]&0x0f)<<4;
    /*Data[4]*/
    Message->Data[4] = USART_RX_Buffer[USART_RX_WaitProcess][19]&0x0f;
    Message->Data[4] |= (USART_RX_Buffer[USART_RX_WaitProcess][20]&0x0f)<<4;
    /*Data[5]*/
    Message->Data[5] = USART_RX_Buffer[USART_RX_WaitProcess][21]&0x0f;
    Message->Data[5] |= (USART_RX_Buffer[USART_RX_WaitProcess][22]&0x0f)<<4;
    /*Data[6]*/
    Message->Data[6] = USART_RX_Buffer[USART_RX_WaitProcess][23]&0x0f;
    Message->Data[6] |= (USART_RX_Buffer[USART_RX_WaitProcess][24]&0x0f)<<4;
    /*Data[7]*/
    Message->Data[7] = USART_RX_Buffer[USART_RX_WaitProcess][25]&0x0f;
    Message->Data[7] |= (USART_RX_Buffer[USART_RX_WaitProcess][26]&0x0f)<<4;

    //Release buffer
    USART_RX_Buffer_State[USART_RX_WaitProcess] = RX_WAITLOAD;
    USART_RX_WaitProcess++;
    if(USART_RX_WaitProcess >= USART_RX_Buffer_MAX )//Greater than maximum value
        USART_RX_WaitProcess = 0;//Return 0
}
/*******************************************************************************
* Function Name  : USART_Load_Buffer
* Description    : Load data, ready to send
* Input          : MESSAGEÐÍÖ¸Õë
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-20
* Modifier         : None
* Date           : None
*******************************************************************************/
void USART_Load_Buffer(MESSAGE *Message)
{
    uint32_t ID,Sum;
    uint8_t i;
    ID = AssembleID(Message);

    USART_TX_Buffer[USART_TX_WaitLoad][0]=0xFF;//Start byte

    USART_TX_Buffer[USART_TX_WaitLoad][1] = (((uint8_t)(ID>>28))&0x0f)|0x30;//
    USART_TX_Buffer[USART_TX_WaitLoad][2] = (((uint8_t)(ID>>24))&0x0f)|0x30;//
    USART_TX_Buffer[USART_TX_WaitLoad][3] = (((uint8_t)(ID>>20))&0x0f)|0x30;//
    USART_TX_Buffer[USART_TX_WaitLoad][4] = (((uint8_t)(ID>>16))&0x0f)|0x30;//
    USART_TX_Buffer[USART_TX_WaitLoad][5] = (((uint8_t)(ID>>12))&0x0f)|0x30;//
    USART_TX_Buffer[USART_TX_WaitLoad][6] = (((uint8_t)(ID>>8))&0x0f)|0x30;//
    USART_TX_Buffer[USART_TX_WaitLoad][7] = (((uint8_t)(ID>>4))&0x0f)|0x30;//
    USART_TX_Buffer[USART_TX_WaitLoad][8] = ((uint8_t)(ID)&0x0f)|0x30;//

    USART_TX_Buffer[USART_TX_WaitLoad][9] = 0x30;//
    USART_TX_Buffer[USART_TX_WaitLoad][10] = (Message->MessageNum & 0x0f)|0x30;//

    USART_TX_Buffer[USART_TX_WaitLoad][11] = (Message->Data[0]&0x0f)|0x30;//
    USART_TX_Buffer[USART_TX_WaitLoad][12] = ((Message->Data[0]>>4)&0x0f)|0x30;//

    USART_TX_Buffer[USART_TX_WaitLoad][13] = (Message->Data[1]&0x0f)|0x30;//
    USART_TX_Buffer[USART_TX_WaitLoad][14] = ((Message->Data[1]>>4)&0x0f)|0x30;//
    /*Data[2]*/
    USART_TX_Buffer[USART_TX_WaitLoad][15] = (Message->Data[2]&0x0f)|0x30;//
    USART_TX_Buffer[USART_TX_WaitLoad][16] = ((Message->Data[2]>>4)&0x0f)|0x30;//
    /*Data[3]*/
    USART_TX_Buffer[USART_TX_WaitLoad][17] = (Message->Data[3]&0x0f)|0x30;//
    USART_TX_Buffer[USART_TX_WaitLoad][18] = ((Message->Data[3]>>4)&0x0f)|0x30;//
    /*Data[4]*/
    USART_TX_Buffer[USART_TX_WaitLoad][19] = (Message->Data[4]&0x0f)|0x30;//
    USART_TX_Buffer[USART_TX_WaitLoad][20] = ((Message->Data[4]>>4)&0x0f)|0x30;//
    /*Data[5]*/
    USART_TX_Buffer[USART_TX_WaitLoad][21] = (Message->Data[5]&0x0f)|0x30;//
    USART_TX_Buffer[USART_TX_WaitLoad][22] = ((Message->Data[5]>>4)&0x0f)|0x30;//
    /*Data[6]*/
    USART_TX_Buffer[USART_TX_WaitLoad][23] = (Message->Data[6]&0x0f)|0x30;//
    USART_TX_Buffer[USART_TX_WaitLoad][24] = ((Message->Data[6]>>4)&0x0f)|0x30;//
    /*Data[7]*/
    USART_TX_Buffer[USART_TX_WaitLoad][25] = (Message->Data[7]&0x0f)|0x30;//
    USART_TX_Buffer[USART_TX_WaitLoad][26] = ((Message->Data[7]>>4)&0x0f)|0x30;//
    Sum = 0;
    for(i=1; i<11+Message->MessageNum; i++)
        Sum    += USART_TX_Buffer[USART_TX_WaitLoad][i];
    //Lowest 8-bit
    USART_TX_Buffer[USART_TX_WaitLoad][11+(Message->MessageNum*2) + 3] = (uint8_t)Sum;
    USART_TX_Buffer[USART_TX_WaitLoad][11+(Message->MessageNum*2) + 2] = (uint8_t)(Sum >> 8);
    USART_TX_Buffer[USART_TX_WaitLoad][11+(Message->MessageNum*2) + 1] = (uint8_t)(Sum >> 16);//
    USART_TX_Buffer[USART_TX_WaitLoad][11+(Message->MessageNum*2) + 0] = (uint8_t)(Sum >> 24);//Highest 8-bit

    USART_TX_Buffer[USART_TX_WaitLoad][11+(Message->MessageNum*2) + 4] = 0xfe;//Ending sign

    USART_TX_Buffer_State[USART_TX_WaitLoad] = TX_WAITSEND;
    USART_TX_WaitLoad++;
    if(USART_TX_WaitLoad > USART_TX_Buffer_MAX)
        USART_TX_WaitLoad = 0;
}
/*******************************************************************************
* Function Name  : USART_RX_Process
* Description    : Processing data received by USART
* Input          : None
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-20
* Modifier         : None
* Date           : None
*******************************************************************************/
void USART_RX_Process(void)
{

    MESSAGE USART_RX_Message;
    uint8_t State;
    while(USART_RX_Buffer_State[USART_RX_WaitProcess] == RX_WAITPROCESS) { //Waiting for processing
        USART_Load_Message(&USART_RX_Message);
        /*********Processing function is showing below, same as CAN********/
        if((USART_RX_Message.MessageNum == 0) &&
                (USART_RX_Message.DeviceID != BroadcastID)) { //if it is message && not broadcast address
            State = Read_Process(&USART_RX_Message);
            /*****************Data copy, ready to reply*********************/
            if(State == 0)
                return ;
            USART_RX_Message.MessageNum = 8;
            USART_Load_Buffer(&USART_RX_Message);
        }
        else if( USART_RX_Message.MessageType == CAN_MSGTYPE_EMERG ||
                 USART_RX_Message.MessageType == CAN_MSGTYPE_NORMAL) //Emergency info or normal message, write in (Normally used for emergency shut down)
            State=Write_Process(&USART_RX_Message);
        else if(USART_RX_Message.MessageType == CAN_MSGTYPE_CONFIG) {
            State=Config_Process(&USART_RX_Message);
            USART_RX_Message.MessageNum = 0;
            /*****************data copy, ready to reply*********************/
            if((USART_RX_Message.MessageID != BroadcastID)&&
                    (State != 0)) { //non-broadcasting address, and sucessfully processed
                USART_Load_Buffer(&USART_RX_Message);
            }
        }
        else if(USART_RX_Message.MessageType == CAN_MSGTYPE_UPDATE) {
            ;
        }
        else {
            ;
        }
    }

}
/*******************************************************************************
* Function Name  : USART_TX_Process
* Description    : USART sending data, DMA type
* Input          : None
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-21
* Modifier         : None
* Date           : None
*******************************************************************************/
void USART_TX_Process(void)
{
    uint8_t BufferSize;
    volatile uint16_t Count;
    /* DMA1 Channel1 (triggered by USART1 Tx event) Config */

    Count = DMA1_Channel4->CNDTR; // Obtain current DMA byte number
    if(USART_TX_Buffer_State[USART_TX_WaitSend] == TX_WAITLOAD)// If wait for loading, there is no message wait for sending
        return ;//return
    else if((USART_TX_Buffer_State[USART_TX_WaitSend] == TX_WAITSEND)&&
            (Count ==0)) { // If msg is waiting for send, and the previous one has been sent

        BufferSize =  ((USART_TX_Buffer[USART_TX_WaitSend][10]&0x0f)*2) + 16;//Buffer size
        USART_TX_Buffer_State[USART_TX_WaitSend] = TX_SENDING;// Set sending state
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

        //DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART1->DR));
        // 0 11 00 00 1 0 0 1 0 0 0 0
        // 0x3090
        //D14: M2M 0: non-memory to memory model 1£ºMemory to memory model
        //D13D12:Channel priority 00£ºLowest 11£ºHighest
        //D11D10:RAM data width 00:8 01:16 10:32 11:reserved
        //D9D8: peripheral data width 00:8 01:16 10:32 11:reserved
        //D7: 0£ºRAM address not increasing 1£ºRAM address increasing
        //D6: 0£ºPeripheral address not increasing 1£ºPeripheral address increasing
        //D5: 0£ºNon-loop 1£ºLoop
        //D4: 0£ºRead from peripheral device 1: Read from memory
        //D3: 0£ºForbid transmission error interruption 1£ºStart transmission error interruption
        //D2: 0£ºForbid half transmission interruption 1£ºStart half tansmission interruption
        //D1: 0£ºForbid finishing interruption 1£ºStart finishing interruption
        //D0: 0 :Idle 1£ºstart
        DMA1_Channel4->CCR  = 0x3090;
        DMA1_Channel4->CNDTR = BufferSize;//transmission data size
        DMA1_Channel4->CPAR = (uint32_t)&(USART1->DR);//peripheral address
        DMA1_Channel4->CMAR = (uint32_t)(USART_TX_Buffer[USART_TX_WaitSend]);//RAM address

        /*There is no RAM to RAM transmission setting in DMA*/
        USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);//USART1 send request
        DMA1_Channel4->CCR |=0x01;//
    }
    else if((USART_TX_Buffer_State[USART_TX_WaitSend] == TX_SENDING)&&
            (Count == 0 )) { // Sending buffer state (finished)
        DMA_Cmd(DMA1_Channel4, DISABLE);
        USART_TX_Buffer_State[USART_TX_WaitSend] = TX_WAITLOAD;
        USART_TX_WaitSend++;
        if(USART_TX_WaitSend > USART_TX_Buffer_MAX)
            USART_TX_WaitSend = 0;
    }
    else
        ;

}
/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : USART1 receives interrupted functon
* Input          : None
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-17
* Modifier         : None
* Date           : None
*******************************************************************************/
void USART1_IRQHandler(void)
{
    uint8_t ch;
    uint32_t temp;
    static uint8_t RxNum = 0;//receive chars number counter
    static uint32_t Sum = 0;
    ch = (uint8_t)(USART1->DR); // read byte
    if(ch == 0xff) {       // If it is start sign
        RxNum = 0 ;// Empty receiving counter
        Sum = 0;
        USART_RX_Buffer_State[USART_RX_WaitLoad]=RX_LOADING;//Loading receiving state
    }
    if(USART_RX_Buffer_State[USART_RX_WaitLoad] == RX_LOADING) { //If it is receiving state
        USART_RX_Buffer[USART_RX_WaitLoad][RxNum++]=ch;// Receiving data
        Sum    += ch;//Calculate sum
    }
    if((RxNum >= 32)||
            ((ch == 0xfe)&&(RxNum >= 16)))
        //Info byte reached 32 or ending signal 0xfe appears and data exceeding 16 byte,
        //At least 12 chars, 1 start, 8 ID, 2 DLC, 4 accumulated sum, 1 ending, or there is error
    {
        temp =*(uint32_t *)(USART_RX_Buffer[USART_RX_WaitLoad]+ RxNum -5);
        Sum = Sum -0xff -0xfe;// Minus starting and ending sign
        Sum -= *(USART_RX_Buffer[USART_RX_WaitLoad]+ RxNum -5);
        Sum -= *(USART_RX_Buffer[USART_RX_WaitLoad]+ RxNum -4);
        Sum -= *(USART_RX_Buffer[USART_RX_WaitLoad]+ RxNum -3);
        Sum -= *(USART_RX_Buffer[USART_RX_WaitLoad]+ RxNum -2);

        if(Sum != temp) {
            USART_RX_Buffer_State[USART_RX_WaitLoad]=RX_WAITLOAD;//Reset buffer
            return ;
        }
        else {
            USART_RX_Buffer_State[USART_RX_WaitLoad]=RX_WAITPROCESS;
            USART_RX_WaitLoad++;
            if(USART_RX_WaitLoad >= USART_RX_Buffer_MAX)
                USART_RX_WaitLoad = 0;
        }
    }
}

/**
  * Close the Doxygen usart_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen usart group.
  *    @}
*/

/* End of usart.c */
