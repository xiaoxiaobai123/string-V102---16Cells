/**
  ******************************************************************************
  * @file        BESS_3-0_AHM_can_V_1-0.h
  * @author      Nystrom Engineering
  * @version     2.0.0
  * @date        2014-01-27
  * @copyright   Powin Energy
  * @brief       The include file for the Controller Automation Network (CAN) bus.
  ******************************************************************************
  */

/*-----------------------------------------------------------------------------*/
/* Define to prevent recursive inclusion                                       */
/*-----------------------------------------------------------------------------*/
#ifndef CAN_H_
#define CAN_H_

/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
#include "BESS_3-0_AHM_device_lib_V_1-0.h"

#include "BESS_3-0_AHM_stringdata_V_1-0.h"
/** @addtogroup can
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Exported Types                                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup can_Exported_Types
  * @{
  */
typedef struct {
    uint32_t ID;
    uint8_t DLC;
    union {
        struct {
            uint32_t DataL;
            uint32_t DataH;
        } longwords;
        struct {
            uint16_t data0;
            uint16_t data1;
            uint16_t data2;
            uint16_t data3;
        } shortwords;
        uint8_t bytes[8];
    } data;
} BUFFER;

typedef struct MESSAGE {
    uint8_t ReserveField;    /**<  */

    uint8_t StringID;        /**<  */

    uint8_t MessageType;        /**<  */
    uint8_t DeviceType;        /**<  */
    int8_t DeviceID;        /**<  */
    uint8_t MessageID;        /**<  */
    uint8_t MessageNum;        /**<  */
    uint8_t Data[8];            /**<  */
} MESSAGE;

extern MESSAGE AlarmMessage;
extern MESSAGE WarningMessage;
extern MESSAGE ErrorMessage;   

#if(0)
typedef struct {
    uint8_t head;       // index of next free entry to put if count < sz
    uint8_t tail;       // index of oldest entry to get
    uint8_t count;      // how many entries are present
    uint8_t sz;         // how many entries reserved in *data array
    BUFFER volatile *data;       // pointer to space for fifo data
} CanFifo;

#else
typedef struct {
    uint16_t head;       // index of next free entry to put if count < sz
    uint16_t tail;       // index of oldest entry to get
    uint16_t count;      // how many entries are present
    uint16_t sz;         // how many entries reserved in *data array
    BUFFER volatile *data;       // pointer to space for fifo data
} CanFifo;
#endif
/**
  * Close the Doxygen can__Exported_Types group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Constants                                                          */
/*-----------------------------------------------------------------------------*/
/** @defgroup can_Exported_Constants
  * @{
  */




#define SUPPORT_CAN_811        1
#if(SUPPORT_CAN_811)
    //String High/Low Cell Voltage & High/Low Cell Temp
    //#define STRING_HL_CELL_VOLTAGE_TEMP_CMD_ID      0x20
    //String Cell Voltage Message
    #define STRING_CELL_VOLTAGE_CMD_ID              0x52//0x21
    //String Cell Temp Message
    #define STRING_CELL_TEMP_CMD_ID                 0x53//0x22
    //String Cell SOC Message
    //#define STRING_CELL_SOC_CMD_ID                  0x23
    //String SOC
    //#define STRING_SOC_CMD_ID                       0x34
    //String Current /  Votlage / Contact Status
    //#define STRING_CURRENT_VOLTAGE_CONTACT_STATUS_CMD_ID    0x30
    //String Amp Hour
    //#define STRING_AMP_HOUR_CMD_ID                  0x31
    //String  DC power / Walt Hour
    //#define STRING_DC_POWER_WALT_HOUR_CMD_ID        0x32
#else
    //String High/Low Cell Voltage & High/Low Cell Temp
    #define STRING_HL_CELL_VOLTAGE_TEMP_CMD_ID      0x60
    //String Cell Voltage Message
    #define STRING_CELL_VOLTAGE_CMD_ID              0x61
    //String Cell Temp Message
    #define STRING_CELL_TEMP_CMD_ID                 0x62
    //String Cell SOC Message
    #define STRING_CELL_SOC_CMD_ID                  0x63
    //String SOC
    #define STRING_SOC_CMD_ID                       0x64
    //String Current /  Votlage / Contact Status
    #define STRING_CURRENT_VOLTAGE_CONTACT_STATUS_CMD_ID    0x01
    //String Amp Hour
    #define STRING_AMP_HOUR_CMD_ID                  0x03
    //String  DC power / Walt Hour
    #define STRING_DC_POWER_WALT_HOUR_CMD_ID        0x05
#endif
//#define RCC_APB1Periph_CAN RCC_APB1Periph_CAN1    /**<  */
//#define CAN CAN1                                /**<  */

#define CAN_RX_BUFFER_LEN        512//(256)//(128)//(64) 
#define CAN_TX_BUFFER_LEN        512//(CELL_NUMBER*BP_NUMBER*3 + 128)//(128)

#define StringIDMASK             0xe0
#define BatteryIDMASK             (0xff-0xe0)

/**Info frame type is shown below, used by CAN and USART to process message**/
/*Reserved ID Must be all 0 5-bit*/
#define    ReserveID    0x00   //CAN bus and USART ID
/*Message Type ID 4-bit*/
#define CAN_MSGTYPE_EMERG 0X00 //Emergency ID (Controller sends Emergency message)
#define AlarmID        0x01 //Alarming ID (Used for uploading alarm info)
#define CAN_MSGTYPE_NORMAL    0x02 //Data ID (Normal transmission and query data)
#define CAN_MSGTYPE_CONFIG    0x03 //Configuration ID (Host computer configuration and lower computer responding)
#define CAN_MSGTYPE_UPDATE    0x04 //Update ID (Hardware upgrade)
/*Device type ID 5-bit*/
#define    AhmType    0x04   //Cell type ID: 0x01
/*Address ID's boradcasting ID 8-bit*/
#define    BroadcastID 0x00   //Broadcasting ID£¬0xFF change to 0 to follow the documentation sam 2011/may/05

/*******ID definition below*********/
//Message type
#define ID_EMERGENCY    (0x00)//
#define CAN_MSGTYPE_ALARM         (0x01)//
#define ID_NORMAL        (0x02)//
#define ID_CONFIG        (0x03)//
#define ID_UPDATE        (0x04)//

//Device type
#define ID_CONTROLLER    (0x00)//
#define ID_PACK            (0x01)//
#define ID_CHARGER        (0x02)//
#define ID_INVERTER        (0x03)//
#define ID_AHCOUNT        (0x04)// AH counter
#define ID_RELAY        (0x05)//

#define CAN_MSGID_CMD 0

#define CAN_MSGNUM_CMD 1
//The current status of receiving buffer
#define RX_WAITLOAD     (0)// Idle, wait for loading
#define RX_LOADING     (1)    // Loading
#define RX_WAITPROCESS (2)    // Wait for processing
#define RX_PROCESSING (3)    // Processing

//Current sending buffer's status
#define TX_WAITLOAD    (0)// Idle, wait for loading
#define TX_LOADING    (1)    // Loading
#define TX_WAITSEND    (2)// Wait to send
#define TX_SENDING    (3)    // Sending

// define bits in status byte
#define	WDOG_REC		0x80
#define	COMM_ERR		0x40
#define AD7327_COMM_ERR	0x02
#define	AD7327_REF_FAIL	0x01
// define bits in startupstatus byte
#define	EEPROM_ERROR	0x04

#define CAN_TX_BLOCK_TIMER_INIT_VALUE 0xCC00000L

/**
  * Close the Doxygen can_Exported_Constants group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Macros                                                             */
/*-----------------------------------------------------------------------------*/
/** @defgroup can_Exported_Macros
  * @{
  */

/**
  * Close the Doxygen can_Exported_Macros group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup can_Exported_Variables
  * @{
  */
/*CAN Buffer information, all information recevied is not processed buffer, in case of lagging*/

uint8_t can_buf_avail(void);  // return 0, 1, 2 for first available buffer or 0xff if none available
void send_autodata(uint8_t idx);
void CAN_Send(BUFFER volatile *SendData, uint8_t idx);  // idx is return value of can_buf_avail

extern uint8_t Self_ID;
extern volatile CanFifo can_rx_fifo;
extern volatile CanFifo can_rx_fifo2;
extern volatile CanFifo can_tx_fifo;

//extern uint32_t Serial_Number;
//extern uint8_t dev_id;
extern uint8_t suppress_can_tx;
extern uint32_t can_tx_block_timer;
extern uint8_t raw_req;    // when non-zero, can has req'd a raw sample

// counts up from systick. When reaches some threshold, causes some
// alarm condition because host is gone. Or at least that's how it works
// on the BMS. Since this board can't shut its own power off, not sure
// what we'll do if that happens here. pjn 2014-4-19
extern uint32_t CANTimer;

/**
  * Close the Doxygen can_Exported_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Function Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup can_Exported_Functions
  * @{
  */

// added clk parameter for automatic prescaler calculation
// pjn 2014-2-6
void CAN_Initialize(RCC_ClocksTypeDef *clk);
void CAN_Device_Init(RCC_ClocksTypeDef *clk);
void CAN_Data_Init(void);
void CAN_config(void);
void CAN_TX(void);
void CAN_TX2(void);
void CAN_RX_Process(void);
void CAN_RX_Process2(void);
void USB_HP_CAN1_TX_IRQHandler(void);
void USB_LP_CAN_RX0_IRQHandler(void);
void CAN1_RX1_IRQHandler(void);
void CAN1_SCE_IRQHandler(void);
uint32_t AssembleID(MESSAGE* message);
void DisassembleID(MESSAGE* message,uint32_t ID);
uint8_t Read_Process(MESSAGE * Message);
uint8_t Write_Process(MESSAGE * Message);
uint8_t Config_Process(MESSAGE * Message);
uint8_t GET_Device_ID(void);

void fifo_init(volatile CanFifo *f, volatile BUFFER *buffer, uint16_t buffer_len);
BUFFER volatile *fifo_get(volatile CanFifo *f);
BUFFER volatile *fifo_alloc(volatile CanFifo *f);

void block_can_tx(void);
void unblock_can_tx(void);

void do_CAN_auto_Ex( void );
void send_autodata_Ex(uint8_t idx);

void CAN_make_send_2_BP (BUFFER volatile * rxbuf2);
void CAN_make_send_bypass_2_BP(uint8_t *buf, uint8_t length, uint8_t msg_id, uint8_t device_id);
void CAN_make_send_2_Array (uint8_t *buf, uint8_t length, uint8_t msg_id);
void CAN_make_send_from_BP (uint8_t *buf, uint8_t length, uint8_t msg_id, int8_t bp_id);

uint16_t measureStringVoltage(void);

int16_t getStringCurrent(void);

void checkCurrent(void);
/**
  * Close the Doxygen can_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* End of the C bindings section for C++ compilers.                            */
/*-----------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* CAN_H_ */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen can group.
  *    @}
*/
