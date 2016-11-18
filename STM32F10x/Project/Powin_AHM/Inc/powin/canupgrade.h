/**

*/ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_UPGRADE_H
#define __CAN_UPGRADE_H



/* Includes ------------------------------------------------------------------*/
//#include "stdafx.h"

#include <stdio.h> 
#include <stdlib.h>
#include <stdint.h>
//#include <windows.h>
#ifdef _WINDOWS
#include "stdafx.h"
#else
#include "stdbool.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
#endif
//#include<BaseTsd.h>

typedef unsigned char boolean;
//#define true    0x1
//#define false   0x0



/* Exported types ------------------------------------------------------------*/
typedef struct
{
    uint8_t id;
    uint8_t enable;
    boolean ackFlag;
} CANUpgradeACKRecord;

/* Exported constants --------------------------------------------------------*/
#define SUPPORT_SERVER_UPGRADE      1
#define SUPPORT_CLIENT_UPGRADE      0

#if(SUPPORT_SERVER_UPGRADE)
    #define SERVER_UPGRADE_BP       0
    #define SERVER_UPGRADE_STRING   1

    #if(SERVER_UPGRADE_BP && SERVER_UPGRADE_STRING)
        #error just select only one SERVER_UPGRADE_BP or SERVER_UPGRADE_STRING
    #endif
#endif

#if(SUPPORT_CLIENT_UPGRADE)
    #define CLIENT_UPGRADE_TYPE_BP       1
    #define CLIENT_UPGRADE_TYPE_STRING   2
#endif

/* Private macro -------------------------------------------------------------*/
#define PART_BURN_LEN  2048//一定要8的倍數
//#define PART_BURN_LEN  13

//#define DEBUG_UPGRADE_STAGE_INIT    0x1
//#define DEBUG_UPGRADE_STAGE_WRITE   0x2    
//#define DEBUG_UPGRADE_STAGE_BURN    0x3
//static uint8_t upgradeServerStage = DEBUG_UPGRADE_STAGE_INIT;
#define UPGRADE_SERVER_SUPPORT_NUM  22
//Flag in dada[0], data len = 1.
#define UPGRADE_CMD_ACK         0x79
#define UPGRADE_CMD_NACK        0x1F
#define UPGRADE_CMD_REQUEST     0xF1
#define UPGRADE_CMD_RESET       0x97

//flag index
#define UPGRADE_CMD_FLAG_INDEX          0x00

//ack nack status id
#define UPGRADE_STATUS_ID_INDEX         0x01

//get type version index
#define UPGRADE_CMD_VERSION_INDEX       0x01
#define UPGRADE_CMD_TYPE_INDEX          0x02

//read memory index
#define UPGRADE_CMD_MEM_ADDRESS_BYTE_0_INDEX  0x01
#define UPGRADE_CMD_MEM_ADDRESS_BYTE_1_INDEX  0x02
#define UPGRADE_CMD_MEM_ADDRESS_BYTE_2_INDEX  0x03
#define UPGRADE_CMD_MEM_ADDRESS_BYTE_3_INDEX  0x04
#define UPGRADE_CMD_READ_MEM_SIZE_INDEX       0x05
#define UPGRADE_CMD_READ_MEM_SIZE_INDEX_2     0x06

//check sum index
#define UPGRADE_CHECKSUM_ID_INDEX         0x02


//---------------------------------------------------------
//status
#define UPGRADE_STATUS_NONE                  0xff
#define UPGRADE_STATUS_STANDBY               0x0
#define UPGRADE_STATUS_READY_TO_RECEIVE      0x1  //server
#define UPGRADE_STATUS_SENDING_DATA          0x2
#define UPGRADE_STATUS_WAITING_TO_SEND       0x3  //client  //not ack type, is request type
#define UPGRADE_STATUS_WAITING_FOR_RECEIVED  0x4  //client
#define UPGRADE_STATUS_WAITING_FOR_BURNED    0x5  //client
#define UPGRADE_STATUS_WAITING_TO_BURN       0x6  //server
#define UPGRADE_STATUS_FINISH_TO_BURN        0x7
//#define UPGRADE_STATUS_WAITING_FOR_ACK       0x7


//**************  Firmware Info   *****************
#define FIRMWARE_INFO_CMD_TYPE_INDEX        0x00
#define FIRMWARE_INFO_CMD_BOOT_FLAG_INDEX   0x01
#define FIRMWARE_INFO_CMD_VER_INDEX         0x02


// Firmware CMD TYPE
#define FIRMWARE_INFO_CMD_TYPE_QUERY    0x00
#define FIRMWARE_INFO_CMD_TYPE_SET      0x01
#define FIRMWARE_INFO_CMD_TYPE_ACK      0x02

//**************  Command Action   *****************
#define COMMAND_ACTION_CMD_TYPE_INDEX        0x00

// Command Action TYPE(COMMAND_ACTION_CMD_TYPE_INDEX)
#define COMMAND_ACTION_CMD_TYPE_REBOOT    0x01
#define COMMAND_ACTION_CMD_TYPE_UPGRADE_BYPASS_BP       0x02
#define COMMAND_ACTION_CMD_TYPE_UPGRADE_JUST_STRING          0x03
/* Exported functions ------------------------------------------------------- */


#if(SUPPORT_SERVER_UPGRADE)
boolean CanServerGetBPBypassMode(void);
void CanServerSetBPBypassMode(boolean mode);
void CanServerCommandActionProcess(uint8_t* data, uint8_t dataLen);
void CanServerFirmwareInfoProcess(uint8_t* data, uint8_t dataLen);
void CanUpgradeServerProcess(uint8_t* data, uint8_t dataLen);
void SendFirmwareInfo(void);
#endif//#if(SUPPORT_SERVER_UPGRADE)

#if(SUPPORT_CLIENT_UPGRADE)
void CanUpgradeStart(uint32_t srcAddress, uint32_t destAddress, uint32_t length);
void CanUpgradeStop();
void CanClientFirmwareInfoProcess(uint8_t* data, uint8_t dataLen, uint8_t targetDeviceId);
void CanUpgradeClientProcess(uint8_t* data, uint8_t dataLen, uint8_t targetDeviceId);
void CanUpgradeSetEnable(uint8_t* data, uint8_t dataLen);
void CanUpgradeSetClientType(int type);
uint8_t CanUpgradeGetClientType(void);
void CanUpgradeClientMainProcess(void);
#endif//#if(SUPPORT_String_UPGRADE)

#endif /* __CAN_UPGRADE_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
