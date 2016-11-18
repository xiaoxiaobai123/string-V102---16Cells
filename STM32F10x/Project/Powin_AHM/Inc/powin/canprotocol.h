/**

*/ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h> 
#include <stdlib.h>
#include <stdint.h>
#include <windows.h>
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define STM32_ID_LEN            12
#define STM32_ID_HALF_LEN       (STM32_ID_LEN/2)

//filter
#define CAN_FILTER_INDEX_0      0x0
#define CAN_FILTER_INDEX_1      0x1
#define CAN_FILTER_INDEX_2      0x2
#define CAN_FILTER_INDEX_3      0x3
#define CAN_FILTER_INDEX_4      0x4
#define CAN_FILTER_NUM          (CAN_FILTER_INDEX_4 + 1)

//ID define
#define CAN_IDENTIFIER_LEN                29
//#define CAN_IDENTIFIER_B_LEN                18
//#define CAN_IDENTIFIER_A_START_BIT          CAN_IDENTIFIER_B_LEN
//#define CAN_IDENTIFIER_B_START_BIT          0

#define CAN_CMD_RESERVED_ID_START_BIT           26
#define CAN_CMD_RESERVED_ID_LEN                 3

#define CAN_CMD_MESSAGE_GROUP_ID_START_BIT      22
#define CAN_CMD_MESSAGE_GROUP_ID_LEN            4

#define CAN_CMD_DEVICE_TYPE_ID_START_BIT        18
#define CAN_CMD_DEVICE_TYPE_ID_LEN              4

#define CAN_CMD_STRING_ID_START_BIT             14
#define CAN_CMD_STRING_ID_LEN                   4

#define CAN_CMD_DEVICE_ID_START_BIT             8
#define CAN_CMD_DEVICE_ID_LEN                   6

#define CAN_CMD_MESSAGE_ID_START_BIT            0
#define CAN_CMD_MESSAGE_ID_LEN                  8



//device type
#define DEVICE_TYPE_ARRAY           0x0
#define DEVICE_TYPE_STRING          0x1
#define DEVICE_TYPE_BATTERY_PACK    0x2
#define DEVICE_TYPE_RELAY_BOARD     0x3
#define DEVICE_TYPE_ERROR           0xf
//for test 
//#define DEVICE_TYPE_BATTERY_PACK_CLONE    0x3

#define DEVICE_TYPE_ERR             0xff

//DEVICE_TYPE_ID value
#define CAN_CMD_DEVICE_TYPE_ID_VALUE_ARRAY          0x02 //dA
#define CAN_CMD_DEVICE_TYPE_ID_VALUE_STRING         0x04 //dS
#define CAN_CMD_DEVICE_TYPE_ID_VALUE_BATTERY_PACK   0x05 //dB   
#define CAN_CMD_DEVICE_TYPE_ID_VALUE_RELAY_BOARD    0x03 //dR 

//Empty ID value
#define CAN_CMD_NULL_ID_VALUE                  0x0 

/* Private macro -------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */

typedef struct
{
    uint32_t StdId;  /*!< Specifies the standard identifier.
                     This parameter can be a value between 0 to 0x7FF. */

    uint32_t ExtId;  /*!< Specifies the extended identifier.
                     This parameter can be a value between 0 to 0x1FFFFFFF. */

    uint8_t IDE;     /*!< Specifies the type of identifier for the message that
                     will be transmitted. This parameter can be a value
                     of @ref CAN_identifier_type */

    uint8_t RTR;     /*!< Specifies the type of frame for the message that will
                     be transmitted. This parameter can be a value of
                     @ref CAN_remote_transmission_request */

    uint8_t DLC;     /*!< Specifies the length of the frame that will be
                     transmitted. This parameter can be a value between
                     0 to 8 */

    uint8_t Data[8]; /*!< Contains the data to be transmitted. It ranges from 0
                     to 0xFF. */
} CanTxMsg;

/**
* @brief  CAN Rx message structure definition
*/

typedef struct
{
    uint32_t StdId;  /*!< Specifies the standard identifier.
                     This parameter can be a value between 0 to 0x7FF. */

    uint32_t ExtId;  /*!< Specifies the extended identifier.
                     This parameter can be a value between 0 to 0x1FFFFFFF. */

    uint8_t IDE;     /*!< Specifies the type of identifier for the message that
                     will be received. This parameter can be a value of
                     @ref CAN_identifier_type */

    uint8_t RTR;     /*!< Specifies the type of frame for the received message.
                     This parameter can be a value of
                     @ref CAN_remote_transmission_request */

    uint8_t DLC;     /*!< Specifies the length of the frame that will be received.
                     This parameter can be a value between 0 to 8 */

    uint8_t Data[8]; /*!< Contains the data to be received. It ranges from 0 to
                     0xFF. */

    uint8_t FMI;     /*!< Specifies the index of the filter the message stored in
                     the mailbox passes through. This parameter can be a
                     value between 0 to 0xFF */
} CanRxMsg;
uint8_t CanDriverGetLocalDeviceId(void);
boolean PrepareCanMsgHeader(CanTxMsg* txMsg, 
                                uint8_t msgGroupId, 
                                uint8_t deviceTypeId, 
                                uint8_t arrayId, 
                                uint8_t stringId, 
                                uint8_t deviceId,
                                uint8_t messageId);                                
                                
boolean GetCanMsgHeader(CanRxMsg* rxMsg, 
                                uint8_t* msgGroupId, 
                                uint8_t* deviceTypeId, 
                                uint8_t* arrayId, 
                                uint8_t* stringId, 
                                uint8_t* deviceId,
                                uint8_t* messageId);
     


#endif /* __CAN_PROTOCOL_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
