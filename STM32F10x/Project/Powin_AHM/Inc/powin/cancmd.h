/**
 
*/ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAM_CMD_H
#define __CAM_CMD_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32fx.h"
#include <stdio.h>
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define CAN_CMD_BROADCAST      0xff
//*****   message group id   *****//
#define CMD_GROUP_ID_DATA              0x04

//*****   message id   (from which device)*****//
//-- relay board --
//Relay <-> Array
#define CMD_ID_RELAY_BOARD_WORKING              0x07
#define CMD_ID_RELAY_BOARD_RESTART              0x03

//-- array -- 
//Array <-> Relay
#define CMD_ID_ARRAY_START_UP                   0x05
#define CMD_ID_ARRAY_WORKING                    0x07
//Array <-> String
#define CMD_ID_ARRAY_QUERY_STRING               0xef
#define CMD_ID_ARRAY_QUERY_BP                   0xee
#define CMD_ID_ARRAY_TARGET_VOLTAGE             0x20
#define CMD_ID_ARRAY_SET_CONTACTOR              0x05

//-- BP --
//BP <-> String
//#define CMD_ID_BP_VOLTAGE                       0x42
//#define CMD_ID_BP_TEMP                          0x43
//#define CMD_ID_BP_BALANCE_STATUS                0x44
//#define CMD_ID_BP_BALANCE_CHARGE                0x45
//#define CMD_ID_BP_CELL_VOLTAGE_1_4              0x48
//#define CMD_ID_BP_CELL_VOLTAGE_5_8              0x49
//#define CMD_ID_BP_CELL_VOLTAGE_9_12             0x4a
//#define CMD_ID_BP_CELL_TEMP_1_4                 0x4c
//#define CMD_ID_BP_CELL_TEMP_5_8                 0x4d
//#define CMD_ID_BP_CELL_TEMP_9_12                0x4e
#define CMD_ID_SERVER_UPGRADE_WRITE_MEMORY      0x78
#define CMD_ID_SERVER_FIRMWARE_INFO                 0x79
#define CMD_ID_SERVER_COMMAND_ACTION                0x80

//-- String --
//String <-> Array
//#define CMD_ID_STRING_CURRENT                       0x51
//#define CMD_ID_STRING_HLA_CELL_VOLTAGE              0x52
//#define CMD_ID_STRING_HLA_CELL_TEMP                 0x53
//#define CMD_ID_STRING_POWER_ENERGY                  0x54
//#define CMD_ID_STRING_BALANCE_CHARGE                0x55
//#define CMD_ID_STRING_USEAGE_HISTORY                0x56
#define CMD_ID_CLIENT_UPGRADE_WRITE_MEMORY          0x78
#define CMD_ID_CLIENT_FIRMWARE_INFO                 0x79
#define CMD_ID_CLIENT_COMMAND_ACTION                0x80

//String <-> BP

/* Private macro -------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */


#endif /* __CAM_CMD_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
