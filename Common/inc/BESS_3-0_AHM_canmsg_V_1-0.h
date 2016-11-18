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
#ifndef CAN_MSG_H_
#define CAN_MSG_H_

/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
#include <stdio.h> 
#include <stdlib.h>
#include <stdint.h>
/** @addtogroup can
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Exported Types                                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup can_Exported_Types
  * @{
  */


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
#define CAN_MSG_OVER_VOLTAGE_SETTING    0xE0
#define CAN_MSG_UNDER_VOLTAGE_SETTING   0xE1
#define CAN_MSG_OVER_TEMP_SETTING       0xE2
#define CAN_MSG_UNDER_TEMP_SETTING      0xE3
#define CAN_MSG_HCDT_SETTING            0xE4 //High Cell Delta Temp(Diff Temp)
#define CAN_MSG_HCTR_SETTING            0xE5 //High Cell Temp Rise
#define CAN_MSG_HCR_TEMP_SETTING        0xE6 //High Charge Rate
#define CAN_MSG_HDR_TEMP_SETTING        0xE7 //High Discharge Rate

#define CAN_MSG_BP_CELL_VOLTAGE  0x42


#define USE_CAN_40  1

#if(USE_CAN_40)
    #define ID_RSVD(v) ((v & 0x7) << 26)
    #define ID_MSGTYPE(v) ((v & 0x0f) << 22)
    #define ID_DVCTYPE(v) ((v & 0x0f) << 18)
    #define ID_STRING(v) ((v & 0x0f) << 14)
    #define ID_DVCID(v) ((v & 0x3f) << 8)
    #define ID_MSG(v) ((v & 0xff) << 0)
#else
    #define ID_RSVD(v) ((v & 0x1) << 28)
    #define ID_STRING(v) ((v & 0x0f) << 24)

    #define ID_MSGTYPE(v) ((v & 0x0f) << 20)
    #define ID_DVCTYPE(v) ((v & 0x0f) << 16)
    #define ID_DVCID(v) ((v & 0xff) << 8)
    #define ID_MSG(v) ((v & 0xff) << 0)
#endif

#define MAKE_ID(rsvd, stringid, msgtype, dvctype, dvcid, msg) (ID_RSVD(rsvd) | ID_STRING(stringid) | ID_MSGTYPE(msgtype) | \
                 ID_DVCTYPE(dvctype) | ID_DVCID(dvcid) | ID_MSG(msg))



//#define MSG_TYPE 2
//#define DVC_TYPE 4
//#define BP_TYPE  1
//#define ARRAY_TYPE  7

#define MSG_TYPE 2
#define STRING_TYPE 4
#define BP_TYPE  5
#define ARRAY_TYPE  2
#define RELAY_TYPE  3
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
uint8_t DecodeFour16BitData(uint8_t* data, uint16_t* data0, uint16_t* data1, uint16_t* data2, uint16_t* data3);
uint8_t DecodeTwo16BitData(uint8_t* data, uint16_t* data0, uint16_t* data1);
/**
  * Close the Doxygen can_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* End of the C bindings section for C++ compilers.                            */
/*-----------------------------------------------------------------------------*/


#endif /* CAN_MSG_H_ */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen can group.
  *    @}
*/
