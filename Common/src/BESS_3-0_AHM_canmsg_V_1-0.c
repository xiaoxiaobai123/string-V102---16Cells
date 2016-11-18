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
#include <stdint.h>
#include "BESS_3-0_AHM_canmsg_V_1-0.h"


#ifdef STM32F10X_CL  //for WaveShare F107
 
#else

#endif

static void switchTwoU8(uint8_t* data0, uint8_t* data1)
{
    uint8_t u8Temp;
    u8Temp = *data0;
    *data0 = *data1;
    *data1 = u8Temp;    
}
uint8_t DecodeFour16BitData(uint8_t* data, uint16_t* data0, uint16_t* data1, uint16_t* data2, uint16_t* data3)
{
    *data0 = ((data[1])&0xff)  | ((data[0]&0xff) << 8);
    *data1 = ((data[3])&0xff)  | ((data[2]&0xff) << 8);
    *data2 = ((data[5])&0xff)  | ((data[4]&0xff) << 8);
    *data3 = ((data[7])&0xff)  | ((data[6]&0xff) << 8);
    
    
    return 8;
}
uint8_t DecodeTwo16BitData(uint8_t* data, uint16_t* data0, uint16_t* data1)
{
    
    *data0 = ((data[1])&0xff)  | ((data[0]&0xff) << 8);
    *data1 = ((data[3])&0xff)  | ((data[2]&0xff) << 8);
    //temp
    switchTwoU8(&data[0], &data[1]);
    switchTwoU8(&data[2], &data[3]);
    
    return 4;
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
