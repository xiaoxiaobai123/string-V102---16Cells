/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_conf.h 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Library configuration file.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MEMORY_CONF_H
#define __MEMORY_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment/Comment the line below to enable/disable peripheral header file inclusion */
#ifdef POWIN_BOOTER
    #define STRING_VERSION    244
#else
    #define STRING_VERSION    102
#endif

#define STRING_RELEASE   1

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
//#define USER_FLASH_AP_1_ADDRESS 0x8002000 //8K
//#define USER_FLASH_AP_2_ADDRESS 0x8022000 //138K
#define USER_FLASH_BOOTER_ADDRESS   0x8000000 //0K(size 32K)
#define USER_FLASH_AP_1_ADDRESS     0x8010000 //64K(size 64K)
#define USER_FLASH_AP_2_ADDRESS     0x8020000 //128K(size 64K)
#define USER_FLASH_AP_3_ADDRESS     0x8030000 //192K(size 64K)
//#define USER_FLASH_AP_4_ADDRESS 0x8020000 //192K(size 32K)
#define FLASH_BOOTER_ID 0x01//0xff
#define FLASH_AP_1_ID 0x02//0x1 
#define FLASH_AP_2_ID 0x03//0x2  
#define FLASH_AP_3_ID 0x04//0x3 

/*
0x1FFFF800: 0x5AA5
0x1FFFF802: 0xFFFF 
0x1FFFF804: 0xFFFF
0x1FFFF806: 0xFFFF
0x1FFFF808: 0xFFFF
0x1FFFF80A: 0xFFFF
0x1FFFF80C: 0xFFFF
0x1FFFF80E: 0xFFFF
*/
#define USER_FLASH_OPTION_BYTE_ADDRESS 0x1FFFF804


/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */


#endif /* __MEMORY_CONF_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
