/**
  ******************************************************************************
  * @file        BESS_3-0_AHM_device_lib_V_1-0.h
  * @author      Nystrom Engineering
  * @version     V2.0.0
  * @date        2014-01-27
  * @copyright   Powin Engineering
  * @brief       The include file to define all the peripheral chip and on-board devices.
  ******************************************************************************
  */

/*-----------------------------------------------------------------------------*/
/* Define to prevent recursive inclusion                                       */
/*-----------------------------------------------------------------------------*/
#ifndef DEVICE_LIB_H_
#define DEVICE_LIB_H_

/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
#include "stdio.h"
#include "string.h"

#include "stm32f10x.h"
#include "core_cm3.h"
#include "system_stm32f10x.h"

#include "stm32f10x_adc.h"
#include "stm32f10x_can.h"
#include "stm32f10x_rcc.h"

//#include "BESS_3-0_AHM_adc_V_1-0.h"
#include "BESS_3-0_AHM_main_V_1-0.h"
#include "BESS_3-0_AHM_can_V_1-0.h"
#include "BESS_3-0_AHM_gpio_V_1-0.h"
//#include "BESS_3-0_AHM_i2c_eeprom_V_1-0.h"
#include "BESS_3-0_AHM_systick_V_1-0.h"
#include "BESS_3-0_AHM_watchdog_V_1-0.h"
//#include "BESS_3-0_AHM_usart_V_1-0.h"
#include "BESS_3-0_AHM_nvic_V_1-0.h"

/** @addtogroup device_lib device_lib
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Exported Types                                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup device_lib_Exported_Types
  * @{
  */
//typedef enum {FALSE = 0, TRUE = !FALSE} bool;

/**
  * Close the Doxygen device_lib__Exported_Types group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Constants                                                          */
/*-----------------------------------------------------------------------------*/
/** @defgroup device_lib_Exported_Constants
  * @{
  */
#define __CAN1_USED__        /**< Enable CAN1 in StdPeriph_Lib */
#ifdef  __CAN1_USED__
#define CANx CAN1
#else /*__CAN2_USED__*/
#define CANx CAN2
#endif  /* __CAN1_USED__ */

/*-----------------------------------------------------------------------------*/
/* BMS LED definitions                                                         */
/*-----------------------------------------------------------------------------*/
#define BLINK_PORT      GPIOC
#define BLINK_PIN       10
#define BLINK_RCC_BIT   RCC_APB2Periph_GPIOC
#define BLINK_TICKS     SYSTICK_FREQUENCY_HZ/2

/**
  * Close the Doxygen device_lib_Exported_Constants group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Macros                                                             */
/*-----------------------------------------------------------------------------*/
/** @defgroup device_lib_Exported_Macros
  * @{
  */

#define ASIZE(a) (sizeof(a) / sizeof(a[0]))




/**
  * Close the Doxygen device_lib_Exported_Macros group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup device_lib_Exported_Variables
  * @{
  */
extern uint32_t uwTimingDelay;

/**
  * Close the Doxygen device_lib_Exported_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Function Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup device_lib_Exported_Functions
  * @{
  */

extern void Device_Initialize(void);
extern void Delay(volatile uint32_t nCount);
extern void DelayBlocking(uint32_t nTime);
extern void TimingDelay_Decrement(void);


/**
  * Close the Doxygen device_lib_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* End of the C bindings section for C++ compilers.                            */
/*-----------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* DEVICE_LIB_H_ */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen device_lib group.
  *    @}
*/
