/**
 ******************************************************************************
  * @file        BESS_3-0_AHM_systick_V_1-0.h
  * @author      Nystrom Engineering
  * @version     2.0.0
  * @date        Jan 27, 2014
  * @copyright   Powin Energy
  * @brief       This file is used for defining the System Timer Functions
  ******************************************************************************
  */

/*-----------------------------------------------------------------------------*/
/* Define to prevent recursive inclusion                                       */
/*-----------------------------------------------------------------------------*/
#ifndef SYSTICK_H_
#define SYSTICK_H_

/*-----------------------------------------------------------------------------*/
/* When using C++ compiler, make sure that all definitions have a C binding.   */
/*-----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif

/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
#include "BESS_3-0_AHM_device_lib_V_1-0.h"

/** @addtogroup systick systick
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Exported Types                                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup systick_Exported_Types
  * @{
  */

/**
  * Close the Doxygen systick__Exported_Types group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Constants                                                          */
/*-----------------------------------------------------------------------------*/
/** @defgroup systick_Exported_Constants
  * @{
  */
#define SYSTICK_FREQUENCY_HZ       1000 /**< SysTick is set for 1mSec period (i.e. 1000Hz) */
/**
  * Close the Doxygen systick_Exported_Constants group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Macros                                                             */
/*-----------------------------------------------------------------------------*/
/** @defgroup systick_Exported_Macros
  * @{
  */

/**
  * Close the Doxygen systick_Exported_Macros group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup systick_Exported_Variables
  * @{
  */

extern uint32_t milli_count;	// used by sampling routines to count passage of time
extern uint16_t LED1_Timer;
extern uint16_t LED2_Timer;
extern int seconds;

/**
  * Close the Doxygen systick_Exported_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Function Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup systick_Exported_Functions
  * @{
  */

extern void SysTick_Initialize(void);
extern void SysTick_Handler(void);
/**
  * Close the Doxygen adc_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* End of the C bindings section for C++ compilers.                            */
/*-----------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* SYSTICK_H_ */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen systick group.
  *    @}
*/

