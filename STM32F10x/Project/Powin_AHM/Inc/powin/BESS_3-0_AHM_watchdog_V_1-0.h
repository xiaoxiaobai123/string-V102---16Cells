/**
  ******************************************************************************
  * @file        BESS_3-0_AHM_watchdog_V_1-0.h
  * @author      Nystrom Engineering
  * @version     2.0.0
  * @date        Jan 28, 2014
  * @copyright   Powin Energy
  * @brief       The include file for all the watchdog timer functions.
  ******************************************************************************
  */

/*-----------------------------------------------------------------------------*/
/* Define to prevent recursive inclusion                                       */
/*-----------------------------------------------------------------------------*/
#ifndef WATCHDOG_H_
#define WATCHDOG_H_

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


/** @addtogroup watchdog watchdog
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Exported Types                                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup watchdog_Exported_Types
  * @{
  */

/**
  * Close the Doxygen watchdog__Exported_Types group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Constants                                                          */
/*-----------------------------------------------------------------------------*/
/** @defgroup watchdog_Exported_Constants
  * @{
  */

/**
  * Close the Doxygen watchdog_Exported_Constants group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Macros                                                             */
/*-----------------------------------------------------------------------------*/
/** @defgroup watchdog_Exported_Macros
  * @{
  */

/**
  * Close the Doxygen watchdog_Exported_Macros group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/* Exported Function Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup watchdog_Exported_Functions
  * @{
  */
extern void Exar_WatchDog_Set(void);
extern void IWDG_WatchDog_Set(void);
extern void WWDG_WatchDog_Set(void);

extern uint32_t Exar_WatchDog_Get(void);
extern uint32_t IWDG_WatchDog_Get(void);
extern uint32_t WWDG_WatchDog_Get(void);

extern void Exar_WatchDog_Initialize(void);
extern void IWDG_WatchDog_Initialize(void);
extern void WWDG_WatchDog_Initialize(void);

/**
  * Close the Doxygen watchdog_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* End of the C bindings section for C++ compilers.                            */
/*-----------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H_ */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen watchdog group.
  *    @}
*/
