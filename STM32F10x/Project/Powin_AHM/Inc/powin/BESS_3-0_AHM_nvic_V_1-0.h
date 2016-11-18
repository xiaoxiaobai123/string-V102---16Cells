/**
  ******************************************************************************
  * @file        BESS_3-0_AHM_nvic_V_1-0.h
  * @author      Nystrom Engineering
  * @version     2.0.0
  * @date        2014-02-01
  * @copyright   Powin Energy
  * @brief       The include file for the Nested Vectored Interrupt Controller functions.
  ******************************************************************************
  */

/*-----------------------------------------------------------------------------*/
/* Define to prevent recursive inclusion                                       */
/*-----------------------------------------------------------------------------*/
#ifndef NVIC_H_
#define NVIC_H_

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


/** @addtogroup nvic nvic
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Exported Types                                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup nvic_Exported_Types
  * @{
  */

#define NVIC_SETPRIMASK __disable_irq            /**<  */
#define NVIC_RESETPRIMASK __enable_irq            /**<  */

/**
  * Close the Doxygen nvic__Exported_Types group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Constants                                                          */
/*-----------------------------------------------------------------------------*/
/** @defgroup nvic_Exported_Constants
  * @{
  */

/**
  * Close the Doxygen nvic_Exported_Constants group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Macros                                                             */
/*-----------------------------------------------------------------------------*/
/** @defgroup nvic_Exported_Macros
  * @{
  */

/**
  * Close the Doxygen nvic_Exported_Macros group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup nvic_Exported_Variables
  * @{
  */

/**
  * Close the Doxygen nvic_Exported_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Function Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup nvic_Exported_Functions
  * @{
  */

/**
  * Close the Doxygen nvic_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* End of the C bindings section for C++ compilers.                            */
/*-----------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* NVIC_H_ */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen nvic group.
  *    @}
*/
