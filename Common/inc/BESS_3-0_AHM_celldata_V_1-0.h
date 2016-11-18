/**
  ******************************************************************************
  * @file        BESS_3-0_AHM_gpio_V_1-0.h
  * @author      Nystrom Engineering
  * @version     2.0.0
  * @date        2014-01-27
  * @copyright   Powin Engineering
  * @brief       The include file for the General Purpose I/O pins.
  ******************************************************************************
  */

/*-----------------------------------------------------------------------------*/
/* Define to prevent recursive inclusion                                       */
/*-----------------------------------------------------------------------------*/
#ifndef CELL_DATA_H_
#define CELL_DATA_H_
#include <stdio.h> 
#include <stdlib.h>
#include <stdint.h>
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


/** @addtogroup gpio gpio
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Exported Types                                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup gpio_Exported_Types
  * @{
  */
//Volt	Temp	Resistor
typedef struct 
{
    uint16_t    volt;  
    int16_t     temp; 
    uint8_t     resistor;
} CellData;


/**
  * Close the Doxygen gpio__Exported_Types group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Constants                                                          */
/*-----------------------------------------------------------------------------*/
/** @defgroup gpio_Exported_Constants
  * @{
  */


/**
  * Close the Doxygen gpio_Exported_Constants group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Macros                                                             */
/*-----------------------------------------------------------------------------*/
/** @defgroup gpio_Exported_Macros
  * @{
  */
#define CELL_DATA_RESISTOR_ON       0x1
#define CELL_DATA_RESISTOR_OFF      0x0
/**
  * Close the Doxygen gpio_Exported_Macros group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup gpio_Exported_Variables
  * @{
  */

/**
  * Close the Doxygen gpio_Exported_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Function Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup gpio_Exported_Functions
  * @{
  */

/**
  * Close the Doxygen gpio_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* End of the C bindings section for C++ compilers.                            */
/*-----------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* CELL_DATA_H_ */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen gpio group.
  *    @}
*/
