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
#ifndef GPIO_H_
#define GPIO_H_

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

#define LED1 GPIOE, GPIO_Pin_3
#define LED2 GPIOE, GPIO_Pin_4
#define ADC_CSn GPIOA, GPIO_Pin_3

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
extern void GPIO_Initialize(void);
extern void RunLight(FunctionalState state);
extern void WDT_State(FunctionalState WDTState);
extern void WDT_SW(void);
extern void Power_ON_OFF(FunctionalState PowerState);
extern void Volt_Gate(FunctionalState GateStage);
extern void Bal_Switch(uint32_t Switch);

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

#endif /* GPIO_H_ */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen gpio group.
  *    @}
*/
