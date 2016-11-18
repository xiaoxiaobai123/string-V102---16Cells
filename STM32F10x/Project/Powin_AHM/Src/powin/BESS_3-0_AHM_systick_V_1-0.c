/**
 ******************************************************************************
  * @file       systick.c
  * @author     Nystrom Engineering
  * @version    2.0
  * @date       2014-01-27
  * @copyright  Powin Energy
  * @brief      The file provides of all of the SysTick system timer functions.
  * @page       SysTick System Timer (SysTick) Functions
  * @section    systick_intro Introduction
  * @par
  * SysTick System Timer functions \n
  *
  ******************************************************************************
  */

/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
#include "BESS_3-0_AHM_device_lib_V_1-0.h"

uint32_t milli_count;	// used by sampling routines to count passage of time
uint16_t LED1_Timer  = 0;
uint16_t LED2_Timer  = 0;

/** @addtogroup systick systick
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Private Types                                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup systick_Private_Types
  * @{
  */

/**
  * Close the Doxygen systick__Private_Types group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private_Constants                                                           */
/*-----------------------------------------------------------------------------*/
/** @defgroup systick_Private_Constants
  * @{
  */

/**
  * Close the Doxygen systick_Private_Constants group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private Variable Declarations                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup systick_Private_Variables
  * @{
  */
/**
  * Close the Doxygen systick_Private_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private Function Declarations                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup systick_Private_Functions
  * @{
  */

/**
  * Close the Doxygen systick_Private_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup systick_Exported_Variables
  * @{
  */

uint32_t WorkSecond = 0;
int seconds = 0;    /**< seconds is a "running time" counter for debug message time stamps */
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

/*-----------------------------------------------------------------------------*/
/**
 * @fn        void SysTick_Initialize(void)
 * @brief    This function performs the SysTick Timer initialization.
 *             Systick is initialized to a 1msec (1000Hz) interrupt rate.
 * @param    None
 * @retval    None
 */
void SysTick_Initialize(void)
{
    /* Use SysTick as reference for the timer */
    SysTick_Config(SystemCoreClock / SYSTICK_FREQUENCY_HZ);
	 NVIC_SetPriority (SysTick_IRQn, 0);      /*抢占优先级0，响应优先级1*/
}


/*-----------------------------------------------------------------------------*/
/**
 * fn        void SysTick_Handler(void)
 * @brief    This function is the SysTick Timer Interrupt Handler.
 * @param    None
 * @retval    None
 */
extern void checkLed(void);
void SysTick_Handler_Powin(void)
{
    #if(1)
    static uint16_t LEDTimerTemp  = 0;
    LEDTimerTemp++;
    if( LEDTimerTemp == 1000 )
    {
        ++seconds;
        LEDTimerTemp = 0;
    }
    #endif
 
     checkLed();

    CANTimer++;        // CAN timeout timer
    milli_count++;     // used by sampling routines to count passage of time
    TimingDelay_Decrement();
    //printf("O");
}

/**
  * Close the Doxygen systick_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen adc group.
  *    @}
*/

/* End of systick.c */
