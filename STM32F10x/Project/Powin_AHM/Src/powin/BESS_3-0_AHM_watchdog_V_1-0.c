/**
  ******************************************************************************
  * @file       watchdog.c
  * @author     Nystrom Engineering
  * @version    2.0.0
  * @date       Jan 28, 2014
  * @copyright  Powin Energy
  * @brief      This file provides all the watchdog timer functions.
  * @page       watchdog_page Watchdog Timer Functions
  *  @section watchdog_intro Introduction
  *  @par
  *    The "watchdog" functions are used to detect a failure of the battery management system \n
  *    to respond, and will cause a complete reset of the BMS board by asserting RST. \n
  *  @par
  *    The Powin BMS incorporates three (3) "watchdog" timer functions : \n
  *    - External Exar SP706R Microprocessor Supervisor
  *    - Internal IWDG Independent Watchdog Timer
  *    - Internal WWDG Windowed Watchdog Timer
  *  @par
  *    These three (3) timers are used in conjunction to insure that the firmware \n
  *    is running in a predictable manner and that the BMS system is responding. \n
  *  @par
  *    The Exar SP706R is a microprocessor (uP) supervisory circuit that monitors \n
  *    the power supplied to digital circuits, such as the STM microprocessor. \n
  *    A microprocessor's reset input starts the uP in a known state. \n
  *    The SP706R/S/T-SP708R/S/T series asserts reset during power-up \n
  *    and prevents code execution errors during power down or brownout conditions. \n
  *    On power-up, once Vcc reaches 1.0 Volts, RESET is a guaranteed logic LOW of 0.4 Volts or less. \n
  *    As Vcc rises, RESET stays LOW. When Vcc rises above the reset threshold, \n
  *    an internal timer releases RESET after 200ms. \n
  *    RESET pulses LOW whenever Vcc dips below the reset threshold, \n
  *    such as in a power brownout condition. \n
  *  @par
  *    The Exar SP706R watchdog circuit monitors the STM microprocessor activity. \n
  *    If the STM microprocessor does not toggle the watchdog input (WDI) within 1.6 seconds, \n
  *    and WDI is not tri-stated, then the WDO line goes LOW and will reset the STM microprocessor. \n
  *    As long as RESET is asserted or the WDI input is tri-stated, \n
  *    the Exar SP706R watchdog timer will stay cleared and will not count. \n
  *    As soon as  RESET is released and WDI is driven HIGH or LOW, \n
  *    then the Exar SP706R watchdog timer will start counting. \n
  *    The STM microcontroller Port C/Bit 0 (PC0) GPIO line is used to toggle the \
  *    Exar SP706R watchdog input (WDI) line within 1.6 seconds to prevent a power-on-reset. \n
  *    BMS Board Jumper J1 (WDT) can be removed to disable the Exar SP706R watchdog timer reset. \n
  *  @par
  *    The STM32 internal independent watchdog (IWDG) is clocked by its own \n
  *    dedicated low-speed clock (LSI) and thus stays active even if the main clock fails. \n
  *    The STM32 internal window watchdog (WWDG) clock is prescaled from the APB1 clock \n
  *    and has a configurable time-window that can be programmed to detect \n
  *    abnormally late or early application behavior. \n
  *  @par
  *    The IWDG is best suited to applications which require the watchdog to run \n
  *    as a totally independent process outside the main application, \n
  *    but to have lower timing accuracy constraints. The WWDG is best suited to \n
  *    applications which require the watchdog to react within an accurate timing window. \n
  *  @par
  *    In the Powin Energy BMS board firmware, the following functions will be used to \n
  *    to interface and control the watchdog timer system : \n
  *    - Exar_WatchDog_Set(void) : Punch (toggle) the external Exar SP706R watchdog timer. \n
  *    - IWDG_Watchdog_Set(void) : Punch (toggle) the TM32 internal independent watchdog (IWDG). \n
  *    - WWDG_Watchdog_Set(void) : Punch (toggle) the TM32 internal windowing watchdog (IWDG). \n
  *    - WatchDog_Set(void)      : Punch (toggle) all watchdog timers. \n
  *  @par
  *
  ******************************************************************************
  */
/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
#include "BESS_3-0_AHM_device_lib_V_1-0.h"

/** @addtogroup watchdog watchdog
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Private Types                                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup watchdog_Private_Types
  * @{
  */

/**
  * Close the Doxygen watchdog__Private_Types group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private_Constants                                                           */
/*-----------------------------------------------------------------------------*/
/** @defgroup watchdog_Private_Constants
  * @{
  */

/**
  * Close the Doxygen watchdog_Private_Constants group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private Variable Declarations                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup watchdog_Private_Variables
  * @{
  */

/**
  * Close the Doxygen watchdog_Private_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private Function Declarations                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup watchdog_Private_Functions
  * @{
  */

/**
  * Close the Doxygen watchdog_Private_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup watchdog_Exported_Variables
  * @{
  */

/*-----------------------------------------------------------------------------*/
/**
 * @fn        void IWDG_WatchDog_Initialize(void)
 * @brief    This function initializes the independent watchdog timer (IWDG).
 * @param    None
 * @retval    None
 */
void IWDG_WatchDog_Initialize(void)
{
    uint32_t LsiFreq = 40000;

    /* Enable write access to IWDG_PR and IWDG_RLR registers */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    /* IWDG counter clock: LSI/32 */
    IWDG_SetPrescaler(IWDG_Prescaler_32);

    /* Set counter reload value to obtain 250ms IWDG TimeOut.
    Counter Reload Value = 250ms/IWDG counter clock period
    = 250ms / (LSI/32)
    = 0.25s / (LsiFreq/32)
    = LsiFreq/(32 * 4)
    = LsiFreq/128
    */
    IWDG_SetReload(LsiFreq/128);

    /* Reload IWDG counter */
    IWDG_ReloadCounter();

    /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
    IWDG_Enable();

}
/*-----------------------------------------------------------------------------*/
/**
 * @fn        void IWDG_WatchDog_Set(void)
 * @brief    This function Punches (toggles) the independent watchdog timer (IWDG).
 * @param    None
 * @retval    None
 */
void IWDG_WatchDog_Set(void)
{
    /* Reload IWDG counter */
    IWDG_ReloadCounter();
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn        uint32_t IWDG_WatchDog_Get(void)
 * @brief    This function checks if the system has resumed from an IWDG reset.
 * @param    None
 * @retval    1 if resumed from an IWDG reset, 0 otherwise
 */
uint32_t IWDG_WatchDog_Get(void)
{
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) {
        /* Clear reset flags */
        RCC_ClearFlag();
        return(1);
    }
    else {
        return(0);
    }
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn        void Exar_WatchDog_Initialize(void)
 * @brief    This function initializes the external Exar SP706R watchdog timer GPIO.
 * @param    None
 * @retval    None
 */
void Exar_WatchDog_Initialize(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;      // Watchdog driver port
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //        Push-pull output
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn        void Exar_WatchDog_Deinitialize(void)
 * @brief    This function is intended to disable the external Exar SP706R watchdog timer GPIO.
 *             @par
 *             Note that if the WDT Jumper is installed, this function will have no effect. \n
 *             In order for the external Exar watchdog to be truly disabled, \n
 *             another GPIO line must be used (asserted) to physically disable the device. \n
 *             In short, this routine has no real effect (if that was the original intention.)
 * @param    None
 * @retval    None
 */
void Exar_WatchDog_Deinitialize(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;      // Watchdog driver port
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //        Push-pull output
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn        void Exar_WatchDog_Set(void)
 * @brief    This function Punches (toggles) the external Exar SP706R watchdog timer.
 * @param    None
 * @retval    None
 */
void Exar_WatchDog_Set(void)
{
//    GPIO_SetBits(WDT);
    //Delay(50);
//    GPIO_ResetBits(WDT);
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn        uint32_t Exar_WatchDog_Get(void)
 * @brief    This function checks if the system has resumed from an external reset.
 * @param    None
 * @retval    1 if resumed from an external reset, 0 otherwise
 */
uint32_t Exar_WatchDog_Get(void)
{
    if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET) {
        /* Clear reset flags */
        RCC_ClearFlag();
        return(1);
    }
    else {
        return(0);
    }
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn        void WWDG_WatchDog_Initialize(void)
 * @brief    This function initializes the window watchdog timer (WWDG).
 * @param    None
 * @retval    None
 */
void WWDG_WatchDog_Initialize(void)
{
    /* WWDG configuration */
    /* Enable WWDG clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

    /* On Medium devices, WWDG clock counter = (PCLK1 (72MHz)/4096)/8 = 2197 Hz (~4551 us)  */
    /* On other devices, WWDG clock counter = (PCLK1(36MHz)/4096)/8 = 1099 Hz (~910 us)  */
    WWDG_SetPrescaler(WWDG_Prescaler_8);

    /* Set Window value to 80; WWDG counter should be refreshed only when the counter
    is below 80 (and greater than 64) otherwise a reset will be generated */
    WWDG_SetWindowValue(80);

    /* - On Value line devices,
    Enable WWDG and set counter value to 127, WWDG timeout = ~4551 us * 64 = 291.26 ms
    In this case the refresh window is: ~4551us * (127-80) = 291.26 ms < refresh window < ~4551us * 64 = 291.26ms
    - On other devices
    Enable WWDG and set counter value to 127, WWDG timeout = ~910 us * 64 = 58.25 ms
    In this case the refresh window is: ~910 us * (127-80) = 42.77 ms < refresh window < ~910 us * 64 = 58.25ms
    */
    WWDG_Enable(127);
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn        void WWDG_WatchDog_Set(void)
 * @brief    This function Punches (toggles) the window watchdog timer (WWDG).
 * @param    None
 * @retval    None
 */
void WWDG_WatchDog_Set(void)
{
    /* Update WWDG counter */
    WWDG_SetCounter(127);
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn        uint32_t WWDG_WatchDog_Get(void)
 * @brief    This function checks if the system has resumed from an WWDG reset.
 * @param    None
 * @retval    1 if resumed from an WWDG reset, 0 otherwise
 */
uint32_t WWDG_WatchDog_Get(void)
{
    if (RCC_GetFlagStatus(RCC_FLAG_WWDGRST) != RESET) {
        /* Clear reset flags */
        RCC_ClearFlag();
        return(1);
    }
    else {
        return(0);
    }
}

/**
  * Close the Doxygen watchdog_Exported_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Function Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup watchdog_Exported_Functions
  * @{
  */

/**
  * Close the Doxygen watchdog_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen watchdog group.
  *    @}
*/

/* End of watchdog.c */
