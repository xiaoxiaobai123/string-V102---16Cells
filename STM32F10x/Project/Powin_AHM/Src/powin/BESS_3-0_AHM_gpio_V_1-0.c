/**
 ******************************************************************************
  * @file        gpio.c
  * @author      Nystrom Engineering
  * @version     2.0.0
  * @date        2014-01-27
  * @copyright   Powin Energy
  * @brief       This file provides all the General Purpose Input/Output Pin functions.
  * @page        gpio_page General Purpose Input/Output Pin Functions
  * @section     gpio_intro Introduction
  * @par
  *  General Purpose Input/Output Pins \n
  *
 ******************************************************************************
*/

/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
#include "BESS_3-0_AHM_device_lib_V_1-0.h"

/** @addtogroup gpio gpio
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Private Types                                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup gpio_Private_Types
  * @{
  */

/**
  * Close the Doxygen gpio__Private_Types group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private_Constants                                                           */
/*-----------------------------------------------------------------------------*/
/** @defgroup gpio_Private_Constants
  * @{
  */

/**
  * Close the Doxygen gpio_Private_Constants group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private Variable Declarations                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup gpio_Private_Variables
  * @{
  */

/**
  * Close the Doxygen gpio_Private_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private Function Declarations                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup gpio_Private_Functions
  * @{
  */

/**
  * Close the Doxygen gpio_Private_Functions group.
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

/*-----------------------------------------------------------------------------*/
/**
 * @fn     void GPIO_Initialize(void)
 * @brief  This function performs the GPIO configuration, initialization, and setup
 * @detail Configure all unused GPIO port pins in Analog Input mode
           (with floating input trigger OFF) in order to reduce the power consumption
           and to increase the device immunity against EMI/EMC.
 */
void GPIO_Initialize(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clock for Ports(s)A,B,C,D,E */
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA |
        RCC_APB2Periph_GPIOB |
        RCC_APB2Periph_GPIOC |
        RCC_APB2Periph_GPIOD |
        RCC_APB2Periph_GPIOE,
        ENABLE);

    /* Configure all unused GPIO port pins in Analog Input mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;

    /* Configure all GPIO using the default GPIO_InitStructure */
    /** @todo Verify the default GPIO_InitStructure used here - what is it initialized with? */
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    GPIO_Init(GPIOB,&GPIO_InitStructure);
    GPIO_Init(GPIOC,&GPIO_InitStructure);
    GPIO_Init(GPIOD,&GPIO_InitStructure);
    GPIO_Init(GPIOE,&GPIO_InitStructure);

    /////////
    // Port A
    // gpios
    GPIO_InitStructure.GPIO_Pin =
        GPIO_Pin_2  |  // RELAY
        GPIO_Pin_3  |  // CSn    (chip select to A/D converter)
		GPIO_Pin_11 |
        0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // Push-pull output
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // spi
    GPIO_InitStructure.GPIO_Pin =
        GPIO_Pin_5  |  // spi sck
        GPIO_Pin_6  |  // spi miso
        GPIO_Pin_7  |  // spi mosi
        0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // SPI push-pull
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /////////
    // Port B
    // i2c
    GPIO_InitStructure.GPIO_Pin =
        GPIO_Pin_10 |  // SCL
        GPIO_Pin_11 |  // SDA
        0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; // Push-pull output
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // CAN
//    GPIO_InitStructure.GPIO_Pin =
//    	GPIO_Pin_8 |  // CAN RX
//        0;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // input, no pull
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//    GPIO_InitStructure.GPIO_Pin =
//    	GPIO_Pin_9 |  // CAN TX
//        0;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Push-pull output
//    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure the "E" output ports */
    GPIO_InitStructure.GPIO_Pin =
        GPIO_Pin_3  |      // LED1
        GPIO_Pin_4  |      // LED2
        0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //        Push-pull output
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/**
  * Close the Doxygen gpio_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen gpio group.
  *    @}
*/

/* End of gpio.c */
