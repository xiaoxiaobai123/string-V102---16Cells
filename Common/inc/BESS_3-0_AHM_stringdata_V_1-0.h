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
#ifndef STRING_DATA_H_
#define STRING_DATA_H_
#include <stdio.h> 
#include <stdlib.h>
#include <stdint.h>
#include "BESS_3-0_AHM_bpdata_V_1-0.h"
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

#define BP_NUMBER   22 //(In a String)
#define CELL_NUMBER 16
/** @addtogroup gpio gpio
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Exported Types                                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup gpio_Exported_Types
  * @{
  */
typedef struct 
{
    BPData          bp[BP_NUMBER]; 
    int32_t         DefaultCellAH;           // Add by jason,2015年8月23日16:16:16
	int32_t         DefaultCellKWH;           // Add by jason,2015年8月27日13:34:54
    uint16_t        highCellVolt;           //Calculate from BPData
    uint16_t        lowCellVolt;            //Calculate from BPData
    uint16_t        averageCellVolt;        //Calculate from BPData

    uint16_t        measureStringVoltage;
    uint16_t        calculateStringVoltage; //Calculate from BPData
	
	uint16_t		dcbusVoltage;
    uint16_t        soc;

    int16_t         highCellTemp;           //Calculate from BPData
    int16_t         lowCellTemp;            //Calculate from BPData
    int16_t         averageCellTemp;        //Calculate from BPData

    uint16_t        maxCellDeltaT;          //Calculate from BPData
    uint16_t        maxCellTempRise;        //Calculate from BPData

    uint8_t         positiveContactor;
    uint8_t         negativeContactor;

    uint16_t        groundfaultvoltage1;
	uint16_t        groundfaultvoltage2;
	uint16_t        groundLeakge; 
    
    int16_t         stringCurrent; 

    int16_t         AHTimer;                   //Add by jason,2015年8月26日14:00:49
	uint16_t         CalAHTimer;
	
    uint16_t        ah;                          
    uint16_t        ahCounter;
    uint32_t        kW;
    uint32_t        kWh;
    uint16_t         StringID;
	uint8_t         SelectedBPID;
	uint8_t         ArrayID;
	uint16_t        capacity;
	uint16_t        chargeefficiency;
	
    uint32_t        averageBalanceCharger;

    uint16_t        totalChargekWh;
    uint16_t        totalDischargekWh;

    uint16_t        balancingTargetVoltage;  //from System controller
    uint16_t        balancingTargetSOC;  //from System controller
	
	int32_t          AHTemp;
	int32_t          KWHTemp;
	
	uint8_t          state;
	uint8_t          mode;

} StringData;
extern volatile StringData mStringData;  //Add by jason,2015年8月23日17:29:41

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
//#define CELL_DATA_RESISTOR_ON       0x1
//#define CELL_DATA_RESISTOR_OFF      0x0
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

#endif /* STRING_DATA_H_ */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen gpio group.
  *    @}
*/
