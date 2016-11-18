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
#ifndef BP_DATA_H_
#define BP_DATA_H_

#include <stdio.h> 
#include <stdlib.h>
#include <stdint.h>
#include "BESS_3-0_AHM_celldata_V_1-0.h"


/*-----------------------------------------------------------------------------*/
/* When using C++ compiler, make sure that all definitions have a C binding.   */
/*-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/

#define CELL_NUMBER   16 //(in a BP)


#define CONFIG_INDEX_OVER_SELL_VOLTAGE      0x0//0xE0 Over Cell Voltage
#define CONFIG_INDEX_UNDER_SELL_VOLTAGE     0x1//0xE1 Under Cell Voltage
#define CONFIG_INDEX_OVER_SELL_TEMP         0x2//0xE2 Over Cell Temp
#define CONFIG_INDEX_UNDER_SELL_TEMP        0x3//0xE3 Under Cell Temp
#define CONFIG_INDEX_HCDT                   0x4//0xE4 HCDT : High Cell Delta Temp(Diff Temp)
#define CONFIG_INDEX_HCTR                   0x5//0xE5 HCTR : High Cell Temp Rise
#define CONFIG_INDEX_HCR                    0x6//0xE6 HCR : High Charge Rate
#define CONFIG_INDEX_HDR                    0x7//0xE7 HDR : High Discharge Rate
#define CONFIG_INDEX_MAX                    (CONFIG_INDEX_HDR+1)


#define CHECK_STATUS_IDLE                   0x0
#define CHECK_STATUS_WARNING                0x1
#define CHECK_STATUS_ALARM                  0x2


#define CHECK_WARNING_FLAG_FALSE             0x0
#define CHECK_WARNING_FLAG_TRUE              0x1

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
    uint8_t currentStatus;// = CHECK_STATUS_IDLE;
    uint8_t checkWarningFlag;// = CHECK_STATUS_IDLE;
} AlarmWarningStatus;

//Cell	Cell Volt	BP Voltage	Cell Temp	Max Cell T	Max Cell Temp Rise
//Balance Charger	Warranty Tracker	Max Charge Current	Max Discharge Current	Warranty
typedef struct 
{
    CellData        cell[CELL_NUMBER]; 
    
    uint8_t         balanceChargerStatus;
    
    uint16_t        highCellVolt; 
    uint16_t        lowCellVolt;
    uint16_t        averageCellVolt; 

    uint16_t        measureBPVoltage;
    uint16_t        calculateBPVoltage;

    int16_t         highCellTemp; 
    int16_t         lowCellTemp;
    int16_t         averageCellTemp; 

/*add by jason Xu,2015年8月26日09:29:18*/	
    int16_t         maxCellVolt;
	int16_t         maxCellTemp;
	int16_t         minCellVolt;
	int16_t         minCellTemp;
	
	
/*--------------------------------------*/

	uint16_t        cellDeltaT;
    uint16_t        cellTempRise[CELL_NUMBER];
    int16_t        maxCellTempRise;

    uint32_t        balanceChargerOnTime;

    uint32_t        warrantyTracker;
	
/*add by jason Xu,2015年8月25日10:13:59*/	
	uint8_t         ProductYear;           
	uint8_t         ProductMonth;
	uint8_t         ProductDay;
	uint8_t         WarrantyValid;
/*---------------------------------------*/

    int16_t         maxChargeCurrent; 
    int16_t         maxDischargeCurrent;

    uint32_t        warranty;
    uint16_t        balancingTargetVoltage;   //from System controller
    uint16_t        balancingTargetSOC;    //from System controller
    
    AlarmWarningStatus alarmWarningStatus[CONFIG_INDEX_MAX];
	
	uint8_t          bpstatus;
	uint8_t          bpalarmcount;
	uint8_t          bpwarningcount;
	uint8_t          bperrorcount;
	uint32_t         kwh;
} BPData;


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


#endif /* BP_DATA_H_ */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen gpio group.
  *    @}
*/
