/*
 * BESS_3-0_AHM_main_V_1-0.h
 *
 *  Created on: Jun 24, 2014
 *      Author: Yosh
 */

/*-----------------------------------------------------------------------------*/
/* Define to prevent recursive inclusion                                       */
/*-----------------------------------------------------------------------------*/
#ifndef MAIN_H_
#define MAIN_H_
#include <stdint.h>
/*-----------------------------------------------------------------------------*/
/* Exported Types                                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup can_Exported_Types
  * @{
  */

/**
  * Close the Doxygen can__Exported_Types group.
  * @}
  */
 

/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup main_Exported_Variables
  * @{
  */
extern int32_t sys_damps;				//system current in deci-amps
extern int32_t sys_dvolts;				//system voltage in deci-volts
extern int32_t sys_cwatts;				//system power in centi-watts
extern int32_t capacity_count;			//appears to be system centi-amp hours
extern int32_t cwatthour_count;			//appears to be system centi-watt hours
extern int32_t capacity_count_remainder;
extern int32_t cwatthour_count_remainder;
extern float i_shunt_value;     // this is in siemens so it can be multiplied, but stored in ohms in EEPROM
extern uint32_t last_can_msg;
extern int8_t Switch_Status;			//
extern int8_t startup_status;			//The POST Status for the board?
extern uint8_t status;					//General status of the board?



/**
  * Close the Doxygen main_Exported_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Constants                                                          */
/*-----------------------------------------------------------------------------*/
/** @defgroup can_Exported_Constants
  * @{
  */
#define RELAY_PIN GPIOA,GPIO_Pin_2

#define SW_REV_MIN 0	//Software Revision minor
#define SW_REV_MAJ 2	//Software Revision major
#define HW_REV_MIN 0	//Hardware revision minor
#define HW_REV_MAJ 5	//Hardware revision major
/**
	* Close the Doxygen main_Exported_Variables group.
	* @}
	*/
// by sam
#define BOARD_TYPE_EVB  0x0
#define BOARD_TYPE_POWIN_1   0x1

#define CURRENT_BOARD_TYPE  BOARD_TYPE_EVB


#endif /* MAIN_H_ */
