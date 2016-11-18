/**
  ******************************************************************************
  * @file        BESS_3-0_AHM_adc_V_1-0.h
  * @author      Nystrom Engineering
  * @version     2.0.0
  * @date        2014-01-27
  * @copyright   Powin Energy
  * @brief       The include file for the  Analog to Digital Converter (ADC) functions.
  ******************************************************************************
  */

/*-----------------------------------------------------------------------------*/
/* Define to prevent recursive inclusion                                       */
/*-----------------------------------------------------------------------------*/
#ifndef ADC_H_
#define ADC_H_

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
#include "BESS_3-0_AHM_device_lib_V_1-0.h"

/** @addtogroup adc
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Exported Types                                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup adc_Exported_Types
  * @{
  */

/**
  * Close the Doxygen adc__Exported_Types group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Constants                                                          */
/*-----------------------------------------------------------------------------*/
/** @defgroup adc_Exported_Constants adc_Exported_Constants
  * @{
  */
#define VOLT_SAMPLE_AVG_DEPTH  4           /**< Maximum voltage sampling number */
#define VOLT_SAMPLE_AVG_SHIFT  2          //!< log2(VOLT_SAMPLE_AVG_DEPTH)
#define TEMP_SAMPLE_AVG_DEPTH  4          /**< Maximum temperature sampling number */
#define TEMP_SAMPLE_AVG_SHIFT  2          //!< log2(TEMP_SAMPLE_AVG_DEPTH)
#define SAMPLE_EXCLUSION        0         //!< if non-zero, throws away high and low samples
#define ADC_VREF                3.0

#define Cell0ADResultToVolt (14652)        /**< Cell 0 voltage, corresponding to one AD value */
#define Cell1ADResultToVolt (14652)        /**< Cell 1 voltage, corresponding to one AD value */
#define Cell2ADResultToVolt (21978)        /**< Cell 2 voltage, corresponding to one AD value */

#define CellADResultToVolt    (10920)        /**< Cell's voltage, corresponding to one AD value, 1.0910mV */
#define PackADResultToVolt    (163198)    /**< Battery pack voltage, corresponding to one AD value, 16.3198mV */

#define VoltSampleTime        ADC_SampleTime_28Cycles5 /**< ?Settling time for VoltSample */
#define TempSampleTime        ADC_SampleTime_13Cycles5 /**< ?Settling time for TempSample */

/**
  * Close the Doxygen adc_Exported_Constants group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Macros                                                             */
/*-----------------------------------------------------------------------------*/
/** @defgroup adc_Exported_Macros adc_Exported_Macros
  * @{
  */

/**
  * Close the Doxygen adc_Exported_Macros group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup adc_Exported_Variables adc_Exported_Variables
  * @{
  */
extern uint16_t VoltSampDelay;
extern uint16_t TempSampDelay;
extern uint16_t samples_complete;
extern uint16_t TempUpdate;
extern uint16_t VoltSampleGateDelay;

/**
  * Close the Doxygen adc_Exported_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Function Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup adc_Exported_Functions adc_Exported_Functions
  * @{
  */
void ADC_Initialize(void);
int16_t ADCToTemp(uint16_t ADCResult);
uint16_t Get_ADCResult(uint16_t avg_depth, uint8_t avg_shift);
int32_t sample_task(void);
extern void Update_Temp(void);
extern void SelfCal(void);
extern void SelfCheck(void);

// these are the values that adc_run_request can assume.
// the normal one scales the readings by mv/count and cal factors;
// the raw one stores raw a/d counts for calibration purposes.
#define ADC_RUN_NORMAL 1
#define ADC_RUN_RAW    2
extern uint8_t adc_run_request;  // when non-zero, causes ADC state machine to activate. When done,
// ADC state machine checks this flag and generates CAN response.
/**
  * Close the Doxygen adc_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* End of the C bindings section for C++ compilers.                            */
/*-----------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* ADC_H_ */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen adc group.
  *    @}
*/
