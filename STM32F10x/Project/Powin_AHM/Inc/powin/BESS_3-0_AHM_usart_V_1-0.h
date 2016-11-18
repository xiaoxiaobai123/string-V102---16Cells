/**
  ******************************************************************************
  * @file        BESS_3-0_AHM_usart_V_1-0.h
  * @author      Nystrom Engineering
  * @version     2.0.0
  * @date        Jan 27, 2014
  * @copyright   Powin Engineering
  * @brief       The include file for the Universal Serial Asynchronous Receiver Transmitter (USART).
  ******************************************************************************
  */

/*-----------------------------------------------------------------------------*/
/* Define to prevent recursive inclusion                                       */
/*-----------------------------------------------------------------------------*/
#ifndef USART_H_
#define USART_H_

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
#include "BESS_3-0_AHM_can_V_1-0.h"

/** @addtogroup usart usart
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Exported Types                                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup usart_Exported_Types
  * @{
  */

/**
  * Close the Doxygen usart__Exported_Types group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Constants                                                          */
/*-----------------------------------------------------------------------------*/
/** @defgroup usart_Exported_Constants
  * @{
  */
#define USART_RX_Buffer_MAX  (16)
#define USART_TX_Buffer_MAX  (32)
/**
  * Close the Doxygen usart_Exported_Constants group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Macros                                                             */
/*-----------------------------------------------------------------------------*/
/** @defgroup usart_Exported_Macros
  * @{
  */

/**
  * Close the Doxygen usart_Exported_Macros group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup usart_Exported_Variables
  * @{
  */
/* USART receiving and sending buffer info showing below,
all the received interruption function will not be processed, in case of delay*/
extern uint8_t USART_RX_Buffer[USART_RX_Buffer_MAX][32];
extern uint8_t USART_RX_WaitLoad;
extern uint8_t USART_RX_WaitProcess;
extern uint8_t USART_RX_Buffer_State[USART_RX_Buffer_MAX];
//Current receiving buffer status
//00:EMPTY
//01:RECEIVEING
//02:RECEIVED
//03:PROCESSING

extern uint8_t USART_TX_Buffer[USART_TX_Buffer_MAX][32];
extern uint8_t USART_TX_WaitLoad;
extern uint8_t USART_TX_WaitSend;
extern uint8_t USART_TX_Buffer_State[USART_TX_Buffer_MAX];
//Current sending buffer status
//00:EMPTY
//01:LOADING
//02:LAODED
//03:SENDING

/**
  * Close the Doxygen usart_Exported_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Function Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup usart_Exported_Functions
  * @{
  */
extern void USART_config(void);
extern void USART_Device_Init(void);
extern void USART_Data_Init(void);
extern void USART_Load_Buffer(MESSAGE *Message);
extern void USART_Load_Message(MESSAGE *Message);
extern void USART_RX_Process(void);
extern void USART_TX_Process(void);
extern void USART1_IRQHandler(void);
/**
  * Close the Doxygen usart_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* End of the C bindings section for C++ compilers.                            */
/*-----------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* USART_H_ */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen usart group.
  *    @}
*/
