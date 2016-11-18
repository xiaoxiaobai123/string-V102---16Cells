/**
  ******************************************************************************
  * @file    USART/HyperTerminal_Interrupt/stm32f10x_it.h 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PRINTF_DEBUG_H
#define __PRINTF_DEBUG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define DEBUG_UART_1    0x0
#define DEBUG_UART_3    0x2
#define DEBUG_UART_5    0x5
#define DEBUG_TARGET_UART   DEBUG_UART_5  

#if (DEBUG_TARGET_UART == DEBUG_UART_1)
  #define DEBUG_USARTx_IRQHandler   USART1_IRQHandler
  #define DEBUG_UART   USART1
  #define DEBUG_USARTx_IRQn   USART1_IRQn
#elif (DEBUG_TARGET_UART == DEBUG_UART_3)
  #define DEBUG_USARTx_IRQHandler   USART3_IRQHandler
  #define DEBUG_UART                USART3
  #define DEBUG_USARTx_IRQn         USART3_IRQn
#elif (DEBUG_TARGET_UART == DEBUG_UART_5)
  #define DEBUG_USARTx_IRQHandler   UART5_IRQHandler
  #define DEBUG_UART                UART5
  #define DEBUG_USARTx_IRQn         UART5_IRQn
#else
  #define DEBUG_USARTx_IRQHandler
#endif

#define DEBUG_UART_TxBufferSize   (countof(TxBuffer) - 1)
#define DEBUG_UART_RxBufferSize   0x20

/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Exported functions ------------------------------------------------------- */

void PrintfInit(void);

#endif /* __PRINTF_DEBUG_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
