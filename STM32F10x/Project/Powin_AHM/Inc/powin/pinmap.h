/*
 * pinmap.h
 *
 *  Created on: Jun 4, 2015
 *      Author: Yosh
 */

#ifndef PINMAP_H_
#define PINMAP_H_

#include "stm32f10x_conf.h"

/*-----------------------------------------------------------------------------*/
/* Exported Constants                                                          */
/*-----------------------------------------------------------------------------*/
/** @defgroup can_Exported_Constants
  * @{
  */

 

#define IVAMP_PORT			GPIOA
#define IAMP1_PIN			GPIO_Pin_0
#define IAMP2_PIN			GPIO_Pin_1
#define VAMP_PIN			GPIO_Pin_2

#define Contactor_Port      GPIOA
#define CONT1_PORT			GPIOA
#define CONT1_PIN           GPIO_Pin_4

#define CONT1_SNSn_PORT     GPIOA
#define CONT1_SNSn_PIN      GPIO_Pin_5

#define CONT2_PORT          GPIOA
#define CONT2_PIN			GPIO_Pin_6

#define CONT2_SNSn_PORT     GPIOA
#define CONT2_SNSn_PIN      GPIO_Pin_7

#define LED1_PORT		GPIOB
#define LED1_PIN		GPIO_Pin_7
//#define LED2_PORT		GPIOA
//#define LED2_PIN		GPIO_Pin_9



#define STATUS_LED_GREEN_PORT		GPIOB
#define STATUS_LED_GREEN_PIN		GPIO_Pin_9
#define STATUS_LED_RED_PORT		GPIOB
#define STATUS_LED_RED_PIN		GPIO_Pin_8

#define WDG_Port  GPIOC
 
#define WDT  GPIOC , GPIO_Pin_0		//PA11  WatchDog
#endif /* PINMAP_H_ */
