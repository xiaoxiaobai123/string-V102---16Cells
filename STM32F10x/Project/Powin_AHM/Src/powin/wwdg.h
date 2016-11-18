/**

*/ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WWDG_H
#define __WWDG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */

void WWDGInit(void);
void WWDGDeInit(void);

void IWDGInit(void);
void IWDGDeInit(void);
#endif /* __WWDG_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
