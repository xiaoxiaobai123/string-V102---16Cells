/**
  ******************************************************************************
  * @file    USART/HyperTerminal_Interrupt/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "booterlib.h"
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "memoryconfig.h"
void SetBootConfig(uint8_t flag)
{
    
    FLASH_Status status;
    printf("SetBootConfig: 0x%x...\r\n", flag);
    FLASH_Unlock();
    FLASH_EraseOptionBytes();
    status = FLASH_ProgramOptionByteData(USER_FLASH_OPTION_BYTE_ADDRESS, flag);
    FLASH_Lock();
    printf("SetBootConfig check again : 0x%x (status = 0x%x)...\r\n", (*(__IO uint16_t*)USER_FLASH_OPTION_BYTE_ADDRESS)&0xff, status);
   
}

uint8_t GetBootConfig(void)
{
    return (*(__IO uint16_t*)USER_FLASH_OPTION_BYTE_ADDRESS)&0xff;
}

uint8_t ShowBootConfig(void)
{
    int i = 0;
    for (i=0; i<0x10; i = i+2)
    {
        printf(" OptionBytes [0x%08x]: 0x%04x)...\r\n", 0x1FFFF800 + i, (*(__IO uint16_t*)(0x1FFFF800 + i))&0xffff);
        
    }
    return (*(__IO uint16_t*)USER_FLASH_OPTION_BYTE_ADDRESS)&0xff;
}

/*
#define RCC_FLAG_HSIRDY                  ((uint8_t)0x21)
#define RCC_FLAG_HSERDY                  ((uint8_t)0x31)
#define RCC_FLAG_PLLRDY                  ((uint8_t)0x39)
#define RCC_FLAG_LSERDY                  ((uint8_t)0x41)
#define RCC_FLAG_LSIRDY                  ((uint8_t)0x61)
#define RCC_FLAG_PINRST                  ((uint8_t)0x7A)
#define RCC_FLAG_PORRST                  ((uint8_t)0x7B)
#define RCC_FLAG_SFTRST                  ((uint8_t)0x7C)
#define RCC_FLAG_IWDGRST                 ((uint8_t)0x7D)
#define RCC_FLAG_WWDGRST                 ((uint8_t)0x7E)
#define RCC_FLAG_LPWRRST                 ((uint8_t)0x7F)

    Bit31: LPWRRSTF – Low-power reset flag
    Set by hardware when a Low-power management reset occurs.
    Bit30: WWDGRSTF – Window watchdog reset flag
    Set by hardware when a window watchdog reset occurs.
    Bit29: IWDGRSTF – Independent watchdog reset flag
    Set by hardware when an independent watchdog reset from VDD domain occurs.
    Bit28: SFTRSTF – Software reset flag
    Set by hardware when a software reset occurs.
    Bit27: PORRSTF – POR/PDR reset flag
    Set by hardware when a POR/PDR reset occurs.
    Bit26: PINRSTF – PIN reset flag
    Set by hardware when a reset from the NRST pin occurs.
    Bit25: BORRSTF – BOR reset flag
    Set by hardware when a POR/PDR or BOR reset occurs.

******************  Bit definition for RCC_CSR register  ******************* 
#define  RCC_CSR_LSION                       ((uint32_t)0x00000001)        !< Internal Low Speed oscillator enable 
#define  RCC_CSR_LSIRDY                      ((uint32_t)0x00000002)        !< Internal Low Speed oscillator Ready 
#define  RCC_CSR_RMVF                        ((uint32_t)0x01000000)        !< Remove reset flag 
#define  RCC_CSR_PINRSTF                     ((uint32_t)0x04000000)        !< PIN reset flag 
#define  RCC_CSR_PORRSTF                     ((uint32_t)0x08000000)        !< POR/PDR reset flag 
#define  RCC_CSR_SFTRSTF                     ((uint32_t)0x10000000)        !< Software Reset flag 
#define  RCC_CSR_IWDGRSTF                    ((uint32_t)0x20000000)        !< Independent Watchdog reset flag 
#define  RCC_CSR_WWDGRSTF                    ((uint32_t)0x40000000)        !< Window watchdog reset flag 
#define  RCC_CSR_LPWRRSTF                    ((uint32_t)0x80000000)        !< Low-Power reset flag 
*/
uint8_t GetResetSource(void)
{
    uint8_t reval = 0x0;
    if (PWR_GetFlagStatus(PWR_FLAG_SB))
    {
        printf("System resumed from STANDBY mode\r\n");
        reval = PWR_FLAG_SB;
    }

     
    if (RCC_GetFlagStatus(RCC_FLAG_SFTRST))
    {
        printf("- Software Reset -\r\n");    
        reval = RCC_FLAG_SFTRST;
    }
     
    if (RCC_GetFlagStatus(RCC_FLAG_PORRST))
    {
        printf("- Power-On-Reset -\r\n");    
        reval = RCC_FLAG_PORRST;
    }
     
    if (RCC_GetFlagStatus(RCC_FLAG_PINRST)) // Always set, test other cases first
    {
        printf("- External Pin Reset -\r\n");    
        reval = RCC_FLAG_PINRST;
    }
     
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
    {
        printf("- Watchdog Reset -\r\n");    
        reval = RCC_FLAG_IWDGRST;
    }
     
    if (RCC_GetFlagStatus(RCC_FLAG_WWDGRST) != RESET)
    {
        printf("- Window Watchdog Reset -\r\n");    
        reval = RCC_FLAG_WWDGRST;
    }
     
    if (RCC_GetFlagStatus(RCC_FLAG_LPWRRST) != RESET)
    {
        printf("- Low Power Reset -\r\n");    
        reval = RCC_FLAG_LPWRRST;
    }
       
    //if (RCC_GetFlagStatus(RCC_FLAG_BORRST) != RESET) // F4 Usually set with POR
    //    printf("Brown-Out Reset\r\n");
       
    RCC_ClearFlag(); // The flags cleared after use
    return reval;
}

