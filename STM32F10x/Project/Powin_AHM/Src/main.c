/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
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
#include "stm32f10x.h"
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "printfdebug.h"
#include "memoryconfig.h"
#include "booterlib.h"

#include "wwdg.h"
#ifdef POWIN_BOOTER
    #define ENABLE_BOOTER   1 
#else
    #define ENABLE_BOOTER   0 
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define mainNORMAL_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;


#if(USER_FREERTOS)
static void vNormalTask( void *pvParameters )
{
    //vTaskDelay(2000); 
    printf("Going...\r\n");  
    for(;;)
    {
        printf("|");  
        vTaskDelay(500); 
        send_autodata_Ex_2_BP(0x42, 0);
    }
}
#else
extern int mainpowin(void);

#endif

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */


int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */    
    PrintfInit();
    #if(ENABLE_BOOTER) 
    printf("\n\r************************************************************************************\r\n");
    printf("    ****    %s  %s  %s(Ver. %d)    ****\r\n", "Powin String(eface) BOOTER Starting", __DATE__, __TIME__, STRING_VERSION);
    printf("************************************************************************************\r\n");
    #else
    #ifdef POWIN_NORMAL   
    printf("\n\r************************************************************************************\r\n");
    printf("    -**-    %s  %s  %s(Ver. %d)    -**-\r\n", "Powin String(eface) Normal Starting", __DATE__, __TIME__, STRING_VERSION);
    printf("************************************************************************************\r\n");
    #else
    printf("\n\r************************************************************************************\r\n");
    printf("    ----    %s  %s  %s(Ver. %d)    ----\r\n", "Powin String(eface) Starting", __DATE__, __TIME__, STRING_VERSION);
    printf("************************************************************************************\r\n");
    #endif
    #endif
    /* Add your application code here
     */

    #if(ENABLE_BOOTER) 
    {
        uint8_t resetSource = GetResetSource();
        if((resetSource == RCC_FLAG_IWDGRST) || (resetSource == RCC_FLAG_WWDGRST))
        //if(resetSource == RCC_FLAG_WWDGRST)
        {
            printf(" !!!  RCC_FLAG_IWDGRST (boot flag: 0x%x) !!!\r\n", GetBootConfig());
            switch(GetBootConfig())
            {            
                case FLASH_AP_1_ID:  
                    printf(" !!!  set FLASH_BOOTER_ID !!!\r\n");
                    SetBootConfig(FLASH_BOOTER_ID);
                    break; 
                case FLASH_AP_2_ID:  
                    printf(" !!!  set FLASH_AP_1_ID !!!\r\n");
                    SetBootConfig(FLASH_BOOTER_ID);
                    break;
            } 
        }
        else 
        {
            printf(" !!!  No WatchDog Source !!!\r\n");
        }
    }
    #endif 
    WWDGDeInit();
    #if(ENABLE_BOOTER) 
    
    //SetBootConfig(FLASH_AP_1_ID);
    //if(!checkPauseKey())
    printf("*************************\r\n");
    printf("*  boot flag: 0x%02x *\r\n", GetBootConfig());
    printf("*************************\r\n");
    if(1)
    {
        uint32_t targetAddress = 0x0; 
        uint8_t bootFlagifError = 0x0;        

        switch(GetBootConfig())
        {
            case FLASH_BOOTER_ID://jump to booter
                printf("Booter Normal boot_2...\r\n");
                //SetBootConfig(FLASH_AP_1_ID);
                break;
            //default:
            //    SetBootConfig(FLASH_AP_1_ID);
            case FLASH_AP_1_ID:             
                targetAddress = USER_FLASH_AP_1_ADDRESS;
                bootFlagifError = FLASH_BOOTER_ID;
                break;
            case FLASH_AP_2_ID:
                targetAddress = USER_FLASH_AP_2_ADDRESS;
                //bootFlagifError = FLASH_AP_1_ID;
                bootFlagifError = FLASH_BOOTER_ID;
                break;
            case FLASH_AP_3_ID:
                targetAddress = USER_FLASH_AP_3_ADDRESS;
                //bootFlagifError = FLASH_AP_1_ID;
                bootFlagifError = FLASH_BOOTER_ID;
                break;
            default:
                SetBootConfig(FLASH_BOOTER_ID);
            break;
        }
        
        printf("address: 0x%08x (0x%08x)...\r\n", (*(__IO uint32_t*)targetAddress) & 0x2FFE0000, targetAddress);
        if(targetAddress != 0)
        {
            if (((*(__IO uint32_t*)targetAddress) & 0x2FFE0000 ) == 0x20000000)
            {
                /* Jump to user application */
                JumpAddress = *(__IO uint32_t*) (targetAddress + 4);
                printf("jump to image: 0x%x...\r\n", targetAddress);
                WWDGInit();
                Jump_To_Application = (pFunction) JumpAddress;
                /* Initialize user application's Stack Pointer */
                __set_MSP(*(__IO uint32_t*) targetAddress);
                Jump_To_Application();
                /* do nothing */
                while(1); 
            }
            else
            {
                printf("No firmware in this address, bootFlagifError = %d...\r\n", bootFlagifError);
                if(bootFlagifError != 0)
                    SetBootConfig(bootFlagifError); 
                
            }
        }
        else
        {
            printf("targetAddress = 0, ignore\r\n");
        }
    }
    else
    {
        printf("Booter Normal boot...\r\n");
    }
    #else
        #ifndef POWIN_NORMAL        
        switch(GetBootConfig())
        {           
            case FLASH_AP_1_ID:             
                SCB->VTOR = FLASH_BASE | USER_FLASH_AP_1_ADDRESS;
                break;
            case FLASH_AP_2_ID:
                SCB->VTOR = FLASH_BASE | USER_FLASH_AP_2_ADDRESS;
                break;
            case FLASH_AP_3_ID:
                SCB->VTOR = FLASH_BASE | USER_FLASH_AP_3_ADDRESS;
                break;
        }   
// 		IWDGInit();	       		
        #endif
    #endif


 
    while (1)
    {
		mainpowin();
    }
    
    
  
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
