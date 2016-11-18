/**
*/ 
/* Includes ------------------------------------------------------------------*/
#include "wwdg.h"
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void WWDGInit(void)
{
    /* WWDG configuration */
    /* Enable WWDG clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

    /* On Value line devices, WWDG clock counter = (PCLK1 (24MHz)/4096)/8 = 732 Hz (~1366 us)  */
    /* On other devices, WWDG clock counter = (PCLK1(36MHz)/4096)/8 = 1099 Hz (~910 us)  */
    WWDG_SetPrescaler(WWDG_Prescaler_8);

    /* Set Window value to 80; WWDG counter should be refreshed only when the counter
        is below 80 (and greater than 64) otherwise a reset will be generated */
    WWDG_SetWindowValue(80);

    /* - On Value line devices,
        Enable WWDG and set counter value to 127, WWDG timeout = ~1366 us * 64 = 87.42 ms 
        In this case the refresh window is: ~1366us * (127-80) = 64.20 ms < refresh window < ~1366us * 64 = 87.42ms
        - On other devices
        Enable WWDG and set counter value to 127, WWDG timeout = ~910 us * 64 = 58.25 ms 
        In this case the refresh window is: ~910 us * (127-80) = 42.77 ms < refresh window < ~910 us * 64 = 58.25ms     
    */
    WWDG_Enable(127);    
    printf("-- WWDGInit --\r\n");    
}

void WWDGDeInit(void)
{
    RCC_ClearFlag();
    WWDG_DeInit();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE);
    printf("-- WWDGDeInit --\r\n");
}

void IWDGInit(void)
{
    __IO uint32_t LsiFreq = 40000;
    /* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
     dispersion) */
    /* Enable write access to IWDG_PR and IWDG_RLR registers */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    #if(1)
    /* IWDG counter clock: LSI/256 */
    IWDG_SetPrescaler(IWDG_Prescaler_256);

    /* Set counter reload value to obtain 250ms IWDG TimeOut.
        Counter Reload Value = 250ms/IWDG counter clock period
                          = 250ms / (LSI/256)
                          = 0.25s / (LsiFreq/256)
                          = LsiFreq/(256 * 4)
                          = LsiFreq/128
    */
    IWDG_SetReload(LsiFreq/32);
    #else
    /* IWDG counter clock: LSI/32 = 1250*/
    IWDG_SetPrescaler(IWDG_Prescaler_32);

    /* Set counter reload value to obtain 250ms IWDG TimeOut.
        Counter Reload Value = 250ms/IWDG counter clock period
                          = 250ms / (LSI/32)
                          = 0.25s / (LsiFreq/32)
                          = LsiFreq/(32 * 4)
                          = LsiFreq/128
    */
    IWDG_SetReload(LsiFreq/128);
    #endif

    /* Reload IWDG counter */
    IWDG_ReloadCounter();

    /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
    IWDG_Enable();
    printf("-- IWDGInit --\r\n");    
}

void IWDGDeInit(void)
{
    RCC_ClearFlag();
    WWDG_DeInit();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE);
    printf("-- WWDGDeInit --\r\n");
}


