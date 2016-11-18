/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
#include "stdio.h"
#include "stm32f10x_it.h"
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "BESS_3-0_AHM_stringdata_V_1-0.h"
#include "NewStringGPIO.h"
#include "messageid.h"
#include "led.h"
#include "contactor.h"
#include "DataProcess.h"
extern void xPortSysTickHandler(void);
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void vPortSVCHandler( void );
void SVC_Handler(void)
{
    //by sam
    vPortSVCHandler();
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
    printf("DebugMon_Handler\r\n");
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void xPortPendSVHandler( void );
void PendSV_Handler(void)
{
    //by sam
    xPortPendSVHandler();
}

void RedLight(void)
{
	GPIO_ResetBits(FRONT_LED_RED_PORT, FRONT_LED_RED_PIN);
	GPIO_SetBits(FRONT_LED_GRN_PORT, FRONT_LED_GRN_PIN);	
}

void YellowLight(void)
{
	GPIO_ResetBits(FRONT_LED_RED_PORT, FRONT_LED_RED_PIN);	
	GPIO_ResetBits(FRONT_LED_GRN_PORT, FRONT_LED_GRN_PIN);	
	
}

void GreenLight(void)
{
	GPIO_ResetBits(FRONT_LED_GRN_PORT, FRONT_LED_GRN_PIN);	
	GPIO_SetBits(FRONT_LED_RED_PORT, FRONT_LED_RED_PIN);   	
}

void RedLightBlinking(void)
{
	GPIO_WriteBit(RedIndictor, (BitAction) !GPIO_ReadOutputDataBit(RedIndictor));	
	GPIO_SetBits(FRONT_LED_GRN_PORT, FRONT_LED_GRN_PIN);	
}

void YellowLightBlinking(void)
{
	GPIO_WriteBit(GreenIndictor, (BitAction) !GPIO_ReadOutputDataBit(GreenIndictor)); 
	GPIO_WriteBit(RedIndictor, (BitAction) !GPIO_ReadOutputDataBit(RedIndictor));    	
}
void GreenLightBlinking(void)
{
	GPIO_WriteBit(GreenIndictor, (BitAction) !GPIO_ReadOutputDataBit(GreenIndictor));
	GPIO_SetBits(FRONT_LED_RED_PORT, FRONT_LED_RED_PIN);	
}
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//for Powin by sam
/*
   changed by jason 2016年1月15日10:33:57


*/
u16 WriteWKHEEPROM5minTimer = 0;
u16 WdtTimer=0;
u16 HeartBeatTimer = 0;
u16 QueryExtremeValueTimer = 0;
u16 CheckStatusTimer = 0;
u16 LedTimer = 0;
u8 LedFlag = 0;
u16 BootCount = 0;
u8 Boot10sFlag = 0;
u16 TimerForAWEProcess = 0;
u16 StringAWETimer = 0;
void SysTick_Handler_Powin(void);
void SysTick_Handler(void)
{
    //by sam
    //printf("SysTick_Handler\r\n");
	WdtTimer++;
	QueryExtremeValueTimer++;
	mStringData.AHTimer++;
	HeartBeatTimer++;
	CheckStatusTimer++;
	LedTimer++;
	ToArray.Led1sCount++;
	TargetValueCounter++;
	if(TargetValueCounter >= TargerValueFromArrayTimeOut)            /*10 ms*/
	{
		TargetValueCountTimeOutFlag = 1;				
	}
	if(ToArray.Led1sCount > 30000)
		ToArray.Led1sCount = 1000;
	if(LedTimer >= 1000)             /*1s*/
	{
		ProgrameRunTimer++;
		if(ProgrameRunTimer >= 120)
		{
			ProgrameRun2minsFlag = 1;
		}
		StringAWETimer++;
		 
		
		TimerForAWEProcess++;
		IWDG_ReloadCounter();
		WriteWKHEEPROM5minTimer++;        /*在充放电 期间 每 5min 写一次 EEPROM，保证WKH等同步*/
		SyncCount++;
		BootCount++;                /* boot 开机 使用*/
		if(BootCount > 10)          /*10s 时间到*/
		{
			Boot10sFlag = 1;
		}
		LedTimer = 0;
		
 		GPIO_WriteBit(SysIndicator, (BitAction) !GPIO_ReadOutputDataBit(SysIndicator));
 
		if(Idle5minCountFlag == 1)
		{
			Idle5minCount++;
			if(Idle5minCount >= 300)
			{
				Idle5minCountFlag = 0;
				Idle5minFlag = 1;
				Idle5minCount = 0;
			}
		}
 		
 		if(SystemLightBlinkingExitFlag == 1)   
 		{
 	 
			if(Count.Alarm > 0 || Count.BpToStringAlarm > 0)                                    //when has alarm
			{
 			 
				RedLight();
				LedFlag = 0;
			}

			else if(Count.Warning > 0 || Count.BpToStringWarning > 0 || getPostitiveContactor() == 0x00 || getPostitiveContactor2() == 0x00)
			{
 				
				if( (Count.Warning > 0 || Count.BpToStringWarning > 0) && (getPostitiveContactor() == 0xFF || getPostitiveContactor2() == 0xFF)) /*黄灯常亮*/
				{	
					YellowLight();
					LedFlag = 0;
				}
				else if(getPostitiveContactor() == 0x00 || getPostitiveContactor2() == 0x00)    /*黄灯闪烁，contactor 打开时  */
				{
					if(LedFlag == 0)
					{
						LedFlag = 1;
						YellowLight();					
					}
					YellowLightBlinking();	
				}
					
 			
			}
			else
			{
 			
				GreenLight();
				LedFlag = 0;	
			}
 		}
 		
		else       
		{
			InitSystemLightBlinking(); 
		}
 
 
	}
	
 
}
 
void TIM2_IRQHandler(void)
{    
	int i = 0;
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源 
		for(i = 0;i < 10;i++) /*bp的AWE判断*/
		{
			if(Flag_AutoUpdateStringAWE[i] == 1)
			{
				 Count_AutoUpdateStringAWE[i]++;				
			}
		}
		
		for(i = 0;i < 13;i++)
		{
			if(Flag_AutpUpdateBPAWE[i] == 1)
			{
				Count_AutoUpdateBPAWE[i]++;
			}
		}
	}
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
