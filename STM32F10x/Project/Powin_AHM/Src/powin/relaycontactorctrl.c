 #include <stdio.h> 
#include <stdlib.h>
#include "BESS_3-0_AHM_device_lib_V_1-0.h"
#include "BESS_3-0_AHM_can_V_1-0.h"
#include "BESS_3-0_AHM_main_V_1-0.h"
#include "BESS_3-0_AHM_systick_V_1-0.h"
//#include "BESS_3-0_AHM_i2c_eeprom_V_1-0.h"
#include "BESS_3-0_AHM_ad7327_V_1-0.h"
#include "BESS_3-0_AHM_canmsg_V_1-0.h"
#include "BESS_3-0_AHM_config_V_1-0.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "adc.h"
#include "contactor.h"
#include "led.h"
#include "messageid.h"
#include <math.h>
#include <NewStringADC.h>
#include "pinmap.h"
#include "i2c_ee.h"
#include "DataProcess.h"

void Relay_Set(u8 Relayx, u8 OnOff)
{
    if(Relayx==RELAY1)
    {
        if(OnOff==RELAY_ON)
        {
            GPIO_SetBits(Relay1_Ctrl_PORT,Relay1_Ctrl_PIN);
        }
        else
        {
            GPIO_ResetBits(Relay1_Ctrl_PORT,Relay1_Ctrl_PIN);
        }
    }
    else
    {
        if(OnOff==RELAY_ON)
        {
            GPIO_SetBits(Relay2_Ctrl_PORT,Relay2_Ctrl_PIN);
        }
        else
        {
            GPIO_ResetBits(Relay2_Ctrl_PORT,Relay2_Ctrl_PIN);
        }
    }
}

void RelayAllTurnOff(void)
{
#if  POWIN_AP
	Relay_Set(RELAY1, 0);
	Relay_Set(RELAY2, 0);
#endif	
}

void RelayAllTurnOn(void)
{
#if  POWIN_AP
	Relay_Set(RELAY1, 1);
	Relay_Set(RELAY2, 1);
#endif	
}
void Bess_SystemMode(void)
{
	if(fabs(mStringData.stringCurrent) <= 100)
	{
		mStringData.mode = Idle_Mode;
	}
	else if(mStringData.stringCurrent > 100)
	{
		mStringData.mode = Charge_Mode;
	}
	else if(mStringData.stringCurrent < -100)
	{
		mStringData.mode = DisCharge_Mode;
	}
	
}

/*
	如果DCBUS电压和String voltage之差在5V以内，则可以 turn on(close) contactor

*/
void DetermineContactorState()
{
		if(MaintenanceModeFlag == 0)
		{
		 
			if(fabs(mStringData.dcbusVoltage - GetStringMeasuredVoltage()) < MismatchVoltageThreshold && Count.Alarm == 0 && Count.BpToStringAlarm == 0 && mStringData.measureStringVoltage > 50) //||  GetDCBusVoltage() == 0)
			{
				
				{
					static u16 MismatchCount = 0;
					MismatchCount++;
					if(MismatchCount >= 2)
					{
						 
						ContactorAllTurnOn();
					}
				}
			}	
		}
 
}

/*
  string 主动向Array controller 询问 是否有 当前string controller 是否 permission 去close contactor
  在DCBUS电压为0(< 50V)的情况下，string 会去主动 询问 permission

*/
void StringQueryContactorClosedPermission()
{
//	if(!is_contactor_1_closed() && !is_contactor_2_closed() && Count.Alarm == 0 && Count.BpToStringAlarm == 0)
//	{
		
	if(mStringData.measureStringVoltage >= 50 && mStringData.dcbusVoltage <= 50 && !is_contactor_1_closed() && !is_contactor_2_closed() && Count.Alarm == 0 && Count.BpToStringAlarm == 0)   /**/
		{
			uint8_t buf[8] = {0};
			uint8_t msg_id = 0x0;
			uint8_t dataLen = 0;
			MsgGroupID = ID_Configure;
			msg_id = MsgID_ContactorClosedPermission;
			dataLen = 2;
			buf[0] = mStringData.ArrayID;
			buf[1] = Self_ID;
			CAN_make_send_2_Array( buf, dataLen, msg_id);
		}
//	}
	
}
