
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

MESSAGE SysHVolAlarmMessage;
MESSAGE SysHVolWarningMessage;
MESSAGE SysLVolAlarmMessage;
MESSAGE SysLVolWarningMessage;
MESSAGE SysHTempAlarmMessage;
MESSAGE SysHTempWarningMessage;
MESSAGE SysLTempAlarmMessage;
MESSAGE SysLTempWarningMessage;

MESSAGE SysHCellDeltaTempAlarmMessage;
MESSAGE SysHCellDeltaTempWarningMessage;
MESSAGE SysHCellTempRiseAlarmMessage;
MESSAGE SysHCellTempRiseWarningMessage;
MESSAGE SysChargeStringCurrentAlarmMessage;
MESSAGE SysChargeStringCurrentWarningMessage;
MESSAGE SysDischargeStringCurrentAlarmMessage;
MESSAGE SysDischargeStringCurrentWarningMessage;

MESSAGE SysBPLoseOfCommunicationErrorMessage;       /*2016年8月29日10:15:50*/

s16 CmpValue_ChargeStringCurrentAlarm = 0;
s16 CmpValue_ChargeStringCurrentWarning = 0;
s16 CmpValue_DischargeStringCurrentAlarm = 0;
s16 CmpValue_DischargeStringCurrentWarning = 0;   

u8 IdleWriteKWHTOEEPROM_FLag = 0;
void Bess_GetHighDeltaTemperature()
{
    int i = 0;
//	int32_t tmpValue = 0;
//	for(i = 0; i<BP_NUMBER; i++)
//    {
//        if(mStringData.bp[i].cellDeltaT != 0 && BpLossCommunicationWarningFlag[i] == 0)
//		{
//			tmpValue = tmpValue + mStringData.bp[i].cellDeltaT;
//			realcount++;
//		}
//    }
//	if(realcount > 0)
//	{
//		mStringData.maxCellDeltaT = tmpValue / realcount; 	
//	}	
	
	mStringData.maxCellDeltaT = mStringData.bp[0].cellDeltaT;
    for(i = 0; i<BP_NUMBER; i++)
    {
        if(mStringData.bp[i].cellDeltaT != 0)
		{
			if(BpLossCommunicationWarningFlag[i] == 0) /*未失联*/
			{
				if(mStringData.bp[i].cellDeltaT > mStringData.maxCellDeltaT)
				{
					mStringData.maxCellDeltaT = mStringData.bp[i].cellDeltaT;
				}
			}
		}
    }
}

void Bess_GetDatas(void)
{
	Bess_GetAverageTemperature();
	Bess_GetHighLowTemperature();
 	Bess_GetHighDeltaTemperature();
}
/*
	得到BP的平均电压
*/

void GetBPAverageVoltage()
{
	int i = 0,tmpValue = 0,realcount = 0;
	for(i = 0;i < BP_NUMBER;i++)
	{
		if(mStringData.bp[i].averageCellVolt != 0 && BpLossCommunicationWarningFlag[i] == 0)
		{
			tmpValue = tmpValue + mStringData.bp[i].averageCellVolt;
			realcount++;
		}
	}
	if(realcount > 0)
	{
		mStringData.averageCellVolt = tmpValue / realcount;
	}
//	for(i = 0;i < BP_NUMBER;i++)
//	{
//		mStringData.bp[i].averageCellVolt = 0;
//	}
	
}
void Bess_GetAverageTemperature()
{
    int i = 0,realcount = 0;
	uint32_t tmpValue = 0;
	for(i = 0; i<BP_NUMBER; i++)
    {
        if(mStringData.bp[i].averageCellTemp != 0 && BpLossCommunicationWarningFlag[i] == 0)
		{
			tmpValue = tmpValue + mStringData.bp[i].averageCellTemp;
			realcount++;
		}
    }
	if(realcount > 0)
	{
		mStringData.averageCellTemp = tmpValue / realcount; 	
	}
//	for(i = 0;i < BP_NUMBER;i++)
//	{
//		mStringData.bp[i].averageCellTemp = 0;
//	}	
}
void Bess_GetHighLowTemperature()
{
	int i = 0;
	mStringData.highCellTemp = 0;
    for(i = 0; i<BP_NUMBER; i++)
    {
        
		if(mStringData.bp[i].highCellTemp != -2500)
		{
			if(BpLossCommunicationWarningFlag[i] == 0)   /*未失联比较，为1 则失联不进行比较*/
			{
				if(mStringData.bp[i].highCellTemp > mStringData.highCellTemp)	
				{
					mStringData.highCellTemp = mStringData.bp[i].highCellTemp;
				}
			}
		}
    }
    mStringData.lowCellTemp = 2500;
    for(i = 0; i<BP_NUMBER; i++)
    {
        if(mStringData.bp[i].lowCellTemp != 2500)
		{
			if(BpLossCommunicationWarningFlag[i] == 0)
			{
				if(mStringData.bp[i].lowCellTemp < mStringData.lowCellTemp)
				{
					mStringData.lowCellTemp = mStringData.bp[i].lowCellTemp;
				}
			}
		}
    }	
	
	mStringData.maxCellTempRise = mStringData.bp[0].maxCellTempRise;
    for(i = 0; i<BP_NUMBER; i++)
    {
        if(mStringData.bp[i].maxCellTempRise != -2500)
		{
			if(BpLossCommunicationWarningFlag[i] == 0) /*未失联*/
			{
				if(mStringData.bp[i].maxCellTempRise > mStringData.maxCellTempRise)
				{
					mStringData.maxCellTempRise = mStringData.bp[i].maxCellTempRise;
				}
			}
		}
    }
}  



void Bess_SOCByVol(void)
{
	
	if((mStringData.ah == 0) || (mStringData.kWh < 3000 && mStringData.lowCellVolt  >= 2900 ))
	{
		if(alreadyCalibrated == 0|| (mStringData.kWh < 3000 && mStringData.lowCellVolt  >= 2900 ))
		{
			static u8 SimpleAHOnce = 0;
			if(SimpleAHOnce <= 20 && mStringData.lowCellVolt >= 2500 && mStringData.lowCellVolt != 10000)
			{
			
				SimpleAHOnce++;
			//	mStringData.ah = 7 * mStringData.highCellVolt - 16590;
			//  	mStringData.kWh =  mStringData.ah * mStringData.measureStringVoltage * 10;
			///		 mStringData.kWh = 33 * mStringData.highCellVolt - 81818;
			//	mStringData.soc = mStringData.ah / 73;
			//	mStringData.averageCellVolt
				if(mStringData.lowCellVolt < 2600)
				{
					mStringData.soc = 0;
					mStringData.ah = mStringData.soc * TotalAH / 100;
					mStringData.kWh = mStringData.ah *mStringData.lowCellVolt/ 1000 * BP_NUMBER * CELL_NUMBER / 100;
				}
				else if(mStringData.lowCellVolt >= 2600 && mStringData.lowCellVolt <2800)
				{
					mStringData.soc = 2;
					mStringData.ah = mStringData.soc * TotalAH / 100;
					mStringData.kWh = mStringData.ah *mStringData.lowCellVolt/ 1000 * BP_NUMBER * CELL_NUMBER / 100;				
					
				}
				else if(mStringData.lowCellVolt >= 2800 && mStringData.lowCellVolt <3100)
				{
					mStringData.soc = 30;
					mStringData.ah = mStringData.soc * TotalAH / 100;
					mStringData.kWh = mStringData.ah *mStringData.lowCellVolt/ 1000 * BP_NUMBER * CELL_NUMBER / 100;			
				}
				else if(mStringData.lowCellVolt >= 3100 && mStringData.lowCellVolt <3200)
				{
					mStringData.soc = 40;
					mStringData.ah = mStringData.soc * TotalAH / 100;
					mStringData.kWh = mStringData.ah *mStringData.lowCellVolt/ 1000 * BP_NUMBER * CELL_NUMBER / 100;				
				}
				if(mStringData.lowCellVolt >= 3200 && mStringData.lowCellVolt< 3250)
				{
					mStringData.soc = 60;
					mStringData.ah = mStringData.soc * TotalAH / 100;
					mStringData.kWh = mStringData.ah *mStringData.lowCellVolt/ 1000 * BP_NUMBER * CELL_NUMBER / 100;
				}
				else if(mStringData.lowCellVolt > 3250 && mStringData.lowCellVolt<= 3300)
				{
					mStringData.soc = 70;
					mStringData.ah = mStringData.soc * TotalAH / 100;
					mStringData.kWh = mStringData.ah *mStringData.lowCellVolt/1000 * BP_NUMBER * CELL_NUMBER / 100;
				}
				else if(mStringData.lowCellVolt > 3300 && mStringData.lowCellVolt<= 3400)
				{
					mStringData.soc = 80;
					mStringData.ah = mStringData.soc * TotalAH / 100;
					mStringData.kWh = mStringData.ah *mStringData.lowCellVolt/1000 * BP_NUMBER * CELL_NUMBER / 100;				
				}
				else if(mStringData.lowCellVolt > 3400 && mStringData.lowCellVolt<= 3600)
				{
					mStringData.soc = 85;
					mStringData.ah = mStringData.soc * TotalAH / 100;
					mStringData.kWh = mStringData.ah *mStringData.lowCellVolt/1000 * BP_NUMBER * CELL_NUMBER / 100;					
				}
				else if(mStringData.lowCellVolt > 3600)
				{
					mStringData.soc = 100;
					mStringData.ah = mStringData.soc * TotalAH / 100;
					mStringData.kWh = mStringData.ah *mStringData.lowCellVolt/1000 * BP_NUMBER * CELL_NUMBER / 100;					
				}
			
			}			
			CopyAHKWHToEEPROMTemp(); 
			StringEEProm_Write(AHKWHEEPageNumber);	
		}
	}
			
}

void InitSystemLightBlinking()
{
  static u8 LedStep = 0;
	switch(LedStep)
	{
		case 0:
				GreenLight();
		break;
		case 1:
				GreenLightBlinking();
		break;
		case 2:
				YellowLight();
		break;
		case 3:	
				YellowLightBlinking();
		break;
		case 4:
				RedLight();
		break;
		case 5:
				RedLightBlinking();
				SystemLightBlinkingExitFlag = 1;
		break;
		 
		default:
				break;
	}
	LedStep++;
}
void SaveWKHTOEEPROM()
{
	
	if(fabs(mStringData.stringCurrent) >= 100)	   /*  当前 处于 充放电状态 */
	{
		if(WriteWKHEEPROM5minTimer >= 300)
		{
			IdleWriteKWHTOEEPROM_FLag = 0;
			WriteWKHEEPROM5minTimer = 0;
			CopyAHKWHToEEPROMTemp(); 
			StringEEProm_Write(AHKWHEEPageNumber);	
		}
	}
	else
	{
		WriteWKHEEPROM5minTimer = 0;	
		if(IdleWriteKWHTOEEPROM_FLag == 0)
		{
			IdleWriteKWHTOEEPROM_FLag = 1;
			CopyAHKWHToEEPROMTemp(); 
			StringEEProm_Write(AHKWHEEPageNumber);	
		//	send_autodata_Ex(35);			
		}
	}
	
}

void Timer2_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //TIM2时钟使能 

 	TIM_TimeBaseStructure.TIM_Period = 2000 - 1; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	  
	TIM_TimeBaseStructure.TIM_Prescaler =36000 - 1; //设置用来作为TIMx时钟频率除数的预分频值  ，72000000 / 36000 = 2000；
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
    TIM_ClearFlag(TIM2, TIM_FLAG_Update); 

	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//使能 TIM 允许更新，触发
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级03级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM2, ENABLE);  //使能TIMx
}

void UpdateAWE()
{
	int i = 0;
	for(i = 0; i < 10;i++)
	{
		int j = 0;
		if(Count_AutoUpdateStringAWE[i] >= 5)
		{
			Count_AutoUpdateStringAWE[i] = 0;
			Flag_AutoUpdateStringAWE[i] = 0;
			
			switch(i)
			{
				case 0:
				MsgGroupID = ID_Alarm;
				CAN_make_send_from_BP(SysHVolAlarmMessage.Data, SysHVolAlarmMessage.MessageNum, SysHVolAlarmMessage.MessageID,SysHVolAlarmMessage.DeviceID  );	
				
				for(j = 0 ;j <BP_NUMBER;j++)
				{
					SysStateAckFlag[j].HVolAlarmAckFlag = 0;
				}				
			
				break;
			
				case 1:
				MsgGroupID = ID_Warning;
				CAN_make_send_from_BP(SysHVolWarningMessage.Data, SysHVolWarningMessage.MessageNum, SysHVolWarningMessage.MessageID,SysHVolWarningMessage.DeviceID  );	
				
				for(j = 0 ;j <BP_NUMBER;j++)
				{
					SysStateAckFlag[j].HVolWarningAckFlag = 0;
				}				
				break;
				
				case 2:
				MsgGroupID = ID_Alarm;
				CAN_make_send_from_BP(SysHTempAlarmMessage.Data, SysHTempAlarmMessage.MessageNum, SysHTempAlarmMessage.MessageID,SysHTempAlarmMessage.DeviceID  );	

				for(j = 0 ;j <BP_NUMBER;j++)
				{
					SysStateAckFlag[j].HTempAlarmAckFlag = 0;
				}				
				break;
				
				case 3:
				MsgGroupID = ID_Warning;
				CAN_make_send_from_BP(SysHTempWarningMessage.Data, SysHTempWarningMessage.MessageNum, SysHTempWarningMessage.MessageID,SysHTempWarningMessage.DeviceID );
				
				for(j = 0 ;j <BP_NUMBER;j++)
				{
					SysStateAckFlag[j].HTempWarningAckFlag = 0;
				}				
				break;
				
				case 4:
				MsgGroupID = ID_Alarm;
				CAN_make_send_from_BP(SysLVolAlarmMessage.Data, SysLVolAlarmMessage.MessageNum, SysLVolAlarmMessage.MessageID,SysLVolAlarmMessage.DeviceID  );	

				for(j = 0 ;j <BP_NUMBER;j++)
				{
					SysStateAckFlag[j].LVolAlarmAckFlag = 0;
				}				
				break;
				
				case 5:
				MsgGroupID = ID_Warning;
				CAN_make_send_from_BP(SysLVolWarningMessage.Data, SysLVolWarningMessage.MessageNum, SysLVolWarningMessage.MessageID,SysLVolWarningMessage.DeviceID  );	

				for(j = 0 ;j <BP_NUMBER;j++)
				{
					SysStateAckFlag[j].LVolWarningAckFlag = 0;
				}				
				break;
				
				case 6:
				MsgGroupID = ID_Alarm;
				CAN_make_send_from_BP(SysLTempAlarmMessage.Data, SysLTempAlarmMessage.MessageNum, SysLTempAlarmMessage.MessageID,SysLTempAlarmMessage.DeviceID  );	
				for(j = 0 ;j <BP_NUMBER;j++)
				{
					SysStateAckFlag[j].LTempAlarmAckFlag = 0;
				}				
				break;
				
				case 7:
				MsgGroupID = ID_Warning;
				CAN_make_send_from_BP(SysLTempWarningMessage.Data, SysLTempWarningMessage.MessageNum, SysLTempWarningMessage.MessageID,SysLTempWarningMessage.DeviceID );	
				for(j = 0 ;j <BP_NUMBER;j++)
				{
					SysStateAckFlag[j].LTempWarningAckFlag = 0;
				}				
				break;
				
				case 8:         /*high average delat error*/

				{
					MESSAGE msg;
					msg.MessageID = ID_Error;
					MsgGroupID = ID_Error;
					msg.Data[0] = Error_HighAveVoltaDelta;
					msg.Data[1] = Update; 
					msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
					msg.Data[3] = 0x00;
					msg.Data[4] = BPHAveDeltaVolErrorbit.Byte1;
					msg.Data[5] = BPHAveDeltaVolErrorbit.Byte2;
					msg.Data[6] = BPHAveDeltaVolErrorbit.Byte3;
					msg.Data[7] = BPHAveDeltaVolErrorbit.Byte4;
					msg.DeviceID = -1;
					CAN_make_send_from_BP(msg.Data,8,MsgID_Error,msg.DeviceID); 					
				}
				for(j = 0 ;j <BP_NUMBER;j++)
				{
					SysStateAckFlag[j].HighAveVoltaDeltaErrorAckFlag = 0;
				}					
				
				break;
				
				case 9:        /*bp boot error*/
				MsgGroupID = ID_Error;	
				{
					MESSAGE bootererror;
					bootererror.MessageID = ID_Error;
					MsgGroupID = ID_Error;
					bootererror.Data[0] = Error_BpBooter;
					bootererror.Data[1] = Update; 
					bootererror.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
					bootererror.Data[3] = 0x00;
					bootererror.Data[4] = BPbooterErrorbit.Byte1;
					bootererror.Data[5] = BPbooterErrorbit.Byte2;
					bootererror.Data[6] = BPbooterErrorbit.Byte3;
					bootererror.Data[7] = BPbooterErrorbit.Byte4;
					bootererror.DeviceID = -1;
					CAN_make_send_from_BP(bootererror.Data,8,MsgID_Error,bootererror.DeviceID); 

					for(j = 0 ;j <BP_NUMBER;j++)
					{
						SysStateAckFlag[j].BPBootError = 0;
					}			 
					
				}
				break;
			}
		}
	}
	
	for(i = 0;i < 13;i++)
	{
		int j = 0;
		if(Count_AutoUpdateBPAWE[i] >= 5)
		{
			Count_AutoUpdateBPAWE[i] = 0;
			Flag_AutpUpdateBPAWE[i] = 0;
			switch(i)
			{
				case 0:
				{
					MESSAGE msg;
					msg.Data[0] = Error_HighCellVolDelta;
					msg.Data[1] = Update;
					msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
					msg.Data[3] = 0x01;
					msg.Data[4] = BPToStringHighCellDeltaErrorbit.Byte1;
					msg.Data[5] = BPToStringHighCellDeltaErrorbit.Byte2;
					msg.Data[6] = BPToStringHighCellDeltaErrorbit.Byte3;
					msg.Data[7] = BPToStringHighCellDeltaErrorbit.Byte4;
					msg.MessageNum = 8;
					msg.MessageID = MsgID_Error;
					msg.DeviceID = -1 ;
					MsgGroupID = ID_Error;			
					CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID );
					
					for( j = 0;j < BP_NUMBER;j++)
					{
						BpToStringFlag[j].HighCellDeltaErrorFlag = 1;  
					}
				}
				break;
				
				case 1:
				{
					MESSAGE BPToStringHVolAlarm;
					BPToStringHVolAlarm.Data[0] = Alarm_CellOverVoltage;
					BPToStringHVolAlarm.Data[1] = Update;
					BPToStringHVolAlarm.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
					BPToStringHVolAlarm.Data[3] = 0x01;     
					BPToStringHVolAlarm.Data[4] = BPToStringHVolAlarmbit.Byte1;
					BPToStringHVolAlarm.Data[5] = BPToStringHVolAlarmbit.Byte2;
					BPToStringHVolAlarm.Data[6] = BPToStringHVolAlarmbit.Byte3;
					BPToStringHVolAlarm.Data[7] = BPToStringHVolAlarmbit.Byte4;
					BPToStringHVolAlarm.MessageNum = 8;
					BPToStringHVolAlarm.MessageID = MsgID_Alarm;
					BPToStringHVolAlarm.DeviceID = -1 ;
					MsgGroupID = ID_Alarm;			
					CAN_make_send_from_BP(BPToStringHVolAlarm.Data, BPToStringHVolAlarm.MessageNum, BPToStringHVolAlarm.MessageID,BPToStringHVolAlarm.DeviceID   );		
					
					for(j = 0 ;j < BP_NUMBER;j++)
					{
						BpToStringFlag[j].HVolAlarmFlag = 1;            
					}					
				}
				break;
				
				case 2:
				{
					MESSAGE BPToStringLVolAlarm;
					BPToStringLVolAlarm.Data[0] = Alarm_CellUnderVoltage;
					BPToStringLVolAlarm.Data[1] = Update;
					BPToStringLVolAlarm.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
					BPToStringLVolAlarm.Data[3] = 0x01;
					BPToStringLVolAlarm.Data[4] = BPToStringLVolAlarmbit.Byte1;
					BPToStringLVolAlarm.Data[5] = BPToStringLVolAlarmbit.Byte2;
					BPToStringLVolAlarm.Data[6] = BPToStringLVolAlarmbit.Byte3;
					BPToStringLVolAlarm.Data[7] = BPToStringLVolAlarmbit.Byte4;
					BPToStringLVolAlarm.MessageNum = 8;
					BPToStringLVolAlarm.MessageID = MsgID_Alarm;
					BPToStringLVolAlarm.DeviceID = -1 ;
					MsgGroupID = ID_Alarm;		
					CAN_make_send_from_BP(BPToStringLVolAlarm.Data, BPToStringLVolAlarm.MessageNum, BPToStringLVolAlarm.MessageID,BPToStringLVolAlarm.DeviceID  );	

					for(j = 0 ;j <BP_NUMBER;j++)
					{
						BpToStringFlag[j].LVolAlarmFlag = 1;
					}
				}
				break;
				
				case 3:
				{
					MESSAGE BPToStringHTempAlarm;
					BPToStringHTempAlarm.Data[0] = Alarm_CellOverTemp;
					BPToStringHTempAlarm.Data[1] = Update;
					BPToStringHTempAlarm.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
					BPToStringHTempAlarm.Data[3] = 0x01;
					BPToStringHTempAlarm.Data[4] = BPToStringHTempAlarmbit.Byte1;
					BPToStringHTempAlarm.Data[5] = BPToStringHTempAlarmbit.Byte2;
					BPToStringHTempAlarm.Data[6] = BPToStringHTempAlarmbit.Byte3;
					BPToStringHTempAlarm.Data[7] = BPToStringHTempAlarmbit.Byte4;
					BPToStringHTempAlarm.MessageNum = 8;
					BPToStringHTempAlarm.MessageID = MsgID_Alarm;
					BPToStringHTempAlarm.DeviceID = -1 ;
					MsgGroupID = ID_Alarm;			
					CAN_make_send_from_BP(BPToStringHTempAlarm.Data, BPToStringHTempAlarm.MessageNum, BPToStringHTempAlarm.MessageID,BPToStringHTempAlarm.DeviceID  );		
					
					for(j = 0 ;j <BP_NUMBER;j++)
					{
						BpToStringFlag[j].HTempAlarmFlag = 1;
					}					
				}				
				break;
				
				case 4:
				{
					MESSAGE BPToStringLTempAlarm;
					BPToStringLTempAlarm.Data[0] = Alarm_CellUnderTemp;
					BPToStringLTempAlarm.Data[1] = Update;
					BPToStringLTempAlarm.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
					BPToStringLTempAlarm.Data[3] = 0x01;
					BPToStringLTempAlarm.Data[4] = BPToStringLTempAlarmbit.Byte1;
					BPToStringLTempAlarm.Data[5] = BPToStringLTempAlarmbit.Byte2;
					BPToStringLTempAlarm.Data[6] = BPToStringLTempAlarmbit.Byte3;
					BPToStringLTempAlarm.Data[7] = BPToStringLTempAlarmbit.Byte4;
					BPToStringLTempAlarm.MessageNum = 8;
					BPToStringLTempAlarm.MessageID = MsgID_Alarm;
					BPToStringLTempAlarm.DeviceID = -1 ;
					MsgGroupID = ID_Alarm;			
					CAN_make_send_from_BP(BPToStringLTempAlarm.Data, BPToStringLTempAlarm.MessageNum, BPToStringLTempAlarm.MessageID,BPToStringLTempAlarm.DeviceID  );	

					for(j = 0 ;j <BP_NUMBER;j++)
					{
						BpToStringFlag[j].LTempAlarmFlag = 1;
					}
				}
				break;
				
				case 5:
				{
					MESSAGE BPToStringHVolWarning;
					BPToStringHVolWarning.Data[0] = Warning_CellOverVoltage;
					BPToStringHVolWarning.Data[1] = Update;
					BPToStringHVolWarning.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
					BPToStringHVolWarning.Data[3] = 0x01;
					BPToStringHVolWarning.Data[4] = BPToStringHVolWarningbit.Byte1;
					BPToStringHVolWarning.Data[5] = BPToStringHVolWarningbit.Byte2;
					BPToStringHVolWarning.Data[6] = BPToStringHVolWarningbit.Byte3;
					BPToStringHVolWarning.Data[7] = BPToStringHVolWarningbit.Byte4;
					BPToStringHVolWarning.MessageNum = 8;
					BPToStringHVolWarning.MessageID = MsgID_Warning;
					BPToStringHVolWarning.DeviceID = -1 ;
					MsgGroupID = ID_Warning;			
					CAN_make_send_from_BP(BPToStringHVolWarning.Data, BPToStringHVolWarning.MessageNum, BPToStringHVolWarning.MessageID,BPToStringHVolWarning.DeviceID );
					
					for(j = 0 ;j <BP_NUMBER;j++)
					{
						BpToStringFlag[j].HVolWarningFlag = 1;
					}					
				}
				break;
				
				case 6:
				{
					MESSAGE BPToStringLVolWarning;
					BPToStringLVolWarning.Data[0] = Warning_CellUnderVoltage;
					BPToStringLVolWarning.Data[1] = Update;
					BPToStringLVolWarning.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
					BPToStringLVolWarning.Data[3] = 0x01;
					BPToStringLVolWarning.Data[4] = BPToStringLVolWarningbit.Byte1;
					BPToStringLVolWarning.Data[5] = BPToStringLVolWarningbit.Byte2;
					BPToStringLVolWarning.Data[6] = BPToStringLVolWarningbit.Byte3;
					BPToStringLVolWarning.Data[7] = BPToStringLVolWarningbit.Byte4;
					BPToStringLVolWarning.MessageNum = 8;
					BPToStringLVolWarning.MessageID = MsgID_Warning;
					BPToStringLVolWarning.DeviceID = -1 ;
					MsgGroupID = ID_Warning;			
					CAN_make_send_from_BP(BPToStringLVolWarning.Data, BPToStringLVolWarning.MessageNum, BPToStringLVolWarning.MessageID,BPToStringLVolWarning.DeviceID  );
						
					for(j = 0 ;j <BP_NUMBER;j++)
					{
						BpToStringFlag[j].LVolWarningFlag= 1;
					}
				}
				break;
				
				case 7:
				{
					MESSAGE BPToStringHTempWarning;
					BPToStringHTempWarning.Data[0] = Warning_CellOverTemp;
					BPToStringHTempWarning.Data[1] = Update;
					BPToStringHTempWarning.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
					BPToStringHTempWarning.Data[3] = 0x01;
					BPToStringHTempWarning.Data[4] = BPToStringHTempWarningbit.Byte1;
					BPToStringHTempWarning.Data[5] = BPToStringHTempWarningbit.Byte2;
					BPToStringHTempWarning.Data[6] = BPToStringHTempWarningbit.Byte3;
					BPToStringHTempWarning.Data[7] = BPToStringHTempWarningbit.Byte4;
					BPToStringHTempWarning.MessageNum = 8;
					BPToStringHTempWarning.MessageID = MsgID_Warning;
					BPToStringHTempWarning.DeviceID = -1 ;
					MsgGroupID = ID_Warning;			
					CAN_make_send_from_BP(BPToStringHTempWarning.Data, BPToStringHTempWarning.MessageNum, BPToStringHTempWarning.MessageID,BPToStringHTempWarning.DeviceID);	
					for(j = 0 ;j <BP_NUMBER;j++)
					{
						BpToStringFlag[j].HTempWarningFlag = 1;
					}					
				}
				break;
				
				case 8:
				{
					MESSAGE BPToStringLTempWarning;
					BPToStringLTempWarning.Data[0] = Warning_CellUnderTemp;
					BPToStringLTempWarning.Data[1] = Update;
					BPToStringLTempWarning.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
					BPToStringLTempWarning.Data[3] = 0x01;
					BPToStringLTempWarning.Data[4] = BPToStringLTempWarningbit.Byte1;
					BPToStringLTempWarning.Data[5] = BPToStringLTempWarningbit.Byte2;
					BPToStringLTempWarning.Data[6] = BPToStringLTempWarningbit.Byte3;
					BPToStringLTempWarning.Data[7] = BPToStringLTempWarningbit.Byte4;
					BPToStringLTempWarning.MessageNum = 8;
					BPToStringLTempWarning.MessageID = MsgID_Warning;
					BPToStringLTempWarning.DeviceID = -1;
					MsgGroupID = ID_Warning;			
					CAN_make_send_from_BP(BPToStringLTempWarning.Data, BPToStringLTempWarning.MessageNum, BPToStringLTempWarning.MessageID,BPToStringLTempWarning.DeviceID);
					for(j = 0 ;j <BP_NUMBER;j++)
					{
						BpToStringFlag[j].LTempWarningFlag = 1;
					}					
				}
				break;
				
				case 9:
				{
					MESSAGE BPToStringChargeStringCurrentAlarm;
					BPToStringChargeStringCurrentAlarm.Data[0] = Alarm_HighChargeRate;
					BPToStringChargeStringCurrentAlarm.Data[1] = Update;
					BPToStringChargeStringCurrentAlarm.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
					BPToStringChargeStringCurrentAlarm.Data[3] = 0x01;
					BPToStringChargeStringCurrentAlarm.Data[4] = BPToStringChargeRateAlarmbit.Byte1;
					BPToStringChargeStringCurrentAlarm.Data[5] = BPToStringChargeRateAlarmbit.Byte2;
					BPToStringChargeStringCurrentAlarm.Data[6] = BPToStringChargeRateAlarmbit.Byte3;
					BPToStringChargeStringCurrentAlarm.Data[7] = BPToStringChargeRateAlarmbit.Byte4;
					
					BPToStringChargeStringCurrentAlarm.MessageNum = 8;
					BPToStringChargeStringCurrentAlarm.MessageID = MsgID_Alarm;
					BPToStringChargeStringCurrentAlarm.DeviceID = -1 ;
					MsgGroupID = ID_Alarm;			
					CAN_make_send_from_BP(BPToStringChargeStringCurrentAlarm.Data, BPToStringChargeStringCurrentAlarm.MessageNum, BPToStringChargeStringCurrentAlarm.MessageID,BPToStringChargeStringCurrentAlarm.DeviceID);	
					
					for(j = 0 ;j <BP_NUMBER;j++)
					{
						BpToStringFlag[j].ChargeStringCurrentAlarmFlag = 1;
					}						
				}
				break;
				
				case 10:
				{
					MESSAGE BPToStringDischargeStringCurrentAlarm;
					BPToStringDischargeStringCurrentAlarm.Data[0] = Alarm_HighDischargeRate;
					BPToStringDischargeStringCurrentAlarm.Data[1] = Update;
					BPToStringDischargeStringCurrentAlarm.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
					BPToStringDischargeStringCurrentAlarm.Data[3] = 0x01;
					BPToStringDischargeStringCurrentAlarm.Data[4] = BPToStringDischargeRateAlarmbit.Byte1;
					BPToStringDischargeStringCurrentAlarm.Data[5] = BPToStringDischargeRateAlarmbit.Byte2;
					BPToStringDischargeStringCurrentAlarm.Data[6] = BPToStringDischargeRateAlarmbit.Byte3;
					BPToStringDischargeStringCurrentAlarm.Data[7] = BPToStringDischargeRateAlarmbit.Byte4;
					
					BPToStringDischargeStringCurrentAlarm.MessageNum = 8;
					BPToStringDischargeStringCurrentAlarm.MessageID = MsgID_Alarm;
					BPToStringDischargeStringCurrentAlarm.DeviceID = -1 ;
					MsgGroupID = ID_Alarm;			
					CAN_make_send_from_BP(BPToStringDischargeStringCurrentAlarm.Data, BPToStringDischargeStringCurrentAlarm.MessageNum, BPToStringDischargeStringCurrentAlarm.MessageID,BPToStringDischargeStringCurrentAlarm.DeviceID);					
					
					for(j = 0 ;j <BP_NUMBER;j++)
					{
						BpToStringFlag[j].DischargeStringCurrentAlarmFlag = 1;
					}					
				}
				
				break;
				
				case 11:
				{
					MESSAGE BPToStringChargeStringCurrentWarning;
					BPToStringChargeStringCurrentWarning.Data[0] = Warning_HighChargeRate;
					BPToStringChargeStringCurrentWarning.Data[1] = Update;
					BPToStringChargeStringCurrentWarning.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
					BPToStringChargeStringCurrentWarning.Data[3] = 0x01;
					BPToStringChargeStringCurrentWarning.Data[4] = BPToStringChargeRateWarningbit.Byte1;
					BPToStringChargeStringCurrentWarning.Data[5] = BPToStringChargeRateWarningbit.Byte2;
					BPToStringChargeStringCurrentWarning.Data[6] = BPToStringChargeRateWarningbit.Byte3;
					BPToStringChargeStringCurrentWarning.Data[7] = BPToStringChargeRateWarningbit.Byte4;
					
					BPToStringChargeStringCurrentWarning.MessageNum = 8;
					BPToStringChargeStringCurrentWarning.MessageID = MsgID_Warning;
					BPToStringChargeStringCurrentWarning.DeviceID = -1 ;
					MsgGroupID = ID_Warning;			
					CAN_make_send_from_BP(BPToStringChargeStringCurrentWarning.Data, BPToStringChargeStringCurrentWarning.MessageNum, BPToStringChargeStringCurrentWarning.MessageID,BPToStringChargeStringCurrentWarning.DeviceID);					
					
					for(j = 0 ;j <BP_NUMBER;j++)
					{
						BpToStringFlag[j].ChargeStringCurrentWarningFlag = 1;
					}					

				}
				break;
				
				case 12:
				{
					MESSAGE BPToStringDischargeStringCurrentWarning;
					BPToStringDischargeStringCurrentWarning.Data[0] = Warning_HighDischargeRate;
					BPToStringDischargeStringCurrentWarning.Data[1] = Update;
					BPToStringDischargeStringCurrentWarning.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
					BPToStringDischargeStringCurrentWarning.Data[3] = 0x01;
					BPToStringDischargeStringCurrentWarning.Data[4] = BPToStringDischargeRateWarningbit.Byte1;
					BPToStringDischargeStringCurrentWarning.Data[5] = BPToStringDischargeRateWarningbit.Byte2;
					BPToStringDischargeStringCurrentWarning.Data[6] = BPToStringDischargeRateWarningbit.Byte3;
					BPToStringDischargeStringCurrentWarning.Data[7] = BPToStringDischargeRateWarningbit.Byte4;
					
					BPToStringDischargeStringCurrentWarning.MessageNum = 8;
					BPToStringDischargeStringCurrentWarning.MessageID = MsgID_Warning;
					BPToStringDischargeStringCurrentWarning.DeviceID = -1 ;
					MsgGroupID = ID_Warning;			
					CAN_make_send_from_BP(BPToStringDischargeStringCurrentWarning.Data, BPToStringDischargeStringCurrentWarning.MessageNum, BPToStringDischargeStringCurrentWarning.MessageID,BPToStringDischargeStringCurrentWarning.DeviceID);					
				
					for(j = 0 ;j <BP_NUMBER;j++)
					{
						BpToStringFlag[j].DischargeStringCurrentWarningFlag = 1;
					}					
				}
				break;
				
			
			}
		}
	}
}

void UpdateBPAWECount()
{
	Count.OldBPHVoltAlarm = CalculateCountOfState(BPToStringHVolAlarmbit.BPbyte);
	Count.OldBPLVoltAlarm = CalculateCountOfState(BPToStringLVolAlarmbit.BPbyte);
	Count.OldBPHTempAlarm = CalculateCountOfState(BPToStringHTempAlarmbit.BPbyte);
	Count.OldBPLTempAlarm = CalculateCountOfState(BPToStringLTempAlarmbit.BPbyte);
	Count.OldBPHChCurrentAlarm = CalculateCountOfState(BPToStringChargeRateAlarmbit.BPbyte);
	Count.OldBPHDisCurrentAlarm = CalculateCountOfState(BPToStringDischargeRateAlarmbit.BPbyte);
	
	Count.OldBPHVoltWarning = CalculateCountOfState(BPToStringHVolWarningbit.BPbyte);
	Count.OldBPLVoltWarning = CalculateCountOfState(BPToStringLVolWarningbit.BPbyte);
	Count.OldBPHTempWarning = CalculateCountOfState(BPToStringHTempWarningbit.BPbyte);
	Count.OldBPLTempWarning = CalculateCountOfState(BPToStringLTempWarningbit.BPbyte);
	Count.OldBPHChCurrentWarning = CalculateCountOfState(BPToStringChargeRateWarningbit.BPbyte);
	Count.OldBPHDisCurrentWarning = CalculateCountOfState(BPToStringDischargeRateWarningbit.BPbyte);
	
	Count.OldBPHighCellVolDeltaError = CalculateCountOfState(BPToStringHighCellDeltaErrorbit.BPbyte);	
	
}

void HighAveDeltaVolErrorJudge()
{
	int i = 0;
	for(i = 0;i< BP_NUMBER;i++)
	{
		if(mStringData.averageCellVolt != 0 && mStringData.bp[i].averageCellVolt !=0 && BpLossCommunicationWarningFlag[i] == 0)
		{
			if(fabs(mStringData.averageCellVolt - mStringData.bp[i].averageCellVolt) <= HighAveDeltaVolErrorThresholdValue)   /*暂时设置为300mv*/
			{
				BPHAveDeltaVolErrorbit.BPbyte  &= ~(1 << i);
	
			}
			else
			{
				StringAWEHappenedFlag.HighAveVoltaDeltaError = 1;
				if(CalculateCountOfState(BPHAveDeltaVolErrorbit.BPbyte) < 1)
				{
					MESSAGE msg;
					BPHAveDeltaVolErrorbit.BPbyte |= 1 << i;		
					msg.MessageID = ID_Error;
					MsgGroupID = ID_Error;
					msg.Data[0] = Error_HighAveVoltaDelta;
					msg.Data[1] = Update; 
					msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
					msg.Data[3] = 0x00;
					msg.Data[4] = BPHAveDeltaVolErrorbit.Byte1;
					msg.Data[5] = BPHAveDeltaVolErrorbit.Byte2;
					msg.Data[6] = BPHAveDeltaVolErrorbit.Byte3;
					msg.Data[7] = BPHAveDeltaVolErrorbit.Byte4;
					msg.DeviceID = -1;
					CAN_make_send_from_BP(msg.Data,8,MsgID_Error,msg.DeviceID); 				
				}	
				BPHAveDeltaVolErrorbit.BPbyte |= 1 << i;				
			}
		}
	}
	
}