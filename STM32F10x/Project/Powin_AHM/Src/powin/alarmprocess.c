/*add by jason ，2015年11月3日11:01:30*/

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
#define AlarmWarningJudgeTimer 1  //(4 * 2)
_WarningAlarmCmpValue CmpValue[BP_NUMBER];
u16 ContactorClosedPermissionFlag;


_SetClrValue SetClrValue;
_BP *BPLoseCommunicationErrorbit;       /*add by jason 2016年8月29日10:00:26*/

_BP *BPHVolAlarmbit;
_BP *BPHVolWarningbit;
_BP *BPHTempAlarmbit;
_BP *BPHTempWarningbit;

_BP *BPLVolAlarmbit;
_BP *BPLVolWarningbit;
_BP *BPLTempAlarmbit;
_BP *BPLTempWarningbit;

_BP *BPCellOfflineErrorbit;
_BP  BPHAveDeltaVolErrorbit;





_BP BPToStringHVolAlarmbit;
_BP BPToStringHVolWarningbit;
_BP BPToStringHTempAlarmbit;
_BP BPToStringHTempWarningbit;

_BP BPToStringLVolAlarmbit;
_BP BPToStringLVolWarningbit;
_BP BPToStringLTempAlarmbit;
_BP BPToStringLTempWarningbit;


_BP BPToStringChargeRateAlarmbit;
_BP BPToStringChargeRateWarningbit;
_BP BPToStringDischargeRateAlarmbit;
_BP BPToStringDischargeRateWarningbit;



/*change by jason,@2016年7月21日16:44:45*/
_BP BPToStringCellOfflineErrorbit;

_BP BPToStringHighCellDeltaErrorbit;

_BP BPLoseofCommunicationnErrorbit;

u8 ErrorLoseofCommunicationFlag[BP_NUMBER] = {0};
u8 ErrorLoseofCommunicationClrFlag[BP_NUMBER] = {0};

//_BP *BPChargeStringCurrentAlarmbit;
//_BP *BPChargeStringCurrentWarningbit;
//_BP *BPDischargeStringCurrentAlarmbit;
//_BP *BPDischargeStringCurrentWarningbit;                      

_AlarmWarningCount AlarmWarningCount[BP_NUMBER];
_Count Count;
u8 StringState;
u8 ContactorError_Flag = 0;


_SysStateFlag SysStateFlag[BP_NUMBER];
_SysStateAckFlag SysStateAckFlag[BP_NUMBER];

_SysStateFlag BpToStringFlag[BP_NUMBER];      /*bp to string，string to array*/
_SysStateFlag BpToStringChargeDisChargeFLag;  /*bp to string，string to array*/

_SysStateFlag SysChargeDisChargeState;
_SysStateAckFlag SysChargeDisChargeFlag;

_SysClrAck    WaitClrAckFlag[BP_NUMBER];

 BPAWEHappened BPAWEHappenedFlag;
 StringAWEHappened StringAWEHappenedFlag;
 


MESSAGE ClrAck[BP_NUMBER];
BUFFER setVoltageBufferTemp;
BUFFER setVoltageBufferTemp1;
//MESSAGE SysAlarmWarningMsg;
u8 ContactorOpenWarningAck = 0;
u8 EnterContactorOpenWarningFlag = 0;
u8 ContactorOpenwarningClrAck = 0;

u16 Idle5minFlag = 0;
u16 Idle5minCountFlag = 0;
u16 Idle5minCount = 0;
u16 BalanceVoltage = 3400;
u8  ArrayStringSendTargetVolFlag = 0;            /*0表示 string 来发 均衡目标电压/ 1表示 Array 发送均衡电压*/
u16 ArrayStringSendTargetVol = 0;
/*产生Alarm或者Warning置位*/
u8 BPHVolAlarmFlag[BP_NUMBER] = {0};
u8 BPLVolAlarmFlag[BP_NUMBER] = {0};
u8 BPHTempAlarmFlag[BP_NUMBER] = {0};
u8 BPLTempAlarmFlag[BP_NUMBER] = {0};

u8 StringHVolAlarmFlag[BP_NUMBER] = {0};
u8 StringLVolAlarmFlag[BP_NUMBER] = {0};
u8 StringHTempAlarmFlag[BP_NUMBER] = {0};
u8 StringLTempAlarmFlag[BP_NUMBER] = {0};

u8 BPHVolWarningFlag[BP_NUMBER] = {0};
u8 BPLVolWarningFlag[BP_NUMBER] = {0};
u8 BPHTempWarningFlag[BP_NUMBER] = {0};
u8 BPLTempWarningFlag[BP_NUMBER] = {0};

u8 StringHVolWarningFlag[BP_NUMBER] = {0};
u8 StringLVolWarningFlag[BP_NUMBER] = {0};
u8 StringHTempWarningFlag[BP_NUMBER] = {0};
u8 StringLTempWarningFlag[BP_NUMBER] = {0}; 

u8 BpLossCommunicationWarningFlag[BP_NUMBER] = {0};


u8 SystemLightBlinkingExitFlag = 0;


 

void BMC_OFFLINE_AlarmWarningCheck()
{
	int i,j;
	for(i = 0;i < BP_NUMBER;i++)
	{
		for(j = 0;j <CELL_NUMBER;j++)
		{
			if(BMC_OFFLINE_AlarmSet_Flag[i][j] == 1)
			{
//				contactor1_off();
//				contactor2_off();
		 
				if(BMC_OFFLINE_AlarmSetAck_Flag[i][j] == 0)
				{
					MESSAGE BMCOFFLINETemp;
			
					BMCOFFLINETemp.Data[0] = Alarm_BMCOFFLINE;
					BMCOFFLINETemp.Data[1] = AlarmSet;
					BMCOFFLINETemp.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
					BMCOFFLINETemp.Data[3] = i + 1;     
					BMCOFFLINETemp.Data[4] = j + 1;
					BMCOFFLINETemp.Data[5] = 0x00;
					BMCOFFLINETemp.Data[6] = 0x00;
					BMCOFFLINETemp.Data[7] = BMC_OFFLINE_AlarmReason[i][j];
					BMCOFFLINETemp.MessageNum = 8;
					BMCOFFLINETemp.MessageID = MsgID_Alarm;
					BMCOFFLINETemp.DeviceID = i ;
					MsgGroupID = ID_Alarm;			
					CAN_make_send_from_BP(BMCOFFLINETemp.Data, BMCOFFLINETemp.MessageNum, BMCOFFLINETemp.MessageID,BMCOFFLINETemp.DeviceID   );					
				}
			}
			if(BMC_OFFLINE_AlarmClr_Flag[i][j] == 1)
			{
				
				if(BMC_OFFLINE_AlarmClrAck_Flag[i][j] == 0)	
				{
					MESSAGE BMCOFFLINETemp;
			
					BMCOFFLINETemp.Data[0] = Alarm_BMCOFFLINE;
					BMCOFFLINETemp.Data[1] = AlarmClr;
					BMCOFFLINETemp.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
					BMCOFFLINETemp.Data[3] = i + 1;     
					BMCOFFLINETemp.Data[4] = j + 1;
					BMCOFFLINETemp.Data[5] = 0x00;
					BMCOFFLINETemp.Data[6] = 0x00;
					BMCOFFLINETemp.Data[7] = BMC_OFFLINE_AlarmReason[i][j];
					BMCOFFLINETemp.MessageNum = 8;
					BMCOFFLINETemp.MessageID = MsgID_Alarm;
					BMCOFFLINETemp.DeviceID = i ;
					MsgGroupID = ID_Alarm;			
					CAN_make_send_from_BP(BMCOFFLINETemp.Data, BMCOFFLINETemp.MessageNum, BMCOFFLINETemp.MessageID,BMCOFFLINETemp.DeviceID   );				
				}
			}
			if(BMC_OFFLINE_WarningSet_Flag[i][j] == 1)
			{
				if(BMC_OFFLINE_WarningSetAck_Flag[i][j] == 0)	
				{
					MESSAGE BMCOFFLINETemp;
			
					BMCOFFLINETemp.Data[0] = Warning_BMCOFFLINE;
					BMCOFFLINETemp.Data[1] = WarningSet;
					BMCOFFLINETemp.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
					BMCOFFLINETemp.Data[3] = i + 1;     
					BMCOFFLINETemp.Data[4] = j + 1;
					BMCOFFLINETemp.Data[5] = 0x00;
					BMCOFFLINETemp.Data[6] = 0x00;
					BMCOFFLINETemp.Data[7] = BMC_OFFLINE_WarningReason[i][j];
					BMCOFFLINETemp.MessageNum = 8;
					BMCOFFLINETemp.MessageID = MsgID_Warning;
					BMCOFFLINETemp.DeviceID = i ;
					MsgGroupID = ID_Warning;			
					CAN_make_send_from_BP(BMCOFFLINETemp.Data, BMCOFFLINETemp.MessageNum, BMCOFFLINETemp.MessageID,BMCOFFLINETemp.DeviceID   );				
				}
			}
			if(BMC_OFFLINE_WarningClr_Flag[i][j] == 1)
			{
				if(BMC_OFFLINE_WarningClrAck_Flag[i][j] == 0)
				{
					MESSAGE BMCOFFLINETemp;
			
					BMCOFFLINETemp.Data[0] = Warning_BMCOFFLINE;
					BMCOFFLINETemp.Data[1] = WarningClr;
					BMCOFFLINETemp.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
					BMCOFFLINETemp.Data[3] = i + 1;     
					BMCOFFLINETemp.Data[4] = j + 1;
					BMCOFFLINETemp.Data[5] = 0x00;
					BMCOFFLINETemp.Data[6] = 0x00;
					BMCOFFLINETemp.Data[7] = BMC_OFFLINE_WarningReason[i][j];
					BMCOFFLINETemp.MessageNum = 8;
					BMCOFFLINETemp.MessageID = MsgID_Warning;
					BMCOFFLINETemp.DeviceID = i ;
					MsgGroupID = ID_Warning;			
					CAN_make_send_from_BP(BMCOFFLINETemp.Data, BMCOFFLINETemp.MessageNum, BMCOFFLINETemp.MessageID,BMCOFFLINETemp.DeviceID   );							
				}
			}	

			if(BMC_OFFLINE_ErrorSet_Flag[i][j] == 1)
			{
				if(BMC_OFFLINE_ErrorSetAck_Flag[i][j] == 0)
				{
					MESSAGE BMCOFFLINETemp;
			
					BMCOFFLINETemp.Data[0] = Error_BMCOFFLINE;
					BMCOFFLINETemp.Data[1] = ErrorSet;
					BMCOFFLINETemp.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
					BMCOFFLINETemp.Data[3] = i + 1;     
					BMCOFFLINETemp.Data[4] = j + 1;
					BMCOFFLINETemp.Data[5] = 0x00;
					BMCOFFLINETemp.Data[6] = 0x00;
					BMCOFFLINETemp.Data[7] = BMC_OFFLINE_ErrorReason[i][j];
					BMCOFFLINETemp.MessageNum = 8;
					BMCOFFLINETemp.MessageID = MsgID_Error;
					BMCOFFLINETemp.DeviceID = i ;
					MsgGroupID = ID_Error;			
					CAN_make_send_from_BP(BMCOFFLINETemp.Data, BMCOFFLINETemp.MessageNum, BMCOFFLINETemp.MessageID,BMCOFFLINETemp.DeviceID   );						
					
					
				}
					
			}
			if(BMC_OFFLINE_ErrorClr_Flag[i][j] == 1)
			{
				if(BMC_OFFLINE_ErrorClrAck_Flag[i][j] == 0)
				{
					MESSAGE BMCOFFLINETemp;
			
					BMCOFFLINETemp.Data[0] = Error_BMCOFFLINE;
					BMCOFFLINETemp.Data[1] = ErrorClr;
					BMCOFFLINETemp.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
					BMCOFFLINETemp.Data[3] = i + 1;     
					BMCOFFLINETemp.Data[4] = j + 1;
					BMCOFFLINETemp.Data[5] = 0x00;
					BMCOFFLINETemp.Data[6] = 0x00;
					BMCOFFLINETemp.Data[7] = BMC_OFFLINE_ErrorReason[i][j];
					BMCOFFLINETemp.MessageNum = 8;
					BMCOFFLINETemp.MessageID = MsgID_Error;
					BMCOFFLINETemp.DeviceID = i ;
					MsgGroupID = ID_Error;			
					CAN_make_send_from_BP(BMCOFFLINETemp.Data, BMCOFFLINETemp.MessageNum, BMCOFFLINETemp.MessageID,BMCOFFLINETemp.DeviceID   );						
					
					
				}
					
			}
		}
	}
}





void ReSendStringAWE()
{
	MESSAGE msg;

	{
		
		msg.Data[0] = Alarm_CellOverVoltage;   /*Alarm Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
		msg.Data[3] = 0x00;        /*reserved*/
		
		msg.Data[4] = BPHVolAlarmbit->Byte1;
		msg.Data[5] = BPHVolAlarmbit->Byte2;
		msg.Data[6] = BPHVolAlarmbit->Byte3;
		msg.Data[7] = BPHVolAlarmbit->Byte4;				
		
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Alarm;
		msg.DeviceID = -1;
		MsgGroupID = ID_Alarm;
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);
	}

	{
		msg.Data[0] = Alarm_CellUnderVoltage;   /*Alarm Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
		msg.Data[3] = 0x00;        /*reserved*/
		
		msg.Data[4] = BPLVolAlarmbit->Byte1;
		msg.Data[5] = BPLVolAlarmbit->Byte2;
		msg.Data[6] = BPLVolAlarmbit->Byte3;
		msg.Data[7] = BPLVolAlarmbit->Byte4;				
		
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Alarm;
		msg.DeviceID = -1 ;		
		MsgGroupID = ID_Alarm;
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);
	}

	{
		msg.Data[0] = Alarm_CellOverTemp;   /*Alarm Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
		msg.Data[3] = 0x00;        /*reserved*/
		msg.Data[4] = BPHTempAlarmbit->Byte1;
		msg.Data[5] = BPHTempAlarmbit->Byte2;
		msg.Data[6] = BPHTempAlarmbit->Byte3;
		msg.Data[7] = BPHTempAlarmbit->Byte4;				
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Alarm;
		msg.DeviceID = -1 ;	
		MsgGroupID = ID_Alarm;				
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);				
	}

	{
		msg.Data[0] = Alarm_CellUnderTemp;   /*Alarm Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
		msg.Data[3] = 0x00;        /*reserved*/
		msg.Data[4] = BPLTempAlarmbit->Byte1;
		msg.Data[5] = BPLTempAlarmbit->Byte2;
		msg.Data[6] = BPLTempAlarmbit->Byte3;
		msg.Data[7] = BPLTempAlarmbit->Byte4;				
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Alarm;
		msg.DeviceID = -1 ;	
		MsgGroupID = ID_Alarm;
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);				
	}
	 

	{
		msg.Data[0] = Warning_CellOverVoltage;   /*Alarm Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
		msg.Data[3] = 0x00;        /*reserved*/
		
		msg.Data[4] = BPHVolWarningbit->Byte1;
		msg.Data[5] = BPHVolWarningbit->Byte2;
		msg.Data[6] = BPHVolWarningbit->Byte3;
		msg.Data[7] = BPHVolWarningbit->Byte4;				
		
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Warning;
		msg.DeviceID = -1 ;	
		MsgGroupID = ID_Warning;				
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);
	}
                                                     /*有Alarm ，发送Alarm*/
	{
		msg.Data[0] = Warning_CellUnderVoltage;   /*Alarm Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
		msg.Data[3] = 0x00;        /*reserved*/
		
		msg.Data[4] = BPLVolWarningbit->Byte1;
		msg.Data[5] = BPLVolWarningbit->Byte2;
		msg.Data[6] = BPLVolWarningbit->Byte3;
		msg.Data[7] = BPLVolWarningbit->Byte4;				
		
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Warning;
		msg.DeviceID = -1 ;
		MsgGroupID = ID_Warning;
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);
	}

	{
		msg.Data[0] = Warning_CellOverTemp;   /*Alarm Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
		msg.Data[3] = 0x00;        /*reserved*/
		msg.Data[4] = BPHTempWarningbit->Byte1;
		msg.Data[5] = BPHTempWarningbit->Byte2;
		msg.Data[6] = BPHTempWarningbit->Byte3;
		msg.Data[7] = BPHTempWarningbit->Byte4;				
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Warning;
		msg.DeviceID = -1 ;	
		MsgGroupID = ID_Warning;				
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);				
	}


	{
		msg.Data[0] = Warning_CellUnderTemp;   /*Alarm Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);         /*ArrayID + StringID*/
		msg.Data[3] = 0x00;        /*reserved*/
		msg.Data[4] = BPLTempWarningbit->Byte1;
		msg.Data[5] = BPLTempWarningbit->Byte2;
		msg.Data[6] = BPLTempWarningbit->Byte3;
		msg.Data[7] = BPLTempWarningbit->Byte4;				
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Warning;;
		msg.DeviceID = -1 ;	
		MsgGroupID = ID_Warning;				
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);				
	}
	 {
		 
		 
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
	{
		msg.Data[0] = Error_HighCellVolDelta;   /*Error Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);         /*ArrayID + StringID*/
		msg.Data[3] = 0x01;        /*reserved*/
		msg.Data[4] = BPToStringHighCellDeltaErrorbit.Byte1;
		msg.Data[5] = BPToStringHighCellDeltaErrorbit.Byte2;
		msg.Data[6] = BPToStringHighCellDeltaErrorbit.Byte3;
		msg.Data[7] = BPToStringHighCellDeltaErrorbit.Byte4;				
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Error;
		msg.DeviceID = -1 ;	
		MsgGroupID = ID_Error;				
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);				
	
	}
	
	
 
 
/*BP alarm warning error*/
	{
		
		msg.Data[0] = Alarm_CellOverVoltage;   /*Alarm Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
		msg.Data[3] = 0x01;        /*reserved*/
		
		msg.Data[4] = BPToStringHVolAlarmbit.Byte1;
		msg.Data[5] = BPToStringHVolAlarmbit.Byte2;
		msg.Data[6] = BPToStringHVolAlarmbit.Byte3;
		msg.Data[7] = BPToStringHVolAlarmbit.Byte4;				
		
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Alarm;
		msg.DeviceID = -1 ;
		MsgGroupID = ID_Alarm;
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);
	}

	{
		msg.Data[0] = Alarm_CellUnderVoltage;   /*Alarm Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
		msg.Data[3] = 0x01;        /*reserved*/
		
		msg.Data[4] = BPToStringLVolAlarmbit.Byte1;
		msg.Data[5] = BPToStringLVolAlarmbit.Byte2;
		msg.Data[6] = BPToStringLVolAlarmbit.Byte3;
		msg.Data[7] = BPToStringLVolAlarmbit.Byte4;				
		
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Alarm;
		msg.DeviceID = -1 ;		
		MsgGroupID = ID_Alarm;
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);
	}

	{
		msg.Data[0] = Alarm_CellOverTemp;   /*Alarm Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
		msg.Data[3] = 0x01;        /*reserved*/
		msg.Data[4] = BPToStringHTempAlarmbit.Byte1;
		msg.Data[5] = BPToStringHTempAlarmbit.Byte2;
		msg.Data[6] = BPToStringHTempAlarmbit.Byte3;
		msg.Data[7] = BPToStringHTempAlarmbit.Byte4;				
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Alarm;
		msg.DeviceID = -1 ;	
		MsgGroupID = ID_Alarm;				
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);				
	}

	{
		msg.Data[0] = Alarm_CellUnderTemp;   /*Alarm Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
		msg.Data[3] = 1;        /*reserved*/
		msg.Data[4] = BPToStringLTempAlarmbit.Byte1;
		msg.Data[5] = BPToStringLTempAlarmbit.Byte2;
		msg.Data[6] = BPToStringLTempAlarmbit.Byte3;
		msg.Data[7] = BPToStringLTempAlarmbit.Byte4;				
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Alarm;
		msg.DeviceID = -1;	
		MsgGroupID = ID_Alarm;
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);				
	}
	 

	{
		msg.Data[0] = Warning_CellOverVoltage;   /*Alarm Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
		msg.Data[3] = 0x01;        /*reserved*/
		
		msg.Data[4] = BPToStringHVolWarningbit.Byte1;
		msg.Data[5] = BPToStringHVolWarningbit.Byte2;
		msg.Data[6] = BPToStringHVolWarningbit.Byte3;
		msg.Data[7] = BPToStringHVolWarningbit.Byte4;		
		
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Warning;
		msg.DeviceID = -1 ;	
		MsgGroupID = ID_Warning;				
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);
	}
                                                     /*有Alarm ，发送Alarm*/
	{
		msg.Data[0] = Warning_CellUnderVoltage;   /*Alarm Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
		msg.Data[3] = 0x01;        /*reserved*/
		
		msg.Data[4] = BPToStringLVolWarningbit.Byte1;
		msg.Data[5] = BPToStringLVolWarningbit.Byte2;
		msg.Data[6] = BPToStringLVolWarningbit.Byte3;
		msg.Data[7] = BPToStringLVolWarningbit.Byte4;				
		
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Warning;
		msg.DeviceID = -1 ;
		MsgGroupID = ID_Warning;
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);
	}

	{
		msg.Data[0] = Warning_CellOverTemp;   /*Alarm Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
		msg.Data[3] = 1;        /*reserved*/
		msg.Data[4] = BPToStringHTempWarningbit.Byte1;
		msg.Data[5] = BPToStringHTempWarningbit.Byte2;
		msg.Data[6] = BPToStringHTempWarningbit.Byte3;
		msg.Data[7] = BPToStringHTempWarningbit.Byte4;			
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Warning;
		msg.DeviceID = -1 ;	
		MsgGroupID = ID_Warning;				
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);				
	}


	{
		msg.Data[0] = Warning_CellUnderTemp;   /*Alarm Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);         /*ArrayID + StringID*/
		msg.Data[3] = 1;        /*reserved*/
		msg.Data[4] = BPToStringLTempWarningbit.Byte1;
		msg.Data[5] = BPToStringLTempWarningbit.Byte2;
		msg.Data[6] = BPToStringLTempWarningbit.Byte3;
		msg.Data[7] = BPToStringLTempWarningbit.Byte4;				
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Warning;;
		msg.DeviceID = -1;	
		MsgGroupID = ID_Warning;				
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);				
	}	
	
	{
		msg.Data[0] = Error_BpBooter;   /*Alarm Type*/ 
		msg.Data[1] = Update;
		msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);         /*ArrayID + StringID*/
		msg.Data[3] = 0;        /*reserved*/
		msg.Data[4] = BPbooterErrorbit.Byte1;
		msg.Data[5] = BPbooterErrorbit.Byte2;
		msg.Data[6] = BPbooterErrorbit.Byte3;
		msg.Data[7] = BPbooterErrorbit.Byte4;				
		msg.MessageNum = 8;
		msg.MessageID = MsgID_Error;
		msg.DeviceID = -1;	
		MsgGroupID = ID_Error;				
		CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);				
	}
}
void ReCheckLossCommunication()
{
//	u8 i = 0;
//	for(i = 0;i < BP_NUMBER;i++)
//	{
//		if(BpLossCommunicationWarningFlag[i] == 1)      /*表示失联*/
		{
			SysBPLoseOfCommunicationErrorMessage.Data[0] = Error_LossOfCommunication;
			SysBPLoseOfCommunicationErrorMessage.Data[1] = Update;
			SysBPLoseOfCommunicationErrorMessage.Data[2]=  ArrayStringID(mStringData.ArrayID,Self_ID);
			SysBPLoseOfCommunicationErrorMessage.Data[3] = 0x00;
			
			SysBPLoseOfCommunicationErrorMessage.Data[4] = BPLoseCommunicationErrorbit->Byte1 ;
			SysBPLoseOfCommunicationErrorMessage.Data[5] = BPLoseCommunicationErrorbit->Byte2 ;
			SysBPLoseOfCommunicationErrorMessage.Data[6] = BPLoseCommunicationErrorbit->Byte3 ;
			SysBPLoseOfCommunicationErrorMessage.Data[7] = BPLoseCommunicationErrorbit->Byte4 ;	
			
			SysBPLoseOfCommunicationErrorMessage.MessageNum = 8;
			SysBPLoseOfCommunicationErrorMessage.MessageID = MsgID_Error;
			SysBPLoseOfCommunicationErrorMessage.DeviceID = -1;
			MsgGroupID = ID_Error;	
			
			CAN_make_send_from_BP(SysBPLoseOfCommunicationErrorMessage.Data, SysBPLoseOfCommunicationErrorMessage.MessageNum, SysBPLoseOfCommunicationErrorMessage.MessageID,SysBPLoseOfCommunicationErrorMessage.DeviceID  );						
 					
		}

//	}	
}
void ReCheckContactorStatus()
{
	ContactorOpenWarningAck = 0;
	ContactorWarningProcess();
	
	/*
		contactor 因为发生了Alarm，所以contactor open，需要发送 contactor open warning
	*/
	if(Count.Alarm > 0 || Count.BpToStringAlarm > 0)
	{
		mStringData.state = Alarm_State;
		contactor1_off();
		contactor2_off();	
		/*如果contactor 关闭，会发送 contactor warning 给 array*/
		{
			uint8_t warningbuf[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
		
			warningbuf[0] = Warning_ContactorOpen;
			warningbuf[1] = WarningSet;
			warningbuf[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
			//warningbuf[2] = 0x10;
			MsgGroupID = ID_Warning;
			EnterContactorOpenWarningFlag = 1;
			//ContactorOpenWarningAck = 0;
			if(ContactorOpenWarningAck == 0)
				CAN_make_send_from_BP(warningbuf, 3, MsgID_Warning,0x00);
		}
	}
	
	/*获取到的返回值都为0,则表示contactor 处于打开状态(未发生Alarm情况下添加)*/
	if(getPostitiveContactor() == 0x00 && getPostitiveContactor2() == 0x00)   
	{
		uint8_t warningbuf[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
		
		warningbuf[0] = Warning_ContactorOpen;
		warningbuf[1] = WarningSet;
		warningbuf[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
		//warningbuf[2] = 0x10;
		MsgGroupID = ID_Warning;
		EnterContactorOpenWarningFlag = 1;           /*进入了 contactor open warning 的标志位*/
		//ContactorOpenWarningAck = 0;
		if(ContactorOpenWarningAck == 0)
			CAN_make_send_from_BP(warningbuf, 3, MsgID_Warning,0x00);		
		
	}
}

 

/*add by jason 2015年11月3日10:59:40*/
u8 testSyncCount = 0;
void SyncAWE() 
{
	u16 i = 0;
	if(SyncCount == 0 || SyncCount >= 300)     /*300s*/ 
	{
		SyncCount = 1;
		{
			MESSAGE msg;
			for(i = 0;i < BP_NUMBER;i++)
			{
				{
					u8 Bpbit1 = (unsigned char) ((BPToStringHVolAlarmbit.BPbyte & (0x00000001 << i)) >> i);
					u8 Bpbit2 = (unsigned char) ((BPToStringHVolWarningbit.BPbyte & (0x00000001 << i)) >> i);
					u8 Bpbit3 = (unsigned char) ((BPToStringHTempAlarmbit.BPbyte & (0x0001 << i)) >> i);
					u8 Bpbit4 = (unsigned char) ((BPToStringHTempWarningbit.BPbyte & (0x0001 << i)) >> i);
					u8 Bpbit5 = (unsigned char) ((BPToStringLVolAlarmbit.BPbyte & (0x0001 << i)) >> i);
					u8 Bpbit6 = (unsigned char) ((BPToStringLVolWarningbit.BPbyte & (0x0001 << i)) >> i);
					u8 Bpbit7 = (unsigned char) ((BPToStringLTempAlarmbit.BPbyte & (0x0001 << i)) >> i);
					u8 Bpbit8 = (unsigned char) ((BPToStringLTempWarningbit.BPbyte & (0x0001 << i)) >> i);
 
					
					u8 Bpbit11 = (unsigned char)((BPToStringCellOfflineErrorbit.BPbyte & (0x0001 << i)) >> i);
					
					u8 Bpbit12 = (unsigned char)((BPToStringHighCellDeltaErrorbit.BPbyte & (0x0001 << i)) >> i);
					
					
					if(Bpbit1 == 1 || Bpbit3 == 1|| Bpbit5 == 1 || Bpbit7 == 1 || Bpbit11 == 1 || Bpbit12 == 1)                /*有报警，询问BP 是否 Clr*/
					{
						msg.DeviceID =  i;
						msg.MessageID = MsgID_Alarm;
						 MsgGroupID = MsgID_Alarm;
						if(Bpbit1 == 1)
						{
							msg.Data[0] = Alarm_CellOverVoltage;
							msg.Data[1] = AWE_QueryData;
							msg.MessageNum = 2;
							msg.MessageID = MsgID_Alarm;
							
							CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID+1);
							
							testSyncCount++;
						}
						if(Bpbit3 == 1)
						{
							msg.Data[0] = Alarm_CellOverTemp;
							msg.Data[1] = AWE_QueryData;
							msg.MessageNum = 2;
							msg.MessageID = MsgID_Alarm;
							CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID+1);	

							
						}
						if(Bpbit5 == 1)
						{
							msg.Data[0] = Alarm_CellUnderVoltage;
							msg.Data[1] = AWE_QueryData;
							msg.MessageNum = 2;
							msg.MessageID = MsgID_Alarm;
							CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID+1);							
						}	
						if(Bpbit7 == 1)
						{
							msg.Data[0] = Alarm_CellUnderTemp;
							msg.Data[1] = AWE_QueryData;
							msg.MessageNum = 2;
							msg.MessageID = MsgID_Alarm;
							CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID+1);							
						}
						
 
						

					}
					
					if(Bpbit2 == 1 || Bpbit4 == 1|| Bpbit6 == 1 || Bpbit8 == 1 )             /*有Warning，询问是否Clr*/
					{
						msg.DeviceID = i;
						msg.MessageID = MsgID_Warning;
						MsgGroupID = MsgID_Warning;
						if(Bpbit2 == 1)
						{
							msg.Data[0] = Warning_CellOverVoltage;
							msg.Data[1] = AWE_QueryData;
							msg.MessageNum = 2;
							CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID+1);
							testSyncCount++;
						}
						if(Bpbit4 == 1)
						{
							msg.Data[0] = Warning_CellOverTemp;
							msg.Data[1] = AWE_QueryData;
							msg.MessageNum = 2;
							CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID+1);							
						}
						if(Bpbit6 == 1)
						{
							msg.Data[0] = Warning_CellUnderVoltage;
							msg.Data[1] = AWE_QueryData;
							msg.MessageNum = 2;
							CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID+1);							
						}	
						if(Bpbit8 == 1)
						{
							msg.Data[0] = Warning_CellUnderTemp;
							msg.Data[1] = AWE_QueryData;
							msg.MessageNum = 2;
							CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID+1);							
						}							
					}
						if(Bpbit12 == 1)      /*High Vol delta error*/
						{
							msg.DeviceID = i;
							msg.MessageID = MsgID_Error;
							MsgGroupID = MsgID_Error;							
							msg.Data[0] = Error_HighCellVolDelta;
							msg.Data[1] = AWE_QueryData;
							msg.MessageNum = 2;
							CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID+1);
							
						}
 
				}
#if 1			
				msg.DeviceID = i;                                                                 /*Query  Current QWE*/
				msg.MessageID = MsgID_QueryAllAWE;
				msg.MessageNum = 0;
			     MsgGroupID = ID_Command;
				CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID+1);		
#endif	
		
			}
				
			
#if 1
			ReCheckContactorStatus();
			ReCheckLossCommunication();
			ReSendStringAWE();
			 
#endif	
		}
	}
	
}

 
void ContactorWarningProcess()
{
	if(EnterContactorOpenWarningFlag == 1)
	{
		if(is_contactor_1_closed()  && is_contactor_2_closed() )      /* 从 contactor open 退回到 contactor close状态*/
		{
			static u8 contactorclosedcount = 0;
			contactorclosedcount++;
			if(contactorclosedcount >= 5)
			{
				contactorclosedcount = 0;
				ContactorOpenwarningClrAck = 1;													/*如果都关闭了发送clr*/
				ContactorOpenWarningAck = 0;
				EnterContactorOpenWarningFlag = 0;
			}
		}		
	}
	 
	{
		uint8_t warningbuf[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
		 
		warningbuf[0] = Warning_ContactorOpen;
		warningbuf[1] = WarningClr;
		
		warningbuf[2] = ArrayStringID(0x01,Self_ID);
//		warningbuf[3] = 0x66;
 
		MsgGroupID = ID_Warning;
		if(ContactorOpenwarningClrAck == 1)
		{
			CAN_make_send_from_BP(warningbuf, 3, MsgID_Warning,0x00);			
			
		}
	}	
}
void HeartBeatToBP()
{
 	int i = 0;
//	u16 BalanceVoltage = 3200;
	
	MESSAGE ToArrayTargetVolatge;
//	BalanceVoltage = mStringData.averageCellVolt;
//	if(mStringData.mode == Idle_Mode && mStringData.averageCellVolt <= 3200)
//		BalanceVoltage = 3200;
//	BalanceVoltage = 3200;
	if(mStringData.measureStringVoltage > 50)
	{
		mStringData.kW = 60 * mStringData.measureStringVoltage;
	}
   if(getPostitiveContactor() == 0x00 && getPostitiveContactor2() == 0x00)
   {
	  if(GetMaintencanceModeFlagFromSys == 1)
	  {
		  GetMaintencanceModeFlagFromSys = 0;
		  /*发送maintenance ack给*/
		  {
			  if(MaintenanceModeFlag == 1)        /*如果是maintenance set*/
			  {
					uint8_t maintenancedata[8] = {0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
		
	
					MsgGroupID = ID_Data;
					CAN_make_send_from_BP(maintenancedata, 1, AcktoMaintenanceMode,0x00);				  
				  
			  }
			  else                                /*maitenance clr*/
			  {
					uint8_t maintenancedata[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
					MsgGroupID = ID_Data;
					CAN_make_send_from_BP(maintenancedata, 1, AcktoMaintenanceMode,0x00);				  	  
				  
			  }
			  
			  
		  }
		  
	  }
	   
	   
   }
	if(MaintenanceModeFlag == 0)
	{
#if 0		
		if(mStringData.dcbusVoltage <= 50)   /*DCBUS 总线没有电压*/
		{
 
				if(ContactorClosedPermissionFlag == 1)
				{
					ContactorClosedPermissionFlag = 0;
					if(Count.Alarm == 0 && Count.BpToStringAlarm == 0)
					{
						ContactorAllTurnOn();
					}
				}
		}
		else
#endif
		{
			if(ContactorClosedPermissionFlag == 1)
			{
				DetermineContactorState();
			}
		}
	}
#if 1	
	if(mStringData.mode == Idle_Mode )
	{
		Idle5minCountFlag = 1;                                     /*5min 时间标志 b*/
		if(Idle5minFlag == 0 && mStringData.averageCellVolt != 0)
		{
			if(mStringData.averageCellVolt > 2500 && mStringData.averageCellVolt <= 3600)
			{
				BalanceVoltage = mStringData.averageCellVolt;
			}
			else if(mStringData.averageCellVolt <=2500)
			{
				BalanceVoltage = 2500;
			}
			else if(mStringData.averageCellVolt > 3600)
			{
				BalanceVoltage = 3600;
			}
		}
	}
	else if(mStringData.mode == Charge_Mode || mStringData.mode == DisCharge_Mode)
	{
			if(mStringData.averageCellVolt > 2500 && mStringData.averageCellVolt <= 3600)
			{
				BalanceVoltage = mStringData.averageCellVolt;
			}
			else if(mStringData.averageCellVolt <=2500)
			{
				BalanceVoltage = 2500;
			}
			else if(mStringData.averageCellVolt > 3600)
			{
				BalanceVoltage = 3600;
			}
		
	}
	if(TargetValueCountTimeOutFlag == 0) /*target value from array controller 未超时*/
	{
		if(ArrayStringSendTargetVol != 0 && ArrayStringSendTargetVol >= 2500 && ArrayStringSendTargetVol <= 3600)
		{
			BalanceVoltage = ArrayStringSendTargetVol;
		}
		else if(ArrayStringSendTargetVol == 0)
		{
			if(mStringData.averageCellVolt > 2500 && mStringData.averageCellVolt <= 3600)
			{
				BalanceVoltage = mStringData.averageCellVolt;
			}
			else if(mStringData.averageCellVolt <=2500)
			{
				BalanceVoltage = 2500;
			}
			else if(mStringData.averageCellVolt > 3600)
			{
				BalanceVoltage = 3600;
			}
		}
	}
	else
	{
			if(mStringData.averageCellVolt > 2500 && mStringData.averageCellVolt <= 3600)
			{
				BalanceVoltage = mStringData.averageCellVolt;
			}
			else if(mStringData.averageCellVolt <=2500)
			{
				BalanceVoltage = 2500;
			}
			else if(mStringData.averageCellVolt > 3600)
			{
				BalanceVoltage = 3600;
			}
	}
#endif
	

		
	if(QueryExtremeValueTimer >= 2000)               /*2s*/
	{
		
//		StringQueryContactorClosedPermission();
		BMC_OFFLINE_AlarmWarningCheck();
		send_autodata_Ex_2_BP(MsgIDToBP_BPExtremeVoltage, BPBroadCastID); //42
		send_autodata_Ex_2_BP(MsgIDToBP_BPExtremeTemp, BPBroadCastID);    //43
		send_autodata_Ex_2_BP(MsgIDToBP_BPBalTime, BPBroadCastID);   
		ToArray.Led1sCount = 0;
		
		                                         
		setVoltageBufferTemp1.ID = CAN_MESSAGE_ID(ID_Data,STRING_TYPE,Self_ID,0x00,0x20);   /*发送给 bp*/
		setVoltageBufferTemp1.data.bytes[0] = (u8)(BalanceVoltage >> 8);
		setVoltageBufferTemp1.data.bytes[1] = (u8)BalanceVoltage;  
		setVoltageBufferTemp1.data.bytes[2] = 0x00;
		setVoltageBufferTemp1.data.bytes[3] = 0x00;
		if(fabs(mStringData.stringCurrent) >= 0 && fabs(mStringData.stringCurrent) <= 9000)                                                        /*电流滤波处理*/
		{
			setVoltageBufferTemp1.data.bytes[4] = (u8)((mStringData.stringCurrent / 10) >> 8);
			setVoltageBufferTemp1.data.bytes[5] = (u8)(mStringData.stringCurrent / 10);
		}
		setVoltageBufferTemp1.data.bytes[6] = 0x00;   
		setVoltageBufferTemp1.data.bytes[7] = 0x00;
		
		setVoltageBufferTemp1.DLC = 8;		
		CAN_make_send_2_BP(&setVoltageBufferTemp1);	
	
			
		QueryExtremeValueTimer = 0;
		
		ToArrayTargetVolatge.Data[0] = (u8)(BalanceVoltage >> 8);             /*发送给array*/
		ToArrayTargetVolatge.Data[1] = (u8)BalanceVoltage;
		ToArrayTargetVolatge.Data[2] = 0;
		ToArrayTargetVolatge.Data[3] = 0;
		ToArrayTargetVolatge.Data[4] = 0;
		ToArrayTargetVolatge.Data[5] = 0;
		ToArrayTargetVolatge.Data[6] = 0;
		ToArrayTargetVolatge.Data[7] = 0;
		ToArrayTargetVolatge.MessageNum = 8;
		ToArrayTargetVolatge.MessageID = 0x20;
		ToArrayTargetVolatge.DeviceID = 0 ;
		MsgGroupID = ID_Data;
		CAN_make_send_from_BP(ToArrayTargetVolatge.Data, ToArrayTargetVolatge.MessageNum, ToArrayTargetVolatge.MessageID,ToArrayTargetVolatge.DeviceID   );
		  
	}
	GetBPAverageVoltage();
#if 1	
	if(HeartBeatTimer >= 3000)    /*3s 发送一次 heartbeat指令*/
	{
		HeartBeatTimer = 0;
		for(i = 0 ;i < BP_NUMBER;i++)
		{
			Count.BpLoseHeatBeat[i]++;
			if(Count.BpLoseHeatBeat[i] > BPLossOfcommunicationTime)       /*3 * 40 = 120s 有BP 出现 失联情况*/
			{
//				uint8_t ErrorBuf[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
/*for testing*/	BpLossCommunicationWarningFlag[i] = 1;	

				BPLoseCommunicationErrorbit->BPbyte |= 1 << i;        /*set the bit*/
				Count.BpLoseHeatBeat[i] = 0; 
				SysStateFlag[i].BPLoseofCommunicationErrorFlag = 1;
				if(CalculateCountOfState(BPLoseCommunicationErrorbit->BPbyte) <= 1)
				{
					
					SysBPLoseOfCommunicationErrorMessage.Data[0] = Error_LossOfCommunication;
					SysBPLoseOfCommunicationErrorMessage.Data[1] = Update;
					SysBPLoseOfCommunicationErrorMessage.Data[2]=  ArrayStringID(mStringData.ArrayID,Self_ID);
					SysBPLoseOfCommunicationErrorMessage.Data[3] = 0x00;
					
					
					SysBPLoseOfCommunicationErrorMessage.MessageNum = 8;
					SysBPLoseOfCommunicationErrorMessage.MessageID = MsgID_Error;
					SysBPLoseOfCommunicationErrorMessage.DeviceID = i ;
					MsgGroupID = ID_Error;	
					
					if(SysStateAckFlag[i].BPLoseofCommunicationErrorAckFlag == 0)  
					{
						CAN_make_send_from_BP(SysBPLoseOfCommunicationErrorMessage.Data, SysBPLoseOfCommunicationErrorMessage.MessageNum, SysBPLoseOfCommunicationErrorMessage.MessageID,SysBPLoseOfCommunicationErrorMessage.DeviceID  );						
					}	
				}
			}
			else if(BpLossCommunicationWarningFlag[i] == 0 && SysStateFlag[i].BPLoseofCommunicationErrorFlag == 1)
			{
				SysStateFlag[i].BPLoseofCommunicationErrorFlag = 0;
				SysBPLoseOfCommunicationErrorMessage.Data[1] = ErrorClr;
				BPLoseCommunicationErrorbit->BPbyte &= ~(1 << i);         /*clr the bit*/  
				SysBPLoseOfCommunicationErrorMessage.DeviceID = i;
				
				MsgGroupID = ID_Error;
				CAN_make_send_from_BP(SysBPLoseOfCommunicationErrorMessage.Data, SysBPLoseOfCommunicationErrorMessage.MessageNum, SysBPLoseOfCommunicationErrorMessage.MessageID,SysBPLoseOfCommunicationErrorMessage.DeviceID  );	

				SysStateAckFlag[i].BPLoseofCommunicationErrorAckFlag = 0;	
					
				WaitClrAckFlag[i].BPLoseofCommunicationErrorFlag = 1;
			}
 
		}
		send_autodata_Ex_2_BP(MsgIDToBP_HeartBeat,BPBroadCastID);
	}
#endif	
	
}

void AlarmWarnErrorProcess(void)
{
	u8 i = 0;
	static u8 enterRelayTurnOnFlag = 0;

	if(mStringData.mode == Charge_Mode)
	{
		BalanceVoltage = mStringData.averageCellVolt;
	}
	else if(mStringData.mode == DisCharge_Mode)
	{
		BalanceVoltage = mStringData.averageCellVolt;
	}
	else
	{
		 ;
	}
	
	if(mStringData.highCellVolt >= 3650)
	{
		RelayAllTurnOff();
		enterRelayTurnOnFlag = 1;
	}
	else
	{
		if(enterRelayTurnOnFlag == 1)
		{
			if(mStringData.highCellVolt <= 3630)
			{
				enterRelayTurnOnFlag = 0;
				RelayAllTurnOn();
			}
		}
	}
/*判断 contactor 打开还是关闭*/	
//if(Boot10sFlag == 1)
//{	
	if(Count.Alarm > 0 || Count.BpToStringAlarm > 0)
	{
		mStringData.state = Alarm_State;
		contactor1_off();
		contactor2_off();	
		
 
/*如果contactor 关闭，会发送 contactor warning 给 array*/
		{
			uint8_t warningbuf[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
		
			warningbuf[0] = Warning_ContactorOpen;
			warningbuf[1] = WarningSet;
		 
			warningbuf[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
			MsgGroupID = ID_Warning;
			EnterContactorOpenWarningFlag = 1;
			if(ContactorOpenWarningAck == 0)
				CAN_make_send_from_BP(warningbuf, 3, MsgID_Warning,0x00);
		}
	}
	else if(Count.Warning > 0 || Count.BpToStringWarning > 0)
	{
		mStringData.state = Warning_State;
 
	}
	else if(Count.Error > 0 || Count.BpToStringError > 0)
	{
		mStringData.state = Error_State;
 
	}
	else
	{
		mStringData.state = Normal_State;
 		
	}
	if(getPostitiveContactor() == 0x00 && getPostitiveContactor2() == 0x00) /*contactor 都open*/
	{
		uint8_t warningbuf[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
		
		warningbuf[0] = Warning_ContactorOpen;
		warningbuf[1] = WarningSet;
		warningbuf[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
		//warningbuf[2] = 0x10;
		MsgGroupID = ID_Warning;
		EnterContactorOpenWarningFlag = 1;
 
		if(ContactorOpenWarningAck == 0)
			CAN_make_send_from_BP(warningbuf, 3, MsgID_Warning,0x00);		
		
	}	
	else
	{
		ContactorOpenWarningAck = 0;
	
	}
		
 	ContactorWarningProcess();
	

/*contactor驱动端口和contactor sensor*/
	{
#if 1		
		if(GPIO_ReadInputDataBit(CONT1_SNSn_PORT, CONT1_SNSn_PIN) == GPIO_ReadOutputDataBit(CONT1_PORT, CONT1_PIN) \
		   || GPIO_ReadInputDataBit(CONT2_SNSn_PORT, CONT2_SNSn_PIN) == GPIO_ReadOutputDataBit(CONT2_PORT, CONT2_PIN))
			 
		{
			
			static u8 ContactorErrorCount = 0;
				
			uint8_t ErrorBuf[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
#if 1 /*For test*/
			
			uint8_t Data1 = GPIO_ReadInputDataBit(CONT1_SNSn_PORT, CONT1_SNSn_PIN);
			uint8_t Data2 = GPIO_ReadOutputDataBit(CONT1_PORT, CONT1_PIN);
			uint8_t Data3 = GPIO_ReadInputDataBit(CONT2_SNSn_PORT, CONT2_SNSn_PIN);
			uint8_t Data4 = GPIO_ReadOutputDataBit(CONT2_PORT, CONT2_PIN);
#endif			
			ContactorErrorCount++;
			if(ContactorErrorCount >= 20)
			{	
				ContactorErrorCount = 0;
				ErrorBuf[0] = Error_ContactorError;
				ErrorBuf[1] = ErrorSet;
				ErrorBuf[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
				MsgGroupID = ID_Error;
				if(ContactorError_Flag == 0)
					CAN_make_send_from_BP(ErrorBuf, 8, MsgID_Error,0x00);
			}
		}
#endif	
	}	
		
/*string controller determine the awe by itself */	
	for(i = 0;i < BP_NUMBER;i++)
	{	
		if(mStringData.bp[i].highCellVolt >= CmpValue[i].HVoltageAlarm)  /*High Voltage Alarm*/
		{
			StringAWEHappenedFlag.HVoltageAlarm = 1;
			AlarmWarningCount[i].HVoltAlarm++;
			if(AlarmWarningCount[i].HVoltAlarm >= AlarmWarningJudgeTimer)
			{
				AlarmWarningCount[i].HVoltAlarm = 0;
				AlarmWarningCount[i].HVoltWarning = 0;
				AlarmWarningCount[i].HVoltClr = 0;
				SysStateFlag[i].HVolAlarmFlag = 1;
				if(SysStateFlag[i].HVolWarningFlag == 1) /*系统退出warning来到alarm状态,发送Warning clr信息*/
				{
					SysStateFlag[i].HVolWarningFlag = 0;
					
					SysHVolWarningMessage.Data[1] = Update;                /*清除相应BP的warning*/
					BPHVolWarningbit->BPbyte &= ~(1 << i);         /*clr the bit*/
					SysHVolWarningMessage.DeviceID = i ;
					//MsgGroupID = ID_Warning;
					//CAN_make_send_from_BP(SysHVolWarningMessage.Data, SysHVolWarningMessage.MessageNum, SysHVolWarningMessage.MessageID,SysHVolWarningMessage.DeviceID  );					
					SysStateAckFlag[i].HVolAlarmAckFlag = 0;
				}
				
				contactor1_off(); // turn off contactor			
				contactor2_off(); // turn off contactor				
				
				CmpValue[i].HVoltageAlarm = SetClrValue.ClrCellOverVoltageAlarm + 1;
				CmpValue[i].HVoltageWarning = SetClrValue.SetCellOverVoltageWarning;
				
				SysHVolAlarmMessage.Data[0] = Alarm_CellOverVoltage;   /*Alarm Type*/ 
				SysHVolAlarmMessage.Data[1] = Update;
				SysHVolAlarmMessage.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
				SysHVolAlarmMessage.Data[3] = 0x00;        /*reserved*/
				
				
				
				SysHVolAlarmMessage.MessageNum = 8;
				SysHVolAlarmMessage.MessageID = MsgID_Alarm;
				SysHVolAlarmMessage.DeviceID = i ;
				MsgGroupID = ID_Alarm;
				
				Count.OldHVoltAlarm = CalculateCountOfState(BPHVolAlarmbit->BPbyte);
				
				if(CalculateCountOfState(BPHVolAlarmbit->BPbyte) == 0)
				{
					BPHVolAlarmbit->BPbyte |= 1 << i;        /*set the bit*/
					CAN_make_send_from_BP(SysHVolAlarmMessage.Data, SysHVolAlarmMessage.MessageNum, SysHVolAlarmMessage.MessageID,SysHVolAlarmMessage.DeviceID  );
				}
				BPHVolAlarmbit->BPbyte |= 1 << i;        /*set the bit*/
				
			}
			 
		}
		else if(mStringData.bp[i].highCellVolt >= CmpValue[i].HVoltageWarning)  /*High Voltage warning*/
		{
			 
			StringAWEHappenedFlag.HVoltageWarning = 1;
			AlarmWarningCount[i].HVoltWarning++;
			if(AlarmWarningCount[i].HVoltWarning >= AlarmWarningJudgeTimer)
			{
				AlarmWarningCount[i].HVoltWarning = 0;
				AlarmWarningCount[i].HVoltAlarm = 0;
				AlarmWarningCount[i].HVoltClr = 0;
				SysStateFlag[i].HVolWarningFlag = 1;
				if(SysStateFlag[i].HVolAlarmFlag == 1)
				{
					SysStateFlag[i].HVolAlarmFlag = 0;
					SysHVolAlarmMessage.Data[1] = Update;
					BPHVolAlarmbit->BPbyte &= ~(1 << i);        /*clr the bit*/  
					SysHVolAlarmMessage.DeviceID = i;
//					MsgGroupID = ID_Alarm;
//					CAN_make_send_from_BP(SysHVolAlarmMessage.Data, SysHVolAlarmMessage.MessageNum, SysHVolAlarmMessage.MessageID,SysHVolAlarmMessage.DeviceID  );	
					SysStateAckFlag[i].HVolWarningAckFlag = 0;
				}
				
			
				CmpValue[i].HVoltageAlarm = SetClrValue.SetCellOverVoltageAlarm;
				CmpValue[i].HVoltageWarning = SetClrValue.ClrCellOverVoltageWarning + 1;
//				CmpValue[i].LVoltageAlarm = SetClrValue.SetCellUnderVoltageAlarm;
//				CmpValue[i].LVoltageWarning = SetClrValue.SetCellUnderVoltageWarning;	
				
				SysHVolWarningMessage.Data[0] = Warning_CellOverVoltage;   /*Alarm Type*/ 
				SysHVolWarningMessage.Data[1] = Update;
				SysHVolWarningMessage.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
				SysHVolWarningMessage.Data[3] = 0x00;        /*reserved*/
				
				
	
				SysHVolWarningMessage.MessageNum = 8;
				SysHVolWarningMessage.MessageID = MsgID_Warning;
				SysHVolWarningMessage.DeviceID = i ;
				MsgGroupID = ID_Warning;
 			
				Count.OldHVoltWarning = CalculateCountOfState(BPHVolWarningbit->BPbyte);
				
				if(CalculateCountOfState(BPHVolWarningbit->BPbyte) == 0)
				{
					BPHVolWarningbit->BPbyte |= 1 << i;        /*set the bit*/
					CAN_make_send_from_BP(SysHVolWarningMessage.Data, SysHVolWarningMessage.MessageNum, SysHVolWarningMessage.MessageID,SysHVolWarningMessage.DeviceID );					
				}
				BPHVolWarningbit->BPbyte |= 1 << i;        /*set the bit*/
				
			}
		}
		else
		{
			
			AlarmWarningCount[i].HVoltWarning = 0;
			AlarmWarningCount[i].HVoltAlarm = 0;
			AlarmWarningCount[i].HVoltClr++;
			if(AlarmWarningCount[i].HVoltClr >= AlarmWarningJudgeTimer)
			{
				AlarmWarningCount[i].HVoltClr = 0;
				
				if(SysStateFlag[i].HVolAlarmFlag == 1)
				{
					Count.OldHVoltAlarm = CalculateCountOfState(BPHVolAlarmbit->BPbyte);
					SysStateFlag[i].HVolAlarmFlag = 0;
					SysHVolAlarmMessage.Data[1] = Update;
					BPHVolAlarmbit->BPbyte &= ~(1 << i);         /*clr the bit*/  
					SysHVolAlarmMessage.DeviceID = i;
//					MsgGroupID = ID_Alarm;
//					CAN_make_send_from_BP(SysHVolAlarmMessage.Data, SysHVolAlarmMessage.MessageNum, SysHVolAlarmMessage.MessageID,SysHVolAlarmMessage.DeviceID  );	
					CmpValue[i].HVoltageAlarm= SetClrValue.SetCellOverVoltageAlarm;
					SysStateAckFlag[i].HVolAlarmAckFlag = 0;
				
				}			
				if(SysStateFlag[i].HVolWarningFlag == 1) /*系统退出warning来到alarm状态,发送Warning clr信息*/
				{
					Count.OldHVoltWarning = CalculateCountOfState(BPHVolWarningbit->BPbyte);
					SysStateFlag[i].HVolWarningFlag = 0;
					
					SysHVolWarningMessage.Data[1] = Update;                /*清除相应BP的warning*/
					BPHVolWarningbit->BPbyte &= ~(1 << i);         /*clr the bit*/
					SysHVolWarningMessage.DeviceID = i ;
//					MsgGroupID = ID_Warning;
//					CAN_make_send_from_BP(SysHVolWarningMessage.Data, SysHVolWarningMessage.MessageNum, SysHVolWarningMessage.MessageID,SysHVolWarningMessage.DeviceID  );	
					CmpValue[i].HVoltageWarning = SetClrValue.SetCellOverVoltageWarning;
					SysStateAckFlag[i].HVolWarningAckFlag = 0;
				}	
			}
		}
		
		if(mStringData.bp[i].lowCellVolt <= CmpValue[i].LVoltageAlarm && (mStringData.bp[i].lowCellVolt != 0))/*Low Voltage Alarm 2400*/
		{
			 
			StringAWEHappenedFlag.LVoltageAlarm = 1;
			AlarmWarningCount[i].LVoltAlarm++;
			if(AlarmWarningCount[i].LVoltAlarm >= AlarmWarningJudgeTimer)
			{
				AlarmWarningCount[i].LVoltAlarm = 0;
				AlarmWarningCount[i].LVoltWarning = 0;
				AlarmWarningCount[i].LVoltClr = 0;
				SysStateFlag[i].LVolAlarmFlag = 1;
				if(SysStateFlag[i].LVolWarningFlag == 1)  /* clr Low voltage warning */
				{
					SysStateFlag[i].LVolWarningFlag = 0;
					SysLVolWarningMessage.Data[1] = Update;       /* clr low vol warning*/
					BPLVolWarningbit->BPbyte &= ~(1 << i);        /*clr the bit*/
					SysHVolWarningMessage.DeviceID = i ;         
//					MsgGroupID = ID_Warning;
//					CAN_make_send_from_BP(SysLVolWarningMessage.Data, SysLVolWarningMessage.MessageNum, SysLVolWarningMessage.MessageID,SysLVolWarningMessage.DeviceID   );
					SysStateAckFlag[i].LVolAlarmAckFlag = 0;
				}
		
					contactor1_off(); // turn off contactor			
					contactor2_off(); // turn off contactor				
	
//					CmpValue[i].HVoltageAlarm = SetClrValue.SetCellOverVoltageAlarm;
//					CmpValue[i].HVoltageWarning = SetClrValue.SetCellOverVoltageWarning;
					CmpValue[i].LVoltageAlarm = SetClrValue.ClrCellUnderVoltageAlarm - 1;
					CmpValue[i].LVoltageWarning = SetClrValue.SetCellUnderVoltageWarning;
		
					
					SysLVolAlarmMessage.Data[0] = Alarm_CellUnderVoltage;   /*Alarm Type*/ 
					SysLVolAlarmMessage.Data[1] = Update;
					SysLVolAlarmMessage.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
					SysLVolAlarmMessage.Data[3] = 0x00;        /*reserved*/
					
					
	
					SysLVolAlarmMessage.MessageNum = 8;
					SysLVolAlarmMessage.MessageID = MsgID_Alarm;
					SysLVolAlarmMessage.DeviceID = i ;
					MsgGroupID = ID_Alarm;
					Count.OldLVoltAlarm =  CalculateCountOfState(BPLVolAlarmbit->BPbyte);
					
					if(CalculateCountOfState(BPLVolAlarmbit->BPbyte) == 0)
					{
						BPLVolAlarmbit->BPbyte |= 1 << i;        /*Set the bit*/
						CAN_make_send_from_BP(SysLVolAlarmMessage.Data, SysLVolAlarmMessage.MessageNum, SysLVolAlarmMessage.MessageID,SysLVolAlarmMessage.DeviceID );
					}
					BPLVolAlarmbit->BPbyte |= 1 << i;        /*Set the bit*/
					
			}
				
		}
		else if(mStringData.bp[i].lowCellVolt <= CmpValue[i].LVoltageWarning) /*Low Voltage Warning*/
		{
			StringAWEHappenedFlag.LVoltageWarning = 1; 
			AlarmWarningCount[i].LVoltWarning++;
			if(AlarmWarningCount[i].LVoltWarning >= AlarmWarningJudgeTimer)
			{
				AlarmWarningCount[i].LVoltAlarm = 0;
				AlarmWarningCount[i].LVoltWarning = 0;
				AlarmWarningCount[i].LVoltClr = 0;
				SysStateFlag[i].LVolWarningFlag = 1;
				
				if(SysStateFlag[i].LVolAlarmFlag == 1)  /* clr Low voltage Alarm */
				{
					SysStateFlag[i].LVolAlarmFlag = 0;
					SysLVolAlarmMessage.Data[1] = Update;       
					BPLVolAlarmbit->BPbyte &= ~(1 << i);         
					SysHVolAlarmMessage.DeviceID = i ;         
//					MsgGroupID = ID_Alarm;
//					CAN_make_send_from_BP(SysLVolAlarmMessage.Data, SysLVolAlarmMessage.MessageNum, SysLVolAlarmMessage.MessageID,SysLVolAlarmMessage.DeviceID  );
					SysStateAckFlag[i].LVolWarningAckFlag = 0;
				}
				

				CmpValue[i].LVoltageAlarm = SetClrValue.SetCellUnderVoltageAlarm;
				CmpValue[i].LVoltageWarning = SetClrValue.ClrCellUnderVoltageWarning - 1;	
				
				SysLVolWarningMessage.Data[0] = Warning_CellUnderVoltage;   /*Warning Type*/ 
				SysLVolWarningMessage.Data[1] = Update;
				SysLVolWarningMessage.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
				SysLVolWarningMessage.Data[3] = 0x00;        /*reserved*/
				
				
				
				SysLVolWarningMessage.MessageNum = 8;
				SysLVolWarningMessage.MessageID = MsgID_Warning;
				SysLVolWarningMessage.DeviceID = i ;
				MsgGroupID = ID_Warning;
				Count.OldLVoltWarning = CalculateCountOfState(BPLVolWarningbit->BPbyte);
				
				if(CalculateCountOfState(BPLVolWarningbit->BPbyte) == 0)
				{
					BPLVolWarningbit->BPbyte |= 1 << i;        /*set the bit*/
					CAN_make_send_from_BP(SysLVolWarningMessage.Data, SysLVolWarningMessage.MessageNum, SysLVolWarningMessage.MessageID,SysLVolWarningMessage.DeviceID  );
				}
				BPLVolWarningbit->BPbyte |= 1 << i;        /*set the bit*/
				
			} 
		}
		else                                                  /*进入正常电压范围，由Warning进入或者一直处于这个状态*/
		{
			AlarmWarningCount[i].LVoltAlarm = 0;
			AlarmWarningCount[i].LVoltWarning = 0;
			AlarmWarningCount[i].LVoltClr++;
			if(AlarmWarningCount[i].LVoltClr >= AlarmWarningJudgeTimer)
			{
				AlarmWarningCount[i].LVoltClr = 0;
				if(SysStateFlag[i].LVolWarningFlag == 1)  /* clr Low voltage warning */
				{
					Count.OldLVoltWarning = CalculateCountOfState(BPLVolWarningbit->BPbyte);
					AlarmWarningCount[i].LVoltClr = 0;
					SysStateFlag[i].LVolWarningFlag = 0;
					SysLVolWarningMessage.Data[1] = Update;       /* clr low vol warning*/
					BPLVolWarningbit->BPbyte &= ~(1 << i);         /*clr the bit*/
					SysLVolWarningMessage.DeviceID = i ;         
//					MsgGroupID = ID_Warning;
//					CAN_make_send_from_BP(SysLVolWarningMessage.Data, SysLVolWarningMessage.MessageNum, SysLVolWarningMessage.MessageID,SysLVolWarningMessage.DeviceID  );
					CmpValue[i].LVoltageWarning = SetClrValue.SetCellUnderVoltageWarning;
					SysStateAckFlag[i].LVolWarningAckFlag = 0;
				}			
				if(SysStateFlag[i].LVolAlarmFlag == 1)  /* clr Low voltage Alarm */
				{
					Count.OldLVoltAlarm =  CalculateCountOfState(BPLVolAlarmbit->BPbyte);
					SysStateFlag[i].LVolAlarmFlag = 0;
					SysLVolAlarmMessage.Data[1] = Update;       
					BPLVolAlarmbit->BPbyte &= ~(1 << i);         
					SysLVolAlarmMessage.DeviceID = i ;         
//					MsgGroupID = ID_Alarm;
//					CAN_make_send_from_BP(SysLVolAlarmMessage.Data, SysLVolAlarmMessage.MessageNum, SysLVolAlarmMessage.MessageID,SysLVolAlarmMessage.DeviceID  );
					CmpValue[i].LVoltageAlarm = SetClrValue.SetCellUnderVoltageAlarm;
					SysStateAckFlag[i].LVolAlarmAckFlag = 0;
				}
			}
		}
	
 		
		if(mStringData.bp[i].highCellTemp >= CmpValue[i].HTempAlarm)  /*High Temp Alarm*/
		{
			StringAWEHappenedFlag.HTempAlarm = 1; 
			AlarmWarningCount[i].HTempAlarm++;
			if(AlarmWarningCount[i].HTempAlarm >= AlarmWarningJudgeTimer)
			{
				AlarmWarningCount[i].HTempAlarm = 0;
				AlarmWarningCount[i].HTempWarning = 0;
				AlarmWarningCount[i].HTempClr = 0;
				SysStateFlag[i].HTempAlarmFlag = 1;
				if(SysStateFlag[i].HTempWarningFlag == 1)
				{
					SysStateFlag[i].HTempWarningFlag = 0;
					SysHTempWarningMessage.Data[1] = Update;       
					BPHTempWarningbit->BPbyte &= ~(1 << i);         
					SysHTempWarningMessage.DeviceID = i ;         
//					MsgGroupID = ID_Warning;
//					CAN_make_send_from_BP(SysHTempWarningMessage.Data, SysHTempWarningMessage.MessageNum, SysHTempWarningMessage.MessageID,SysHTempWarningMessage.DeviceID  );
					SysStateAckFlag[i].HTempAlarmAckFlag = 0; /*更换新的状态时候，状态要重新置位*/
				}
				contactor1_off(); // turn off contactor			
				contactor2_off(); // turn off contactor				
				
				CmpValue[i].HTempAlarm = SetClrValue.ClrCellOverTempAlarm + 1;
				CmpValue[i].HTempWarning = SetClrValue.SetCellOverTempWarning;
				
	
				SysHTempAlarmMessage.Data[0] = Alarm_CellOverTemp;   /*Alarm Type*/ 
				SysHTempAlarmMessage.Data[1] = Update;
				SysHTempAlarmMessage.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
				SysHTempAlarmMessage.Data[3] = 0x00;        /*reserved*/
				
				
				
				SysHTempAlarmMessage.MessageNum = 8;
				SysHTempAlarmMessage.MessageID = MsgID_Alarm;
				SysHTempAlarmMessage.DeviceID = i  ;
				MsgGroupID = ID_Alarm;

				
				Count.OldHTempAlarm = CalculateCountOfState(BPHTempAlarmbit->BPbyte);
				
				if(CalculateCountOfState(BPHTempAlarmbit->BPbyte) == 0)
				{
					BPHTempAlarmbit->BPbyte |= 1 << i;        /*set the bit*/
					CAN_make_send_from_BP(SysHTempAlarmMessage.Data, SysHTempAlarmMessage.MessageNum, SysHTempAlarmMessage.MessageID,SysHTempAlarmMessage.DeviceID  );
				}
				BPHTempAlarmbit->BPbyte |= 1 << i;        /*set the bit*/
			}
		}
		else if(mStringData.bp[i].highCellTemp >= CmpValue[i].HTempWarning)  /*High Temp warning*/
		{
			StringAWEHappenedFlag.HTempWarning = 1; 
			AlarmWarningCount[i].HTempWarning++;
			if(AlarmWarningCount[i].HTempWarning >= AlarmWarningJudgeTimer)
			{
				AlarmWarningCount[i].HTempAlarm = 0;
				AlarmWarningCount[i].HTempWarning = 0;
				AlarmWarningCount[i].HTempClr = 0;
				SysStateFlag[i].HTempWarningFlag = 1; 
				if(SysStateFlag[i].HTempAlarmFlag == 1)      /* exit high temp alarm*/
				{
					SysStateFlag[i].HTempAlarmFlag = 0;
					SysHTempAlarmMessage.Data[1] = Update;
					BPHTempAlarmbit->BPbyte &= ~(1 << i); 
					SysHTempAlarmMessage.DeviceID = i ; 
//					MsgGroupID = ID_Alarm;
//					CAN_make_send_from_BP(SysHTempAlarmMessage.Data, SysHTempAlarmMessage.MessageNum, SysHTempAlarmMessage.MessageID,SysHTempAlarmMessage.DeviceID  );
					SysStateAckFlag[i].HTempWarningAckFlag = 1;
				}
				
				CmpValue[i].HTempAlarm = SetClrValue.SetCellOverTempAlarm;
				CmpValue[i].HTempWarning = SetClrValue.ClrCellOverTempWarning + 1;
//				CmpValue[i].LTempAlarm = SetClrValue.SetCellUnderTempAlarm;
//				CmpValue[i].LTempWarning = SetClrValue.SetCellUnderTempWarning;
				
				SysHTempWarningMessage.Data[0] = Warning_CellOverTemp;   /*warning Type*/ 
				SysHTempWarningMessage.Data[1] = Update;
				SysHTempWarningMessage.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
				SysHTempWarningMessage.Data[3] = 0x00;        /*reserved*/
				
				
	
				SysHTempWarningMessage.MessageNum = 8;
				SysHTempWarningMessage.MessageID = MsgID_Warning;
				SysHTempWarningMessage.DeviceID = i  ;
				MsgGroupID = ID_Warning;
				Count.OldHTempWarning = CalculateCountOfState(BPHTempWarningbit->BPbyte);
				
				if(CalculateCountOfState(BPHTempWarningbit->BPbyte) == 0)
				{
					BPHTempWarningbit->BPbyte |= 1 << i;        /*set the bit*/
					CAN_make_send_from_BP(SysHTempWarningMessage.Data, SysHTempWarningMessage.MessageNum, SysHTempWarningMessage.MessageID,SysHTempWarningMessage.DeviceID );					
				}
				BPHTempWarningbit->BPbyte |= 1 << i;        /*set the bit*/
				
		  }
		}
		else 
		{
			AlarmWarningCount[i].HTempAlarm = 0;
			AlarmWarningCount[i].HTempWarning = 0;
			AlarmWarningCount[i].HTempClr++;
			if(AlarmWarningCount[i].HTempClr >= AlarmWarningJudgeTimer)
			{
				AlarmWarningCount[i].HTempClr = 0;
				if(SysStateFlag[i].HTempAlarmFlag == 1)      /* exit high temp alarm*/
				{
					Count.OldHTempAlarm = CalculateCountOfState(BPHTempAlarmbit->BPbyte);
					SysStateFlag[i].HTempAlarmFlag = 0;
					SysHTempAlarmMessage.Data[1] = Update;
					BPHTempAlarmbit->BPbyte &= ~(1 << i); 
					SysHTempAlarmMessage.DeviceID = i ; 
//					MsgGroupID = ID_Alarm;
//					CAN_make_send_from_BP(SysHTempAlarmMessage.Data, SysHTempAlarmMessage.MessageNum, SysHTempAlarmMessage.MessageID,SysHTempAlarmMessage.DeviceID  );
					CmpValue[i].HTempAlarm = SetClrValue.SetCellOverTempAlarm ;
					SysStateAckFlag[i].HTempAlarmAckFlag = 0;
				}
				if(SysStateFlag[i].HTempWarningFlag == 1)
				{
					Count.OldHTempWarning = CalculateCountOfState(BPHTempWarningbit->BPbyte);
					SysStateFlag[i].HTempWarningFlag = 0;
					SysHTempWarningMessage.Data[1] = Update;       
					BPHTempWarningbit->BPbyte &= ~(1 << i);         
					SysHTempWarningMessage.DeviceID = i ;         
//					MsgGroupID = ID_Warning;
//					CAN_make_send_from_BP(SysHTempWarningMessage.Data, SysHTempWarningMessage.MessageNum, SysHTempWarningMessage.MessageID,SysHTempWarningMessage.DeviceID  );
					CmpValue[i].HTempWarning = SetClrValue.SetCellOverTempWarning;
					SysStateAckFlag[i].HTempWarningAckFlag = 0;
				}	
			}
		}
		
		if(mStringData.bp[i].lowCellTemp <= CmpValue[i].LTempAlarm)/*Low Temp Alarm*/
		{
			StringAWEHappenedFlag.LTempAlarm = 1; 
			AlarmWarningCount[i].LTempAlarm++;
			if(AlarmWarningCount[i].LTempAlarm >=AlarmWarningJudgeTimer )
			{
				AlarmWarningCount[i].LTempAlarm = 0;
				AlarmWarningCount[i].LTempWarning = 0;
				AlarmWarningCount[i].LTempClr = 0;
				SysStateFlag[i].LTempAlarmFlag = 1;
				if(SysStateFlag[i].LTempWarningFlag == 1)
				{
					SysStateFlag[i].LTempWarningFlag = 0;
					SysLTempWarningMessage.Data[1] = Update;
					BPLTempWarningbit->BPbyte &= ~(1 << i);
					SysLTempWarningMessage.DeviceID = i;
//					MsgGroupID = ID_Warning;
//					CAN_make_send_from_BP(SysLTempWarningMessage.Data, SysLTempWarningMessage.MessageNum, SysLTempWarningMessage.MessageID,SysLTempWarningMessage.DeviceID  );
					SysStateAckFlag[i].LTempAlarmAckFlag =0;
				}
	
				contactor1_off(); // turn off contactor			
				contactor2_off(); // turn off contactor				
				
//				CmpValue[i].HTempAlarm = SetClrValue.SetCellOverTempAlarm;
//				CmpValue[i].HTempWarning = SetClrValue.SetCellOverTempWarning;
				CmpValue[i].LTempAlarm = SetClrValue.ClrCellUnderTempAlarm - 1;
				CmpValue[i].LTempWarning = SetClrValue.SetCellUnderTempWarning;
				
				SysLTempAlarmMessage.Data[0] = Alarm_CellUnderTemp;   /*Alarm Type*/ 
				SysLTempAlarmMessage.Data[1] = Update;
				SysLTempAlarmMessage.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
				SysLTempAlarmMessage.Data[3] = 0x00;        /*reserved*/
				
				
	
				SysLTempAlarmMessage.MessageNum = 8;
				SysLTempAlarmMessage.MessageID = MsgID_Alarm;
				SysLTempAlarmMessage.DeviceID = i  ;
				MsgGroupID = ID_Alarm;
 
				Count.OldLTempAlarm = CalculateCountOfState(BPLTempAlarmbit->BPbyte);
				
				if(CalculateCountOfState(BPLTempAlarmbit->BPbyte) == 0)
				{
					BPLTempAlarmbit->BPbyte |= 1 << i;        /*set the bit*/
					CAN_make_send_from_BP(SysLTempAlarmMessage.Data, SysLTempAlarmMessage.MessageNum, SysLTempAlarmMessage.MessageID,SysLTempAlarmMessage.DeviceID );
				}
				BPLTempAlarmbit->BPbyte |= 1 << i;        /*set the bit*/
				
			}
		}
		else if(mStringData.bp[i].lowCellTemp <= CmpValue[i].LTempWarning) /*Low Voltage Warning*/
		{
			 StringAWEHappenedFlag.LTempWarning = 1;
			AlarmWarningCount[i].LTempWarning++;
			if(AlarmWarningCount[i].LTempWarning >= AlarmWarningJudgeTimer)
			{
				AlarmWarningCount[i].LTempWarning = 0;
				AlarmWarningCount[i].LTempAlarm = 0;
				AlarmWarningCount[i].LTempClr = 0;
				SysStateFlag[i].LTempWarningFlag = 1; 
				if(SysStateFlag[i].LTempAlarmFlag == 1)
				{
					SysStateFlag[i].LTempAlarmFlag = 0;
					SysLTempAlarmMessage.Data[1] = Update;
					BPLTempAlarmbit->BPbyte &= ~(1 << i);
					SysLTempAlarmMessage.DeviceID = i;
//					MsgGroupID = ID_Alarm;
//					CAN_make_send_from_BP(SysLTempAlarmMessage.Data, SysLTempAlarmMessage.MessageNum, SysLTempAlarmMessage.MessageID,SysLTempAlarmMessage.DeviceID );	
					SysStateAckFlag[i].LTempWarningAckFlag = 0;
				}
//				CmpValue[i].HTempAlarm = SetClrValue.SetCellOverTempAlarm;
//				CmpValue[i].HTempWarning = SetClrValue.SetCellOverTempWarning;
				CmpValue[i].LTempAlarm = SetClrValue.SetCellUnderTempAlarm;
				CmpValue[i].LTempWarning = SetClrValue.ClrCellUnderTempWarning - 1;
				
				SysLTempWarningMessage.Data[0] = Warning_CellUnderTemp;   /*Alarm Type*/ 
				SysLTempWarningMessage.Data[1] = Update;
				SysLTempWarningMessage.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
				SysLTempWarningMessage.Data[3] = 0x00;        /*reserved*/
				
				
		
				SysLTempWarningMessage.MessageNum = 8;
				SysLTempWarningMessage.MessageID = MsgID_Warning;
				SysLTempWarningMessage.DeviceID = i  ;
				MsgGroupID = ID_Warning;
 			
//				if(SysStateAckFlag[i].LTempWarningAckFlag == 0)
//				{
//					if(StringUnderTempWarningSendTimer[i] % 30 == 0)
//					{
//						CAN_make_send_from_BP(SysLTempWarningMessage.Data, SysLTempWarningMessage.MessageNum, SysLTempWarningMessage.MessageID,SysLTempWarningMessage.DeviceID  );			
//						if(StringUnderTempWarningSendTimer[i] >= 30)
//						{
//							StringUnderTempWarningSendTimer[i] = 1;
//						}
//					}
//					StringUnderTempWarningSendTimer[i]++;
//				}
				Count.OldLTempWarning = CalculateCountOfState(BPLTempWarningbit->BPbyte);
				
				if(CalculateCountOfState(BPLTempWarningbit->BPbyte) == 0)
				{
					BPLTempWarningbit->BPbyte |= 1 << i;        /*set the bit*/
					CAN_make_send_from_BP(SysLTempWarningMessage.Data, SysLTempWarningMessage.MessageNum, SysLTempWarningMessage.MessageID,SysLTempWarningMessage.DeviceID  );
				}
				BPLTempWarningbit->BPbyte |= 1 << i;        /*set the bit*/
		   }
		}
		else
		{
			AlarmWarningCount[i].LTempWarning = 0;
			AlarmWarningCount[i].LTempAlarm = 0;
			AlarmWarningCount[i].LTempClr++;
			if(AlarmWarningCount[i].LTempClr >= AlarmWarningJudgeTimer)
			{
				AlarmWarningCount[i].LTempClr = 0;
				if(SysStateFlag[i].LTempAlarmFlag == 1)
				{
					Count.OldLTempAlarm = CalculateCountOfState(BPLTempAlarmbit->BPbyte);
					
					SysStateFlag[i].LTempAlarmFlag = 0;
					SysLTempAlarmMessage.Data[1] = Update;
					BPLTempAlarmbit->BPbyte &= ~(1 << i);
					SysLTempAlarmMessage.DeviceID = i;
//					MsgGroupID = ID_Alarm;
//					CAN_make_send_from_BP(SysLTempAlarmMessage.Data, SysLTempAlarmMessage.MessageNum, SysLTempAlarmMessage.MessageID,SysLTempAlarmMessage.DeviceID  );	
					CmpValue[i].LTempAlarm = SetClrValue.SetCellUnderTempAlarm;
					SysStateAckFlag[i].LTempAlarmAckFlag =0;
				}
				if(SysStateFlag[i].LTempWarningFlag == 1)
				{
					Count.OldLTempWarning = CalculateCountOfState(BPLTempWarningbit->BPbyte);
					SysStateFlag[i].LTempWarningFlag = 0;
					SysLTempWarningMessage.Data[1] = Update;
					BPLTempWarningbit->BPbyte &= ~(1 << i);
					SysLTempWarningMessage.DeviceID = i;
//					MsgGroupID = ID_Warning;
//					CAN_make_send_from_BP(SysLTempWarningMessage.Data, SysLTempWarningMessage.MessageNum, SysLTempWarningMessage.MessageID,SysLTempWarningMessage.DeviceID  );
					CmpValue[i].LTempWarning = SetClrValue.SetCellUnderTempWarning;
					SysStateAckFlag[i].LTempWarningAckFlag = 0;
				}	
			}
		}	

 
    
    }		

		 
			
			 
			
 	Count.Alarm = CalculateCountOfState(BPHVolAlarmbit->BPbyte) + CalculateCountOfState(BPHTempAlarmbit->BPbyte)\
					+ CalculateCountOfState(BPLVolAlarmbit->BPbyte) + CalculateCountOfState(BPLTempAlarmbit->BPbyte) \
					+ OverChargeCount\
					+ OVerDischargeCount + BMC_OFFLINE_AlarmCount;
	
 	Count.Warning = CalculateCountOfState(BPHVolWarningbit->BPbyte) + CalculateCountOfState(BPHTempWarningbit->BPbyte)\
					+ CalculateCountOfState(BPLVolWarningbit->BPbyte) + CalculateCountOfState(BPLTempWarningbit->BPbyte) |
					+ BMC_OFFLINE_WarningCount;
	
	Count.BpToStringAlarm  = CalculateCountOfState(BPToStringHVolAlarmbit.BPbyte) + CalculateCountOfState(BPToStringHTempAlarmbit.BPbyte)\
					+ CalculateCountOfState(BPToStringLVolAlarmbit.BPbyte) + CalculateCountOfState(BPToStringLTempAlarmbit.BPbyte) \
					+ OverChargeCount + OVerDischargeCount;
	
	Count.BpToStringWarning  = CalculateCountOfState(BPToStringHVolWarningbit.BPbyte) + CalculateCountOfState(BPToStringHTempWarningbit.BPbyte)\
					+ CalculateCountOfState(BPToStringLVolWarningbit.BPbyte) + CalculateCountOfState(BPToStringLTempWarningbit.BPbyte) \
					+ OverChargeCount + OVerDischargeCount;	

	
/*High Charge String Current Alarm */
	if(mStringData.stringCurrent / 100 >= CmpValue_ChargeStringCurrentAlarm)
	{
		 
		AlarmWarningCount[0].HChCurrentAlarm++;
	
		if(AlarmWarningCount[0].HChCurrentAlarm >= AlarmWarningJudgeTimer)
		{
			AlarmWarningCount[0].HChCurrentAlarm = 0;
			AlarmWarningCount[0].HChCurrentWarning = 0;
			AlarmWarningCount[0].HChargeCurrentClr = 0;
			AlarmWarningCount[0].HDisCurrentAlarm = 0;
			AlarmWarningCount[0].HDisCurrentWarning = 0;
			AlarmWarningCount[0].HDischargeCurrentClr = 0;
			SysChargeDisChargeState.ChargeStringCurrentAlarmFlag = 1;
			if(SysChargeDisChargeState.ChargeStringCurrentWarningFlag == 1)
			{
				SysChargeDisChargeState.ChargeStringCurrentWarningFlag = 0;
				SysChargeStringCurrentWarningMessage.Data[1] = WarningClr;
				SysChargeStringCurrentWarningMessage.DeviceID = 0x00;               /*bpid = 0*/
				MsgGroupID = ID_Warning;
				CAN_make_send_from_BP(SysChargeStringCurrentWarningMessage.Data, SysChargeStringCurrentWarningMessage.MessageNum, SysChargeStringCurrentWarningMessage.MessageID,SysChargeStringCurrentWarningMessage.DeviceID  );			
				SysChargeDisChargeFlag.ChargeStringCurrentAlarmAckFlag = 0;
			}
			
			
			contactor1_off(); // turn off contactor			
			contactor2_off(); // turn off contactor				
			
			CmpValue_ChargeStringCurrentAlarm = SetClrValue.ClrHighChargeRateAlarm;
			CmpValue_ChargeStringCurrentWarning = SetClrValue.SetHighChargeRateWarning;
			CmpValue_DischargeStringCurrentAlarm = SetClrValue.SetHighDisChargeRateAlarm;
			CmpValue_DischargeStringCurrentWarning = SetClrValue.SetHighDisChargeRateWarning;
			
			SysChargeStringCurrentAlarmMessage.Data[0] = Alarm_HighChargeRate;   /*Alarm Type*/ 
			SysChargeStringCurrentAlarmMessage.Data[1] = AlarmSet;
			SysChargeStringCurrentAlarmMessage.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
			SysChargeStringCurrentAlarmMessage.Data[3] = 0x00;        /*reserved*/
			
			
			SysChargeStringCurrentAlarmMessage.Data[4] = 0x00;
			SysChargeStringCurrentAlarmMessage.Data[5] = 0x00;
			SysChargeStringCurrentAlarmMessage.Data[6] = 0x00;
			SysChargeStringCurrentAlarmMessage.Data[7] = 0x00;
			
			SysChargeStringCurrentAlarmMessage.MessageNum = 8;
		
			SysChargeStringCurrentAlarmMessage.MessageID = MsgID_Alarm;
			SysChargeStringCurrentAlarmMessage.DeviceID = 0x00;
			MsgGroupID = ID_Alarm;
			if(SysChargeDisChargeFlag.ChargeStringCurrentAlarmAckFlag == 0)
				CAN_make_send_from_BP(SysChargeStringCurrentAlarmMessage.Data, SysChargeStringCurrentAlarmMessage.MessageNum, SysChargeStringCurrentAlarmMessage.MessageID,SysChargeStringCurrentAlarmMessage.DeviceID );		
		}
	}
/*High Charge String Current Warning*/	
	else if(mStringData.stringCurrent / 100 >= CmpValue_ChargeStringCurrentWarning)
	{
		 
		AlarmWarningCount[0].HChCurrentWarning++;
		if(AlarmWarningCount[0].HChCurrentWarning >= AlarmWarningJudgeTimer)
		{
			AlarmWarningCount[0].HChCurrentAlarm = 0;
			AlarmWarningCount[0].HChCurrentWarning = 0;
			AlarmWarningCount[0].HChargeCurrentClr = 0;
			AlarmWarningCount[0].HDisCurrentAlarm = 0;
			AlarmWarningCount[0].HDisCurrentWarning = 0;
			AlarmWarningCount[0].HDischargeCurrentClr = 0;
			
			SysChargeDisChargeState.ChargeStringCurrentWarningFlag = 1;
			if(SysChargeDisChargeState.ChargeStringCurrentAlarmFlag == 1)
			{
				SysChargeDisChargeState.ChargeStringCurrentAlarmFlag = 0;
				SysChargeStringCurrentAlarmMessage.Data[1] = AlarmClr;
				SysChargeStringCurrentAlarmMessage.DeviceID = 0x00;               /*bpid = 0*/
				MsgGroupID = ID_Alarm;
				CAN_make_send_from_BP(SysChargeStringCurrentAlarmMessage.Data, SysChargeStringCurrentAlarmMessage.MessageNum, SysChargeStringCurrentAlarmMessage.MessageID,SysChargeStringCurrentAlarmMessage.DeviceID  );			
				SysChargeDisChargeFlag.ChargeStringCurrentWarningAckFlag = 0;
			}
			
			CmpValue_ChargeStringCurrentAlarm = SetClrValue.SetHighChargeRateAlarm;
			CmpValue_ChargeStringCurrentWarning = SetClrValue.ClrHighChargeRateWarning;
			CmpValue_DischargeStringCurrentAlarm = SetClrValue.SetHighDisChargeRateAlarm;
			CmpValue_DischargeStringCurrentWarning = SetClrValue.SetHighDisChargeRateWarning;
			
			SysChargeStringCurrentWarningMessage.Data[0] = Warning_HighChargeRate;   /*Alarm Type*/ 
			SysChargeStringCurrentWarningMessage.Data[1] = WarningSet;
			SysChargeStringCurrentWarningMessage.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
			SysChargeStringCurrentWarningMessage.Data[3] = 0x00;        /*reserved*/
			
			
			SysChargeStringCurrentWarningMessage.Data[4] = 0x00;
			SysChargeStringCurrentWarningMessage.Data[5] = 0x00;
			SysChargeStringCurrentWarningMessage.Data[6] = 0x00;
			SysChargeStringCurrentWarningMessage.Data[7] = 0x00;
			
			SysChargeStringCurrentWarningMessage.MessageNum = 8;
			SysChargeStringCurrentWarningMessage.MessageID = MsgID_Warning;
			SysChargeStringCurrentWarningMessage.DeviceID = 0x00;
			MsgGroupID = ID_Warning;
			if(SysChargeDisChargeFlag.ChargeStringCurrentWarningAckFlag == 0)
				CAN_make_send_from_BP(SysChargeStringCurrentWarningMessage.Data, SysChargeStringCurrentWarningMessage.MessageNum, SysChargeStringCurrentWarningMessage.MessageID,SysChargeStringCurrentWarningMessage.DeviceID );			
		}
	}
/*High Discharge String Current Alarm*/
	else if(mStringData.stringCurrent / 100 <= CmpValue_DischargeStringCurrentAlarm)
	{
		 
		AlarmWarningCount[0].HDisCurrentAlarm++;
		if(AlarmWarningCount[0].HDisCurrentAlarm >= AlarmWarningJudgeTimer)
		{
			AlarmWarningCount[0].HChCurrentAlarm = 0;
			AlarmWarningCount[0].HChCurrentWarning = 0;
			AlarmWarningCount[0].HChargeCurrentClr = 0;
			AlarmWarningCount[0].HDisCurrentAlarm = 0;
			AlarmWarningCount[0].HDisCurrentWarning = 0;
			AlarmWarningCount[0].HDischargeCurrentClr = 0;
			
			SysChargeDisChargeState.DischargeStringCurrentAlarmFlag = 1;
			if(SysChargeDisChargeState.DischargeStringCurrentWarningFlag == 1)
			{
				SysChargeDisChargeState.DischargeStringCurrentWarningFlag = 0;
				SysDischargeStringCurrentWarningMessage.Data[1] = WarningClr;
				SysDischargeStringCurrentWarningMessage.DeviceID = 0x00;               /*bpid = 0*/
				MsgGroupID = ID_Warning;
				CAN_make_send_from_BP(SysDischargeStringCurrentWarningMessage.Data, SysDischargeStringCurrentWarningMessage.MessageNum, SysDischargeStringCurrentWarningMessage.MessageID,SysDischargeStringCurrentWarningMessage.DeviceID );			
				SysChargeDisChargeFlag.DischargeStringCurrentAlarmAckFlag = 0;
			}				
			contactor1_off(); // turn off contactor			
			contactor2_off(); // turn off contactor		
			
			CmpValue_ChargeStringCurrentAlarm = SetClrValue.SetHighChargeRateAlarm;
			CmpValue_ChargeStringCurrentWarning = SetClrValue.SetHighChargeRateWarning;
			CmpValue_DischargeStringCurrentAlarm = SetClrValue.ClrHighDisChargeRateAlarm;
			CmpValue_DischargeStringCurrentWarning = SetClrValue.SetHighDisChargeRateWarning;
			
			SysDischargeStringCurrentAlarmMessage.Data[0] = Alarm_HighDischargeRate;   /*Alarm Type*/ 
			SysDischargeStringCurrentAlarmMessage.Data[1] = AlarmSet;
			SysDischargeStringCurrentAlarmMessage.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
			SysDischargeStringCurrentAlarmMessage.Data[3] = 0x00;        /*reserved*/
			
			
			SysDischargeStringCurrentAlarmMessage.Data[4] = 0x00;
			SysDischargeStringCurrentAlarmMessage.Data[5] = 0x00;
			SysDischargeStringCurrentAlarmMessage.Data[6] = 0x00;
			SysDischargeStringCurrentAlarmMessage.Data[7] = 0x00;
			
			SysDischargeStringCurrentAlarmMessage.MessageNum = 8;
			SysDischargeStringCurrentAlarmMessage.MessageID = MsgID_Alarm;
			SysDischargeStringCurrentAlarmMessage.DeviceID = 0x00;
			MsgGroupID = ID_Alarm;
			if(SysChargeDisChargeFlag.DischargeStringCurrentAlarmAckFlag == 0)
				CAN_make_send_from_BP(SysDischargeStringCurrentAlarmMessage.Data, SysDischargeStringCurrentAlarmMessage.MessageNum, SysDischargeStringCurrentAlarmMessage.MessageID,SysDischargeStringCurrentAlarmMessage.DeviceID );			
		}
	}
/*Discharge High Current Warning*/	
	else if(mStringData.stringCurrent / 100 <= CmpValue_DischargeStringCurrentWarning)
	{
		 
		AlarmWarningCount[0].HDisCurrentWarning++;
		if(AlarmWarningCount[0].HDisCurrentWarning >= AlarmWarningJudgeTimer)
		{
			AlarmWarningCount[0].HChCurrentAlarm = 0;
			AlarmWarningCount[0].HChCurrentWarning = 0;
			AlarmWarningCount[0].HChargeCurrentClr = 0;
			AlarmWarningCount[0].HDisCurrentAlarm = 0;
			AlarmWarningCount[0].HDisCurrentWarning = 0;
			AlarmWarningCount[0].HDischargeCurrentClr = 0;
			
			SysChargeDisChargeState.DischargeStringCurrentWarningFlag = 1;
			if(SysChargeDisChargeState.DischargeStringCurrentAlarmFlag == 1)
			{
				SysChargeDisChargeState.DischargeStringCurrentAlarmFlag = 0;
				SysDischargeStringCurrentAlarmMessage.Data[1] = WarningClr;
				SysDischargeStringCurrentAlarmMessage.DeviceID = 0x00;               /*bpid = 0*/
				MsgGroupID = ID_Warning;
				CAN_make_send_from_BP(SysDischargeStringCurrentAlarmMessage.Data, SysDischargeStringCurrentAlarmMessage.MessageNum, SysDischargeStringCurrentAlarmMessage.MessageID,SysDischargeStringCurrentAlarmMessage.DeviceID  );			
				SysChargeDisChargeFlag.DischargeStringCurrentWarningAckFlag = 0;
			}			 
			
			CmpValue_ChargeStringCurrentAlarm = SetClrValue.SetHighChargeRateAlarm;
			CmpValue_ChargeStringCurrentWarning = SetClrValue.SetHighChargeRateWarning;
			CmpValue_DischargeStringCurrentAlarm = SetClrValue.SetHighDisChargeRateAlarm;
			CmpValue_DischargeStringCurrentWarning = SetClrValue.ClrHighDisChargeRateWarning;
			
			SysDischargeStringCurrentWarningMessage.Data[0] = Warning_HighDischargeRate;   /*Alarm Type*/ 
			SysDischargeStringCurrentWarningMessage.Data[1] = WarningSet;
			SysDischargeStringCurrentWarningMessage.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);          /*ArrayID + StringID*/
			SysDischargeStringCurrentWarningMessage.Data[3] = 0x00;        /*reserved*/
			
			
			SysDischargeStringCurrentWarningMessage.Data[4] = 0x00;
			SysDischargeStringCurrentWarningMessage.Data[5] = 0x00;
			SysDischargeStringCurrentWarningMessage.Data[6] = 0x00;
			SysDischargeStringCurrentWarningMessage.Data[7] = 0x00;
			
			SysDischargeStringCurrentWarningMessage.MessageNum = 8;
			SysDischargeStringCurrentWarningMessage.MessageID = MsgID_Warning;
			SysDischargeStringCurrentWarningMessage.DeviceID = 0x00;
			MsgGroupID = ID_Warning;   
			if(SysChargeDisChargeFlag.DischargeStringCurrentWarningAckFlag == 0)
			/*uint8_t *buf, uint8_t length, uint8_t msg_id, uint8_t bp_id*/
				CAN_make_send_from_BP(SysDischargeStringCurrentWarningMessage.Data, SysDischargeStringCurrentWarningMessage.MessageNum, SysDischargeStringCurrentWarningMessage.MessageID,SysDischargeStringCurrentWarningMessage.DeviceID );			
		}
	}
	
}

 
#if 1
void StringAWE()/*30s query 做一次*/
{
	int i = 0,Acksum = 0;
	if(CalculateCountOfState(BPHVolAlarmbit->BPbyte) >= 1) /*只要有任何一个 over vol发生，判断是否大于1*/
	{
		 
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].HVolAlarmAckFlag;				
		}
		if(Acksum == 0)
		{
			MsgGroupID = ID_Alarm;
			CAN_make_send_from_BP(SysHVolAlarmMessage.Data, SysHVolAlarmMessage.MessageNum, SysHVolAlarmMessage.MessageID,SysHVolAlarmMessage.DeviceID  );			
		}	
		 
	}
	else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].HVolAlarmAckFlag;				
		}
		if(Acksum == 0)
		{
			if(StringAWEHappenedFlag.HVoltageAlarm == 1)
			{
				MsgGroupID = ID_Alarm;
				CAN_make_send_from_BP(SysHVolAlarmMessage.Data, SysHVolAlarmMessage.MessageNum, SysHVolAlarmMessage.MessageID,SysHVolAlarmMessage.DeviceID  );					
			}
		}
		else
		{
			 StringAWEHappenedFlag.HVoltageAlarm = 0;
		}
	}
	
	if(CalculateCountOfState(BPHVolWarningbit->BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].HVolWarningAckFlag;
		}
		if(Acksum == 0)
		{
			MsgGroupID = ID_Warning;
			CAN_make_send_from_BP(SysHVolWarningMessage.Data, SysHVolWarningMessage.MessageNum, SysHVolWarningMessage.MessageID,SysHVolWarningMessage.DeviceID  );
		}
	}
	else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].HVolWarningAckFlag;
		}
		if(Acksum == 0)
		{
			if(StringAWEHappenedFlag.HVoltageWarning == 1)
			{
				MsgGroupID = ID_Warning;
				CAN_make_send_from_BP(SysHVolWarningMessage.Data, SysHVolWarningMessage.MessageNum, SysHVolWarningMessage.MessageID,SysHVolWarningMessage.DeviceID  );
			}
		}		
		else
		{
			StringAWEHappenedFlag.HVoltageWarning = 0;
		}
	}
	
	if(CalculateCountOfState(BPHTempAlarmbit->BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].HTempAlarmAckFlag;
		}
		if(Acksum == 0)
		{
			MsgGroupID = ID_Alarm;
			CAN_make_send_from_BP(SysHTempAlarmMessage.Data, SysHTempAlarmMessage.MessageNum, SysHTempAlarmMessage.MessageID,SysHTempAlarmMessage.DeviceID  );		
		}
	}
	else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].HTempAlarmAckFlag;
		}
		if(Acksum == 0)
		{
			if(StringAWEHappenedFlag.HTempAlarm == 1)
			{
				MsgGroupID = ID_Alarm;
				CAN_make_send_from_BP(SysHTempAlarmMessage.Data, SysHTempAlarmMessage.MessageNum, SysHTempAlarmMessage.MessageID,SysHTempAlarmMessage.DeviceID  );		
			}
		}
		else
		{
			StringAWEHappenedFlag.HTempAlarm = 0;
		}
		
	}
 
	if(CalculateCountOfState(BPHTempWarningbit->BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].HTempWarningAckFlag;
		}
		if(Acksum == 0)
		{
			MsgGroupID = ID_Warning;
			CAN_make_send_from_BP(SysHTempWarningMessage.Data, SysHTempWarningMessage.MessageNum, SysHTempWarningMessage.MessageID,SysHTempWarningMessage.DeviceID );
		}
	}
	else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].HTempWarningAckFlag;
		}
		if(Acksum == 0)
		{
			if(StringAWEHappenedFlag.HTempWarning == 1)
			{
				MsgGroupID = ID_Warning;
				CAN_make_send_from_BP(SysHTempWarningMessage.Data, SysHTempWarningMessage.MessageNum, SysHTempWarningMessage.MessageID,SysHTempWarningMessage.DeviceID );
				
			}
			
		}	
		else
		{
			StringAWEHappenedFlag.HTempWarning = 0;
		}
		
	}

	if(CalculateCountOfState(BPLVolAlarmbit->BPbyte) >= 1) /*只要有任何一个 over vol发生，判断是否大于1*/
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].LVolAlarmAckFlag;				
		}
		if(Acksum == 0)
		{
			if(StringAWEHappenedFlag.HVoltageAlarm == 1)
			{
				MsgGroupID = ID_Alarm;
				CAN_make_send_from_BP(SysLVolAlarmMessage.Data, SysLVolAlarmMessage.MessageNum, SysLVolAlarmMessage.MessageID,SysLVolAlarmMessage.DeviceID  );			
			}
		}	
		else
		{
			StringAWEHappenedFlag.HVoltageAlarm = 0;
		}
	}
	
	if(CalculateCountOfState(BPLVolWarningbit->BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].LVolWarningAckFlag;
		}
		if(Acksum == 0)
		{
			MsgGroupID = ID_Warning;
			CAN_make_send_from_BP(SysLVolWarningMessage.Data, SysLVolWarningMessage.MessageNum, SysLVolWarningMessage.MessageID,SysLVolWarningMessage.DeviceID  );
		}
	}
	else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].LVolWarningAckFlag;
		}
		if(Acksum == 0)
		{
			if(StringAWEHappenedFlag.LVoltageWarning == 1)
			{
				MsgGroupID = ID_Warning;
				CAN_make_send_from_BP(SysLVolWarningMessage.Data, SysLVolWarningMessage.MessageNum, SysLVolWarningMessage.MessageID,SysLVolWarningMessage.DeviceID  );
			}
		}
		else
		{
			StringAWEHappenedFlag.LVoltageWarning = 0;
		}
		
	}
	if(CalculateCountOfState(BPLTempAlarmbit->BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].LTempAlarmAckFlag;
		}
		if(Acksum == 0)
		{
			MsgGroupID = ID_Alarm;
			CAN_make_send_from_BP(SysLTempAlarmMessage.Data, SysLTempAlarmMessage.MessageNum, SysLTempAlarmMessage.MessageID,SysLTempAlarmMessage.DeviceID  );		
		}
	}
	else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].LTempAlarmAckFlag;
		}
		if(Acksum == 0)
		{
			if(StringAWEHappenedFlag.LTempAlarm == 1)
			{
				MsgGroupID = ID_Alarm;
				CAN_make_send_from_BP(SysLTempAlarmMessage.Data, SysLTempAlarmMessage.MessageNum, SysLTempAlarmMessage.MessageID,SysLTempAlarmMessage.DeviceID  );		
			}
		}	
		else
		{
			StringAWEHappenedFlag.LTempAlarm = 0;
		}
		
	}
	if(CalculateCountOfState(BPLTempWarningbit->BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].LTempWarningAckFlag;
		}
		if(Acksum == 0)
		{
			MsgGroupID = ID_Warning;
			CAN_make_send_from_BP(SysLTempWarningMessage.Data, SysLTempWarningMessage.MessageNum, SysLTempWarningMessage.MessageID,SysLTempWarningMessage.DeviceID );
		}
	}	
	else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].LTempWarningAckFlag;
		}
		if(Acksum == 0)
		{
			if(StringAWEHappenedFlag.LTempWarning == 1)
			{
				MsgGroupID = ID_Warning;
				CAN_make_send_from_BP(SysLTempWarningMessage.Data, SysLTempWarningMessage.MessageNum, SysLTempWarningMessage.MessageID,SysLTempWarningMessage.DeviceID );
			}
		}	
		else
		{
			StringAWEHappenedFlag.LTempWarning = 0;
		}
		
	}
	
	if(CalculateCountOfState(BPbooterErrorbit.BPbyte) >= 1)
	{
		for(i = 0;i <BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].BPBootError;
		}
		if(Acksum == 0)
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
		}
	}
	else
	{
		for(i = 0;i <BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].BPBootError;
		}	
		if(Acksum == 0)
		{
			if(StringAWEHappenedFlag.BPBootError == 1)
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
			}
		}
		else
		{
			StringAWEHappenedFlag.BPBootError = 0;
		}
	}
	
	if(CalculateCountOfState(BPHAveDeltaVolErrorbit.BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].HighAveVoltaDeltaErrorAckFlag;
		}
		if(Acksum == 0)
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
	}
	else
	{
		for(i = 0;i <BP_NUMBER;i++)
		{
			Acksum += SysStateAckFlag[i].HighAveVoltaDeltaErrorAckFlag;
		}		
		if(Acksum == 0)
		{
			if(StringAWEHappenedFlag.HighAveVoltaDeltaError == 1)
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
		}
		else
		{
			StringAWEHappenedFlag.HighAveVoltaDeltaError = 0;
		}
	}	
}
#endif
void StringAWEIntervalProcess()
{
	if(StringAWETimer >= 30)          /*30s判断一次发生alarm未收到ACK，是否重新发送*/
	{
		StringAWETimer = 0;
		StringAWE();
		BPAwe();
	}
}

void BPAwe() /*BP AWE 30s 发送一次*/  
{
	int i = 0,Acksum = 0;
	MESSAGE msg;
	if(CalculateCountOfState(BPToStringHighCellDeltaErrorbit.BPbyte) >= 1)
	{
		for( i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].HighCellDeltaErrorFlag;
			
		}
		if(Acksum > 0)
		{
			MsgGroupID = ID_Error;
			{ 
				
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
				
			}
		}	
	}
	else
	{
		for( i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].HighCellDeltaErrorFlag;			
		}
		if(Acksum > 0)
		{
			if(BPAWEHappenedFlag.HCellVolDeltaError == 1)
			{ 
	 
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
				
			}
		}	
		else
		{
			BPAWEHappenedFlag.HCellVolDeltaError = 0;
		}
	}
	 
	
	if(CalculateCountOfState(BPToStringHVolAlarmbit.BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].HVolAlarmFlag;
		}
		if(Acksum > 0)
		{
			msg.Data[0] = Alarm_CellOverVoltage;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x01;     
			msg.Data[4] = BPToStringHVolAlarmbit.Byte1;
			msg.Data[5] = BPToStringHVolAlarmbit.Byte2;
			msg.Data[6] = BPToStringHVolAlarmbit.Byte3;
			msg.Data[7] = BPToStringHVolAlarmbit.Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Alarm;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Alarm;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID   );	
		}	
	}
	else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].HVolAlarmFlag;
		}
		if(Acksum > 0)
		{
			if(BPAWEHappenedFlag.HVoltageAlarm == 1)
			{
				msg.Data[0] = Alarm_CellOverVoltage;
				msg.Data[1] = Update;
				msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
				msg.Data[3] = 0x01;     
				msg.Data[4] = BPToStringHVolAlarmbit.Byte1;
				msg.Data[5] = BPToStringHVolAlarmbit.Byte2;
				msg.Data[6] = BPToStringHVolAlarmbit.Byte3;
				msg.Data[7] = BPToStringHVolAlarmbit.Byte4;
				msg.MessageNum = 8;
				msg.MessageID = MsgID_Alarm;
				msg.DeviceID = -1 ;
				MsgGroupID = ID_Alarm;			
				CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID   );	
			}
		}
		else
		{
			BPAWEHappenedFlag.HVoltageAlarm = 0;
		}
		
	}
	 
	
	
	if(CalculateCountOfState(BPToStringLVolAlarmbit.BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].LVolAlarmFlag;
		}
		if(Acksum > 0)
		{
			msg.Data[0] = Alarm_CellUnderVoltage;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringLVolAlarmbit.Byte1;
			msg.Data[5] = BPToStringLVolAlarmbit.Byte2;
			msg.Data[6] = BPToStringLVolAlarmbit.Byte3;
			msg.Data[7] = BPToStringLVolAlarmbit.Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Alarm;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Alarm;		
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID  );				
		}
	}
	else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].LVolAlarmFlag;
		}
		if(Acksum > 0)
		{
			if(BPAWEHappenedFlag.LVoltageAlarm == 1)
			{
				msg.Data[0] = Alarm_CellUnderVoltage;
				msg.Data[1] = Update;
				msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
				msg.Data[3] = 0x01;
				msg.Data[4] = BPToStringLVolAlarmbit.Byte1;
				msg.Data[5] = BPToStringLVolAlarmbit.Byte2;
				msg.Data[6] = BPToStringLVolAlarmbit.Byte3;
				msg.Data[7] = BPToStringLVolAlarmbit.Byte4;
				msg.MessageNum = 8;
				msg.MessageID = MsgID_Alarm;
				msg.DeviceID = -1 ;
				MsgGroupID = ID_Alarm;		
				CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID  );	
			}
		}	
		else
		{
			BPAWEHappenedFlag.LVoltageAlarm = 0;
		}
		
	}
	
	if(CalculateCountOfState(BPToStringHTempAlarmbit.BPbyte) >= 1)
	{
		for(i = 0; i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].HTempAlarmFlag;
		}
		if(Acksum > 0)
		{
			msg.Data[0] = Alarm_CellOverTemp;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringHTempAlarmbit.Byte1;
			msg.Data[5] = BPToStringHTempAlarmbit.Byte2;
			msg.Data[6] = BPToStringHTempAlarmbit.Byte3;
			msg.Data[7] = BPToStringHTempAlarmbit.Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Alarm;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Alarm;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID  );				
		}	
	}
	else
	{
		for(i = 0; i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].HTempAlarmFlag;
		}
		if(Acksum > 0)
		{
			if(BPAWEHappenedFlag.HTempAlarm == 1)
			{
				msg.Data[0] = Alarm_CellOverTemp;
				msg.Data[1] = Update;
				msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
				msg.Data[3] = 0x01;
				msg.Data[4] = BPToStringHTempAlarmbit.Byte1;
				msg.Data[5] = BPToStringHTempAlarmbit.Byte2;
				msg.Data[6] = BPToStringHTempAlarmbit.Byte3;
				msg.Data[7] = BPToStringHTempAlarmbit.Byte4;
				msg.MessageNum = 8;
				msg.MessageID = MsgID_Alarm;
				msg.DeviceID = -1 ;
				MsgGroupID = ID_Alarm;			
				CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID  );	
			}
		}
		else
		{
			BPAWEHappenedFlag.HTempAlarm = 0;
		}
	}
	
	if(CalculateCountOfState(BPToStringLTempAlarmbit.BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].LTempAlarmFlag;
		}
		if(Acksum > 0)
		{
			msg.Data[0] = Alarm_CellUnderTemp;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringLTempAlarmbit.Byte1;
			msg.Data[5] = BPToStringLTempAlarmbit.Byte2;
			msg.Data[6] = BPToStringLTempAlarmbit.Byte3;
			msg.Data[7] = BPToStringLTempAlarmbit.Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Alarm;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Alarm;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID  );			
		}
	}
	else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].LTempAlarmFlag;
		}
		if(Acksum > 0)
		{
			if(BPAWEHappenedFlag.LTempAlarm == 1)
			{
				msg.Data[0] = Alarm_CellUnderTemp;
				msg.Data[1] = Update;
				msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
				msg.Data[3] = 0x01;
				msg.Data[4] = BPToStringLTempAlarmbit.Byte1;
				msg.Data[5] = BPToStringLTempAlarmbit.Byte2;
				msg.Data[6] = BPToStringLTempAlarmbit.Byte3;
				msg.Data[7] = BPToStringLTempAlarmbit.Byte4;
				msg.MessageNum = 8;
				msg.MessageID = MsgID_Alarm;
				msg.DeviceID = -1 ;
				MsgGroupID = ID_Alarm;			
				CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID  );
			}
		}
		else
		{
			BPAWEHappenedFlag.LTempAlarm = 0;
		}
		
	}
	
	if(CalculateCountOfState(BPToStringHVolWarningbit.BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].HVolWarningFlag;
		}
		if(Acksum > 0)
		{
			msg.Data[0] = Warning_CellOverVoltage;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringHVolWarningbit.Byte1;
			msg.Data[5] = BPToStringHVolWarningbit.Byte2;
			msg.Data[6] = BPToStringHVolWarningbit.Byte3;
			msg.Data[7] = BPToStringHVolWarningbit.Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Warning;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Warning;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID );						
		}
	}
	else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].HVolWarningFlag;
		}
		if(Acksum > 0)
		{
			if(BPAWEHappenedFlag.HVoltageWarning == 1)
			{
				msg.Data[0] = Warning_CellOverVoltage;
				msg.Data[1] = Update;
				msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
				msg.Data[3] = 0x01;
				msg.Data[4] = BPToStringHVolWarningbit.Byte1;
				msg.Data[5] = BPToStringHVolWarningbit.Byte2;
				msg.Data[6] = BPToStringHVolWarningbit.Byte3;
				msg.Data[7] = BPToStringHVolWarningbit.Byte4;
				msg.MessageNum = 8;
				msg.MessageID = MsgID_Warning;
				msg.DeviceID = -1 ;
				MsgGroupID = ID_Warning;			
				CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID );	
			}
		}	
		else
		{
			BPAWEHappenedFlag.HVoltageWarning = 0;
		}
	}
	
	if(CalculateCountOfState(BPToStringLVolWarningbit.BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].LVolWarningFlag;
		}
		if(Acksum > 0)
		{
			msg.Data[0] = Warning_CellUnderVoltage;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringLVolWarningbit.Byte1;
			msg.Data[5] = BPToStringLVolWarningbit.Byte2;
			msg.Data[6] = BPToStringLVolWarningbit.Byte3;
			msg.Data[7] = BPToStringLVolWarningbit.Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Warning;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Warning;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID  );						
		}
	}
	else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].LVolWarningFlag;
		}
		if(Acksum > 0)
		{
			if(BPAWEHappenedFlag.LVoltageWarning == 1)
			{
				msg.Data[0] = Warning_CellUnderVoltage;
				msg.Data[1] = Update;
				msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
				msg.Data[3] = 0x01;
				msg.Data[4] = BPToStringLVolWarningbit.Byte1;
				msg.Data[5] = BPToStringLVolWarningbit.Byte2;
				msg.Data[6] = BPToStringLVolWarningbit.Byte3;
				msg.Data[7] = BPToStringLVolWarningbit.Byte4;
				msg.MessageNum = 8;
				msg.MessageID = MsgID_Warning;
				msg.DeviceID = -1 ;
				MsgGroupID = ID_Warning;			
				CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID  );		
			}
		}	
		else
		{
			BPAWEHappenedFlag.LVoltageWarning = 0;
		}
	}
	
	if(CalculateCountOfState(BPToStringHTempWarningbit.BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].HTempWarningFlag;
		}
		if(Acksum > 0)
		{
			msg.Data[0] = Warning_CellOverTemp;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringHTempWarningbit.Byte1;
			msg.Data[5] = BPToStringHTempWarningbit.Byte2;
			msg.Data[6] = BPToStringHTempWarningbit.Byte3;
			msg.Data[7] = BPToStringHTempWarningbit.Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Warning;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Warning;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID   );				
		}
		
	}
	else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].HTempWarningFlag;
		}
		if(Acksum > 0)
		{
			if(BPAWEHappenedFlag.HTempWarning == 1)
			{
				msg.Data[0] = Warning_CellOverTemp;
				msg.Data[1] = Update;
				msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
				msg.Data[3] = 0x01;
				msg.Data[4] = BPToStringHTempWarningbit.Byte1;
				msg.Data[5] = BPToStringHTempWarningbit.Byte2;
				msg.Data[6] = BPToStringHTempWarningbit.Byte3;
				msg.Data[7] = BPToStringHTempWarningbit.Byte4;
				msg.MessageNum = 8;
				msg.MessageID = MsgID_Warning;
				msg.DeviceID = -1 ;
				MsgGroupID = ID_Warning;			
				CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID   );		
			}
		}	
		else
		{
			BPAWEHappenedFlag.HTempWarning = 0;
		}
		
	}
	
	if(CalculateCountOfState(BPToStringLTempWarningbit.BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].LTempWarningFlag;
		}
		if(Acksum > 0)
		{
			msg.Data[0] = Warning_CellUnderTemp;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringLTempWarningbit.Byte1;
			msg.Data[5] = BPToStringLTempWarningbit.Byte2;
			msg.Data[6] = BPToStringLTempWarningbit.Byte3;
			msg.Data[7] = BPToStringLTempWarningbit.Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Warning;
			msg.DeviceID = -1;
			MsgGroupID = ID_Warning;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID );			
		}
	}
	else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].LTempWarningFlag;
		}
		if(Acksum > 0)
		{
			if(BPAWEHappenedFlag.LTempWarning == 1)
			{
				msg.Data[0] = Warning_CellUnderTemp;
				msg.Data[1] = Update;
				msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
				msg.Data[3] = 0x01;
				msg.Data[4] = BPToStringLTempWarningbit.Byte1;
				msg.Data[5] = BPToStringLTempWarningbit.Byte2;
				msg.Data[6] = BPToStringLTempWarningbit.Byte3;
				msg.Data[7] = BPToStringLTempWarningbit.Byte4;
				msg.MessageNum = 8;
				msg.MessageID = MsgID_Warning;
				msg.DeviceID = -1;
				MsgGroupID = ID_Warning;			
				CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID );	
			}
		}
		else
		{
			BPAWEHappenedFlag.LTempWarning = 0;
		}
	}
	
	if(CalculateCountOfState(BPToStringChargeRateAlarmbit.BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].ChargeStringCurrentAlarmFlag;
		}
		if(Acksum > 0)
		{
			msg.Data[0] = Alarm_HighChargeRate;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringChargeRateAlarmbit.Byte1;
			msg.Data[5] = BPToStringChargeRateAlarmbit.Byte2;
			msg.Data[6] = BPToStringChargeRateAlarmbit.Byte3;
			msg.Data[7] = BPToStringChargeRateAlarmbit.Byte4;
			
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Alarm;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Alarm;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);			
		}
	}
	else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].ChargeStringCurrentAlarmFlag;
		}
		if(Acksum > 0)
		{
			if(BPAWEHappenedFlag.HChargeRateAlarm == 1)
			{
				msg.Data[0] = Alarm_HighChargeRate;
				msg.Data[1] = Update;
				msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
				msg.Data[3] = 0x01;
				msg.Data[4] = BPToStringChargeRateAlarmbit.Byte1;
				msg.Data[5] = BPToStringChargeRateAlarmbit.Byte2;
				msg.Data[6] = BPToStringChargeRateAlarmbit.Byte3;
				msg.Data[7] = BPToStringChargeRateAlarmbit.Byte4;
				
				msg.MessageNum = 8;
				msg.MessageID = MsgID_Alarm;
				msg.DeviceID = -1 ;
				MsgGroupID = ID_Alarm;			
				CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);			
			}
		}
		else
		{
			BPAWEHappenedFlag.HChargeRateAlarm = 0;
		}
	}
	
	if(CalculateCountOfState(BPToStringDischargeRateAlarmbit.BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].DischargeStringCurrentAlarmFlag;
		}
		if(Acksum > 0)
		{
			msg.Data[0] = Alarm_HighDischargeRate;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringDischargeRateAlarmbit.Byte1;
			msg.Data[5] = BPToStringDischargeRateAlarmbit.Byte2;
			msg.Data[6] = BPToStringDischargeRateAlarmbit.Byte3;
			msg.Data[7] = BPToStringDischargeRateAlarmbit.Byte4;
			
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Alarm;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Alarm;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);			
		}		
	}
    else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].DischargeStringCurrentAlarmFlag;
		}
		if(Acksum > 0)
		{
			if(BPAWEHappenedFlag.HDischargeRateAlarm == 1)
			{
				msg.Data[0] = Alarm_HighDischargeRate;
				msg.Data[1] = Update;
				msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
				msg.Data[3] = 0x01;
				msg.Data[4] = BPToStringDischargeRateAlarmbit.Byte1;
				msg.Data[5] = BPToStringDischargeRateAlarmbit.Byte2;
				msg.Data[6] = BPToStringDischargeRateAlarmbit.Byte3;
				msg.Data[7] = BPToStringDischargeRateAlarmbit.Byte4;
				
				msg.MessageNum = 8;
				msg.MessageID = MsgID_Alarm;
				msg.DeviceID = -1 ;
				MsgGroupID = ID_Alarm;			
				CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);			
			}
		}	
		else
		{
			BPAWEHappenedFlag.HDischargeRateAlarm = 0;
		}
	}
	if(CalculateCountOfState(BPToStringChargeRateWarningbit.BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].ChargeStringCurrentWarningFlag;
		}
		if(Acksum > 0)
		{
			msg.Data[0] = Warning_HighChargeRate;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringChargeRateWarningbit.Byte1;
			msg.Data[5] = BPToStringChargeRateWarningbit.Byte2;
			msg.Data[6] = BPToStringChargeRateWarningbit.Byte3;
			msg.Data[7] = BPToStringChargeRateWarningbit.Byte4;
			
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Warning;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Warning;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);			
		}
	}
	else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].ChargeStringCurrentWarningFlag;
		}
		if(Acksum > 0)
		{
			if(BPAWEHappenedFlag.HChargeRateWarning == 1)
			{
				msg.Data[0] = Warning_HighChargeRate;
				msg.Data[1] = Update;
				msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
				msg.Data[3] = 0x01;
				msg.Data[4] = BPToStringChargeRateWarningbit.Byte1;
				msg.Data[5] = BPToStringChargeRateWarningbit.Byte2;
				msg.Data[6] = BPToStringChargeRateWarningbit.Byte3;
				msg.Data[7] = BPToStringChargeRateWarningbit.Byte4;
				
				msg.MessageNum = 8;
				msg.MessageID = MsgID_Warning;
				msg.DeviceID = -1 ;
				MsgGroupID = ID_Warning;			
				CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);			
			}
		}	
		else
		{
			BPAWEHappenedFlag.HChargeRateWarning = 0;
		}
	}
	
	if(CalculateCountOfState(BPToStringDischargeRateWarningbit.BPbyte) >= 1)
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].DischargeStringCurrentWarningFlag;
		}
		if(Acksum > 0)
		{
			msg.Data[0] = Warning_HighDischargeRate;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringDischargeRateWarningbit.Byte1;
			msg.Data[5] = BPToStringDischargeRateWarningbit.Byte2;
			msg.Data[6] = BPToStringDischargeRateWarningbit.Byte3;
			msg.Data[7] = BPToStringDischargeRateWarningbit.Byte4;
			
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Warning;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Warning;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);			
		}
	}	 
	else
	{
		for(i = 0;i < BP_NUMBER;i++)
		{
			Acksum += BpToStringFlag[i].DischargeStringCurrentWarningFlag;
		}
		if(Acksum > 0)
		{
			if(BPAWEHappenedFlag.HDischargeRateWarning == 1)
			{
				msg.Data[0] = Warning_HighDischargeRate;
				msg.Data[1] = Update;
				msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
				msg.Data[3] = 0x01;
				msg.Data[4] = BPToStringDischargeRateWarningbit.Byte1;
				msg.Data[5] = BPToStringDischargeRateWarningbit.Byte2;
				msg.Data[6] = BPToStringDischargeRateWarningbit.Byte3;
				msg.Data[7] = BPToStringDischargeRateWarningbit.Byte4;
				
				msg.MessageNum = 8;
				msg.MessageID = MsgID_Warning;
				msg.DeviceID = -1 ;
				MsgGroupID = ID_Warning;			
				CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);			
			}
		}
		else
		{
			BPAWEHappenedFlag.HDischargeRateWarning = 0;
		}		
	}
}

int EnterBPAWE[13] = {0};
int EnterStringAWE[13] = {0};
void BPAWEClear()
{
	int j = 0;
	MESSAGE msg;
	if(CalculateCountOfState(BPToStringHighCellDeltaErrorbit.BPbyte) >= 1)
	{
		EnterBPAWE[0] = 1;		 
	}
	else
	{
		if(EnterBPAWE[0] == 1)
		{
			
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
			EnterBPAWE[0] = 0;	
			for( j = 0;j < BP_NUMBER;j++)
			{
				BpToStringFlag[j].HighCellDeltaErrorFlag = 1;  
			}				
		}		
	}
	
	if(CalculateCountOfState(BPToStringHVolAlarmbit.BPbyte) >= 1)
	{
		EnterBPAWE[1] = 1;
	}
	else
	{
		if(EnterBPAWE[1] == 1)
		{
			msg.Data[0] = Alarm_CellOverVoltage;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x01;     
			msg.Data[4] = BPToStringHVolAlarmbit.Byte1;
			msg.Data[5] = BPToStringHVolAlarmbit.Byte2;
			msg.Data[6] = BPToStringHVolAlarmbit.Byte3;
			msg.Data[7] = BPToStringHVolAlarmbit.Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Alarm;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Alarm;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID   );	
			EnterBPAWE[1] = 0;
			for(j = 0 ;j < BP_NUMBER;j++)
			{
				BpToStringFlag[j].HVolAlarmFlag = 1;
			}			
		}
	}
	
	
	if(CalculateCountOfState(BPToStringLVolAlarmbit.BPbyte) >= 1)
	{
		EnterBPAWE[2] = 1;
	}
	else
	{
		if(EnterBPAWE[2] == 1)
		{
			msg.Data[0] = Alarm_CellUnderVoltage;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringLVolAlarmbit.Byte1;
			msg.Data[5] = BPToStringLVolAlarmbit.Byte2;
			msg.Data[6] = BPToStringLVolAlarmbit.Byte3;
			msg.Data[7] = BPToStringLVolAlarmbit.Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Alarm;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Alarm;		
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID  );			
			EnterBPAWE[2] = 0;
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				BpToStringFlag[j].LVolAlarmFlag = 1;
			}
		}		
	}
	if(CalculateCountOfState(BPToStringHTempAlarmbit.BPbyte) >= 1)
	{
		EnterBPAWE[3] = 1;		
	}
	else
	{
		if(EnterBPAWE[3] == 1)
		{
			msg.Data[0] = Alarm_CellOverTemp;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringHTempAlarmbit.Byte1;
			msg.Data[5] = BPToStringHTempAlarmbit.Byte2;
			msg.Data[6] = BPToStringHTempAlarmbit.Byte3;
			msg.Data[7] = BPToStringHTempAlarmbit.Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Alarm;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Alarm;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID  );			
			
			EnterBPAWE[3] = 0;	
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				BpToStringFlag[j].HTempAlarmFlag = 1;
			}				
		}
		
	}
	
	if(CalculateCountOfState(BPToStringLTempAlarmbit.BPbyte) >= 1)
	{
		EnterBPAWE[4] = 1;
	}
	else
	{
		if(EnterBPAWE[4] == 1)
		{
			msg.Data[0] = Alarm_CellUnderTemp;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringLTempAlarmbit.Byte1;
			msg.Data[5] = BPToStringLTempAlarmbit.Byte2;
			msg.Data[6] = BPToStringLTempAlarmbit.Byte3;
			msg.Data[7] = BPToStringLTempAlarmbit.Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Alarm;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Alarm;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID  );				
			EnterBPAWE[4] = 0;
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				BpToStringFlag[j].LTempAlarmFlag = 1;
			}	
		}
		
	}
	
	if(CalculateCountOfState(BPToStringHVolWarningbit.BPbyte) >= 1)
	{
		EnterBPAWE[5] = 1;
	}
	else
	{
		if(EnterBPAWE[5] == 1)
		{
			msg.Data[0] = Warning_CellOverVoltage;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringHVolWarningbit.Byte1;
			msg.Data[5] = BPToStringHVolWarningbit.Byte2;
			msg.Data[6] = BPToStringHVolWarningbit.Byte3;
			msg.Data[7] = BPToStringHVolWarningbit.Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Warning;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Warning;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID );			
			EnterBPAWE[5] = 0;
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				BpToStringFlag[j].HVolWarningFlag = 1;
			}			
		}
	}
	if(CalculateCountOfState(BPToStringLVolWarningbit.BPbyte) >= 1)
	{
		EnterBPAWE[6] = 1;
	}
	else
	{
		if(EnterBPAWE[6] == 1)
		{
			msg.Data[0] = Warning_CellUnderVoltage;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringLVolWarningbit.Byte1;
			msg.Data[5] = BPToStringLVolWarningbit.Byte2;
			msg.Data[6] = BPToStringLVolWarningbit.Byte3;
			msg.Data[7] = BPToStringLVolWarningbit.Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Warning;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Warning;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);			
			EnterBPAWE[6] = 0;
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				BpToStringFlag[j].LVolWarningFlag= 1;
			}
		}
		
	}
	if(CalculateCountOfState(BPToStringHTempWarningbit.BPbyte) >= 1)
	{
		EnterBPAWE[7] = 1; 
	}
	else
	{
		if(EnterBPAWE[7] == 1)
		{
			msg.Data[0] = Warning_CellOverTemp;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringHTempWarningbit.Byte1;
			msg.Data[5] = BPToStringHTempWarningbit.Byte2;
			msg.Data[6] = BPToStringHTempWarningbit.Byte3;
			msg.Data[7] = BPToStringHTempWarningbit.Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Warning;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Warning;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID   );				
			EnterBPAWE[7] = 0;
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				BpToStringFlag[j].HTempWarningFlag = 1;
			}
		}
		
	}
	if(CalculateCountOfState(BPToStringLTempWarningbit.BPbyte) >= 1)
	{
		EnterBPAWE[8] = 1;
	}
	else
	{
		if(EnterBPAWE[8] == 1)
		{
			msg.Data[0] = Warning_CellUnderTemp;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringLTempWarningbit.Byte1;
			msg.Data[5] = BPToStringLTempWarningbit.Byte2;
			msg.Data[6] = BPToStringLTempWarningbit.Byte3;
			msg.Data[7] = BPToStringLTempWarningbit.Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Warning;
			msg.DeviceID = -1;
			MsgGroupID = ID_Warning;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID );						
			EnterBPAWE[8] = 0;
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				BpToStringFlag[j].LTempWarningFlag = 1;
			}			
		}
	}
	if(CalculateCountOfState(BPToStringChargeRateAlarmbit.BPbyte) >= 1)
	{
		EnterBPAWE[9] = 1;
		 
	}
	else
	{
		if(EnterBPAWE[9] == 1)
		{
			msg.Data[0] = Alarm_HighChargeRate;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringChargeRateAlarmbit.Byte1;
			msg.Data[5] = BPToStringChargeRateAlarmbit.Byte2;
			msg.Data[6] = BPToStringChargeRateAlarmbit.Byte3;
			msg.Data[7] = BPToStringChargeRateAlarmbit.Byte4;
			
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Alarm;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Alarm;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);				
			EnterBPAWE[9] = 0;
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				BpToStringFlag[j].ChargeStringCurrentAlarmFlag = 1;
			}
		}
	}
	
	if(CalculateCountOfState(BPToStringDischargeRateAlarmbit.BPbyte) >= 1)
	{
		EnterBPAWE[10] = 1;
	}	
	else
	{
		if(EnterBPAWE[10] == 1)
		{
			msg.Data[0] = Alarm_HighDischargeRate;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringDischargeRateAlarmbit.Byte1;
			msg.Data[5] = BPToStringDischargeRateAlarmbit.Byte2;
			msg.Data[6] = BPToStringDischargeRateAlarmbit.Byte3;
			msg.Data[7] = BPToStringDischargeRateAlarmbit.Byte4;
			
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Alarm;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Alarm;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);						
			EnterBPAWE[10] = 0;
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				BpToStringFlag[j].DischargeStringCurrentAlarmFlag = 1;
			}			
		}
		
	}
	if(CalculateCountOfState(BPToStringChargeRateWarningbit.BPbyte) >= 1)
	{
		EnterBPAWE[11] = 1;
	}
	else
	{
		if(EnterBPAWE[11] == 1)	
		{
			msg.Data[0] = Warning_HighChargeRate;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringChargeRateWarningbit.Byte1;
			msg.Data[5] = BPToStringChargeRateWarningbit.Byte2;
			msg.Data[6] = BPToStringChargeRateWarningbit.Byte3;
			msg.Data[7] = BPToStringChargeRateWarningbit.Byte4;
			
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Warning;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Warning;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);						
			EnterBPAWE[11] = 0;
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				BpToStringFlag[j].ChargeStringCurrentWarningFlag = 1;
			}	
		}
	}
	if(CalculateCountOfState(BPToStringDischargeRateWarningbit.BPbyte) >= 1)
	{
		EnterBPAWE[12] = 1;
	}		
	else
	{
		if(EnterBPAWE[12] == 1)
		{
			msg.Data[0] = Warning_HighDischargeRate;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);
			msg.Data[3] = 0x01;
			msg.Data[4] = BPToStringDischargeRateWarningbit.Byte1;
			msg.Data[5] = BPToStringDischargeRateWarningbit.Byte2;
			msg.Data[6] = BPToStringDischargeRateWarningbit.Byte3;
			msg.Data[7] = BPToStringDischargeRateWarningbit.Byte4;
			
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Warning;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Warning;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);						
			EnterBPAWE[12] = 0;
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				BpToStringFlag[j].DischargeStringCurrentWarningFlag = 1;
			}
		}
	}
	
	
}

void StringAWEClear()
{	
	int j = 0,Acksum = 0;
	if(CalculateCountOfState(BPHVolAlarmbit->BPbyte) >= 1)
	{
		EnterStringAWE[1] = 1;
	}
	else
	{
		if(EnterStringAWE[1] == 1)
		{
			MESSAGE msg;
			msg.Data[0] = Alarm_CellOverVoltage;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x00;     
			msg.Data[4] = BPHVolAlarmbit->Byte1;
			msg.Data[5] = BPHVolAlarmbit->Byte2;
			msg.Data[6] = BPHVolAlarmbit->Byte3;
			msg.Data[7] = BPHVolAlarmbit->Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Alarm;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Alarm;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID   );	
			EnterStringAWE[1] = 0;
			
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				SysStateAckFlag[j].HVolAlarmAckFlag = 0;
			}				
		}
	}
	
	
	if(CalculateCountOfState(BPLVolAlarmbit->BPbyte) >= 1)
	{
		EnterStringAWE[2] = 1;
	}
	else
	{
		if(EnterStringAWE[2] == 1)
		{
			MESSAGE msg;
			msg.Data[0] = Alarm_CellUnderVoltage;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x00;
			msg.Data[4] = BPLVolAlarmbit->Byte1;
			msg.Data[5] = BPLVolAlarmbit->Byte2;
			msg.Data[6] = BPLVolAlarmbit->Byte3;
			msg.Data[7] = BPLVolAlarmbit->Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Alarm;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Alarm;		
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID  );			
			EnterStringAWE[2] = 0;
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				SysStateAckFlag[j].LVolAlarmAckFlag = 0;
			}			
		}		
	}
	if(CalculateCountOfState(BPHTempAlarmbit->BPbyte) >= 1)
	{
		EnterStringAWE[3] = 1;		
	}
	else
	{
		if(EnterStringAWE[3] == 1)
		{
			MESSAGE msg;
			msg.Data[0] = Alarm_CellOverTemp;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x00;
			msg.Data[4] = BPHTempAlarmbit->Byte1;
			msg.Data[5] = BPHTempAlarmbit->Byte2;
			msg.Data[6] = BPHTempAlarmbit->Byte3;
			msg.Data[7] = BPHTempAlarmbit->Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Alarm;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Alarm;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID  );			
			
			EnterStringAWE[3] = 0;		
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				SysStateAckFlag[j].HTempAlarmAckFlag = 0;
			}			
		}
		
	}
	
	if(CalculateCountOfState(BPLTempAlarmbit->BPbyte) >= 1)
	{
		EnterStringAWE[4] = 1;
	}
	else
	{
		if(EnterStringAWE[4] == 1)
		{
			MESSAGE msg;
			msg.Data[0] = Alarm_CellUnderTemp;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x00;
			msg.Data[4] = BPLTempAlarmbit->Byte1;
			msg.Data[5] = BPLTempAlarmbit->Byte2;
			msg.Data[6] = BPLTempAlarmbit->Byte3;
			msg.Data[7] = BPLTempAlarmbit->Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Alarm;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Alarm;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID  );				
			EnterStringAWE[4] = 0;
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				SysStateAckFlag[j].LTempAlarmAckFlag = 0;
			}			
		}
		
	}
	
	if(CalculateCountOfState(BPHVolWarningbit->BPbyte) >= 1)
	{
		EnterStringAWE[5] = 1;
	}
	else
	{
		if(EnterStringAWE[5] == 1)
		{
			MESSAGE msg;
			msg.Data[0] = Warning_CellOverVoltage;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x00;
			msg.Data[4] = BPHVolWarningbit->Byte1;
			msg.Data[5] = BPHVolWarningbit->Byte2;
			msg.Data[6] = BPHVolWarningbit->Byte3;
			msg.Data[7] = BPHVolWarningbit->Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Warning;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Warning;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID );			
			EnterStringAWE[5] = 0;
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				SysStateAckFlag[j].HVolWarningAckFlag = 0;
			}
		}
	}
	if(CalculateCountOfState(BPLVolWarningbit->BPbyte) >= 1)
	{
		EnterStringAWE[6] = 1;
	}
	else
	{
		if(EnterStringAWE[6] == 1)
		{
			MESSAGE msg;
			msg.Data[0] = Warning_CellUnderVoltage;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x00;
			msg.Data[4] = BPLVolWarningbit->Byte1;
			msg.Data[5] = BPLVolWarningbit->Byte2;
			msg.Data[6] = BPLVolWarningbit->Byte3;
			msg.Data[7] = BPLVolWarningbit->Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Warning;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Warning;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID);			
			EnterStringAWE[6] = 0;
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				SysStateAckFlag[j].LVolWarningAckFlag = 0;
			}			
		}
		
	}
	if(CalculateCountOfState(BPHTempWarningbit->BPbyte) >= 1)
	{
		EnterStringAWE[7] = 1; 
	}
	else
	{
		if(EnterStringAWE[7] == 1)
		{
			MESSAGE msg;
			msg.Data[0] = Warning_CellOverTemp;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x00;
			msg.Data[4] = BPHTempWarningbit->Byte1;
			msg.Data[5] = BPHTempWarningbit->Byte2;
			msg.Data[6] = BPHTempWarningbit->Byte3;
			msg.Data[7] = BPHTempWarningbit->Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Warning;
			msg.DeviceID = -1 ;
			MsgGroupID = ID_Warning;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID   );				
			EnterStringAWE[7] = 0;
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				SysStateAckFlag[j].HTempWarningAckFlag = 0;
			}			
		}
		
	}
	if(CalculateCountOfState(BPLTempWarningbit->BPbyte) >= 1)
	{
		EnterStringAWE[8] = 1;
	}
	else
	{
		if(EnterStringAWE[8] == 1)
		{
			MESSAGE msg;
			msg.Data[0] = Warning_CellUnderTemp;
			msg.Data[1] = Update;
			msg.Data[2] = ArrayStringID(mStringData.ArrayID,Self_ID);                    /*ArrayID+StringID*/
			msg.Data[3] = 0x00;
			msg.Data[4] = BPLTempWarningbit->Byte1;
			msg.Data[5] = BPLTempWarningbit->Byte2;
			msg.Data[6] = BPLTempWarningbit->Byte3;
			msg.Data[7] = BPLTempWarningbit->Byte4;
			msg.MessageNum = 8;
			msg.MessageID = MsgID_Warning;
			msg.DeviceID = -1;
			MsgGroupID = ID_Warning;			
			CAN_make_send_from_BP(msg.Data, msg.MessageNum, msg.MessageID,msg.DeviceID );						
			EnterStringAWE[8] = 0;
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				SysStateAckFlag[j].LTempWarningAckFlag = 0;
			}			
		}
	}
	
	if(CalculateCountOfState(BPbooterErrorbit.BPbyte) >= 1)
	{
		EnterStringAWE[9] = 1;
	}
	else
	{
		if(EnterStringAWE[9] == 1)
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
			EnterStringAWE[9] = 0;	
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				SysStateAckFlag[j].BPBootError = 0;
			}			 
		}
	}

/*add by jason,2016年11月2日11:23:57 Error_HighAveVoltaDelta*/	
	if(CalculateCountOfState(BPHAveDeltaVolErrorbit.BPbyte) >= 1)
	{
		EnterStringAWE[10] = 1;
	}
	else
	{
		if(EnterStringAWE[10] == 1)
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
			EnterStringAWE[10] = 0;	
			for(j = 0 ;j <BP_NUMBER;j++)
			{
				SysStateAckFlag[j].HighAveVoltaDeltaErrorAckFlag = 0;
			}			 
		}
	}	
}
 
/*if the alarm warning or error count changed，if not the first AWE then send it immediately，otherwise sent the message after 5s*/
int Count_AutoUpdateStringAWE[10];
int Count_AutoUpdateBPAWE[13];
unsigned char Flag_AutoUpdateStringAWE[10];
unsigned char Flag_AutpUpdateBPAWE[13];


void AutoUpdateAWEMsg()
{
/*string controller AWE*/	
	if(CalculateCountOfState(BPHVolAlarmbit->BPbyte) >= 1) /*High Vol alarm*/
	{
		if(CalculateCountOfState(BPHVolAlarmbit->BPbyte) != Count.OldHVoltAlarm)
		{
			Flag_AutoUpdateStringAWE[0] = 1;
		}

	}
	else
	{
		Flag_AutoUpdateStringAWE[0] = 0;
	}
	if(CalculateCountOfState(BPHVolWarningbit->BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPHVolWarningbit->BPbyte)  != Count.OldHVoltWarning)
		{
			Flag_AutoUpdateStringAWE[1] = 1;
		}

	}
	else
	{
		Flag_AutoUpdateStringAWE[1] = 0;
	}
	
	if(CalculateCountOfState(BPHTempAlarmbit->BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPHTempAlarmbit->BPbyte) != Count.OldHTempAlarm)
		{
			Flag_AutoUpdateStringAWE[2] = 1;
		}		
	}
	else
	{
		Flag_AutoUpdateStringAWE[2] = 0;
	}
	
	if(CalculateCountOfState(BPHTempWarningbit->BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPHTempWarningbit->BPbyte) != Count.OldHTempWarning)
		{
			Flag_AutoUpdateStringAWE[3] = 1;
		}
	}
	else
	{
		Flag_AutoUpdateStringAWE[3] = 0;
	}
	
	
	if(CalculateCountOfState(BPLVolAlarmbit->BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPLVolAlarmbit->BPbyte) != Count.OldLVoltAlarm)
		{
			Flag_AutoUpdateStringAWE[4] = 1;
		}	
	}
	else
	{
		Flag_AutoUpdateStringAWE[4] = 0;
	}
	
	if(CalculateCountOfState(BPLVolWarningbit->BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPLVolWarningbit->BPbyte) != Count.OldLVoltWarning)
		{
			Flag_AutoUpdateStringAWE[5] = 1;
		}	
	}
	else
	{
		Flag_AutoUpdateStringAWE[5] = 0;
	}
	
	if(CalculateCountOfState(BPLTempAlarmbit->BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPLTempAlarmbit->BPbyte) != Count.OldLTempAlarm)
		{
			Flag_AutoUpdateStringAWE[6] = 1;
		}
	}
	else
	{
		Flag_AutoUpdateStringAWE[6] = 0;
	}
	
	if(CalculateCountOfState(BPLTempWarningbit->BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPLTempWarningbit->BPbyte) != Count.OldLTempWarning)
		{
			Flag_AutoUpdateStringAWE[7] = 1;
		}
	}
	else
	{
		Flag_AutoUpdateStringAWE[7] = 0;
	}
	

/*New function of BP high average vol delta error,add by jason ，2016-10-20 */	
	if(CalculateCountOfState(BPHAveDeltaVolErrorbit.BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPHAveDeltaVolErrorbit.BPbyte) != Count.OldHAveDeltaVolError)
		{
			Flag_AutoUpdateStringAWE[8] = 1;
			Count.OldHAveDeltaVolError = CalculateCountOfState(BPHAveDeltaVolErrorbit.BPbyte);
		}
	}
	else
	{
		Flag_AutoUpdateStringAWE[8] = 0;
	}
	 
	if(CalculateCountOfState(BPbooterErrorbit.BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPbooterErrorbit.BPbyte) != Count.OldBpBootError)
		{
			Flag_AutoUpdateStringAWE[9] = 1;
			Count.OldBpBootError = CalculateCountOfState(BPbooterErrorbit.BPbyte);
		}
		
	}
	else
	{
		Flag_AutoUpdateStringAWE[9] = 0;
	}
	
	
	/*bp controller AWE*/
	if(CalculateCountOfState(BPToStringHighCellDeltaErrorbit.BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPToStringHighCellDeltaErrorbit.BPbyte) != Count.OldBPHighCellVolDeltaError)
		{
			Flag_AutpUpdateBPAWE[0] = 1;
		}
	}
	else
	{
		Flag_AutpUpdateBPAWE[0] = 0;
	}
	
	if(CalculateCountOfState(BPToStringHVolAlarmbit.BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPToStringHVolAlarmbit.BPbyte) != Count.OldBPHVoltAlarm)
		{
			Flag_AutpUpdateBPAWE[1] = 1;
		}
	}
	else
	{
		Flag_AutpUpdateBPAWE[1] = 0;
	}
	
	if(CalculateCountOfState(BPToStringLVolAlarmbit.BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPToStringLVolAlarmbit.BPbyte) != Count.OldBPLVoltAlarm)
		{
			Flag_AutpUpdateBPAWE[2] = 1;
		}		
	}
	else
	{
		Flag_AutpUpdateBPAWE[2] = 0;
	}
	
	if(CalculateCountOfState(BPToStringHTempAlarmbit.BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPToStringHTempAlarmbit.BPbyte) != Count.OldBPHTempAlarm)
		{
			Flag_AutpUpdateBPAWE[3] = 1;
		}
	}
	else
	{
		Flag_AutpUpdateBPAWE[3] = 0;
	}
	
	if(CalculateCountOfState(BPToStringLTempAlarmbit.BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPToStringLTempAlarmbit.BPbyte) != Count.OldBPLTempAlarm)
		{
			Flag_AutpUpdateBPAWE[4] = 1;
		}
	}
	else
	{
		Flag_AutpUpdateBPAWE[4] = 0;
	}
	
	if(CalculateCountOfState(BPToStringHVolWarningbit.BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPToStringHVolWarningbit.BPbyte) != Count.OldBPHVoltWarning)
		{
			Flag_AutpUpdateBPAWE[5] = 1;
		}
	}
	else
	{
		Flag_AutpUpdateBPAWE[5] = 0;
	}
	
	if(CalculateCountOfState(BPToStringLVolWarningbit.BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPToStringLVolWarningbit.BPbyte) != Count.OldBPLVoltWarning)
		{
			Flag_AutpUpdateBPAWE[6] = 1;			
		}
	}
	else
	{
		Flag_AutpUpdateBPAWE[6] = 0;
	}
	
	if(CalculateCountOfState(BPToStringHTempWarningbit.BPbyte) > 1)
	{
		if(CalculateCountOfState(BPToStringHTempWarningbit.BPbyte) != Count.OldBPHTempWarning)
		{
			Flag_AutpUpdateBPAWE[7] = 1;
		}
	}
	else
	{
		Flag_AutpUpdateBPAWE[7] = 0;
	}
	
	if(CalculateCountOfState(BPToStringLTempWarningbit.BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPToStringLTempWarningbit.BPbyte) != Count.OldBPLTempWarning)
		{
			Flag_AutpUpdateBPAWE[8] = 1;
		}
	}
	else
	{
		Flag_AutpUpdateBPAWE[8] = 0;
	}
	
	if(CalculateCountOfState(BPToStringChargeRateAlarmbit.BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPToStringChargeRateAlarmbit.BPbyte) != Count.OldBPHChCurrentAlarm)
		{
			Flag_AutpUpdateBPAWE[9] = 1;			
		}	
	}
	else
	{
		Flag_AutpUpdateBPAWE[9] = 0;
	}
	
	if(CalculateCountOfState(BPToStringDischargeRateAlarmbit.BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPToStringDischargeRateAlarmbit.BPbyte) != Count.OldBPHDisCurrentAlarm)
		{
			Flag_AutpUpdateBPAWE[10] = 1;
		}
	}
	else
	{
		Flag_AutpUpdateBPAWE[10] = 0;
	}
	
	if(CalculateCountOfState(BPToStringChargeRateWarningbit.BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPToStringChargeRateWarningbit.BPbyte) != Count.OldBPHChCurrentWarning)
		{
			Flag_AutpUpdateBPAWE[11] = 1;
		}
	}
	else
	{
		Flag_AutpUpdateBPAWE[11] = 0;
	}
	
	if(CalculateCountOfState(BPToStringDischargeRateWarningbit.BPbyte) >= 1)
	{
		if(CalculateCountOfState(BPToStringDischargeRateWarningbit.BPbyte) != Count.OldBPHDisCurrentWarning)
		{
			Flag_AutpUpdateBPAWE[12] = 1;
		}
	}
	else
	{
		Flag_AutpUpdateBPAWE[12] = 0;
	}
	
	UpdateAWE();
}

