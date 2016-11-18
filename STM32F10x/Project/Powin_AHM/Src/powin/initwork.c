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


int CalculateCountOfState(unsigned int n)
{ 
  int count=0; 
  while(n>0)
  { 
    n &= (n-1); 
    count++; 
  } 
  return count; 
}
void InitCmpValueofAlarmWarning()
{
	int i;
	for(i = 0;i < BP_NUMBER;i++)
	{
		CmpValue[i].HVoltageAlarm = SetClrValue.SetCellOverVoltageAlarm;
		CmpValue[i].HTempAlarm = SetClrValue.SetCellOverTempAlarm;
		CmpValue[i].LVoltageAlarm = SetClrValue.SetCellUnderVoltageAlarm;
		CmpValue[i].LTempAlarm = SetClrValue.SetCellUnderTempAlarm;

		CmpValue[i].HVoltageWarning =  SetClrValue.SetCellOverVoltageWarning;
		CmpValue[i].HTempWarning = SetClrValue.SetCellOverTempWarning;
		CmpValue[i].LVoltageWarning = SetClrValue.SetCellUnderVoltageWarning;
		CmpValue[i].LTempWarning = SetClrValue.SetCellUnderTempWarning;
		
		CmpValue[i].HCellDeltaTempAlarm = SetClrValue.SetHighCellTempDeltaAlarm;
		CmpValue[i].HCellDeltaTempWarning = SetClrValue.SetHighCellTempDeltaWarning;
		
		CmpValue[i].HCellTempRiseAlarm = SetClrValue.SetHighCellTempRiseAlarm;
		CmpValue[i].HCellTempRiseWarning = SetClrValue.SetHighCellTempRiseWarning;		
		
		CmpValue_ChargeStringCurrentAlarm = SetClrValue.SetHighChargeRateAlarm;
		CmpValue_ChargeStringCurrentWarning = SetClrValue.SetHighChargeRateWarning;
		
		CmpValue_DischargeStringCurrentAlarm = SetClrValue.SetHighDisChargeRateAlarm;
		CmpValue_DischargeStringCurrentWarning = SetClrValue.SetHighDisChargeRateWarning;		
	}	
}
void Value_Init()
{
	int i;
	u32 *pData;
	SetClrValue.SetCellOverVoltageAlarm = EnterCellOverVoltageAlarm;
	SetClrValue.SetCellOverTempAlarm = EnterCellOverTempAlarm;
	SetClrValue.SetCellUnderVoltageAlarm = EnterCellUnderVoltageAlarm;
	SetClrValue.SetCellUnderTempAlarm = EnterCellUnderTempAlarm;
	SetClrValue.SetCellOverVoltageWarning = EnterCellOverVoltageWarning;
	SetClrValue.SetCellOverTempWarning = EnterCellOverTempWarning;
	SetClrValue.SetCellUnderVoltageWarning = EnterCellUnderVoltageWarning;
	SetClrValue.SetCellUnderTempWarning = EnterCellUnderTempWarning;
	
	SetClrValue.SetHighCellTempDeltaAlarm = EnterHighCellTempDeltaAlarm;
	SetClrValue.SetHighCellTempDeltaWarning = EnterHighCellTempDeltaWarning;
	SetClrValue.SetHighCellTempRiseAlarm = EnterHighCellTempRiseAlarm;
	SetClrValue.SetHighCellTempRiseWarning = EnterHighCellTempRiseWarning;
	
	SetClrValue.SetHighChargeRateAlarm = EnterHighChargeRateAlarm;
	SetClrValue.SetHighChargeRateWarning = EnterHighChargeRateWarning;
	SetClrValue.SetHighDisChargeRateAlarm = EnterHighDisChargeRateAlarm;
	SetClrValue.SetHighDisChargeRateWarning = EnterHighDisChargeRateWarning;
	
	SetClrValue.ClrCellOverVoltageAlarm = ExitCellOverVoltageAlarm;
	SetClrValue.ClrCellOverTempAlarm = ExitCellOverTempAlarm;
	SetClrValue.ClrCellUnderVoltageAlarm = ExitCellUnderVoltageAlarm;
	SetClrValue.ClrCellUnderTempAlarm = ExitCellUnderTempAlarm;
	SetClrValue.ClrCellOverVoltageWarning = ExitCellOverVoltageWarning;
	SetClrValue.ClrCellOverTempWarning = ExitCellOverTempWarning;
	SetClrValue.ClrCellUnderVoltageWarning = ExitCellUnderVoltageWarning;
	SetClrValue.ClrCellUnderTempWarning =ExitCellUnderTempWarning;
	             
	SetClrValue.ClrHighCellTempDeltaAlarm = ExitHighCellTempDeltaAlarm;
	SetClrValue.ClrHighCellTempDeltaWarning = ExitHighCellTempDeltaTempWarning;
	SetClrValue.ClrHighCellTempRiseAlarm = ExitHighCellTempRiseAlarm;
	SetClrValue.ClrHighCellTempRiseWarning = ExitHighCellTempRiseWarning;
	             
	SetClrValue.ClrHighChargeRateAlarm = ExitHighChargeRateAlarm;
	SetClrValue.ClrHighChargeRateWarning = ExitHighChargeRateWarning;
	SetClrValue.ClrHighDisChargeRateAlarm = ExitHighDisChargeRateAlarm;
	SetClrValue.ClrHighDisChargeRateWarning = ExitHighDisChargeRateWarning;		
	InitCmpValueofAlarmWarning();
	//System space initialized to 0
	pData = (u32 *)(&SysStateAckFlag);
	i = sizeof(SysStateAckFlag)/4;
	while(i--)
	*(pData + i)= 0; 
	
	pData = (u32 *)(&SysStateFlag);
	i = sizeof(SysStateFlag)/4;
	while(i--)
	*(pData + i)= 0; 	
	
	
	
	BPHVolAlarmbit = (_BP* )&SysHVolAlarmMessage.Data[4];
	BPHVolWarningbit = (_BP* )&SysHVolWarningMessage.Data[4];
	BPHTempAlarmbit = (_BP* )&SysHTempAlarmMessage.Data[4];
	BPHTempWarningbit = (_BP* )&SysHTempWarningMessage.Data[4];
	BPLVolAlarmbit = (_BP* )&SysLVolAlarmMessage.Data[4];
	BPLVolWarningbit = (_BP* )&SysLVolWarningMessage.Data[4];
	BPLTempAlarmbit = (_BP* )&SysLTempAlarmMessage.Data[4];
	BPLTempWarningbit = (_BP* )&SysLTempWarningMessage.Data[4];	

	BPLoseCommunicationErrorbit = (_BP*)&SysBPLoseOfCommunicationErrorMessage.Data[4];
	
 
	
	                            
/*保证 不会 在没有读到的情况下报警*/	
	for(i = 0 ;i < BP_NUMBER;i++)
	{
		mStringData.bp[i].highCellVolt = 0;
		mStringData.bp[i].lowCellVolt =  10000;
		mStringData.bp[i].highCellTemp = -2500; 
		mStringData.bp[i].lowCellTemp = 2500;
		mStringData.bp[i].maxCellTempRise = -2500;
	}
	
	mStringData.ArrayID = 0x01;                                           /*2015年11月10日11:22:17 by Jason，ArrayID 默认为1，等待新的更改方案*/
	

}
void MemCopy(u8 *Dest,u8 *Src,u32 Num)
{
	u32 Count=0;
	while(Count < Num)
	{
		*(Dest+Count)=*(Src+Count);
		Count++;
	}
}
/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length.
* Output         : None
* Return         : None
*******************************************************************************/
void CanDelay(uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}
