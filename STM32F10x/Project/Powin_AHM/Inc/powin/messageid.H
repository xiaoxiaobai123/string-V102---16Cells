#ifndef MESSAGEID_H_
#define MESSAGEID_H_

#include "stm32f10x.h"
#include "BESS_3-0_AHM_stringdata_V_1-0.h"
#include "BESS_3-0_AHM_can_V_1-0.h"


#define HighAveDeltaVolErrorThresholdValue  50
#define RelayQueryMsgID  0x10

// 2015年10月11日17:36:30 
#define TotalAH    7000       /*60AH*/
#define TotalKWH   60000      /*50 KWH*/
//#define DefaultSoc    

#define BPLossOfcommunicationTime   40        //3 * 40 s


#define _Debug 0


#define ControlBalancing 0x57

/*2016年6月7日8:31:42 maintenance mode message  id by jason */
#define AcktoMaintenanceMode    0x13
#define EnterIntoMaintenanceMode 0x22


/*2016年6月1日8:33:40 @byjason*/
#define GiveContactorClosedPermission 0xAA
#define TakeBackConactorClosePermission 0xCC



#define CellMsg(Cell,CellID) 	(CellID << 8) | Cell


#define IDLE_BalancingVoltageThreshold 3200

#define MsgID_QueryAllAWE 0x11       /*0x06 change to 0x11,@2016年4月22日11:06:54 byjason*/
 // this file written by jason 2015年8月31日16:45:19
#define MsgID_Alarm       0x00            
#define MsgID_Warning     0x01		   
#define MsgID_Error       0x02           

#define MsgID_PCSState    0x07

#define MsgID_ContactorClosedPermission 0x08
#define DCBUS_Voltage     0x09

#define Normal_State      0x00
#define Warning_State     0x0F
#define Error_State       0xF0
#define Alarm_State       0xFF
 
#define Idle_Mode         0x00
#define Charge_Mode       0x0F
#define DisCharge_Mode    0xF0

#define Alarm_CellOverVoltage   0x00
#define Alarm_CellUnderVoltage  0x01
#define Alarm_CellOverTemp      0x02
#define Alarm_CellUnderTemp     0x03
#define Alarm_HighCellTempDelta 0x04
#define Alarm_HighCellTempRise  0x05
#define Alarm_HighChargeRate    0x06
#define Alarm_HighDischargeRate 0x07
#define Alarm_HighGroundLeakageCurrent 0x08
#define Alarm_SmokeAlarm        0x09
#define Alarm_HighBESSUnitTemp  0x0A
#define Alarm_LowBESSUnitTemp   0x0B
#define Alarm_GenericBatteryPackID     0x0C
#define Alarm_BMCOFFLINE               0x0D


#define Warning_CellOverVoltage   0x00
#define Warning_CellUnderVoltage  0x01
#define Warning_CellOverTemp      0x02
#define Warning_CellUnderTemp     0x03
#define Warning_HighCellTempDelta  0x04
#define Warning_HighCellTempRise  0x05
#define Warning_HighChargeRate    0x06
#define Warning_HighDischargeRate 0x07
#define Warning_HighGroundLeakageCurrent 0x08
#define Warning_OpenDoor          0x09
#define Warning_HighBESSUnitTemp  0x0A
#define Warning_LowBESSUnitTemp   0x0B
#define Warning_ContactorOpen     0x0C
#define Warning_BMCOFFLINE        0x0D

#define Error_LossOfCommunication 0x00
#define Error_MeasuredCalculatedVoltageMismatch 0x01

#define Error_ContactorError      0x03

#define Error_BMCMISSING          0x05

#define Error_BMCOFFLINE          0x05

#define Error_HighCellVolDelta     0x0C
#define Error_BpBooter            0x0D
#define Error_HighAveVoltaDelta   0x0E
#define Update 0xF0
#define UpdateAck 0xF0

#define AlarmSet    0xFF
#define AlarmClr    0x00
#define AlarmUpdate 0xF0
#define AlarmAck    0x0F
#define AlarmClrAck    0x0E
#define AlarmSetAck    0x0F

#define WarningSet  	0xFF
#define WarningClr  	0x00
#define WarningUpdate 	0xF0
#define WarningClrAck   0x0E
#define WarningSetAck  	0x0F

#define ErrorSet    0xFF
#define ErrorClr  	0x00
#define ErrorUpdate	0xF0
#define ErrorSetAck  	0x0F
#define ErrorClrAck 0x0E


#define BPBroadCastID              0x00
#define MsgID_StringShuntDown      0x03

#define MsgID_StringRestartCount   0x05
#define MsgID_BPVoltageInfo        0x42
#define MsgID_BPTemperatureInfo    0x43
#define MsgID_BPBalanceChargerTime 0x45

#define MsgID_BPTargetVoltage      0x20
#define MsgID_StringState          0x21

#define MsgID_BpRequestArrayStringID 0x22

#define MsgID_SystemBPSelect       0x40

#define MsgID_GetBPExtremeVol      0x42
#define MsgID_GetBPExtremeTemp     0x43
#define MsgID_StringHeartBeat      0x46
#define MsgID_UpdateSelectedBPMsg  0x50
#define MsgID_StringCurrent		   0x51
#define MsgID_StringVoltageInfo    0x52
#define MsgID_StringTempInfo       0x53
#define MsgID_StringPower          0x54
#define MsgID_StringBalChargingTimeLeakageCurrent   0x55
#define MsgID_StringUsageHistory   0x56

#define MsgID_HelloBPs                0x80
#define MsgID_SetNumbersofBPs         0x81


#define MsgID_SelectedBPID             0xA0
#define MsgID_SelectedBPStatus         0xA1
#define MsgID_SelectedBPCellVol1To4    0xA2
#define MsgID_SelectedBPCellVol5To8    0xA3
#define MsgID_SelectedBPCellVol9To12   0xA4
#define MsgID_SelectedBPCellVol13To16  0xA5

#define MsgID_SelectedBPCellTemp1To4    0xA6
#define MsgID_SelectedBPCellTemp5To8    0xA7
#define MsgID_SelectedBPCellTemp9To12   0xA8
#define MsgID_SelectedBPCellTemp13To16  0xA9
#define MsgID_SelectedBPBalState        0xAA

#define MsgID_SelectedBPWarrantyTracker            0xAB
#define MsgID_SelectedBPCellWarrantyVolTemp         0xAC
#define MsgID_SelectedBPMAXChargeDisachargeCurrent 0xAD


#define MsgIDToBP_bpStatus              0x41
#define MsgIDToBP_BPExtremeVoltage      0x42
#define MsgIDToBP_BPExtremeTemp         0x43
#define MsgIDToBP_BPBalState            0x44
#define MsgIDToBP_BPBalTime             0x45
#define MsgIDToBP_HeartBeat             0x46

#define MsgIDToBP_CellVol1To4           0x48
#define MsgIDToBP_CellVol5To8           0x49
#define MsgIDToBP_CellVol9To12          0x4A
#define MsgIDToBP_CellVol13To16         0x4B

#define MsgIDToBP_CellTemp1To4          0x4C  
#define MsgIDToBP_CellTemp5To8          0x4D
#define MsgIDToBP_CellTemp9To12         0x4E
#define MsgIDToBP_CellTemp13To16        0x4F

#define MsgIDToBP_SelectBPWarrantyTracker     0xDF
#define MsgIDToBP_MaxChargeDischargeCurrent   0xDE

 

#define MsgIDTo_BPWarrantyVoltageTemp           0xC0

#define MsgID_AllWarrantyTracker     0xC1

#define MsgID_BlueBoxSN 0xC2
#define MsgID_BPSN      0xC6
#define MsgID_BPUID		0xC7

#define ID_Alarm     	0x00
#define ID_Warning   	0x01
#define ID_Error	 	0x02
#define ID_Command   	0x03
#define ID_Data		 	0x04
#define ID_Configure 	0x05
#define ID_Upgrade   	0x06
#define ID_Warranty     0x0E

#define Debug_AlarmValue 0xF5
#define Debug_WarningValue 0xF6

#define Debug_BPAlarmCount 0xF7
#define Debug_BPWarningCount 0xF8
#define AWE_QueryData 0xF1
//EEPROM PageNumber 
#define WholeDataEEPageNumber 1  

#define AHKWHEEPageNumber 19
//add by 2015年9月19日09:21:51
#define OverVoltagePageNumber 2
#define UnderVoltagePageNumber 3
#define OverTempPageNumber 4
#define UnderTempPageNumber 5
#define HighTempDeltaPageNumber 6
#define HighTempRisePageNumber 7
#define HighChargeCurrentPageNumber 8
#define HighDisChargeCurrentPageNumber 9
#define EEPROMFlagPageNumber 10

#define MismatchVoltageThreshold         5
#define LeakageVoltageThresholdALarm     5000
#define LeakageVoltageThresholdWarning   4000

#if 1
#define EnterCellOverVoltageAlarm   	3680
#define ExitCellOverVoltageAlarm 		3650
#endif

//for test 2015年11月6日11:42:29 by jason
 

#define EnterCellUnderVoltageAlarm		2400
#define ExitCellUnderVoltageAlarm		2450

#define EnterCellOverTempAlarm			500
#define ExitCellOverTempAlarm			480

#define EnterCellUnderTempAlarm			0
#define ExitCellUnderTempAlarm			20

#define EnterHighCellTempDeltaAlarm		150
#define ExitHighCellTempDeltaAlarm		130

#define EnterHighCellTempRiseAlarm		80
#define ExitHighCellTempRiseAlarm		60

#define EnterHighChargeRateAlarm		620
#define ExitHighChargeRateAlarm			600

#define EnterHighDisChargeRateAlarm		-620
#define ExitHighDisChargeRateAlarm		-600

#define EnterHighGroundLeakageCurrentAlarm	5000
//#define ExitHighGroundLeakageCurrentAlarm	

#define EnterCellOverVoltageWarning  	3600
#define ExitCellOverVoltageWarning		3570

#define EnterCellUnderVoltageWarning	2750
#define ExitCellUnderVoltageWarning		2800

#define EnterCellOverTempWarning		460
#define ExitCellOverTempWarning			440

#define EnterCellUnderTempWarning		 30
#define ExitCellUnderTempWarning		 50

#define EnterHighCellTempDeltaWarning	 120
#define ExitHighCellTempDeltaTempWarning 100

#define EnterHighCellTempRiseWarning	 50
#define ExitHighCellTempRiseWarning		 30

#define EnterHighChargeRateWarning       600
#define ExitHighChargeRateWarning		 550

#define EnterHighDisChargeRateWarning    -600
#define ExitHighDisChargeRateWarning     -550

#define CAN_MESSAGE_ID(MessageGroupID,DeviceTypeID,StringID,DeviceID,MessageID) ((((((((MessageGroupID << 4) + DeviceTypeID) << 4 ) + StringID) << 6) + DeviceID) << 8) + MessageID)

#define ArrayStringID(ArrayID,StringID)  (ArrayID << 4) + StringID
#pragma anon_unions
typedef union
{
	u16 EEPROMWriteFlag;
	struct
	{
		u16 OverVoltageEEPROMFlag:1;
		u16 UnderVoltageEEPROMFlag:1;
		u16 OverTempEEPROMFlag:1;
		u16 UnderTempEEPROMFlag:1;
		u16 HighDeltaTempEEPROMFlag:1;
		u16 HighTempRiseEEPROMFlag:1;
		u16 HighChargeRateEERPOMFlag:1;
		u16 HighDisChargeRateEEPROMFlag:1;
		u16 SOCEEPROMFlag:1;
	};
}_EEPROM;

extern _EEPROM EEPROM_WriteFlag;

typedef struct
{
	s16 HTempWarning;
	s16 HTempAlarm;
	s16 LTempWarning;
	s16 LTempAlarm;
	s16 HVoltageWarning;
	s16 HVoltageAlarm;
	s16 LVoltageWarning;
	s16 LVoltageAlarm;	
	
	s16 HCellTempRiseAlarm;
	s16 HCellTempRiseWarning;
	
	s16 HCellDeltaTempAlarm;
	s16 HCellDeltaTempWarning;
}_WarningAlarmCmpValue;

typedef struct
{
	u8 Alarm; 
	u8 Warning;
	u8 Error;
	u8 BpLoseHeatBeat[BP_NUMBER];
	u8 BpToStringAlarm;
	u8 BpToStringWarning;
	u8 BpToStringError;
	
	u8 OldHVoltAlarm;
	u8 OldLVoltAlarm;
	u8 OldHVoltWarning;
	u8 OldLVoltWarning;
	u8 OldHTempAlarm;
	u8 OldLTempAlarm;
	u8 OldHTempWarning;
	u8 OldLTempWarning;	 
	u8 OldHChCurrentAlarm;
	u8 OldHDisCurrentAlarm;
	u8 OldHChCurrentWarning;
	u8 OldHDisCurrentWarning;
	
	u8 OldHAveDeltaVolError;
	
	u8 OldBpBootError;
	u8 OldBPHVoltAlarm;
	u8 OldBPLVoltAlarm;
	u8 OldBPHVoltWarning;
	u8 OldBPLVoltWarning;
	u8 OldBPHTempAlarm;
	u8 OldBPLTempAlarm;
	u8 OldBPHTempWarning;
	u8 OldBPLTempWarning;
	u8 OldBPHChCurrentAlarm;
	u8 OldBPHDisCurrentAlarm;
	u8 OldBPHChCurrentWarning;
	u8 OldBPHDisCurrentWarning;	
	
	u8 OldBPHighCellVolDeltaError;
	
	
	
	
}_Count;
typedef struct
{
	u8 HVoltAlarm;
	u8 LVoltAlarm;
	u8 HVoltWarning;
	u8 LVoltWarning;
	u8 HTempAlarm;
	u8 LTempAlarm;
	u8 HTempWarning;
	u8 LTempWarning;
	u8 HTempRiseAlarm;
	u8 HTempDelatAlarm;
	u8 HTempRiseWarning;
	u8 HTempDelatWarning;	
	u8 HChCurrentAlarm;
	u8 HDisCurrentAlarm;
	u8 HChCurrentWarning;
	u8 HDisCurrentWarning;

	u8 HVoltClr;
	u8 LVoltClr;
	u8 HTempClr;
	u8 LTempClr;
	u8 HTempRiseClr;
	u8 HTempDeltaClr;
	u8 HChargeCurrentClr;
	u8 HDischargeCurrentClr;
	
	
}_AlarmWarningCount;

extern _AlarmWarningCount AlarmWarningCount[BP_NUMBER];
#pragma anon_unions
typedef union
{
	u32 BPbyte;
	struct
	{
		u32 Byte1:8;
		u32 Byte2:8;
		u32 Byte3:8;
		u32 Byte4:8;
	};
}_BP;
extern _BP BPAlarmbit;
extern _BP BPWarningbit;
extern _BP BPErrorbit;

extern _BP *BPHVolAlarmbit;
extern _BP *BPHVolWarningbit;
extern _BP *BPHTempAlarmbit;
extern _BP *BPHTempWarningbit;

extern _BP *BPLVolAlarmbit;
extern _BP *BPLVolWarningbit;
extern _BP *BPLTempAlarmbit;
extern _BP *BPLTempWarningbit;


extern _BP *BPLoseCommunicationErrorbit;
//extern _BP *BPErrorbit;
extern u16 BootCount;
extern u8 Boot10sFlag;
extern u16 TimerForAWEProcess;
extern _Count Count;
extern u16 SyncCount;
extern _WarningAlarmCmpValue CmpValue[BP_NUMBER];     
typedef struct 
{
	uint8_t AlarmFlag;
	uint8_t WarningFlag;
	uint8_t ErrorFlag;
	uint16_t Led1sCount;
}_StringStateFlag;

typedef struct
{
	uint8_t HVolAlarmAckFlag;
	uint8_t HVolWarningAckFlag;
	uint8_t LVolAlarmAckFlag;
	uint8_t LVolWarningAckFlag;
	uint8_t HTempAlarmAckFlag;
	uint8_t HTempWarningAckFlag;
	uint8_t LTempAlarmAckFlag;
	uint8_t LTempWarningAckFlag;
	
	uint8_t HCellDeltaTempAlarmAckFlag;
	uint8_t HCellDeltaTempWarningAckFlag;
	
	uint8_t HCellTempRiseAlarmAckFlag;
	uint8_t HCellTempRiseWarningAckFlag;
	
	uint8_t ChargeStringCurrentAlarmAckFlag;
	uint8_t ChargeStringCurrentWarningAckFlag;
	
	uint8_t DischargeStringCurrentAlarmAckFlag;
	uint8_t DischargeStringCurrentWarningAckFlag;
	
	uint8_t BmcOfflineErrorAckFlag;            /*add by jason,change the format the same as other AWE,2016年7月21日17:02:55*/
	
	uint8_t BPLoseofCommunicationErrorAckFlag;
	
	uint8_t BPBootError;
	
	uint8_t HighAveVoltaDeltaErrorAckFlag; /*add by jason,new function 2016年11月2日10:42:31*/
	
}_SysStateAckFlag;

typedef struct
{
	uint8_t HVolAlarmFlag;
	uint8_t HVolWarningFlag;
	uint8_t LVolAlarmFlag;
	uint8_t LVolWarningFlag;
	uint8_t HTempAlarmFlag;
	uint8_t HTempWarningFlag;
	uint8_t LTempAlarmFlag;
	uint8_t LTempWarningFlag;	
	
	uint8_t HCellDeltaTempAlarmFlag;
	uint8_t HCellDeltaTempWarningFlag;
	
	uint8_t HCellTempRiseAlarmFlag;
	uint8_t HCellTempRiseWarningFlag;
	
	uint8_t ChargeStringCurrentAlarmFlag;
	uint8_t ChargeStringCurrentWarningFlag;
	
	uint8_t DischargeStringCurrentAlarmFlag;
	uint8_t DischargeStringCurrentWarningFlag;
	
	uint8_t BmcOfflineAlarmFlag;                     /*add by jason,change the format the same as other AWE,2016年7月21日17:03:06*/
	uint8_t BmcOfflineWarningFlag;
	
	uint8_t BmcOfflineErrorFlag;
	
	uint8_t HighCellDeltaErrorFlag;
	
	uint8_t BPLoseofCommunicationErrorFlag;
	
}_SysStateFlag;

typedef struct
{
	uint8_t HVolAlarmFlag;
	uint8_t HVolWarningFlag;
	uint8_t LVolAlarmFlag;
	uint8_t LVolWarningFlag;
	uint8_t HTempAlarmFlag;
	uint8_t HTempWarningFlag;
	uint8_t LTempAlarmFlag;
	uint8_t LTempWarningFlag;	
	
	uint8_t HCellDeltaTempAlarmFlag;
	uint8_t HCellDeltaTempWarningFlag;
	
	uint8_t HCellTempRiseAlarmFlag;
	uint8_t HCellTempRiseWarningFlag;
	
	uint8_t ChargeStringCurrentAlarmFlag;
	uint8_t ChargeStringCurrentWarningFlag;
	
	uint8_t DischargeStringCurrentAlarmFlag;
	uint8_t DischargeStringCurrentWarningFlag;	
	
	uint8_t Bmc_OffLineErrorFlag;           /*add by jason,change the format the same as other AWE,2016-7-21 17:03:32*/
	
	uint8_t HighCellDeltaErrorFlag;
	
	uint8_t BPLoseofCommunicationErrorFlag; /*add by jason,2016年8月29日9:52:03*/
	
}_SysClrAck;

typedef struct
{
	uint16_t  SetCellOverVoltageAlarm;
	uint16_t  ClrCellOverVoltageAlarm;
	uint16_t  SetCellUnderVoltageAlarm;
	uint16_t  ClrCellUnderVoltageAlarm;
	int16_t	  SetCellOverTempAlarm;
	int16_t   ClrCellOverTempAlarm;
	int16_t	  SetCellUnderTempAlarm;
	int16_t	  ClrCellUnderTempAlarm;
	uint16_t  SetHighCellTempDeltaAlarm;
	uint16_t  ClrHighCellTempDeltaAlarm;
	uint16_t  SetHighCellTempRiseAlarm;
	uint16_t  ClrHighCellTempRiseAlarm;
	uint16_t  SetHighChargeRateAlarm;
	uint16_t  ClrHighChargeRateAlarm;
	int16_t	  SetHighDisChargeRateAlarm;
	int16_t   ClrHighDisChargeRateAlarm;
	
	uint16_t  SetCellOverVoltageWarning;
	uint16_t  ClrCellOverVoltageWarning;
	uint16_t  SetCellUnderVoltageWarning;
	uint16_t  ClrCellUnderVoltageWarning;
	int16_t	  SetCellOverTempWarning;
	int16_t   ClrCellOverTempWarning;
	int16_t	  SetCellUnderTempWarning;
	int16_t	  ClrCellUnderTempWarning;
	uint16_t  SetHighCellTempDeltaWarning;
	uint16_t  ClrHighCellTempDeltaWarning;
	uint16_t  SetHighCellTempRiseWarning;
	uint16_t  ClrHighCellTempRiseWarning;
	uint16_t  SetHighChargeRateWarning;
	uint16_t  ClrHighChargeRateWarning;
	int16_t	  SetHighDisChargeRateWarning;
	int16_t   ClrHighDisChargeRateWarning;	
}_SetClrValue;

extern uint8_t alreadyCalibrated;

extern u8 BPHVolAlarmFlag[BP_NUMBER]  ;
extern u8 BPLVolAlarmFlag[BP_NUMBER]  ;
extern u8 BPHTempAlarmFlag[BP_NUMBER] ;
extern u8 BPLTempAlarmFlag[BP_NUMBER] ;
 
extern u8 StringHVolAlarmFlag[BP_NUMBER]  ;
extern u8 StringLVolAlarmFlag[BP_NUMBER]  ;
extern u8 StringHTempAlarmFlag[BP_NUMBER] ;
extern u8 StringLTempAlarmFlag[BP_NUMBER] ;

extern u8 BPHVolWarningFlag[BP_NUMBER]  ;
extern u8 BPLVolWarningFlag[BP_NUMBER]  ;
extern u8 BPHTempWarningFlag[BP_NUMBER] ;
extern u8 BPLTempWarningFlag[BP_NUMBER] ;
 
extern u8 StringHVolWarningFlag[BP_NUMBER]  ;
extern u8 StringLVolWarningFlag[BP_NUMBER]  ;
extern u8 StringHTempWarningFlag[BP_NUMBER] ;
extern u8 StringLTempWarningFlag[BP_NUMBER] ;


extern u8 BpLossCommunicationWarningFlag[BP_NUMBER];





extern _BP *BPHVolAlarmbit;
extern _BP *BPHVolWarningbit;
extern _BP *BPHTempAlarmbit;
extern _BP *BPHTempWarningbit;
  
extern _BP *BPLVolAlarmbit;
extern _BP *BPLVolWarningbit;
extern _BP *BPLTempAlarmbit;
extern _BP *BPLTempWarningbit;
 
 

extern _BP BPHAveDeltaVolErrorbit;
 
extern _BP BPToStringHVolAlarmbit;
extern _BP BPToStringHVolWarningbit;
extern _BP BPToStringHTempAlarmbit;
extern _BP BPToStringHTempWarningbit;
 
extern _BP BPToStringLVolAlarmbit;
extern _BP BPToStringLVolWarningbit;
extern _BP BPToStringLTempAlarmbit;
extern _BP BPToStringLTempWarningbit;
 
 

extern	_BP BPToStringChargeRateAlarmbit;
extern	_BP BPToStringChargeRateWarningbit;
extern	_BP BPToStringDischargeRateAlarmbit;
extern	_BP BPToStringDischargeRateWarningbit;

extern _BP BPToStringCellOfflineErrorbit;

extern _BP BPToStringHighCellDeltaErrorbit;

extern u8 BMC_OFFLINE_AlarmSet_Flag[BP_NUMBER][CELL_NUMBER];     
extern u8 BMC_OFFLINE_AlarmClr_Flag[BP_NUMBER][CELL_NUMBER];

extern u8 BMC_OFFLINE_WarningSet_Flag[BP_NUMBER][CELL_NUMBER];    
extern u8 BMC_OFFLINE_WarningClr_Flag[BP_NUMBER][CELL_NUMBER];

extern u8 BMC_OFFLINE_AlarmSetAck_Flag[BP_NUMBER][CELL_NUMBER];     
extern u8 BMC_OFFLINE_AlarmClrAck_Flag[BP_NUMBER][CELL_NUMBER];

extern u8 BMC_OFFLINE_WarningSetAck_Flag[BP_NUMBER][CELL_NUMBER];    
extern u8 BMC_OFFLINE_WarningClrAck_Flag[BP_NUMBER][CELL_NUMBER];

extern u8 GetMaintencanceModeFlagFromSys;
extern u8 MaintenanceModeFlag;
extern u8 BMC_OFFLINE_AlarmCount;
extern u8 BMC_OFFLINE_WarningCount;

extern u8 BMC_OFFLINE_AlarmReason[BP_NUMBER][CELL_NUMBER]; 
extern u8 BMC_OFFLINE_WarningReason[BP_NUMBER][CELL_NUMBER];


extern u8 BMC_OFFLINE_ErrorSet_Flag[BP_NUMBER][CELL_NUMBER];
extern u8 BMC_OFFLINE_ErrorClr_Flag[BP_NUMBER][CELL_NUMBER];
  
extern u8 BMC_OFFLINE_ErrorSetAck_Flag[BP_NUMBER][CELL_NUMBER];
extern u8 BMC_OFFLINE_ErrorClrAck_Flag[BP_NUMBER][CELL_NUMBER];
 
extern u8 BMC_OFFLINE_ErrorReason[BP_NUMBER][CELL_NUMBER];

extern _BP BPbooterErrorbit;

extern u16 WriteWKHEEPROM5minTimer;
extern u16 Idle5minFlag;
extern u16 Idle5minCount;
extern u16 Idle5minCountFlag;
 

extern _SetClrValue SetClrValue;

void AutoUpdateAWEMsg(void);
void Bess_SOCByVol(void);
void CanDelay(uint32_t nCount);
void EmptyKwhCalibration(void);
extern u16 ContactorClosedPermissionFlag;  
 
void Can_Send_2_CCU(uint32_t ExtendID,uint8_t Len,uint8_t *buf);
void StringQueryContactorClosedPermission(void);
void SyncAWE(void);
void ReSendStringAWE(void);
void Relay_Set(u8 Relayx, u8 OnOff);
extern _SysStateFlag BpToStringChargeDisChargeFLag;  /*bp to string，string to array*/
extern _SysStateFlag BpToStringFlag[BP_NUMBER];
extern _SysClrAck    WaitClrAckFlag[BP_NUMBER];
extern _SysStateFlag SysChargeDisChargeState;
extern _SysStateAckFlag SysChargeDisChargeFlag;
extern _SysStateFlag SysStateFlag[BP_NUMBER];
extern _SysStateAckFlag SysStateAckFlag[BP_NUMBER];
extern _StringStateFlag ToArray;
extern u8 MsgGroupID;
extern u16 CheckStatusTimer;
extern u16 HeartBeatTimer;
extern u16 WdtTimer;
extern u8 RelayState;

extern MESSAGE ClrAck[BP_NUMBER];

extern u16 QueryExtremeValueTimer;
extern u8 SystemLightBlinkingExitFlag;
uint8_t getPostitiveContactor(void);
uint8_t getPostitiveContactor2(void);
void DetermineContactorState(void);
void calBPRestartCount(void);
void AlarmWarnErrorProcess(void);
void StringAWE(void);
void BPAwe(void);
void BPAWEClear(void);
void StringAWEClear(void);
void ArrayAWEAckClear(void);
void StringAWEIntervalProcess(void);
void MemCopy(u8 *Dest,u8 *Src,u32 Num);
void WDT_State(FunctionalState WDTState);
void StringBP_Capacity_update(void);
void Value_Init(void);
int CalculateCountOfState(unsigned int n); 
void HeartBeatToBP(void);
void InitCmpValueofAlarmWarning(void);
/*for grobal use,add by jaosn ,2015年9月2日10:19:01*/
void send_autodata_Ex_2_BP(uint8_t idx, uint8_t bpIndex); // MsgID,BPID
void SaveWKHTOEEPROM(void);        /*add by jason,2016年1月15日10:44:48*/
void ContactorWarningProcess(void);
void ClrAllAWE(void);  /*bp restart 后清除所有的alarm和warning*/
void ReCheckContactorStatus(void);
void ReCheckLossCommunication(void);
void StringAWEProcess(void);
void InitSystemLightBlinking(void);
void RedLight(void);
void YellowLight(void);
void GreenLight(void);
void RedLightBlinking(void);
void YellowLightBlinking(void);
void GreenLightBlinking(void);

void RelayAllTurnOff(void);
void RelayAllTurnOn(void);



/*For Test*/
extern u16 PortADC1,PortADC2; 
extern u8 OverChargeCount ;
extern u8 OVerDischargeCount ;
extern  u8 OverChargeWarningCount;
extern u8 OVerDischargeWarningCount;
extern u8 ContactorError_Flag;
extern int testDCVoltage;
extern u8 ErrorLoseofCommunicationFlag[BP_NUMBER];
extern u8 ContactorOpenWarningAck;
extern u8 EnterContactorOpenWarningFlag;
extern u8 ContactorOpenwarningClrAck;
extern u8 ArrayStringSendTargetVolFlag;
extern u16 ArrayStringSendTargetVol;
#endif
