 #ifndef DATAPROCESS_H_
#define DATAPROCESS_H_

#include "stm32f10x.h"
#include "BESS_3-0_AHM_stringdata_V_1-0.h"
#include "BESS_3-0_AHM_can_V_1-0.h"
#include "BESS_3-0_AHM_can_V_1-0.h"


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
	s16 HChargeRateAlarm;
	s16 HChargeRateWarning;
	s16 HDischargeRateAlarm;
	s16 HDischargeRateWarning;		
	s16 HCellVolDeltaError;
	
}BPAWEHappened;

extern BPAWEHappened BPAWEHappenedFlag;
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
 

	s16 BPBootError;
	s16 HighAveVoltaDeltaError;
}StringAWEHappened;

extern StringAWEHappened StringAWEHappenedFlag;

extern u16 StringAWETimer;
extern uint8_t ProgrameRunTimer;
extern uint8_t ProgrameRun2minsFlag;
extern MESSAGE SysHVolAlarmMessage;
extern MESSAGE SysHVolWarningMessage;
extern MESSAGE SysLVolAlarmMessage;
extern MESSAGE SysLVolWarningMessage;
extern MESSAGE SysHTempAlarmMessage;
extern MESSAGE SysHTempWarningMessage;
extern MESSAGE SysLTempAlarmMessage;
extern MESSAGE SysLTempWarningMessage;
 
extern MESSAGE SysHCellDeltaTempAlarmMessage;
extern MESSAGE SysHCellDeltaTempWarningMessage;
extern MESSAGE SysHCellTempRiseAlarmMessage;
extern MESSAGE SysHCellTempRiseWarningMessage;
extern MESSAGE SysChargeStringCurrentAlarmMessage;
extern MESSAGE SysChargeStringCurrentWarningMessage;
extern MESSAGE SysDischargeStringCurrentAlarmMessage;
extern MESSAGE SysDischargeStringCurrentWarningMessage;

extern MESSAGE SysBPLoseOfCommunicationErrorMessage;
extern s16 CmpValue_ChargeStringCurrentAlarm;
extern s16 CmpValue_ChargeStringCurrentWarning ;
extern s16 CmpValue_DischargeStringCurrentAlarm ;
extern s16 CmpValue_DischargeStringCurrentWarning;   

extern uint32_t TargetValueCounter;       /*add by jason,2016Äê7ÔÂ27ÈÕ10:44:17*/
extern u8 TargetValueCountTimeOutFlag;

extern int Count_AutoUpdateStringAWE[10];
extern int Count_AutoUpdateBPAWE[13];
extern unsigned char Flag_AutoUpdateStringAWE[10];
extern unsigned char Flag_AutpUpdateBPAWE[13];

extern int AWESCAlreadyHappenedFlag[9];
extern int AWEBPAlreadyHappenedFlag[13]; 
#define TargerValueFromArrayTimeOut  10000            // 10s
void Bess_GetDatas(void);
void Bess_GetAverageTemperature(void);
void Bess_GetHighLowTemperature(void);
void GetBPAverageVoltage(void);
void Bess_GetHighDeltaTemperature(void);

void Timer2_Init(void);
void UpdateAWE(void);

void UpdateBPAWECount(void);
void HighAveDeltaVolErrorJudge(void);
#endif
