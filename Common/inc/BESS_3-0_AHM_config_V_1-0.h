

#ifndef CONFIG_INC
#define CONFIG_INC


#include <stdio.h> 
#include <stdlib.h>
#include <stdint.h>
#include "BESS_3-0_AHM_bpdata_V_1-0.h"


#define OVER_VOLTAGE_ALARM_SET_DEFAULT_VALUE        3650
#define OVER_VOLTAGE_ALARM_CLR_DEFAULT_VALUE        3600
#define OVER_VOLTAGE_WARNING_SET_DEFAULT_VALUE      3550
#define OVER_VOLTAGE_WARNING_CLR_DEFAULT_VALUE      3500

#define UNDER_VOLTAGE_ALARM_SET_DEFAULT_VALUE       2250
#define UNDER_VOLTAGE_ALARM_CLR_DEFAULT_VALUE       2300
#define UNDER_VOLTAGE_WARNING_SET_DEFAULT_VALUE     2500//2550
#define UNDER_VOLTAGE_WARNING_CLR_DEFAULT_VALUE     2550//2500

#if(1)
    #define OVER_TEMP_ALARM_SET_DEFAULT_VALUE        480
    #define OVER_TEMP_ALARM_CLR_DEFAULT_VALUE        450
    #define OVER_TEMP_WARNING_SET_DEFAULT_VALUE      430
    #define OVER_TEMP_WARNING_CLR_DEFAULT_VALUE      400
#else
    #define OVER_TEMP_ALARM_SET_DEFAULT_VALUE        280//480
    #define OVER_TEMP_ALARM_CLR_DEFAULT_VALUE        250//450
    #define OVER_TEMP_WARNING_SET_DEFAULT_VALUE      230//430
    #define OVER_TEMP_WARNING_CLR_DEFAULT_VALUE      200//400
#endif

#define UNDER_TEMP_ALARM_SET_DEFAULT_VALUE       20
#define UNDER_TEMP_ALARM_CLR_DEFAULT_VALUE       50
#define UNDER_TEMP_WARNING_SET_DEFAULT_VALUE     70
#define UNDER_TEMP_WARNING_CLR_DEFAULT_VALUE     100

#define HCDT_ALARM_SET_DEFAULT_VALUE        150
#define HCDT_ALARM_CLR_DEFAULT_VALUE        130
#define HCDT_WARNING_SET_DEFAULT_VALUE      120
#define HCDT_WARNING_CLR_DEFAULT_VALUE      100


#define HCTR_ALARM_SET_DEFAULT_VALUE        80
#define HCTR_ALARM_CLR_DEFAULT_VALUE        60
#define HCTR_WARNING_SET_DEFAULT_VALUE      60
#define HCTR_WARNING_CLR_DEFAULT_VALUE      30

#define HCR_ALARM_SET_DEFAULT_VALUE        750
#define HCR_ALARM_CLR_DEFAULT_VALUE        700
#define HCR_WARNING_SET_DEFAULT_VALUE      650
#define HCR_WARNING_CLR_DEFAULT_VALUE      600

#define HDR_ALARM_SET_DEFAULT_VALUE        750
#define HDR_ALARM_CLR_DEFAULT_VALUE        700
#define HDR_WARNING_SET_DEFAULT_VALUE      650
#define HDR_WARNING_CLR_DEFAULT_VALUE      600





// Process variable structure
// one each for v and i
typedef struct 
{
    int scale;
    int offs;
    int threshold;
} Pv;

typedef struct 
{
    //0xE0 Over Cell Voltage
    uint16_t overCellVoltageAlarmSet;
    uint16_t overCellVoltageAlarmClr;
    uint16_t overCellVoltageWarningSet;
    uint16_t overCellVoltageWarningClr;

    //0xE1 Under Cell Voltage
    uint16_t underCellVoltageAlarmSet;
    uint16_t underCellVoltageAlarmClr;
    uint16_t underCellVoltageWarningSet;
    uint16_t underCellVoltageWarningClr;

    //0xE2 Over Cell Temp
    int16_t overCellTempAlarmSet;
    int16_t overCellTempAlarmClr;
    int16_t overCellTempWarningSet;
    int16_t overCellTempWarningClr;

    //0xE3 Under Cell Temp
    int16_t underCellTempAlarmSet;
    int16_t underCellTempAlarmClr;
    int16_t underCellTempWarningSet;
    int16_t underCellTempWarningClr;

    //0xE4 HCDT : High Cell Delta Temp(Diff Temp)
    uint16_t HCDTAlarmSet; 
    uint16_t HCDTAlarmClr;
    uint16_t HCDTWarningSet;
    uint16_t HCDTWarningClr;

    //0xE5 HCTR : High Cell Temp Rise
    uint16_t HCTRAlarmSet; 
    uint16_t HCTRAlarmClr;
    uint16_t HCTRWarningSet;
    uint16_t HCTRWarningClr;

    //0xE6 HCR : High Charge Rate
    uint16_t HCRAlarmSet; 
    uint16_t HCRAlarmClr;
    uint16_t HCRWarningSet;
    uint16_t HCRWarningClr;

    //0xE7 HDR : High Discharge Rate
    uint16_t HDRAlarmSet; 
    uint16_t HDRAlarmClr;
    uint16_t HDRWarningSet;
    uint16_t HDRWarningClr;

} AlarmWarningConfig;



typedef struct {
    Pv i, v;
    AlarmWarningConfig alarmWarningConfig;

} Config;


uint8_t InitConfigValue(Config* mConfig);
uint8_t CheckConfigValue(Config* mConfig, BPData* mBPData, uint8_t bpId);
char* getAlarmWarningStr(uint8_t index);
void sendAlarmWarningMsg(uint8_t msg_id, uint8_t alarmId, uint8_t flag, uint8_t bpId, uint8_t cellId, uint16_t alarmValue);

#endif

