/**
  ******************************************************************************
  * @file        can.c
  * @author      Nystrom Engineering
  * @version     2.0.0
  * @date        Jan 27, 2014
  * @copyright   Powin Energy
  * @brief       This file provides all the CAN-Bus functions.
  * @page        can_page Controller Automation Network (CAN) Functions
  * @section     can_intro Introduction
  * @par
  *  The CAN functions are used to interface with the system Controller Automation Network.  \n
  *
  ******************************************************************************
  */
/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
#include <stdio.h> 
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "BESS_3-0_AHM_canmsg_V_1-0.h"
#include "BESS_3-0_AHM_config_V_1-0.h"
#include "BESS_3-0_AHM_bpdata_V_1-0.h"

#ifdef STM32F10X_CL  //for WaveShare F107
 
#else
//#ifdef STM32F10X_CL  //for WaveShare F107
//    extern void CAN_make_send_BP2 (uint8_t *buf, uint8_t length, uint8_t msg_id, uint8_t bp_id);
//#else
    extern void CAN_make_send_BP (uint8_t *buf, uint8_t length, uint8_t msg_id, uint8_t bp_id);
//#endif

void sendAlarmWarningMsg(uint8_t msg_id, uint8_t alarmId, uint8_t flag, uint8_t bpId, uint8_t cellId, uint16_t alarmValue)
{
    uint8_t buf[8];
    uint8_t dataLen = 0;
    buf[0] = alarmId;
    buf[1] = flag;
    buf[2] = 0x0;
    buf[3] = 0x0;
    buf[4] = bpId+1;
    buf[5] = cellId;
    buf[6] = alarmValue&0xff;
    buf[7] = (alarmValue>>8)&0xff;   
    dataLen = 8;   
//#ifdef STM32F10X_CL  //for WaveShare F107
//    printf("    ~ ==Send Message(AHM)==> msg_id:0x%02x, alarmId:0x%02x, flag:0x%02x, bpId:0x%02x ~\r\n",msg_id, alarmId, flag, bpId);
    
//    CAN_make_send_BP2(buf, dataLen, msg_id, bpId);
//#else
    printf("  ==Send Message(BP)==> msg_id:0x%02x, alarmId:0x%02x, flag:0x%02x, bpId:0x%02x, value:%d ~\r\n",
                        msg_id, alarmId, flag, bpId, alarmValue);
    
    CAN_make_send_BP(buf, dataLen, msg_id, bpId);
//#endif    
   
}

uint8_t InitConfigValue(Config* mConfig)
{
//    int i;
    mConfig->alarmWarningConfig.overCellVoltageAlarmSet     = OVER_VOLTAGE_ALARM_SET_DEFAULT_VALUE;     
    mConfig->alarmWarningConfig.overCellVoltageAlarmClr     = OVER_VOLTAGE_ALARM_CLR_DEFAULT_VALUE;  
    mConfig->alarmWarningConfig.overCellVoltageWarningSet   = OVER_VOLTAGE_WARNING_SET_DEFAULT_VALUE;  
    mConfig->alarmWarningConfig.overCellVoltageWarningClr   = OVER_VOLTAGE_WARNING_CLR_DEFAULT_VALUE;  
         
    mConfig->alarmWarningConfig.underCellVoltageAlarmSet    = UNDER_VOLTAGE_ALARM_SET_DEFAULT_VALUE; 
    mConfig->alarmWarningConfig.underCellVoltageAlarmClr    = UNDER_VOLTAGE_ALARM_CLR_DEFAULT_VALUE;
    mConfig->alarmWarningConfig.underCellVoltageWarningSet  = UNDER_VOLTAGE_WARNING_SET_DEFAULT_VALUE;
    mConfig->alarmWarningConfig.underCellVoltageWarningClr  = UNDER_VOLTAGE_WARNING_CLR_DEFAULT_VALUE;

    mConfig->alarmWarningConfig.overCellTempAlarmSet     = OVER_TEMP_ALARM_SET_DEFAULT_VALUE;  
    mConfig->alarmWarningConfig.overCellTempAlarmClr     = OVER_TEMP_ALARM_CLR_DEFAULT_VALUE;  
    mConfig->alarmWarningConfig.overCellTempWarningSet   = OVER_TEMP_WARNING_SET_DEFAULT_VALUE;
    mConfig->alarmWarningConfig.overCellTempWarningClr   = OVER_TEMP_WARNING_CLR_DEFAULT_VALUE;
                                                              
    mConfig->alarmWarningConfig.underCellTempAlarmSet    = UNDER_TEMP_ALARM_SET_DEFAULT_VALUE;  
    mConfig->alarmWarningConfig.underCellTempAlarmClr    = UNDER_TEMP_ALARM_CLR_DEFAULT_VALUE;  
    mConfig->alarmWarningConfig.underCellTempWarningSet  = UNDER_TEMP_WARNING_SET_DEFAULT_VALUE;
    mConfig->alarmWarningConfig.underCellTempWarningClr  = UNDER_TEMP_WARNING_CLR_DEFAULT_VALUE;
    
    mConfig->alarmWarningConfig.HCDTAlarmSet    = HCDT_ALARM_SET_DEFAULT_VALUE;  
    mConfig->alarmWarningConfig.HCDTAlarmClr    = HCDT_ALARM_CLR_DEFAULT_VALUE;  
    mConfig->alarmWarningConfig.HCDTWarningSet  = HCDT_WARNING_SET_DEFAULT_VALUE;
    mConfig->alarmWarningConfig.HCDTWarningClr  = HCDT_WARNING_CLR_DEFAULT_VALUE;    
    
    mConfig->alarmWarningConfig.HCTRAlarmSet    = HCTR_ALARM_SET_DEFAULT_VALUE;  
    mConfig->alarmWarningConfig.HCTRAlarmClr    = HCTR_ALARM_CLR_DEFAULT_VALUE;  
    mConfig->alarmWarningConfig.HCTRWarningSet  = HCTR_WARNING_SET_DEFAULT_VALUE;
    mConfig->alarmWarningConfig.HCTRWarningClr  = HCTR_WARNING_CLR_DEFAULT_VALUE;    

    mConfig->alarmWarningConfig.HCRAlarmSet    = HCR_ALARM_SET_DEFAULT_VALUE;  
    mConfig->alarmWarningConfig.HCRAlarmClr    = HCR_ALARM_CLR_DEFAULT_VALUE;  
    mConfig->alarmWarningConfig.HCRWarningSet  = HCR_WARNING_SET_DEFAULT_VALUE;
    mConfig->alarmWarningConfig.HCRWarningClr  = HCR_WARNING_CLR_DEFAULT_VALUE;    

    mConfig->alarmWarningConfig.HDRAlarmSet    = HDR_ALARM_SET_DEFAULT_VALUE;  
    mConfig->alarmWarningConfig.HDRAlarmClr    = HDR_ALARM_CLR_DEFAULT_VALUE;  
    mConfig->alarmWarningConfig.HDRWarningSet  = HDR_WARNING_SET_DEFAULT_VALUE;
    mConfig->alarmWarningConfig.HDRWarningClr  = HDR_WARNING_CLR_DEFAULT_VALUE; 
    
    
    return 1;    
}
#define CHECK_TYPE_HIGH  0x0
#define CHECK_TYPE_LOW   0x1




uint8_t CheckConfigValue(Config* mConfig, BPData* mBPData, uint8_t bpId)
{
    //#define CHECK_STATUS_IDLE                   0x0
    //#define CHECK_STATUS_WARNING                0x1
    //#define CHECK_STATUS_ALARM                  0x2
    int32_t alarmSetValue = 0, alarmClrValue = 0, warningSetValue = 0, warningClrValue = 0;
    int32_t targetValue = 0;
    uint16_t targetValue2 = 0;
    int8_t checkType= CHECK_TYPE_HIGH, currentStatusTmp, checkFlagTmp;
    int i;
    for(i = 0; i<CONFIG_INDEX_MAX; i++)
    {
        //AlarmWarningStatus alarmWarningStatus[CONFIG_INDEX_MAX];
        switch(i)
        {
            case CONFIG_INDEX_OVER_SELL_VOLTAGE:
                printf("~ check Over Cell Voltage (0xe0)(BP id:0x%02x) ~\r\n", bpId+1);
                alarmSetValue = mConfig->alarmWarningConfig.overCellVoltageAlarmSet;
                alarmClrValue = mConfig->alarmWarningConfig.overCellVoltageAlarmClr;
                warningSetValue = mConfig->alarmWarningConfig.overCellVoltageWarningSet;
                warningClrValue = mConfig->alarmWarningConfig.overCellVoltageWarningClr;
                checkType= CHECK_TYPE_HIGH;
                targetValue = mBPData->highCellVolt;
                memcpy(&targetValue2, &(mBPData->highCellVolt), sizeof(uint16_t));
                break;
            case CONFIG_INDEX_UNDER_SELL_VOLTAGE:
                printf("~ check Under Cell Voltage (0xe1)(BP id:0x%02x) ~\r\n", bpId+1);
                alarmSetValue = mConfig->alarmWarningConfig.underCellVoltageAlarmSet;
                alarmClrValue = mConfig->alarmWarningConfig.underCellVoltageAlarmClr;
                warningSetValue = mConfig->alarmWarningConfig.underCellVoltageWarningSet;
                warningClrValue = mConfig->alarmWarningConfig.underCellVoltageWarningClr;
                //warningClrValue = mConfig->alarmWarningConfig.underCellVoltageWarningSet;
                //warningSetValue = mConfig->alarmWarningConfig.underCellVoltageWarningClr;
                checkType= CHECK_TYPE_LOW;
                targetValue = mBPData->lowCellVolt;
                memcpy(&targetValue2, &(mBPData->lowCellVolt), sizeof(uint16_t));
                break;
            case CONFIG_INDEX_OVER_SELL_TEMP:
                printf("~ check Over Cell Temp (0xe2)(BP id:0x%02x) ~\r\n", bpId+1);
                alarmSetValue = mConfig->alarmWarningConfig.overCellTempAlarmSet;  
                alarmClrValue = mConfig->alarmWarningConfig.overCellTempAlarmClr;  
                warningSetValue = mConfig->alarmWarningConfig.overCellTempWarningSet;
                warningClrValue = mConfig->alarmWarningConfig.overCellTempWarningClr;
                checkType= CHECK_TYPE_HIGH;
                targetValue = mBPData->highCellTemp;
                memcpy(&targetValue2, &(mBPData->highCellTemp), sizeof(uint16_t));
                break;
            case CONFIG_INDEX_UNDER_SELL_TEMP:
                printf("~ check Under Cell Temp (0xe3)(BP id:0x%02x) ~\r\n", bpId+1);
                alarmSetValue = mConfig->alarmWarningConfig.underCellTempAlarmSet;  
                alarmClrValue = mConfig->alarmWarningConfig.underCellTempAlarmClr;  
                warningSetValue = mConfig->alarmWarningConfig.underCellTempWarningSet;
                warningClrValue = mConfig->alarmWarningConfig.underCellTempWarningClr;
                checkType= CHECK_TYPE_LOW;
                targetValue = mBPData->lowCellTemp;
                memcpy(&targetValue2, &(mBPData->lowCellTemp), sizeof(uint16_t));
                break;
            case CONFIG_INDEX_HCDT:
                printf("~ check High Cell Delta Temp (0xe4)(BP id:0x%02x) ~\r\n", bpId+1);
                alarmSetValue = mConfig->alarmWarningConfig.HCDTAlarmSet;  
                alarmClrValue = mConfig->alarmWarningConfig.HCDTAlarmClr;  
                warningSetValue = mConfig->alarmWarningConfig.HCDTWarningSet;
                warningClrValue = mConfig->alarmWarningConfig.HCDTWarningClr;  
                checkType= CHECK_TYPE_HIGH;
                targetValue = mBPData->cellDeltaT;
                memcpy(&targetValue2, &(mBPData->cellDeltaT), sizeof(uint16_t));
                break;
            case CONFIG_INDEX_HCTR:
                printf("~ check High Cell Temp Rise (0xe5)(BP id:0x%02x) ~\r\n", bpId+1);
                alarmSetValue = mConfig->alarmWarningConfig.HCTRAlarmSet;  
                alarmClrValue = mConfig->alarmWarningConfig.HCTRAlarmClr;  
                warningSetValue = mConfig->alarmWarningConfig.HCTRWarningSet;
                warningClrValue = mConfig->alarmWarningConfig.HCTRWarningClr;  
                checkType= CHECK_TYPE_HIGH;
                targetValue = mBPData->maxCellTempRise;
                memcpy(&targetValue2, &(mBPData->maxCellTempRise), sizeof(uint16_t));
                break;
            case CONFIG_INDEX_HCR:
                printf("~ check High Charge Rate (0xe6)(BP id:0x%02x) ~\r\n", bpId+1);
                alarmSetValue = mConfig->alarmWarningConfig.HCRAlarmSet;  
                alarmClrValue = mConfig->alarmWarningConfig.HCRAlarmClr;  
                warningSetValue = mConfig->alarmWarningConfig.HCRWarningSet;
                warningClrValue = mConfig->alarmWarningConfig.HCRWarningClr;  
                checkType= CHECK_TYPE_HIGH;
                targetValue = mBPData->maxChargeCurrent;
                memcpy(&targetValue2, &(mBPData->maxChargeCurrent), sizeof(uint16_t));
                break;
            case CONFIG_INDEX_HDR:
                printf("~ check High DisCharge Rate (0xe7)(BP id:0x%02x) ~\r\n", bpId+1);
                alarmSetValue = mConfig->alarmWarningConfig.HDRAlarmSet;  
                alarmClrValue = mConfig->alarmWarningConfig.HDRAlarmClr;  
                warningSetValue = mConfig->alarmWarningConfig.HDRWarningSet;
                warningClrValue = mConfig->alarmWarningConfig.HDRWarningClr;  
                checkType= CHECK_TYPE_HIGH;
                targetValue = mBPData->maxDischargeCurrent;
                memcpy(&targetValue2, &(mBPData->maxDischargeCurrent), sizeof(uint16_t));
                break;
            default:
                printf("~ check error, not implement yet ~\r\n");
                break;
                

        }
        if(alarmSetValue != 0)
        {          
            currentStatusTmp = mBPData->alarmWarningStatus[i].currentStatus;
            checkFlagTmp = mBPData->alarmWarningStatus[i].checkWarningFlag;
            if(checkType == CHECK_TYPE_HIGH)
            {                
                switch(mBPData->alarmWarningStatus[i].currentStatus)
                {
//                    case CHECK_STATUS_INIT:
                    case CHECK_STATUS_IDLE:
                        if(targetValue > alarmSetValue)
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_ALARM;//OK
                            mBPData->alarmWarningStatus[i].checkWarningFlag = CHECK_WARNING_FLAG_TRUE;//CHECK_WARNING_FLAG_FALSE;//OK
                            //printf(" ~ --1-> set flag FALSE... ~\r\n");
                        }
                        else if(targetValue > alarmClrValue)
                        {
                            if(mBPData->alarmWarningStatus[i].checkWarningFlag == CHECK_WARNING_FLAG_TRUE)
                            {
                                mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_WARNING;//OK
                            }
                            else
                            {
                                //printf(" ~ --1-> flag = FALSE, ignore set to CHECK_STATUS_WARNING... ~\r\n");
                            }
                        }
                        else if(targetValue > warningSetValue)
                        {
                            if(mBPData->alarmWarningStatus[i].checkWarningFlag == CHECK_WARNING_FLAG_TRUE)
                            {
                                mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_WARNING;//OK
                            }
                            else
                            {
                                //printf(" ~ --1-> flag = FALSE, ignore set to CHECK_STATUS_WARNING... ~\r\n");
                            }
                        }
                        else if(targetValue > warningClrValue)
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_IDLE;//OK                            
                        }
                        else
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_IDLE;//OK
                            mBPData->alarmWarningStatus[i].checkWarningFlag = CHECK_WARNING_FLAG_TRUE;//OK
                            //printf(" ~ --1-> set flag TRUE... ~\r\n");
                        }
                        break;
                    case CHECK_STATUS_WARNING:
                        if(targetValue > alarmSetValue)
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_ALARM;//OK
                            mBPData->alarmWarningStatus[i].checkWarningFlag = CHECK_WARNING_FLAG_TRUE;//CHECK_WARNING_FLAG_FALSE;//OK
                            //printf(" ~ --2-> set flag FALSE... ~\r\n");
                        }
                        else if(targetValue > alarmClrValue)
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_WARNING;//OK                            
                        }
                        else if(targetValue > warningSetValue)
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_WARNING;//OK                            
                        }
                        else if(targetValue > warningClrValue)
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_WARNING;//OK                            
                        }
                        else
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_IDLE;//OK
                            mBPData->alarmWarningStatus[i].checkWarningFlag = CHECK_WARNING_FLAG_TRUE;//OK
                            //printf(" ~ --2-> set flag TRUE... ~\r\n");
                        }
                        break;
                    case CHECK_STATUS_ALARM:
                        if(targetValue > alarmSetValue)
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_ALARM;//OK  
                            mBPData->alarmWarningStatus[i].checkWarningFlag = CHECK_WARNING_FLAG_TRUE;//CHECK_WARNING_FLAG_FALSE;//OK    
                            //printf(" ~ --3-> set flag FALSE... ~\r\n");                            
                        }
                        else if(targetValue > alarmClrValue)
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_ALARM;//OK                                                
                        }
                        else if(targetValue > warningSetValue)
                        {                           
                            if(mBPData->alarmWarningStatus[i].checkWarningFlag == CHECK_WARNING_FLAG_TRUE)
                            {
                                mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_WARNING;//OK
                            }
                            else
                            {
                                mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_IDLE; 
                            }
                                                       
                        }
                        else if(targetValue > warningClrValue)
                        {
                            if(mBPData->alarmWarningStatus[i].checkWarningFlag == CHECK_WARNING_FLAG_TRUE)
                            {
                                mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_WARNING;//OK
                            }
                            else
                            {
                                mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_IDLE; 
                            }
                        }
                        else
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_IDLE;//OK
                            mBPData->alarmWarningStatus[i].checkWarningFlag = CHECK_WARNING_FLAG_TRUE;//OK
                            //printf(" ~ --3-> set flag TRUE... ~\r\n");
                        }
                        break;                    
                }   
            }
            else if(checkType == CHECK_TYPE_LOW)
            {                
                switch(mBPData->alarmWarningStatus[i].currentStatus)
                {
//                    case CHECK_STATUS_INIT:
                    case CHECK_STATUS_IDLE:
                        if(targetValue < alarmSetValue)
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_ALARM;//OK
                            mBPData->alarmWarningStatus[i].checkWarningFlag = CHECK_WARNING_FLAG_TRUE;//CHECK_WARNING_FLAG_FALSE;//OK
                            //printf(" ~ -=1=> set flag FALSE... ~\r\n");
                        }
                        else if(targetValue < alarmClrValue)
                        {
                            if(mBPData->alarmWarningStatus[i].checkWarningFlag == CHECK_WARNING_FLAG_TRUE)
                            {
                                mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_WARNING;//OK
                            }
                            else
                            {
                                //printf(" ~ -=1=> flag = FALSE, ignore set to CHECK_STATUS_WARNING... ~\r\n");
                            }
                        }
                        else if(targetValue < warningSetValue)
                        {
                            if(mBPData->alarmWarningStatus[i].checkWarningFlag == CHECK_WARNING_FLAG_TRUE)
                            {
                                mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_WARNING;//OK
                            }
                            else
                            {
                                //printf(" ~ -=1=> flag = FALSE, ignore set to CHECK_STATUS_WARNING... ~\r\n");
                            }
                        }
                        else if(targetValue < warningClrValue)
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_IDLE;//OK
                        }
                        else
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_IDLE;//OK
                            mBPData->alarmWarningStatus[i].checkWarningFlag = CHECK_WARNING_FLAG_TRUE;//OK
                            //printf(" ~ -=1=> set flag TRUE... ~\r\n");
                        }
                        break;
                    case CHECK_STATUS_WARNING:
                        if(targetValue < alarmSetValue)//2.25
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_ALARM;//OK
                            mBPData->alarmWarningStatus[i].checkWarningFlag = CHECK_WARNING_FLAG_TRUE;//CHECK_WARNING_FLAG_FALSE;//OK
                            //printf(" ~ -=2=> set flag FALSE... ~\r\n");
                        }
                        else if(targetValue < alarmClrValue)//2.30
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_WARNING;//OK
                        }
                        else if(targetValue < warningSetValue)//2.50
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_WARNING;//OK
                        }
                        else if(targetValue < warningClrValue)//2.55
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_WARNING;//OK
                        }
                        else
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_IDLE;//OK
                            mBPData->alarmWarningStatus[i].checkWarningFlag = CHECK_WARNING_FLAG_TRUE;//OK
                            //printf(" ~ -=2=> set flag TRUE... ~\r\n");
                        }
                        break;
                    case CHECK_STATUS_ALARM:
                        if(targetValue < alarmSetValue)
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_ALARM;//OK
                            mBPData->alarmWarningStatus[i].checkWarningFlag = CHECK_WARNING_FLAG_TRUE;//CHECK_WARNING_FLAG_FALSE;//OK    
                            //printf(" ~ -=3=> set flag FALSE... ~\r\n");      
                        }
                        else if(targetValue < alarmClrValue)
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_ALARM;//OK
                        }
                        else if(targetValue < warningSetValue)
                        {
                            if(mBPData->alarmWarningStatus[i].checkWarningFlag == CHECK_WARNING_FLAG_TRUE)
                            {
                                mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_WARNING;//OK
                            }
                            else
                            {
                                mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_IDLE; 
                            }
                        }
                        else if(targetValue < warningClrValue)
                        {
                           if(mBPData->alarmWarningStatus[i].checkWarningFlag == CHECK_WARNING_FLAG_TRUE)
                            {
                                mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_WARNING;//OK
                            }
                            else
                            {
                                mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_IDLE; 
                            }
                        }
                        else
                        {
                            mBPData->alarmWarningStatus[i].currentStatus = CHECK_STATUS_IDLE;
                            mBPData->alarmWarningStatus[i].checkWarningFlag = CHECK_WARNING_FLAG_TRUE;//OK
                            //printf(" ~ -=3=> set flag TRUE... ~\r\n");
                        }
                        break;                    
                }   
            }
            else
            {
                printf(" - check type error~\r\n");
            }
            //if(currentStatusTmp == CHECK_STATUS_INIT)
            //{
            //    printf("~ check type CHECK_STATUS_INIT, ignore it: %d (%d, %d, %d, %d), flag:%d, <%s(%d)> ~\r\n", 
            //                            targetValue, 
            //                            alarmSetValue, alarmClrValue, warningSetValue, warningClrValue,
            //                            mBPData->alarmWarningStatus[i].checkWarningFlag,
            //                            getCurrentStatusStr(mBPData->alarmWarningStatus[i].currentStatus), mBPData->alarmWarningStatus[i].currentStatus);
            //    
            //}
            //else
            {
                if(currentStatusTmp != mBPData->alarmWarningStatus[i].currentStatus)
                {
                    uint8_t msgId = 0x0, flag = 0x0;
                    printf(" - check result: %d (%d, %d, %d, %d), flag:%d->%d,  <%s(%d)> => <%s(%d)> ~\r\n", 
                                        targetValue, 
                                        alarmSetValue, alarmClrValue, warningSetValue, warningClrValue,
                                        checkFlagTmp, mBPData->alarmWarningStatus[i].checkWarningFlag,
                                        getCurrentStatusStr(currentStatusTmp), currentStatusTmp,
                                        getCurrentStatusStr(mBPData->alarmWarningStatus[i].currentStatus), mBPData->alarmWarningStatus[i].currentStatus);
                    //sendAlarmWarningMsg(uint8_t msg_id, uint8_t alarmId, uint8_t flag, uint8_t bpId, uint8_t cellId, uint16_t alarmValue)
                    if(mBPData->alarmWarningStatus[i].currentStatus == CHECK_STATUS_ALARM)
                    {//Alarm Set
                        msgId = 0x00;
                        flag = 0xff;
                        sendAlarmWarningMsg(msgId, i, flag, bpId, 0, targetValue2);
                    }
                    else if(mBPData->alarmWarningStatus[i].currentStatus == CHECK_STATUS_WARNING)
                    {
                        if(currentStatusTmp == CHECK_STATUS_ALARM)
                        {//Alarm Clr
                            msgId = 0x00;
                            flag = 0x00;
                            sendAlarmWarningMsg(msgId, i, flag, bpId, 0, targetValue2);
                        }
                        //Warning Set
                        msgId = 0x01;
                        flag = 0xff;
                        sendAlarmWarningMsg(msgId, i, flag, bpId, 0, targetValue2);
                    }
                    else if(mBPData->alarmWarningStatus[i].currentStatus == CHECK_STATUS_IDLE)
                    {
                        if(currentStatusTmp == CHECK_STATUS_ALARM)
                        {//Alarm Clr
                            msgId = 0x00;
                            flag = 0x00;
                            sendAlarmWarningMsg(msgId, i, flag, bpId, 0, targetValue2);
                        }
                        //else if(currentStatusTmp == CHECK_STATUS_WARNING)
                        //{//Warning Clr
                            msgId = 0x01;
                            flag = 0x00;
                            sendAlarmWarningMsg(msgId, i, flag, bpId, 0, targetValue2);
                        //}
                    }
                    
                }
                else
                {
                    printf(" - check result: %d (%d, %d, %d, %d), flag:%d, <%s(%d)> ~\r\n", 
                                        targetValue, 
                                        alarmSetValue, alarmClrValue, warningSetValue, warningClrValue,
                                        mBPData->alarmWarningStatus[i].checkWarningFlag,
                                        getCurrentStatusStr(mBPData->alarmWarningStatus[i].currentStatus), mBPData->alarmWarningStatus[i].currentStatus);
                }
            }
        }  
        else   
        {
            printf("~ ignore check ~\r\n");
        }            
    }    
    return 1; 
}
#endif

char* getCurrentStatusStr(uint8_t index)
{
    switch(index)
    {
        case CHECK_STATUS_IDLE:
            return "CHECK_STATUS_IDLE";

        case CHECK_STATUS_ALARM:
            return "CHECK_STATUS_ALARM";

        case CHECK_STATUS_WARNING:
            return "CHECK_STATUS_WARNING";

        default:
            return "ERROR CHECK_STATUS";

    }
}


char* getAlarmWarningStr(uint8_t index)
{
    index = index+CAN_MSG_OVER_VOLTAGE_SETTING;
    switch(index)
    {
        case CAN_MSG_OVER_VOLTAGE_SETTING:
            return "<Over Cell Voltage>";
        case CAN_MSG_UNDER_VOLTAGE_SETTING:
            return "<Under Cell Voltage>";
        case CAN_MSG_OVER_TEMP_SETTING:
            return "<Over Cell Temp>";
        case CAN_MSG_UNDER_TEMP_SETTING:
            return "<Under Cell Temp>";
        case CAN_MSG_HCDT_SETTING:
            return "<High Cell Delta Temp(Diff Temp)>";
        case CAN_MSG_HCTR_SETTING:
            return "<High Cell Temp Rise>";
        case CAN_MSG_HCR_TEMP_SETTING:
            return "<High Charge Rate>";
        case CAN_MSG_HDR_TEMP_SETTING:
            return "<High Discharge Rate>";
        

        default:
            return "<ERROR ALARM String>";

    }
}

/**
  * Close the Doxygen can_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen can group.
  *    @}
*/

/* End of can.c */
