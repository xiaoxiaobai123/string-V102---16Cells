#include <stdlib.h>    // abs()
#include "BESS_3-0_AHM_device_lib_V_1-0.h"
#include "BESS_3-0_AHM_gpio_V_1-0.h"
#include "stm32f10x.h"
#include "BESS_3-0_AHM_systick_V_1-0.h"
#include "BESS_3-0_AHM_ad7327_V_1-0.h"
#include "BESS_3-0_AHM_config_V_1-0.h"
#include "BESS_3-0_AHM_can_V_1-0.h"
#include "BESS_3-0_AHM_main_V_1-0.h"
#include "adc.h"
#include "contactor.h"
#include "led.h"
#include "messageid.h"
#include "i2c_ee.h"
#include "canupgrade.h"
#include "NewStringGPIO.h"
#include "NewStringADC.h"
#include "math.h"
#include "pinmap.h"
#include "DataProcess.h"
Config cfg;    // configuration values read from EEPROM
	
int32_t sys_damps = 0;				//system current in deci-amps
int32_t sys_dvolts = 0;				//system voltage in deci-volts
int32_t sys_cwatts = 0;				//system power in centi-watts
int32_t capacity_count = 0;			//appears to be system centi-amp hours
int32_t cwatthour_count = 0;		//appears to be system centi-watt hours
int32_t capacity_count_remainder = 0;
int32_t cwatthour_count_remainder = 0;
float i_shunt_value;
uint32_t last_can_msg;
int8_t Switch_Status = 0;				//
int8_t startup_status;				//The POST Status for the board?
uint8_t status;						//General status of board?

int testDCVoltage;
u16 SyncCount = 0;
/** @defgroup main main
  * @{
  */

#define CAN_MSG_COUNTDOWN  410	/*  */
//by sam
#define CAN_MSG_EX_COUNTDOWN  1000	/*  */
#define DATA_SAVE_INTERVAL 7200	// save data every 2 hours.  Flash memory should last about 20 years at this rate.

//void read_float(float *dest, uint8_t addr, float _default)
//{
//    I2C_EE_BufferRead((uint8_t *)dest, addr, 4);
//    if (!memcmp(dest, "\xff\xff\xff\xff", 4))
//        *dest = _default;
//}

/**
  * @fn         int main(void)
  * @brief      (Amp-Hour Meter)Battery Management System - Firmware Main Entry Point
  * @details    The main entry point for the AHM board firmware.
  * @param      None.
  * @retval     int (Main never returns, a return of int is used to indicate failure)
  */
extern uint16_t totalSendCounter;
extern  void send_autodata_Ex_2_BP(uint8_t idx, uint8_t bpIndex);
uint32_t last_can_msg_ex;// = milli_count;
void checkLed(void)
{
    if ( (milli_count - last_can_msg_ex) >= CAN_MSG_EX_COUNTDOWN)
    {
		static int times = 1;
        last_can_msg_ex = milli_count;
  

        if(times == 1)
        {
            led1_ctl(0);
            times = 0;
        }
        else
        {
            led1_ctl(1);
            times = 1;
        }
		statusLedGreen_blink();             

	}
}

void init_queryawefrombp()
{
	MESSAGE msg;
	MsgGroupID = ID_Command;
	msg.MessageID = 0x06;
	msg.DeviceID = 0x00;
	msg.MessageNum = 0;
	CAN_make_send_bypass_2_BP(msg.Data, msg.MessageNum, msg.MessageID, msg.DeviceID);	
}


int mainpowin(void)
{
    SystemCoreClockUpdate(); // Update global SystemCoreClock variable for initialization functions
    Device_Initialize();
	Value_Init();
	Timer2_Init();
    contactor_init();
    led_init();
 //	WDT_State(ENABLE);   //使能开门狗
	WDT_State(DISABLE);   //  disable watch dog
	I2C_EE_Init();
 	
	ADC1_Configuration();
	ADC1_ContinuousCmd_Enable();  
	
	RS485_Init();
/*EEPROM Read，not use right now*/	
#if 1
	StringEEPromAHKWH_Read();
//	ReadEEPROM_Flag();
//	ReadEEPROMParameters();
#endif
	InitCmpValueofAlarmWarning();
	
    status = 0;
#if 0/*test*/	

//	mStringData.DefaultCellAH = 7300;
//	mStringData.DefaultCellKWH = 36000;
	mStringData.ah = 6000;
		//mStringData.kW = 0;
	mStringData.kWh = 110000;
#endif
	
    SendFirmwareInfo();
 	
	RelayAllTurnOn();
	init_queryawefrombp();

    while (1)
    {
		
		SaveWKHTOEEPROM();
		SyncAWE();
		mStringData.measureStringVoltage = GetStringMeasuredVoltage();
		mStringData.dcbusVoltage = GetDCBusVoltage();
		EmptyKwhCalibration();
		if(ADC_DataUpdatedFalg)
			checkCurrent();
		if(CheckStatusTimer >= 2000)
		{
			CheckStatusTimer = 0;			
			AlarmWarnErrorProcess();  
			Bess_GetDatas();
			HighAveDeltaVolErrorJudge();
		}		
		if(can_rx_fifo.count)
		{
			CAN_RX_Process();     // do the receive datas from bpcs
		}

		if(can_rx_fifo2.count)
		{
 			CAN_RX_Process2();  // do the receive datas from array  
		}

		CAN_TX();                  //发给 bpc
		CAN_TX2();                 //发给 Array
		if((mStringData.measureStringVoltage != 0) && (fabs(mStringData.stringCurrent) >= 100) && (fabs(mStringData.stringCurrent) <= 9000))
		{
			StringBP_Capacity_update();
		}
		if(WdtTimer >= 500)	         /*Watch dog，500ms*/
		{	
			WdtTimer = 0;
			WDT_SW(); 
		}
		HeartBeatToBP();
		StringAWEIntervalProcess();
		BPAWEClear();
		StringAWEClear();
		AutoUpdateAWEMsg();
		UpdateBPAWECount();
 
	}
}

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen main group.
  *    @}
*/

/* End of main.c */
