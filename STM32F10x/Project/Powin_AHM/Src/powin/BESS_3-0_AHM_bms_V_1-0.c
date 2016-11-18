/**
  ******************************************************************************
  * @file        bms.c
  * @author      Nystrom Engineering
  * @version     2.0.0
  * @date        Jan 27, 2014
  * @copyright   Powin Energy
  * @brief       This file provides all the Battery Management System functions.
  * @page        bms_page Battery Management System Functions
  * @section     bms_intro Introduction
  * @par
  *  The BMS functions are used to monitor and control the battery management system.  \n
  *
  ******************************************************************************
  */

/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
#include "device_lib.h"
//#include "bms.h"
/** @addtogroup bms bms
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Private Types                                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup bms_Private_Types
  * @{
  */

/**
  * Close the Doxygen bms__Private_Types group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private_Constants                                                           */
/*-----------------------------------------------------------------------------*/
/** @defgroup bms_Private_Constants
  * @{
  */

/**
  * Close the Doxygen bms_Private_Constants group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private Variable Declarations                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup bms_Private_Variables
  * @{
  */

/**
  * Close the Doxygen bms_Private_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private Function Declarations                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup bms_Private_Functions
  * @{
  */

/**
  * Close the Doxygen bms_Private_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup bms_Exported_Variables
  * @{
  */


/**
  * Close the Doxygen bms_Exported_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Function Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup bms_Exported_Functions
  * @{
  */

/*-----------------------------------------------------------------------------*/
/**
 * @fn       void Bms_Data_Init(void)
 * @brief    Initialize non-EEPROM parameters' BMS value
 */
void Bms_Data_Init(void)
{
    uint16_t i;

    /* Initialize the system space to Zeros */
    memset((void *)&System, 0, sizeof(System));
    memset((void *)&Pack, 0, sizeof(Pack));
    memset((void *)&Cell, 0, sizeof(Cell));

    System.DefaultCellAH = CELL_CAP;
    /*System average cell's AH number, unit in mAH. AH number needs to be broadcasted before calibration status*/
    System.State = (NOTCALIBRATED | SYSTEMIDLE| BALPROHIBIT);
    /*D7D6:N/A
    //D5D4: Standard Calibration status. 00:non-calibration status/calibration over. 01:Calibration status
    //Normally, CPU should stop balanced status before calibration over.
    //D3D2: Charging/discharging status 00: non-charging/discharging status-01:charging:10£»discharging-11:N/A
    //This data is written by CPU, notify PACK to adjust sampling rate and alarm model.
    //D1D0: balanced Model 00:non-balanced/balance over 01:initiative balanced 10:passive balanced
    //This data is written by CPU-PACK take action after receiving the data, which default by 00.
    //Under 00 status, CPU is forbidden controlling the balance switch and stop balancing.*/

    /*Pack structure initialization*/
    Pack.Volt = CELL_DISCHARGE_VOLT * CELL_NUM; //3200*16
    Pack.BalanceSwitch = 0x00000000;
    /* Battery pack cells balance switch. "1" stands for balancing/switch on balancing
    "0" stands for non-balancing/switch off balancing
    There are total 16-bit, each bit stands for 1 cell from low to high (1 to 16), default by 00
    In initiative balanced model, CPU will judge if the balancing finish by reading the data. If all 00, then balancing finished.
    In passive balanced model, CPU will write the data to control the balancing switch. Once the balanced model is set to "00", then stop balancing.
    "D16" stands for PACK charging balance switch. "1": balancing "0": stop balancing*/
    //	Pack.PackBalTime = 0x0000;
    /* Battery Pack balancing time, in the unit of seconds, will be added by 1 after starting*/
    Pack.CellState = 0x0000;
    /* Each two-bit stand for cell's status D1D0: Cell0¡­¡­D31D30:Cell16
    Battery pack cell status  00:normal  01:alarm  10:protect*/
    //Pack.CellVolt[CELLNUM]; Not necessary for initializing
    /*Voltage of each cell in battery pack*/
    //Pack.CellTemp[CELLNUM]; Not necessary for initializing
    /*Temperature of each cell in battery pack-there are totally 8 temperature sensors-
    each sensor detects 2 cells temperature (temperature detector is put inside two cells), accurate to 0.1 Celsius degree */
    Pack.CANTimer = 0;
    /* The unit is mS-CAN bus receives overtime timer. System starts timing after being initialized.
    If the value is greater or equal to PackPara.CANOutTime, PACK will shut down automatically, and requires activated by CPU next time.
    Each time system received CAN info, it erases value. Not necessary to shut down if PackPara.CANOutTime=0. */
    Pack.AlarmTimer = 0;
    /*The unit is mS-Alarm status will retransmit to timer. Each time the alert transmit successfully, it then deletes the status.
    If this value is larger or equal to PackPara.AlarmOutTime-then retransmit the alarm status.*/

    for(i=0; i<CELL_NUM; i++) {
        //Cell[i].VoltADresult;
        /*Cell's voltage AD sampling value*/
        Pack.CellVolt[i]=Cell[i].Volt = 3200;
        /*Cell's voltage*/
        //Cell[i].TempADresult;
        /*Cell's temprature AD sampling value in circuit*/
        Pack.CellTemp[i]=Cell[i].Temp = 250;
        //Temprature of cell, accurate to 0.1 Celcius degree
        Cell[i].State = (CELL_NO_BAL | CELL_TEMP_NORMAL| CELL_VOLT_NORMAL);

    }
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn       void Bms_Para_Init(void)
 * @brief    Write the demand EEPROM initialized value into RAM BMS
 */
void Bms_Para_Init(void)
{
    uint16_t i;

    //System space initialized to 0
    memset((void *)&PackPara, 0, sizeof(PackPara));
    memset((void *)&CellPara, 0, sizeof(CellPara));
    memset((void *)&BmsPara, 0, sizeof(BmsPara));

    PackPara.CellNum = CELL_NUM;
    /*Cell number in battery pack (from 1 to 16)*/
    PackPara.BalCurrent = 1000;
    /*The amount of battery pack's constant flow source, unit in mA*/
    PackPara.Cell0VoltCalNum = 10000;//Cell0's voltage calibration value
    PackPara.Cell1VoltCalNum = 10000;//Cell1's voltage calibration value
    PackPara.Cell2VoltCalNum = 10000;//Cell2's voltage calibration value
    PackPara.VoltCalNum = 10000;
    /*Battery pack's voltage sampling calibration value, Actual value = Sampling value * Calibration value /10000,
    Normally this value is not changed.*/
    PackPara.FastVoltCheckTime = 1000;
    /*Voltage rapid detection time, unit in "ms", default by 1000ms.
    If battery is in charging or discharging, then do rapid detection. This parameter can be modified by CPU.
    Normally this parameter is not changed.*/
    PackPara.SlowVoltCheckTime = 5000;
    /*Voltage slow detection time, unit is "ms", default by 5000ms.
    If battery is in non-charging/discharging, then do slow detection. This parameter can be modified by CPU.
    Normally this parameter is not changed.*/
    PackPara.TempCheckTime = 5000;
    /*Temperature detection time, unit is "ms", default by 5000ms. The detection time can be modified by CPU.
    Normally this parameter is not changed.*/
    PackPara.CANOutTime = 120000;
    /*Unit is mS-total transmission time of CAN bus is defaulted by 120s.
    If the time is 0, then no overtime control. If the time exceeds 120s,
    then PACK will turn off automatically (considered as CAN bus error)*/
    PackPara.AlarmOutTime = 5000;
    /* Unit is ms, Alarming for overtime, defaulted by 5000ms = 5s.
    If the time is 0, then upload one alarming status.
    The alarm and protect status will re-upload if it exceeds this time.*/
    PackPara.WorkHour = 0;
    /*Working Hour, unit is "Hour". (32-bit), add 1 when hour increases and write to EEPROM*/
    PackPara.ProductTimeYear = 2010;
    /* Battery pack product time: Year (2-bit), Month (1-bit), Day (1-bit)*/
    PackPara.ProductTimeMonth = 12;
    PackPara.ProductTimeDay = 20;
    PackPara.SeriesNum = 0x01;
    /*Battery pack series number*/
    PackPara.ClientCode = 0x01;
    /*Customer Code*/
    PackPara.SoftVer = 0x02;
    /*Software version number, "1" for first, "2" for second version*/

    for(i=0;  i < CELL_NUM ; i++) {
        //    CellPara[i].RealCapAH = CELL_CAP;
        //    CellPara[i].RealCapWH = (uint32_t)((CELL_CAP / 1000)* CELL_DISCHARGE_VOLT );
        //Actual cell's capacity

        CellPara[i].VoltCalNum = 10000;
        //Cell's voltage calibration value, Actual value = Sampling value * Calibration Value/10000.
        //Normally this value is not changed.*/
        CellPara[i].BalResist = 5000;
        // Balanced resist value, unit is mOhm
    }



    BmsPara.PackOVAlarm = CELL_NUM*3650;
    //Pack over voltage alarm, unit is 1mV. Given signal for alarm, reduce signal required
    BmsPara.PackOVAlarmRecover = CELL_NUM*3600 ;
    //Pack over voltage alarm recovery value
    BmsPara.PackOVProtect = CELL_NUM*3800;
    //Pack over voltage alarm protection value, charger shut down
    BmsPara.PackOVProtectRecover = CELL_NUM*3600 ;
    //Pack over voltage protect recovery value
    BmsPara.PackUVAlarm = CELL_NUM*2400;
    //Pack under voltage alarm value, reduce discharging current required
    BmsPara.PackUVAlarmRecover = CELL_NUM*2600;
    //Pack under voltage alarm recovery value
    BmsPara.PackUVProtect = CELL_NUM*2000;
    //Pack under voltage protection value, shut down discharging circuit required
    BmsPara.PackUVProtectRecover = CELL_NUM*2600;//Once discharging protection open, must be recharged to allow discharging
    //Pack under voltage recovery value
    BmsPara.ChgOCAlarm = ((uint32_t)CELL_CAP*8)/10; //0.8C
    //Charging over current alarm value, reduce charging current required, unit is mA
    BmsPara.ChgOCAlarmRecover = CELL_CAP/2;//0.5C
    //Charging over current alarm recovery value
    BmsPara.ChgOCProtect = CELL_CAP;//1C
    //Charging over current protection value, shut down charger required
    //    BmsPara.ChgOCprotectrecover = ((uint32_t)CELL_CAP*8)/100; //0.8C
    //Charging over current protection recovery value

    BmsPara.DisChgOCAlarm =((uint32_t)CELL_CAP*3)/2;//1.5C
    //Charging over current alarm value, reduce discharging current required, unit is mA
    BmsPara.DisChgOCAlarmRecover =CELL_CAP;//1C
    //Discharging over current alarm recovery value
    BmsPara.DisChgOCProtect =CELL_CAP*2;//2C
    //Discharging over current protect value, shut down electronic equipment required.
    //    BmsPara.DisChgOCprotectrecover =((uint32_t)CELL_CAP*3)/20;//0.5C
    //Discharging over current protect recovery value


    BmsPara.SmallCurChg = CELL_CAP / 100;//    0.1C
    //Small current charging value, unit is mA. Small current required in low temperature, alarm for exceeding this value

    BmsPara.CellOVAlarm = 3650;
    //Cell over voltage alarm value, unit is mV, Alarm signal given, reduce charging current 3.60 required
    BmsPara.CellOVAlarmRecover = 3600;
    //Cell over voltage alarm recovery value 3.55
    BmsPara.CellOVProtect = 3800;
    //Cell over voltage alarm protection value, shut down charger when over voltage
    BmsPara.CellOVProtectRecover = 3600;
    //Cell over voltage protection recovery value
    BmsPara.CellUVAlarm = 2400;
    //Cell under voltage alarm value, reduce discharging current required
    BmsPara.CellUVAlarmRecover = 2600;
    //Cell under voltage alarm recovery value
    BmsPara.CellUVProtect = 2000;
    //Cell under voltage protection value, shut down discharging circuit required
    BmsPara.CellUVProtectRecover = 2600;//Once discharging protection starts, requires re-charging to allow discharging
    //Cell under voltage protection recovery value

    BmsPara.PackBalStartVoltDif = 50;//
    //PACK balance start voltage difference
    BmsPara.PackBalStopVoltDif = 25;
    //PACK balance stop voltage difference
    //PACK balance controls the difference within PackBalStopVoltDif
    //between the lowest voltage in the PACK with the highest of other
    //PACK's lowest voltage
    //

    BmsPara.CellBalStartVoltDif = 50;
    //Cell's balance start voltage difference
    BmsPara.CellBalStopVoltDif =  25;
    //Cell's balance stop voltage difference
    //Cell's balance controls cell voltage difference under CellBalStopVoltDif
    //Criterion is the lowest voltage of the cell in PACK
    //There are charging balance, discharging balance and rest for the balance,
    //so controller command is required for scheduling start time



    BmsPara.CellDischgOTAlarm = 750;
    //Cell's discharging over temperature alarm    value
    BmsPara.CellDischgOTAlarmRecover = 700;
    //Cell's discharging over temperature alarm recovery value
    BmsPara.CellDischgOTProtect = 800;
    //Cell's discharging over temperature protection value
    BmsPara.CellDischgOTProtectRecover = 700;
    //Cell's discharging over temperature protection recovery value

    BmsPara.CellDischgUTAlarm = -150;
    //Cell's discharging low temperature alarm value
    BmsPara.CellDischgUTAlarmRecover = -100;
    //Cell's discharging low temperature alarming recovery value
    BmsPara.CellDischgUTProtect = -200;
    //Cell's discharging low temperature protection value
    BmsPara.CellDischgUTProtectRecover = -100;
    //Cell's discharging low temperature protection recovery value

    BmsPara.CellOTAlarm = 600;
    //Cell's over temperature alarm value
    BmsPara.CellOTAlarmRecover = 550;
    //Cell's over temperature alarm recovery value
    BmsPara.CellOTProtect = 650;
    //Cell's over temperature protection value
    BmsPara.CellOTProtectRecover = 550;
    //Cell's over temperature protection recovery value

    BmsPara.CellUTAlarm = -100;
    //Cell's under temperature alarm value
    BmsPara.CellUTAlarmRecover = -50;
    //Cell's under temperature alarm recovery value
    BmsPara.CellUTProtect = -150;
    //Cell's under temperature protection value
    BmsPara.CellUTProtectRecover = -50;
    //Cell's under protection recovery value

    BmsPara.CellChgOTAlarm = 600;
    //Cell's charging over temperature alarm value
    BmsPara.CellChgOTAlarmRecover = 550;
    //Cell's charging over temperature alarm recovery value
    BmsPara.CellChgOTProtect = 650;
    //Cell's charging over temperature protection value
    BmsPara.CellChgOTProtectRecover = 550;
    //Cell's charging over temperature protection recovery value

    BmsPara.CellChgUTAlarm = 0;
    //Cell's charging under temperature protective entrance value
    BmsPara.CellChgUTAlarmRecover = 50;
    //Cell's charging under temperature protective exit value
    //Cell's Trickle charge temperature exit value
    BmsPara.CellChgUTProtect = -50;
    //Cell's charging under temperature protective value
    BmsPara.CellChgUTProtectRecover = 50;
    //Cell charging under temperature protective recovery value

    BmsPara.CellChgSmallCurTemp = 50;
    //Cell's charging small current temperature entrance value
    BmsPara.CellChgSmallCurTempRecover = 100;

    // Cell's charging small current temperature exit value
    // Self_ID = GET_Device_ID();    // Add software support the device ID change 2011/May/18
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn       uint32_t Para_CRC_Verify(void)
 * @brief    Verify BMS parameters, including PackPara,CellPara,BmsPara
 * @return   CRC value after calculation
 */
uint32_t Para_CRC_Verify(void)
{
    uint16_t i;
    uint32_t *pData;
    CRC->CR = 0x01;//Reset CRC calculator

    pData = (uint32_t *)(&PackPara);     // Get PackPara address
    for(i=0; i<(sizeof(PackPara)/4)-4; i++) //Calculate PackPara, but not working hour, production time,
        //software version and customer code
        CRC->DR = *(pData + i);

    pData = (uint32_t *)(CellPara);
    for(i=0; i<(sizeof(CellPara)/4); i++) //Get CellPara address
        CRC->DR = *(pData + i);    // Calculating

    pData = (uint32_t *)(&BmsPara);
    for(i=0; i<(sizeof(BmsPara)/4); i++) //Get BmsPara address
        CRC->DR = *(pData + i);    // Calculating

    return(CRC->DR);
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn       void SaveAllEEPromData(void)
 * @brief    Save all EEPROM data
 */
void SaveAllEEPromData(void)
{
    uint8_t Verify[EE_VERIFY_NUM];
    uint16_t i;
    uint32_t CRCResult;

    I2C_EE_BufferWrite((uint8_t *)(&PackPara),EE_PACKPARA_ADDR,sizeof(PackPara));
    I2C_EE_BufferWrite((uint8_t *)(&CellPara),EE_CELLPARA_ADDR,sizeof(CellPara));
    I2C_EE_BufferWrite((uint8_t *)(&BmsPara),EE_BMSPARA_ADDR,sizeof(BmsPara));
    for(i=0; i<EE_VERIFY_NUM; i++) {
        Verify[i]=EE_SELF_WRITE;// Self-write
    }
    I2C_EE_BufferWrite(Verify,EE_VERIFY_ADDR,EE_VERIFY_NUM);

    CRCResult = Para_CRC_Verify();// Calculating CRC value
    I2C_EE_BufferWrite((uint8_t*)(&CRCResult),EE_CRC_VERIFY_ADDR,4);
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn       void Bms_Initialize(void)
 * @brief    Initialize BMS parameter to see if it is the first time start, and if read data from EEPROM
 */
void Bms_Initialize(void)
{
    uint8_t Verify[EE_VERIFY_NUM];
    volatile uint16_t i;
    uint32_t CRCResult;
    //    uint32_t EE_CRC;
    uint32_t ID;
    BUFFER volatile *txbuf;

    //    uint8_t Temp[16];

    MESSAGE EEPROMErrorMessage;


    I2C_EE_BufferRead(Verify,EE_VERIFY_ADDR,EE_VERIFY_NUM);//read verification number to see if it is the first time start
    for(i=0; i<EE_VERIFY_NUM; i++) {
        if((Verify[i]!= EE_SELF_WRITE)&&
                (Verify[i]!= EE_CALED) )// If one number is not through self-write or calibrated, then it is the first time start.
            break;
    }
    if((i != EE_VERIFY_NUM) || (GET_Device_ID() == 0)) { //if first start and 0 address , initialize the data
        Bms_Para_Init(); //initialize parameters to original values
        SaveAllEEPromData();
    }
    else if(i == EE_VERIFY_NUM) {   //Not first time start
        I2C_EE_BufferRead((uint8_t *)(&PackPara),EE_PACKPARA_ADDR,sizeof(PackPara));
        I2C_EE_BufferRead((uint8_t *)(&CellPara),EE_CELLPARA_ADDR,sizeof(CellPara));
        I2C_EE_BufferRead((uint8_t *)(&BmsPara),EE_BMSPARA_ADDR,sizeof(BmsPara));
        I2C_EE_BufferRead((uint8_t*)(&CRCResult),EE_CRC_VERIFY_ADDR,sizeof(CRCResult));
        if (CRCResult != Para_CRC_Verify()) { //Calculate and compare CRC
            //If EEPROM data is broken-send alarm information, require upper computer to configure,
            //upload the error CRC value in data bit
            EEPROMErrorMessage.ReserveField = 0x00;        //reserved filed
            EEPROMErrorMessage.MessageType = CAN_MSGTYPE_ALARM;      //Message Type
            EEPROMErrorMessage.DeviceType = ID_PACK;       //Device Type
            EEPROMErrorMessage.DeviceID = GET_Device_ID(); //Device ID
            EEPROMErrorMessage.MessageID = 0xfe;           //Message ID
            EEPROMErrorMessage.MessageNum = 4;             //Message Number

            EEPROMErrorMessage.Data[0]=(uint8_t)(CRCResult >>0 );
            EEPROMErrorMessage.Data[1]=(uint8_t)(CRCResult >>8 );
            EEPROMErrorMessage.Data[2]=(uint8_t)(CRCResult >>16 );
            EEPROMErrorMessage.Data[3]=(uint8_t)(CRCResult >>24 );

            ID = AssembleID(&EEPROMErrorMessage);
            txbuf = fifo_alloc(&can_tx_fifo);
            if (txbuf) {
                txbuf->ID = ID;

                txbuf->DataL = CRCResult;
                txbuf->DLC = 4;
            }
            //            USART_Load_Buffer(&EEPROMErrorMessage);//USART sends alarm information
        }
    }

    Bms_Data_Init();//Initialize non-EEPROM parameter's BMS data
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn       void Update_CanContent(void)
 * @brief    Update the voltage,temperature,SOC (AVE,MAX,MIN.High,Low)
 */
void Update_CanContent(void)
{
    uint16_t i;
    int16_t Temp;

    PackPara.HighCellVoltage = 0;
    PackPara.AveCellVoltage    = 0 ;
    PackPara.LowCellVoltage = CELL_VOLT_MAX; //4000 mV

    PackPara.HighCellTemp = -1000;      //-100.0 Celsius degree
    PackPara.AveCellTemp    = 0 ;
    PackPara.LowCellTemp = 1500 ;      //150.0 Celsius degrees

    PackPara.HighCellSOC = -1000;      //-100.0%
    PackPara.AveCellSOC    = 0 ;
    PackPara.LowCellSOC = 1000 ;      //100.0%

    for(i=0 ; i<CELL_NUM; i++) {
        // Update the State of Charge
        CellPara[i].CellAHTotalCounter = PackPara.AHMAHCounter + CellPara[i].CellBalAHCounter;//balance is positive value to AH,sam gao @May 15,2012
        CellPara[i].CellWHTotalCounter = PackPara.AHMWHCounter + CellPara[i].CellBalWHCounter;

        if (PackPara.HighCellVoltage < Cell[i].Volt) {
            PackPara.HighCellVoltage = Cell[i].Volt;
        }
        if (PackPara.LowCellVoltage > Cell[i].Volt) {
            PackPara.LowCellVoltage = Cell[i].Volt;
        }

        PackPara.AveCellVoltage += Cell[i].Volt;


        if (PackPara.HighCellTemp < Cell[i].Temp) {
            PackPara.HighCellTemp = Cell[i].Temp;
        }
        if (PackPara.LowCellTemp > Cell[i].Temp) {
            PackPara.LowCellTemp = Cell[i].Temp;
        }
        PackPara.AveCellTemp += Cell[i].Temp ;

        if (System.DefaultCellAH != 0)
            Temp = (CellPara[i].CellAHTotalCounter*1000) / PackPara.PackAHCapacity  ;
        //  change from System DefaultCellAH to PackPara.PackAHCapacity
        // ??overflow notice ?? check overflow later?? unit
        else
            Temp = 0;

        Pack.CellSoc[i] = Temp;
        if (PackPara.HighCellSOC < Temp) {
            PackPara.HighCellSOC = Temp;
        }
        if (PackPara.LowCellSOC > Temp) {
            PackPara.LowCellSOC = Temp;
        }
        PackPara.AveCellSOC += Temp ;
    }

    PackPara.AveCellSOC/=CELL_NUM;
    PackPara.AveCellVoltage/=CELL_NUM;
    PackPara.AveCellTemp /= CELL_NUM;

    PackPara.BPVoltage = Pack.Volt;
    PackPara.BPTemp = PackPara.HighCellTemp;
    PackPara.BPSOC = PackPara.LowCellSOC;
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn       void Bat_Manage(void)
 * @brief    manage battery, balancing process, alarm process
 */
void Bat_Manage(void)
{
    Alarm_Manage();// Alarm processing after voltage or temperature updated
    Update_CanContent();
    Bat_Bal();
    Bat_Capacity_update();
    // Bat_Cal();
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn       void Alarm_Manage(void)
 * @brief    Alarm management for cell's status
 */
void Alarm_Manage(void)
{
    static uint8_t OldState[CELL_NUM]= {0};
    uint8_t i;
    MESSAGE AlarmMessage;
    uint32_t ID;
    BUFFER volatile *txbuf;

    for(i=0 ; i<CELL_NUM; i++) {
        /*********************Over/Under voltage Alarm management program*******************/
        /*********************First, decide if it is over voltage or not *******************/
        if((Cell[i].Volt <= BmsPara.CellOVAlarmRecover)&& //If voltage is in normal range
                (Cell[i].Volt >= BmsPara.CellUVAlarmRecover)) {
            Cell[i].State = ((Cell[i].State&0xF8)|CELL_VOLT_NORMAL);// Normal status
            Pack.CellState &= (~(uint32_t)(0x11 << (i * 2)));
            Pack.CellState |= (uint32_t)(NORMAL << (i * 2));
        }
        else if(Cell[i].Volt >= BmsPara.CellOVProtect) {   // larger or equal to protective voltage
            Cell[i].State = ((Cell[i].State&0xF8)|CELL_OV_PROTECT);
            Pack.CellState &= (~(uint32_t)(0x11 << (i * 2)));
            Pack.CellState |= (uint32_t)(PROTECT << (i * 2));
        }
        else if((Cell[i].Volt < BmsPara.CellOTProtect)&&
                (Cell[i].Volt >= BmsPara.CellOTAlarm)) { //Smaller than protective voltage, larger or equal to alarm voltage
            if((Cell[i].State&0x07) == CELL_OV_PROTECT) {
                ; //Keep protection if state is in protective status
            }
            else {
                //else goto alarm status
                Cell[i].State = ((Cell[i].State&0xF8)|CELL_OT_ALARM);
                Pack.CellState &= (~(uint32_t)(0x11 << (i * 2)));
                Pack.CellState |= (uint32_t)(ALARM << (i * 2));
            }
        }
        /*********************Decide if it is undervoltage*******************/
        else if(Cell[i].Volt <= BmsPara.CellUVProtect) { //less than or equal to protective voltage
            Cell[i].State = (Cell[i].State&0xF8)|CELL_UV_PROTECT;
            Pack.CellState &= (~(uint32_t)(0x11 << (i * 2)));
            Pack.CellState |= (uint32_t)(PROTECT << (i * 2));
        }
        else if((Cell[i].Volt > BmsPara.CellUVProtect)&&
                (Cell[i].Volt <= BmsPara.CellUVAlarm)) { //If bigger than protection voltage, less or equal to alarm voltage
            if((Cell[i].State&0x07) == CELL_UV_PROTECT) {
                ;//Keep protection if status is in protective status;
            }
            else {   //else goto alarm status
                Cell[i].State = ((Cell[i].State&0x07)|CELL_UV_ALARM);
                Pack.CellState &= (~(uint32_t)(0x11 << (i * 2)));
                Pack.CellState |= (uint32_t)(ALARM << (i * 2));
            }
        }
        else
            ;


        /*********************High/low temperature alarm management program*******************/
        if((System.State&0x0C) == SYSTEMIDLE) { // Idle status
            if((Cell[i].Temp <= BmsPara.CellOTAlarmRecover)&& // If temperature is in normal range
                    (Cell[i].Temp >= BmsPara.CellUTAlarmRecover)) {
                Cell[i].State = ((Cell[i].State&0xC7)|CELL_TEMP_NORMAL);
                Pack.CellState &= (~(uint32_t)(0x03 << (i * 2)));
                Pack.CellState |= (uint32_t)(NORMAL << (i * 2));
            }
            else if(Cell[i].Temp >= BmsPara.CellOTProtect) {   //larger or equal to temperature
                Cell[i].State = ((Cell[i].State&0xC7)|CELL_OT_PROTECT);
                Pack.CellState &= (~(uint32_t)(0x03 << (i * 2)));
                Pack.CellState |= (uint32_t)(PROTECT << (i * 2));
            }
            else if((Cell[i].Temp < BmsPara.CellOTProtect)&&
                    (Cell[i].Temp >= BmsPara.CellOTAlarm)) { //smaller than protective temperature, larger or equal to alarm temperature
                if((Cell[i].State&0x31) == CELL_OT_PROTECT) {
                    ; //Keep protection if status is in protective state
                }
                else {   //else goto alarm status
                    Cell[i].State = ((Cell[i].State&0xC7)|CELL_OT_ALARM);
                    Pack.CellState &= (~(uint32_t)(0x03 << (i * 2)));
                    Pack.CellState |= (uint32_t)(ALARM << (i * 2));
                }
            }
            /*********************Decide if it is under temperature*******************/
            else if(Cell[i].Temp <= BmsPara.CellUTProtect) { //less or equal to protective temperature
                Cell[i].State = ((Cell[i].State&0xC7)|CELL_UT_PROTECT);
                Pack.CellState &= (~((uint32_t)(0x03) << (i * 2)));
                Pack.CellState |= ((uint32_t)(PROTECT) << (i * 2));
            }
            else if((Cell[i].Temp > BmsPara.CellUTProtect)&&
                    (Cell[i].Temp <= BmsPara.CellUTAlarm)) { //larger than protective temperature-less or equal to alarm temperature
                if((Cell[i].State&0x31) == CELL_UT_PROTECT) {
                    ; // Keep protection if state in protective state
                }
                else {   //else goto alarm state
                    Cell[i].State = ((Cell[i].State&0xC7)|CELL_UT_ALARM);
                    Pack.CellState &= (~(uint32_t)(0x03 << (i * 2)));
                    Pack.CellState |= (uint32_t)(ALARM << (i * 2));
                }
            }
        }
        else if((System.State&0x0C) == CHARGING) {   //Charging status
            if((Cell[i].Temp <= BmsPara.CellChgOTAlarmRecover)&& //Decide if temperature is in normal range
                    (Cell[i].Temp >= BmsPara.CellChgUTAlarmRecover)) {
                Cell[i].State = ((Cell[i].State&0xC7)|CELL_TEMP_NORMAL);// Normal
                Pack.CellState &= (~(uint32_t)(0x03 << (i * 2)));
                Pack.CellState |= (uint32_t)(NORMAL << (i * 2));
            }
            else if(Cell[i].Temp >= BmsPara.CellChgOTProtect) {   // Larger or equal to protective temperature
                Cell[i].State = ((Cell[i].State&0xC7)|CELL_OT_PROTECT);
                Pack.CellState &= (~(uint32_t)(0x03 << (i * 2)));
                Pack.CellState |= (uint32_t)(PROTECT << (i * 2));
            }
            else if((Cell[i].Temp < BmsPara.CellChgOTProtect)&&
                    (Cell[i].Temp >= BmsPara.CellChgOTAlarm)) { // Less or equal to protective temperature, larger or equal to alarm temperature
                if((Cell[i].State&0x31) == CELL_OT_PROTECT) {
                    ; // Keep protection if state in protective state
                }
                else {   //else goto alarm status
                    Cell[i].State = ((Cell[i].State&0xC7)|CELL_OT_ALARM);
                    Pack.CellState &= (~(uint32_t)(0x03 << (i * 2)));
                    Pack.CellState |= (uint32_t)(ALARM << (i * 2));
                }
            }
            /*********************Decide if it is low temperature*******************/
            else if(Cell[i].Temp <= BmsPara.CellChgUTProtect) { // Less or equal to protection temperature
                Cell[i].State = ((Cell[i].State&0xC7)|CELL_UT_PROTECT);
                Pack.CellState &= (~(uint32_t)(0x03 << (i * 2)));
                Pack.CellState |= (uint32_t)(PROTECT << (i * 2));
            }
            else if((Cell[i].Temp > BmsPara.CellChgUTProtect)&&
                    (Cell[i].Temp <= BmsPara.CellChgUTAlarm)) { //larger than protective temperature, less or equal to alarm temperature
                if((Cell[i].State&0x31) == CELL_UT_PROTECT) {
                    ; //if state is in protect status, keep the status
                }
                else {   //else goto alarm status
                    Cell[i].State = ((Cell[i].State&0xC7)|CELL_UT_ALARM);
                    Pack.CellState &= (~(uint32_t)(0x03 << (i * 2)));
                    Pack.CellState |= (uint32_t)(ALARM << (i * 2));
                }
            }
            else;
        }
        else if((System.State&0x0C) == DISCHARGING) {   // Discharging status
            if((Cell[i].Temp <= BmsPara.CellDischgOTAlarmRecover)&& // If temperature is in normal range
                    (Cell[i].Temp >= BmsPara.CellDischgUTAlarmRecover)) {
                Cell[i].State = ((Cell[i].State&0xC7)|CELL_TEMP_NORMAL);//Normal
                Pack.CellState &= (~(uint32_t)(0x03 << (i * 2)));
                Pack.CellState |= (uint32_t)(NORMAL << (i * 2));
            }
            else if(Cell[i].Temp >= BmsPara.CellDischgOTProtect) {   // If temperature is larger or equal to protective temperature
                Cell[i].State = ((Cell[i].State&0xC7)|CELL_OT_PROTECT);
                Pack.CellState &= (~(uint32_t)(0x03 << (i * 2)));
                Pack.CellState |= (uint32_t)(PROTECT << (i * 2));
            }
            else if((Cell[i].Temp < BmsPara.CellDischgOTProtect)&&
                    (Cell[i].Temp >= BmsPara.CellDischgOTAlarm)) { //Less than protective temperature and larger than alarm temperature
                if((Cell[i].State&0x31) == CELL_OT_PROTECT) {
                    ; //If state is in protect status, keep it
                }
                else {   // else goto alarm state.
                    Cell[i].State = ((Cell[i].State&0xC7)|CELL_OT_ALARM);
                    Pack.CellState &= (~(uint32_t)(0x03 << (i * 2)));
                    Pack.CellState |= (uint32_t)(ALARM << (i * 2));
                }
            }
            /*********************Decide if state is in low temperature*******************/
            else if(Cell[i].Temp <= BmsPara.CellDischgUTProtect) { //Less or equal to protective temperature
                Cell[i].State = ((Cell[i].State&0xC7)|CELL_UT_PROTECT);
                Pack.CellState &= (~(uint32_t)(0x03 << (i * 2)));
                Pack.CellState |= (uint32_t)(PROTECT << (i * 2));
            }
            else if((Cell[i].Temp > BmsPara.CellDischgUTProtect)&&
                    (Cell[i].Temp <= BmsPara.CellDischgUTAlarm)) { //Larger than protective temperature, and less or equal to alarm temperature
                if((Cell[i].State&0x31) == CELL_UT_PROTECT) {
                    ; //If state is in protect state, keep it.
                }
                else {   //else goto alarm state
                    Cell[i].State = ((Cell[i].State&0xC7)|CELL_UT_ALARM);
                    Pack.CellState &= (~(uint32_t)(0x03 << (i * 2)));
                    Pack.CellState |= (uint32_t)(ALARM << (i * 2));
                }
            }
            else
                ;
        }
        else
            ;

        if((Cell[i].State&0x3f)==(CELL_TEMP_NORMAL|CELL_VOLT_NORMAL)) {
            ; //If all in normal state
        }
        else {
            if((Cell[i].State&0x3f)!=OldState[i]) { // If new state is not equal to old state, then there is error, or error state changed
                Cell[i].AlarmTimer = PackPara.AlarmOutTime;//set once for alarm is out of time
            }
            if(PackPara.AlarmOutTime != 0) {
                if(Cell[i].AlarmTimer >= PackPara.AlarmOutTime) {
                    AlarmMessage.ReserveField = 0x00;    //reserve field
                    AlarmMessage.MessageType = CAN_MSGTYPE_ALARM;//message type
                    AlarmMessage.DeviceType = ID_PACK;    //device type
                    AlarmMessage.DeviceID = Self_ID;    //device ID
                    AlarmMessage.MessageID = 0x30 + (i*4 + 0);
                    AlarmMessage.MessageNum = 5;
                    AlarmMessage.Data[0] =     (uint8_t)(Cell[i].Volt);
                    AlarmMessage.Data[1] = (uint8_t)(Cell[i].Volt >> 8);
                    AlarmMessage.Data[2] =     (uint8_t)(Cell[i].Temp);
                    AlarmMessage.Data[3] = (uint8_t)(Cell[i].Temp >> 8);
                    AlarmMessage.Data[4] =     Cell[i].State;

                    ID = AssembleID(&AlarmMessage);
                    txbuf = fifo_alloc(&can_tx_fifo);
                    if (txbuf) {
                        txbuf->ID = ID;

                        txbuf->DataL = (uint32_t)(Cell[i].Temp) <<16;
                        txbuf->DataL += Cell[i].Volt;
                        txbuf->DataH = Cell[i].State;

                        txbuf->DLC = 5;//2-bit voltage, 2-bit temperature, 1-bit status
                    }
//                    USART_Load_Buffer(&AlarmMessage);//USART send alarm message

                    Cell[i].AlarmTimer = 0;//clear the alarm timer
                }
            }
        }
        OldState[i]=Cell[i].State&0x3f;    // State update
    }
}

void Open_BalanceSwitch(void)
{
    uint8_t i;
    for(i=0; i<CELL_NUM; i++) {
        Cell[i].State &= 0xbf; // 1011 1111
        Cell[i].State |=CELL_NO_BAL;// reset Balance status
        Pack.BalanceSwitch &= ~(1<<i);// reset balance switch
    }
    Pack.BalanceSwitch &= ~(1<<ACBALSWITCHBIT);
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn       void Bat_Bal(void)
 * @brief    Process cell's balance status
 */
void Bat_Bal(void)
{

    switch (System.State & (AUTOBAL|MANUALBAL)) {
        case  AUTOBAL: {
                switch (System.State & (CHARGING|DISCHARGING)) {
                    case CHARGING:
                        Auto_Balance_Charging();
                        break;
                    case DISCHARGING:
                        Auto_Balance_Discharging();
                        break;
                    default:
                        Auto_Balance_SystemIdle();
                        break;
                }
                break;
            }

        case  MANUALBAL:
            Manual_Balance();
            break;

        default:
            Open_BalanceSwitch();
            break;
    }

    Bal_Switch(Pack.BalanceSwitch);
    OldState = System.State;
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn       void Manual_Balance(void)
 * @brief    Manually process cell balancing
 */
void Manual_Balance(void)
{
    // bug - unbalanced parens make inner expr always false, then negates to always
    // open balance switch pjn 2014-2-19
    if (!(((System.State&0x03) == MANUALBAL) && ((OldState&0x03) == MANUALBAL)))
        Open_BalanceSwitch();
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn       void Auto_Balance_Charging(void)
 * @brief    While charging, auto process the cell's balance state
 */
void Auto_Balance_Charging(void)
{
    uint16_t i;
    //Turn-on BP Balancing Power Supply if the BPTargetVoltage 30mV higher than low cell voltage
    if (
        (PackPara.BPTargetVoltage > (PackPara.LowCellVoltage + 30))||
        (PackPara.BPTargetVoltage > (PackPara.AveCellVoltage + 25))
    ) {
        Pack.BalanceSwitch |=(1<<ACBALSWITCHBIT);
    }
    //Close Balancing Resistor for any cell having a voltage 30mV higher than the BPTargetVoltage
    for(i=0; i<CELL_NUM; i++) {
        if (Cell[i].Volt > (PackPara.BPTargetVoltage + 30)) {
            Pack.BalanceSwitch |= (1<<i);   //close cell balancing resistor switches;
        }
    }

    //Open Balancing Resistor for any cell having a voltage within 25mV of the BPTargetVoltage.
    for(i=0; i<CELL_NUM; i++) {
        if (Cell[i].Volt  < (PackPara.BPTargetVoltage+25)) {
            Pack.BalanceSwitch &= (~(1<<i));   //open cell balancing resistor switches;
        }
    }
    //Turn-off BP Balancing Power Supply if low cell voltage is within 25mV of the BPTargetVoltage.
    if (
        (PackPara.LowCellVoltage > (PackPara.BPTargetVoltage - 25))&&
        (PackPara.AveCellVoltage > (PackPara.BPTargetVoltage - 20))
    ) {
        Pack.BalanceSwitch &= (~(1<<ACBALSWITCHBIT));   //open power supply switch;
    }
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn       void Auto_Balance_SystemIdle(void)
 * @brief    Idle, then auto process the balance state£¨copy from Auto_Balance_Charging(), remove all SOC related code
 */
void Auto_Balance_SystemIdle(void)
{
    uint16_t i;
    //Turn-on BP Balancing Power Supply if the BPTargetVoltage 15mV higher than low cell voltage
    if ( (PackPara.BPTargetVoltage > (PackPara.LowCellVoltage+15)) ) {
        Pack.BalanceSwitch |=(1<<ACBALSWITCHBIT);
    }

    //Close Balancing Resistor for any cell having a voltage 15mV higher than the BPTargetVoltage
    for(i=0; i<CELL_NUM; i++) {
        if (Cell[i].Volt > (PackPara.BPTargetVoltage + 15)) {
            Pack.BalanceSwitch |= (1<<i);   //close cell balancing resistor switches;
        }
    }

    //Open Balancing Resistor for any cell having a voltage within 10mV of the BPTargetVoltage.
    for(i=0; i<CELL_NUM; i++) {
        if (Cell[i].Volt  < (PackPara.BPTargetVoltage+10)) {
            Pack.BalanceSwitch &= (~(1<<i));   //open cell balancing resistor switches;
        }
    }
    //Turn-off BP Balancing Power Supply if low cell voltage is within 10mV of the BPTargetVoltage.
    if (PackPara.LowCellVoltage > (PackPara.BPTargetVoltage - 10)) {
        Pack.BalanceSwitch &= (~(1<<ACBALSWITCHBIT));   //open power supply switch;
    }
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn       void Auto_Balance_Discharging(void)
 * @brief    Discharging, auto-process the cell's balance state£¨copy from Auto_Balance_Charging(), remove all SOC related code
 */
void Auto_Balance_Discharging(void)
{
    uint16_t i;
    //Turn-on BP Balancing Power Supply if the BPTargetVoltage is 30mV higher than low cell voltage
    if (
        (PackPara.BPTargetVoltage > (PackPara.LowCellVoltage+30))
    ) {
        Pack.BalanceSwitch |=(1<<ACBALSWITCHBIT);
    }
    //Close Balancing Resistor for any cell having a voltage 30mV higher than the BPTargetVoltage
    for(i=0; i<CELL_NUM; i++) {
        if (Cell[i].Volt > (PackPara.BPTargetVoltage + 30)) {
            Pack.BalanceSwitch |= (1<<i);   //close cell balancing resistor switches;
        }
    }

    //Open Balancing Resistor for any cell having a voltage within 25mV of the BPTargetVoltage.
    for(i=0; i<CELL_NUM; i++) {
        if (Cell[i].Volt  < (PackPara.BPTargetVoltage+25)) {
            Pack.BalanceSwitch &= (~(1<<i));   //open cell balancing resistor switches;
        }
    }
    //Turn-off BP Balancing Power Supply if low cell voltage is within 25mV of the BPTargetVoltage.
    if (PackPara.LowCellVoltage > (PackPara.BPTargetVoltage - 25)) {
        Pack.BalanceSwitch &= (~(1<<ACBALSWITCHBIT));   //open power supply switch;
    }
}


/*-----------------------------------------------------------------------------*/
/**
 * @fn       void Bat_Capacity_update(void)
 * @brief    Calculate and update balance capacitor
 */
void Bat_Capacity_update(void)
{

    uint8_t i=0;
    int32_t temp;

    if(Pack.BalMMTimer >=1000) {
        // Balance time reaches 1s
        temp = Pack.BalMMTimer;
        Pack.BalMMTimer = 0;//Reset Balance timer. Temperature - 1000 means that it will drift for every ms it takes longer for each time.
        //temp - 1000;

        for(i=0; i<CELL_NUM; i++) {
            Cell[i].BalTime++; //balance time+1

            if( Pack.BalanceSwitch & (0x01<<i)) { // Discharging balance
                Cell[i].BalAHTemp -= Cell[i].Volt*1000/4200; //CellPara[i].BalResist; change to a const temporarily Sam@2012,May 5
                //I=V/R
                Cell[i].BalWHTemp -= Cell[i].Volt*Cell[i].Volt/4200; //CellPara[i].BalResist;change to a const temporarily Sam@2012,May 5
                //P=V*V/R

            }

            if((Pack.BalanceSwitch & (1<<ACBALSWITCHBIT)) != 0) { //Charging balance
                Cell[i].BalAHTemp += PackPara.BalCurrent;
                //I=V/R
                Cell[i].BalWHTemp += Cell[i].Volt*PackPara.BalCurrent/1000;
                //P=V*V/R
            }

            if((Cell[i].BalTime >= 10)&&(Cell[i].BalTime % 10 ==0)) { // 10 second

                temp=    Cell[i].BalAHTemp;
                CellPara[i].CellBalAHCounter += temp/3600;
                Cell[i].BalAHTemp = temp%3600;

                temp=    Cell[i].BalWHTemp;
                CellPara[i].CellBalWHCounter += temp/3600;
                Cell[i].BalWHTemp = temp%3600;
            }
        }
    }
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn       void Bat_Cal(void)
 * @brief    Calibrate the Battery
 */
void Bat_Cal(void)
{
    static uint8_t OldState = 0;
    uint8_t i;
    if(((OldState & 0xef)==NOTCALIBRATED) &&
            ((System.State &0xef)==CALIBRATED)) {
        for(i=0; i<CELL_NUM; i++) {
            //CellPara[i].RealCapAH = System.CellAH + Cell[i].BalAH; //System.CellAH is AHM AH counter value; Need to add Cell AH counter
            //CellPara[i].RealCapWH =    System.CellWH + Cell[i].BalWH; //System.CellWH is AHM WH counter value; Need to add Cell WH counter
            //I2C_EE_BufferWrite((uint8_t *)(&CellPara[i].RealCapAH),EE_CELLPARA_ADDR+(i*(sizeof(CELLPARA))),8);
        }
    }
}

/**
  * Close the Doxygen bms_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen bms group.
  *    @}
*/

/* End of bms.c */
