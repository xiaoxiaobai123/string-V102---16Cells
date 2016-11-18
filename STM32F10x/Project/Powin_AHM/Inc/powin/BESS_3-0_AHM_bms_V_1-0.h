/**
  ******************************************************************************
  * @file        bms.h
  * @author      Nystrom Engineering
  * @version     2.0.0
  * @date        2014-01-27
  * @copyright   Powin Engineering
  * @brief       The include file for Battery Management System functions.
  ******************************************************************************
  */

/*-----------------------------------------------------------------------------*/
/* Define to prevent recursive inclusion                                       */
/*-----------------------------------------------------------------------------*/
#ifndef BMS_H_
#define BMS_H_

/*-----------------------------------------------------------------------------*/
/* When using C++ compiler, make sure that all definitions have a C binding.   */
/*-----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif

/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
#include "device_lib.h"

/** @addtogroup bms bms
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Exported Types                                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup bms_Exported_Types
  * @{
  */

/*=============================================================================*/
/*
 * SYSTEM parameter structure
This structure has only one entity in SystemPara,
used for saving system value, operation on the PACK.
The CPU broadcasts this data
*/
#if(1)//by sam
typedef struct{
        // msgid 0x01
        uint16_t reserved01; /**< reserved01 */
        uint16_t reserved02; /**< reserved02 */
        uint16_t reserved03; /**< reserved03 */
        uint16_t reserved04; /**< reserved04 */

        // msgid 0x02
        int32_t DefaultCellAH; /**< System cell's AH number, unit is mAH. System should broadcast this value before calibration finished. */
        int32_t reserved11;    /**< reserved11 */
        // msgid 0x03
        uint8_t State; /**< State
             - D7D6D5: None Sense
             - D4:     Calibration state 
	                   0: non-Calibration state/Calibration done 
			           1: Calibration state Normally, CPU should stop balance state before calibration done.
             - D3D2:   charging/discharging states 
	                   00:non-charging/non-discharging 
			           01:charging, 
			           10:discharging, 
			           11:N/A
                       This data is written by CPU, to inform PACK adjusting sampling rate and alarm type
             - D1D0:   balance model 
	                   00:non-balancing/finishing balance 
			           01:initiative balance 
			           10:passive balance
			           11:NA?
                       This data is writen by CPU, take actions after receiving PACK, default 00. 
			           In "00" state, not allowed for CPU to control balance switch, and stop balancing
        */
        volatile uint8_t ee_state; /**< ee_state - meaningless, as far as I can tell - pjn 2014-3-3 */
        volatile uint16_t Reserved2;  /**< Reserved2 */
        volatile uint32_t Reserved3;  /**< Reserved3 */
        /**************CAN ID above is 0x03*******************/

        /**************CAN ID above is 0x04*******************/

        /**************CAN ID above is 0x05*******************/

        /**************CAN ID above is 0x06*******************/

        /*************CAN ID 0x07 reserved*******************/
//	uint8_t array[24]; /** Note : the struct will be padded to 24 bytes */
} SYSTEM_t;
#else
typedef union {
	struct {
        // msgid 0x01
        uint16_t reserved01; /**< reserved01 */
        uint16_t reserved02; /**< reserved02 */
        uint16_t reserved03; /**< reserved03 */
        uint16_t reserved04; /**< reserved04 */

        // msgid 0x02
        int32_t DefaultCellAH; /**< System cell's AH number, unit is mAH. System should broadcast this value before calibration finished. */
        int32_t reserved11;    /**< reserved11 */
        // msgid 0x03
        uint8_t State; /**< State
             - D7D6D5: None Sense
             - D4:     Calibration state 
	                   0: non-Calibration state/Calibration done 
			           1: Calibration state Normally, CPU should stop balance state before calibration done.
             - D3D2:   charging/discharging states 
	                   00:non-charging/non-discharging 
			           01:charging, 
			           10:discharging, 
			           11:N/A
                       This data is written by CPU, to inform PACK adjusting sampling rate and alarm type
             - D1D0:   balance model 
	                   00:non-balancing/finishing balance 
			           01:initiative balance 
			           10:passive balance
			           11:NA?
                       This data is writen by CPU, take actions after receiving PACK, default 00. 
			           In "00" state, not allowed for CPU to control balance switch, and stop balancing
        */
        volatile uint8_t ee_state; /**< ee_state - meaningless, as far as I can tell - pjn 2014-3-3 */
        volatile uint16_t Reserved2;  /**< Reserved2 */
        volatile uint32_t Reserved3;  /**< Reserved3 */
        /**************CAN ID above is 0x03*******************/

        /**************CAN ID above is 0x04*******************/

        /**************CAN ID above is 0x05*******************/

        /**************CAN ID above is 0x06*******************/

        /*************CAN ID 0x07 reserved*******************/
    };
//	uint8_t array[24]; /** Note : the struct will be padded to 24 bytes */
} SYSTEM_t;
#endif
/*=============================================================================*/
/* Battery pack structure
This structure in PACK has only one entity in RAM, stands for the whole battery pack, not included in EEPROM*/
typedef struct{
    // msg 0x08
    uint16_t Volt;
    uint16_t GndCounts;//ground measurement of kelvin point of cell 0
    uint16_t Cell1Volt;//Cell1 anode voltage
    uint16_t Cell2Volt;//Cell2 anode voltage
    // msg 0x09
    uint32_t BalanceSwitch;
    /* battery pack cell balancing switch 1: balancing/turn balance switch on 0: no balancing/turn balance switch off
        There are total 16-bit, each bit stands for 1 cell from low to high (1 to 16), default by 00
        In initiative balanced model, CPU will judge if the balancing finish by reading the data. If all 00, then balancing finished.
        In passive balanced model, CPU will write the data to control the balancing switch. Once the balanced model is set to "00", then stop balancing.
        "D16" stands for PACK charging balance switch. "1": balancing "0": stop balancing
    */

    int16_t    HeatSinkTemp;/*Heat sink temperature */
    int16_t    PowerSupplyTemp;/*Power supply temperature, charging power temperature*/
    // msg 0x0a
    int32_t rsvd[2];
    // msg 0x0b
    uint32_t CellState;
    /* Each two bit stand for cell's state: D1D0:Cell0¡­¡­D31D30:Cell16
    Battery pack's state: 00£ºnormal  01£ºalarm  10£ºprotective*/
    volatile uint32_t Reserved6;
    // each can message holds 8 bytes, 4 16-bit words, so an array of
    // 16 16-bit words takes 4 message ids for all 32 bytes
    // msg 0x0c, 0x0d, 0x0e, 0x0f
    uint16_t CellVolt[16];  /*Each cell's voltage in battery pack*/
    // msg 0x10, 0x11, 0x12, 0x13
    int16_t CellTemp[16];  // each cell's temperature (deg. C * 10)
    // msg 0x14, 0x15, 0x16, 0x17
    int16_t CellSoc[16];   // cell state-of-charge
    // msg 0x18
    uint16_t    BalMMTimer;//battery pack balance millisecond timer
    uint32_t CANTimer;
        /*Unit is mS£¬CAN bus receives overtime timer. Timing after inilization, 
            if this value is greater or equal to PackPara.CANOutTime, PACK will 
            shut off automatically. Next time start requires CPU activation. and 
            requires activitied by CPU next time. Each time system received CAN 
            info, it erases value. Not necessary to shut down if PackPara.CANOutTime=0.*/
    uint32_t AlarmTimer;
            /*The unit is mS£¬Alarm status will retransmit to timer. Each time the alert 
            transmite successfully, it then deletes the status. If this value is larger 
            or equal to PackPara.AlarmOutTime£¬then retransmit the alarm status.*/
    uint16_t rsvd1[3];    // even this up to 8-byte boundary
         // this was added 2014-3-18 by pjn. This structure is not saved in EEPROM,
         // so the size can harmlessly be changed. All other CAN-readable/writeable
         // structures are multiples of 8-bytes in size, so now this one is too.
} PACK;

/*=============================================================================*/
/*Battery pack parameter structure
Below is Pack's configuration parameter data, stored in EEPROM. Each time activation will read RAM,CPU will configure data and write to EEPROM except address and cell's number*/
typedef struct{

    uint8_t CellNum;
    /*Cell number in Battery pack, from 1 to 16. 1 to 8 conntect to ADC1, 9-16 connect to ADC2. CPU cannot change this data.*/
    volatile uint8_t Reserved1;
    volatile uint16_t Reserved2;
    int32_t pack_scale;      // AD scaling to mV*10000
    /***************CAN ID above is 0x20*******************/
    uint16_t BalCurrent;
    /*Battery pack 's initiative balance current, unit is mA*/
    volatile uint16_t Cell0VoltCalNum;//Cell0 voltage calibration value
    volatile uint16_t Cell1VoltCalNum;//Cell1 voltage calibration value
    volatile uint16_t Cell2VoltCalNum;//Cell2 voltage calibration value

    /***************CAN ID above is 0x21*******************/
    uint16_t VoltCalNum;
    /*Battery pack sampling calibration value, Actual value = Sampling value * Calibration value /10000,
        Normally this value is not changed.*/
    uint16_t reserved13;
    int32_t pack_offset;     // AD offset to mV * 100
    /***************CAN ID is 0x22*******************/
    uint16_t FastVoltCheckTime;
    /*Fast voltage check time, unit is mS, default 100 mS. If battery is in charging or discharging, then fast check, this parameter can modified by CPU. Normally we don change this parameter*/
    uint16_t SlowVoltCheckTime;
    /*Slow Voltage check time, unit is mS, default by 1000 ms. If battery is in non-charging /non-discharging, then slow check, This parameter can be modified by CPU. Normally we don change this parameter*/
    uint16_t TempCheckTime;
    /*Temperature check time, unit is mS, default by 1000 ms. Temperature check time is adjusted by CPU command. Normally this parameter is not changed*/
    volatile uint16_t Reserved7;
    /***************CAN ID above is 0x23*******************/
    uint32_t CANOutTime;
    /*unit is mS£¬CAN bus transmission overtime value, default by 30000=30S. If value = 0£¬then no overtime control, else PACK will shut down automatically by CPU shut down or CAN bus error*/
    uint32_t AlarmOutTime;
    /*unit is mS, alarm out time, default by 2000=2S. If it is 0, then upload alarm state once. Re-upload alarm and protection state if exceeding this time*/
    /***************ÒÔÉÏCAN IDÎª0x24*******************/
    uint32_t WorkHour;
    /*Working hour, unit is hour£¨32-bit£©, add 1 each hour and write to EEPROM*/
    uint16_t ProductTimeYear;
    /*Battery pack product time: year (2-bit), month (1-bit), day (1-bit)*/
    volatile uint8_t ProductTimeMonth;
    volatile uint8_t ProductTimeDay;
    /***************CAN ID above is 0x25*******************/
    uint32_t SeriesNum;
    /*Battery pack series number*/
    uint16_t ClientCode;
    /*Client code*/
    uint8_t SoftVer;
    /*Software version, "1" stands for the first version, "2" stands for the second*/
    volatile uint8_t DevID;     // Add software support the device ID change 2011/May/18
    /***************CAN ID is 0x26*******************/
    // Add new variable for SOC 2011-04-25 Sam

    uint16_t HighCellVoltage;
    uint16_t LowCellVoltage;
    uint16_t AveCellVoltage;
    uint16_t BPVoltage;
    /***************CAN ID above is 0x27*******************/
    int16_t HighCellSOC;
    int16_t LowCellSOC;
    int16_t AveCellSOC;
    int16_t BPSOC;
    /***************CAN ID above is 0x28*******************/
    int16_t HighCellTemp;
    int16_t LowCellTemp;
    int16_t AveCellTemp;
    int16_t BPTemp;
    /***************CAN ID above is 0x29*******************/
    uint16_t BPTargetVoltage;
    int16_t BPTargetSOC;
    uint32_t Reserved10;
    /***************CAN ID above is 0x2A*******************/
    int32_t AHMAHCounter;
    int32_t AHMWHCounter;
    /***************CAN ID above is 0x2B*******************/
    int32_t PackAHCapacity;             //add for Calibration , sam@April 24 2012
    int32_t PackWHCapacity;
    /***************CAN ID above is 0x2C*******************/

    /***************CAN ID reserved for 0x2C-0x2F*******************/
} PACKPARA;

/*=============================================================================*/
/*BMS system parameter value, saving in EEPROM. Read from RAM when initializing, and update by CPU*/
typedef struct{

    uint16_t PackOVAlarm;
    //Pack over voltage alarm, unit is 1mV. Alarm signal will be given to reduce charging current
    uint16_t    PackOVAlarmRecover;
    //Pack over voltage alarm recovery value
    uint16_t    PackOVProtect;
    //Pack over voltage protect value. Charger will be shut down when over voltage
    uint16_t    PackOVProtectRecover;
    //Pack over voltage protect recovery value
    /***************CAN ID above is 0xB0*******************/

    uint16_t    PackUVAlarm;
    //Pack under voltage alarm value, required reducing discharging current
    uint16_t    PackUVAlarmRecover;
    //Pack under voltage alarm recovery value
    uint16_t    PackUVProtect;
    //Pack under voltage protect value, required shut down discharging curcuit
    uint16_t    PackUVProtectRecover;
    //Pack under voltage recovery value
    /***************CAN ID above is 0xB1*******************/

    uint32_t ChgOCAlarm;
    // Charging over current alarm value, require reduce charging current, unit is mA
    uint32_t ChgOCAlarmRecover;
    //Charging over current alarm value recovery value
    /***************CAN ID above is 0xB2*******************/
    uint32_t ChgOCProtect;
    //Charging over current protect value, shut down charger required.
    uint16_t ChgOCprotectRecoverTime;
    //Charging over current protect recovery time
    uint16_t Reserved1;
    /***************CAN ID above is 0xB3*******************/

    uint32_t DisChgOCAlarm;
    //Discharging over current alarm, reducing discharging current required, unit is mA.
    uint32_t DisChgOCAlarmRecover;
    //Discharging over current alarm recovery value
    /***************CAN ID above is 0xB4*******************/
    uint32_t DisChgOCProtect;
    //Discharging over current protection value, shut down electronic device required
    uint16_t DisChgOCprotectRecoverTime;
    //Discharging over current protect recovery time
    uint16_t Reserved2;
    /***************CAN ID is 0xB5*******************/


    volatile uint16_t SmallCurChg;
    volatile uint16_t Reserved3;
    volatile uint32_t Reserved4;
    /***************CAN ID is 0xB6*******************/

    uint16_t CellOVAlarm;
    //Cell's over voltage alarm, unit is mV. Alarming signal is given, reduce charging current 3.60 required
    uint16_t    CellOVAlarmRecover;
    // Cell's over voltage alarm recovery value             3.50
    uint16_t    CellOVProtect;
    //Cell's over voltage protect value, shut down charger when over voltage    3.80
    uint16_t    CellOVProtectRecover;            //3.50
    //Cell's over voltage protection recovery value
    /***************CAN ID above is 0xB7*******************/

    uint16_t    CellUVAlarm;
    //Cell's under voltage value alarm, reducing current required 2400
    uint16_t    CellUVAlarmRecover;
    //Cell's under voltage alarm recovery value                 2600
    uint16_t    CellUVProtect;
    //Cell's under voltage protector value, shut down discharging circuit required     2000
    uint16_t    CellUVProtectRecover;
    //Cell's under voltage protect recovery value                     2600
    /***************CAN ID above is 0xB8*******************/

    uint16_t    PackBalStartVoltDif;
    //PACK balancing start voltage differences
    uint16_t    PackBalStopVoltDif;
    //PACK balancing start voltage differences
    //PACK balancing goal is to control the voltage differences in PACK under the PackBalStopVoltDif, criterion is the lowest voltage in PACK
    uint16_t    CellBalStartVoltDif;
    //Cell's balancing start voltage difference
    uint16_t    CellBalStopVoltDif;
    //Cell's balancing stop voltage differences
    //Cell's balancing goal is to control the voltage differences in PACK under the PackBalStopVoltDif
    //criterion is the lowest voltage in PACK's voltage
    //All the balancing processes requires command from controller to start, because there are three balance states: charging balance, discharging balance, and standing balance.
    /***************CAN ID above is 0xB9*******************/

    int16_t CellDischgOTAlarm;
    //Cell discharging over temperature alarm
    int16_t CellDischgOTAlarmRecover;
    //Cell's discharging over temperature alarm recovery value
    int16_t CellDischgOTProtect;
    //Cell's discharging over temperature protection value
    int16_t CellDischgOTProtectRecover;
    //Cell's discharging over temperature protection recovery value
    /***************CAN ID above is 0xBA*******************/

    int16_t CellDischgUTAlarm;
    //Cell's discharging under temperature alarm value
    int16_t CellDischgUTAlarmRecover;
    //Cell's discharging under temperature alarm recovery value
    int16_t CellDischgUTProtect;
    //Cell's discharging under temperature protection value
    int16_t CellDischgUTProtectRecover;
    //Cell's discharging under temperature protection recovery value
    /***************CAN ID above is 0xBB*******************/

    int16_t CellOTAlarm;
    //Cell's over temperature alarm value
    int16_t CellOTAlarmRecover;
    //Cell's over temperature alarm recovery value
    int16_t CellOTProtect;
    //Cell's over temperature protection value
    int16_t CellOTProtectRecover;
    //Cell's over temperature protect recovery value
    /***************CAN ID above is 0xBC*******************/

    int16_t CellUTAlarm;
    //Cell's under temperature alarm
    int16_t CellUTAlarmRecover;
    //Cell's under temperature alarm recovery value
    int16_t CellUTProtect;
    //Cell's under temperature protector
    int16_t CellUTProtectRecover;
    //Cell's under temperature protect recovery value
    /***************CAN ID above is 0xBD*******************/

    int16_t CellChgOTAlarm;
    //Cell's charging over temperature alarm
    int16_t CellChgOTAlarmRecover;
    //Cell's charging over temperature alarm recovery value
    int16_t CellChgOTProtect;
    //Cell's charging over temperature protect value
    int16_t CellChgOTProtectRecover;
    //Cell's charging over temperature protect recovery value
    /***************CAN ID above is 0xBE*******************/
    int16_t CellChgUTAlarm;
    //Cell's charging under temperature alarm
    int16_t CellChgUTAlarmRecover;
    //Cell's charging under temperature alarm recovery value
    int16_t CellChgUTProtect;
    //Cell's charging under temperature protect value
    int16_t CellChgUTProtectRecover;
    //Cell's charging under temperature protect recovery value
    /***************CAN ID above is 0xBF*******************/
    int16_t CellChgSmallCurTemp;
    int16_t CellChgSmallCurTempRecover;
    uint32_t Reserved5;
    /***************CAN ID above is 0xC0*******************/
} BMSPARA;

/*=============================================================================*/
/*Cell structure
Cell parameter structure
PACK configuration parameter data is showing below, saving in EEPROM. Read RAM each time start*/
typedef struct {
    int32_t ad_scale;        // ad scale to mv * 10000
    int32_t ad_offset;       // ad offset in mv * 100

    /**********CAN ID above is 0x70+(Cell's number-1)*4+0**********************/
    uint16_t VoltCalNum;
    //Cell's voltage calibration value, Actual value = sampling value * 10000/Calibration value. Normally this value is not changed*/
    uint16_t BalResist;
    //Balancing Resist, unit is mOhm
    volatile uint32_t Reserved1;
    /**********CAN ID above is 0x70+(Cell's number-1)*4+1**********************/
    volatile int32_t CellAHTotalCounter;
    volatile int32_t CellWHTotalCounter;
    /**********CAN ID is 0x70+(Cell's number-1)*4+2**********************/
    volatile int32_t CellBalAHCounter;
    volatile int32_t CellBalWHCounter;
    /**********CAN ID above is 0x70+(Cell's number-1)*4+3**********************/
} CELLPARA;

/*=============================================================================*/
//cell's structure
typedef struct {

    uint16_t Volt;
    /*Cell's voltage*/
    int16_t Temp;
    //Cell's temperature, accurate to 0.1 Celsius degree
    uint8_t State;
    /*D7:N/A
    //D7: balancing state 0: non-balancing 1:balancing
    //D5D4D3:
    000:Noraml temperature range
    100:High temperature alarm
    101:High temperature protection
    110:Low temperature alarm
    110:Low temperature protection
    //D2D1D0:
    000:Normal voltage
    100:Over voltage alarm
    101:Over voltage alarm
    110:Under voltage alarm
    111:Under voltage alarm
    */
    volatile uint8_t Reserved1;
    volatile uint16_t Reserved2;
    /**********Cell's voltage, current, state ID is 0x30+(Cell number-1)*4+0**********************/
    uint32_t BalTime;
    /*Accumulated time for balancing cell, unit is second. Highly real-time mission, empty before balance state begin*/
    uint16_t AlarmTimer;
    /*Alarm timer, empty after each alarming*/
    uint16_t Reserved3;
    /**********balance time ID is 0x30+(Cell number-1)*4+1**********************/
    int32_t reserved21;
    int32_t reserved22;
    /**********Balance AH and WH number, ID is 0x30+(Cell number-1)*4+2**********************/
    int32_t BalWHTemp;
    /*Balance WH number temporary counter, empty before balance state start*/
    int32_t BalAHTemp;
    /*Balance AH number temporary counter, empty before balance state start*/
    /**********Balance temporary AH number and temporary WH ID is 0x30+(Cell's number-1)*4+3**********************/
}CELL;


/**
  * Close the Doxygen bms__Exported_Types group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Constants                                                          */
/*-----------------------------------------------------------------------------*/
/** @defgroup bms_Exported_Constants
  * @{
  */
/**
 *  Command ID sent to the BMS, CAN ID = 0x00
 *  */
#define CLOSE_BP                 0        /**<  */
#define CALIBATED_EMPTY_BATTERY 1        /**<  */
#define CALIBATED_FULL_BATTERY     2        /**<  */
#define BACKUP_EEPROM             3        /**<  */
#define RUN_ADC_PROCESS         4
#define RUN_ADC_PROCESS_RAW     5
#define BMS_INHIBIT             6
#define CANCMD_FORCE_VMEASURE_ON 7
#define CANCMD_FORCE_VMEASURE_OFF 8


#define SELF_CALIBATING         0xf0    /**<  */
#define SELF_CHECK                 0xf1    /**<  */
#define SET_EEPROM_DEFAULT         0xf2    /**<  */


/**
 *  The Macro definition of BMS parameter addresses in EEPROM
 *  */
#define EE_FIRST         (0xFF) /**< First time start address. */
#define EE_SELF_WRITE   (0x01) /**< Parameters never being written. The parameters in EEPROM are initialized by MCU */
#define EE_CALED        (0x02) /**< Parameters being written. The data in EEPROM is written and calibrated by host computer */


#define EE_CRC_VERIFY_ADDR (0x07f0)
/*CRC verification bit address, which save the CRC results of valid data, use STM32 hardware computing
Production: X32 + X26 + X23 + X22 + X16 + X12 + X11 + X10 +X8 + X7 + X5 + X4 + X2+ X1  */
#define EE_VERIFY_ADDR   (0x07f8) //This address save verification data that stored in RAM, to decide if it is the first time start
#define EE_VERIFY_NUM     (0x08) //verify the byte's number

#define EE_PACKPARA_ADDR (0x0000) //PACKPARA structure and start address  0x00 preserved 256-bit space
#define EE_CELLPARA_ADDR (0x0100) //CELLPARA structural saving start address 0x0100=256 preserved 512-bit saving space
#define EE_BMSPARA_ADDR  (0x0300) //BMSPARA structural saving start address 0x0300=768


#define VAROFFSET(STRUCTs,VARs) ((unsigned long)(&(((STRUCTs *)0)->VARs)))


/*The Macro definition below is related to BMS initialization*/
#define CELL_NUM (16) // Cell's number
#define CELL_CAP (40000)// Default by 40AH battery, change to 40Ah from 24Ah 2011/May/09
#define CELL_DISCHARGE_VOLT (3200)// discharging platform is defaulted by 3.2V, fit for phosphoric acid iron battery
#define CELL_VOLT_MIN (2000) //Battery minimum discharging voltage 2.000V
#define CELL_VOLT_MAX (4000) //Battery maximum charging voltage 4.000V

/*The Macro definition below is related to System.state parameter*/
#define CALIBRATED    (0x00) // Calibration finished
#define NOTCALIBRATED    (0x10) // Calibrating

#define SYSTEMIDLE    (0x00) //Idle
#define CHARGING    (0x08) //Charging
#define DISCHARGING    (0x04) //Discharging

#define BALPROHIBIT        (0x00) //balance prohibit
#define    AUTOBAL        (0x01) //automatic balance
#define    MANUALBAL    (0x02) // Manual balance, controlled by controller

/*The Macro definition below is related to Pack.Cellstate parameter*/
#define NORMAL         (0x00)
#define ALARM        (0x01)
#define PROTECT        (0x02)

/*The Macro definition below is related to Cell[i].State parameters*/
     //D7£ºNon sense
#define CELL_NO_BAL (0x00)    //D6:Balance state 0:no balance 1:balancing
#define CELL_BALING (0x40)
    //D5D4D3:
#define    CELL_TEMP_NORMAL    (0x00)//Normal temperature range  00 0000
#define    CELL_OT_ALARM         (0x20) //Over temperature alarm      10 0000
#define    CELL_OT_PROTECT        (0x21) //Over temperature protect 10 1000
#define    CELL_UT_ALARM        (0x30) //Under temperature alarm  11 0000
#define    CELL_UT_PROTECT        (0x31) //Under temperature protect  11 1000
    //D2D1D0:
#define    CELL_VOLT_NORMAL    (0x00)//Normal voltage      000
#define    CELL_OV_ALARM        (0x04)//Over voltage alarm      100
#define    CELL_OV_PROTECT        (0x05)//Over voltage protect      101
#define    CELL_UV_ALARM        (0x06)//Under voltage alarm      110
#define    CELL_UV_PROTECT        (0x07)//Under voltage protect      111
#define ACBALSWITCHBIT 16

/**
  * Close the Doxygen bms_Exported_Constants group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Macros                                                             */
/*-----------------------------------------------------------------------------*/
/** @defgroup bms_Exported_Macros
  * @{
  */

/**
  * Close the Doxygen bms_Exported_Macros group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup bms_Exported_Variables
  * @{
  */
extern SYSTEM_t System;
extern PACK Pack;
extern PACKPARA PackPara;
extern CELL Cell[CELL_NUM];
extern CELLPARA CellPara[CELL_NUM];
extern BMSPARA BmsPara;
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
extern void Bms_Initialize(void);
extern uint32_t Para_CRC_Verify(void);
extern void Bms_Data_Init(void);
extern void Bms_Para_Init(void);
extern void SaveAllEEPromData(void);
extern void Bat_Manage(void);
extern void Alarm_Manage(void);
extern void Bat_Balance(void);
extern void Bat_Cal(void);
extern void Bat_Capacity_update(void);
extern void Bat_Bal(void);
extern void Auto_Balance_Charging(void);
extern void Auto_Balance_Discharging(void);
extern void Auto_Balance_SystemIdle(void);
extern void Manual_Balance(void);
// momentarily interrupt the ac balance charger during measurement. 'open' should
// be TRUE to turn off charging (before measurement) and FALSE to conditionally turn
// it back on after measurement.
void bal_interrupt(int open);

/**
  * Close the Doxygen bms_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* End of the C bindings section for C++ compilers.                            */
/*-----------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* BMS_H_ */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen bms group.
  *    @}
*/
