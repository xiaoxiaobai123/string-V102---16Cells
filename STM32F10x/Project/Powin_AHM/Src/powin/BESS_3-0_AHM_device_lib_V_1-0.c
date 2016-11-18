/**
  ******************************************************************************
  * @file        device_lib.c
  * @author      Nystron Engineering
  * @version     2.0.0
  * @date        Jan 27, 2014
  * @copyright   Powin Energy
  * @brief       This file defines all the peripheral chip and on-board devices.
  * @page        device_lib_page Platform and Device Control Functions
  * @section     device_lib_intro Introduction
  * @par
  *  This file defines all the peripheral chip and on-board devices.  \n
  *
  ******************************************************************************
  */
/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
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
#include "pinmap.h"
#include "BESS_3-0_AHM_stringdata_V_1-0.h"
#include "i2c_ee.h"


/** @addtogroup device_lib device_lib
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Private Types                                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup device_lib_Private_Types
  * @{
  */

/**
  * Close the Doxygen device_lib__Private_Types group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private_Constants                                                           */
/*-----------------------------------------------------------------------------*/
/** @defgroup device_lib_Private_Constants
  * @{
  */

/**
  * Close the Doxygen device_lib_Private_Constants group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private Variable Declarations                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup device_lib_Private_Variables
  * @{
  */

/**
  * Close the Doxygen device_lib_Private_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private Function Declarations                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup device_lib_Private_Functions
  * @{
  */

/**
  * Close the Doxygen device_lib_Private_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup device_lib_Exported_Variables
  * @{
  */
uint32_t uwTimingDelay = 0;

/**
  * Close the Doxygen device_lib_Exported_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Function Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup device_lib_Exported_Functions
  * @{
  */
/*-----------------------------------------------------------------------------*/
/**
 * @fn       void Device_Initialization(void)
 * @brief    This function performs the Platform and Device Initialization.
 * @detail   All subsystems are initialized and configured.
 *           @par
 *           At this stage the microcontroller clock setting is already configured,
 *           this is done through SystemInit() function which is called from the
 *           startup file (startup_cm.c) before the branch to application main.
 *           To reconfigure the default setting of SystemInit() function,
 *           refer to the "system_stm32f10x.c" file.
 */

void SPI_Initialize(RCC_ClocksTypeDef *clks)
{
    SPI_InitTypeDef spi_init;

    spi_init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi_init.SPI_Mode = SPI_Mode_Master;
    spi_init.SPI_DataSize = SPI_DataSize_16b;
    spi_init.SPI_CPOL = SPI_CPOL_High;   // intent: clock = HIGH at falling CSn
    spi_init.SPI_CPHA = SPI_CPHA_1Edge;  // intent: first data bit clocked in at first falling clock edge
    spi_init.SPI_NSS = SPI_NSS_Soft;
    spi_init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    spi_init.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI1, &spi_init);
    SPI_Cmd(SPI1, ENABLE);

}

void Device_Initialize(void)
{
    RCC_ClocksTypeDef clks;

    /* Disable Global Interrupts */
    //NVIC_SETPRIMASK();

    // Enable the CRC generator. This must be done before Bms_Initialize, because
    // that routine reads the EE and checksums the data using this peripheral.

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    /* Configure System Clocks */
    /* NB: this is done in BESS_3-0_AHM_stm32f10x_conf_V_1-0.h */

    /* Initialize all devices */
    RCC_GetClocksFreq(&clks);

    SysTick_Initialize();
    #if(CURRENT_BOARD_TYPE == BOARD_TYPE_EVB)
    #else
    GPIO_Initialize();
    #endif
    CAN_Initialize(&clks);

    SPI_Initialize(&clks);
    #if(CURRENT_BOARD_TYPE == BOARD_TYPE_EVB)
    #else
//    I2C_EE_Initialize();
    #endif
	
    /* Configure Interrupt Controller */
    //NVIC_Config();

    /* Enable Global Interrupts */
    //NVIC_RESETPRIMASK();
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn        void Delay(volatile uint32_t nCount)
 * @brief     This function performs a soft delay count, and is a blocking function.
 * @param     volatile uint32_t nCount - soft delay counter, blocking function.
 * @retval    None
 */
void Delay(volatile uint32_t nCount)
{
    for(; nCount != 0; nCount--) {
        ; // do nothing...
    }
}
#if(0)
/*-----------------------------------------------------------------------------*/
/**
 * @fn       void DelayBlocking(uint32_t nTime)
 * @brief    Inserts a delay time.
 * @param    nTime: specifies the delay time length, in SysTick ticks.
 * @retval   None
 */
void DelayBlocking(uint32_t nTime)
{
    uwTimingDelay = nTime;

    while (uwTimingDelay != 0)
        ;
}
#endif
/*-----------------------------------------------------------------------------*/
/**
 * @fn       void TimingDelay_Decrement(void)
 * @brief    Decrements the TimingDelay variable.
 * @param    None
 * @retval   None
 */
void TimingDelay_Decrement(void)
{
    if (uwTimingDelay != 0x00) {
        uwTimingDelay--;
    }
}

void WDT_State(FunctionalState WDTState)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | 
						   RCC_APB2Periph_GPIOB |
                      	   RCC_APB2Periph_GPIOC |
						   RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_GPIOE,
						   ENABLE);	                 /*Assign clock for GPIOA B C D E */   	
	if(WDTState == DISABLE)
	{
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 	 // Watchdog driver port		 					 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //		Push-pull output
    	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	}
	else
	{
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 	 // Watchdog driver port		 					 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //		Push-pull output
		GPIO_Init(GPIOC, &GPIO_InitStructure);
	}

}
void WDT_SW(void)
{
	uint8_t RunLigntState = GPIO_ReadOutputDataBit(WDT);
	if (RunLigntState  == DISABLE)
		GPIO_SetBits(WDT);
	else
		GPIO_ResetBits(WDT);
}


static uint8_t CalibrationFullAH_EEPROMDo_Once_Flag = 1;  /*EEPROM 只写一次*/
static uint8_t CalibrationEmptyAH_EEPROMDo_Once_Flag = 1;
uint8_t alreadyCalibrated = 0;
uint8_t ProgrameRunTimer = 0;
uint8_t ProgrameRun2minsFlag = 0;
void EmptyKwhCalibration()
{
	if(ProgrameRun2minsFlag == 1)
	{
		if(mStringData.averageCellVolt >= 3550 && mStringData.averageCellVolt <= 3800) /**/
		{
			static uint16_t UnderVoltCount = 0;
			UnderVoltCount++;
			if(UnderVoltCount >= 500)
			{
				UnderVoltCount = 0;
				alreadyCalibrated = 1;
	
	
				mStringData.soc = 95;
				mStringData.ah = mStringData.soc * TotalAH / 100;
				mStringData.kWh = mStringData.ah *mStringData.averageCellVolt/ 1000 * BP_NUMBER * CELL_NUMBER / 100;			
				CopyAHKWHToEEPROMTemp(); 
				StringEEProm_Write(AHKWHEEPageNumber);		
			}		
			
		}	
		if(mStringData.averageCellVolt <= 2800 && mStringData.averageCellVolt > 2600 && mStringData.averageCellVolt > 0) /**/
		{
			if( (mStringData.mode == DisCharge_Mode && mStringData.kWh > 2000) || mStringData.mode == Charge_Mode || mStringData.mode == Idle_Mode)
			{
				static uint16_t UnderVoltCount = 0;
				UnderVoltCount++;
				if(UnderVoltCount >= 500)
				{
					UnderVoltCount = 0;
					alreadyCalibrated = 1;
		
		
					mStringData.soc = 5;
					mStringData.ah = mStringData.soc * TotalAH / 100;
					mStringData.kWh = mStringData.ah *mStringData.averageCellVolt/ 1000 * BP_NUMBER * CELL_NUMBER / 100;			
					CopyAHKWHToEEPROMTemp(); 
					StringEEProm_Write(AHKWHEEPageNumber);		
				}
			}
			
			
			
		}	
		
		else if(mStringData.averageCellVolt <= 2600 && mStringData.averageCellVolt > 2500 && mStringData.averageCellVolt > 0)
		{
			
			if( (mStringData.mode == DisCharge_Mode && mStringData.kWh > 1000) || mStringData.mode == Charge_Mode || mStringData.mode == Idle_Mode)
			{
				static uint16_t UnderVoltCount = 0;
				UnderVoltCount++;
				if(UnderVoltCount >= 500)
				{
					UnderVoltCount = 0;
					alreadyCalibrated = 1;
					
					mStringData.soc = 2;
					mStringData.ah = mStringData.soc * TotalAH / 100;
					mStringData.kWh = mStringData.ah *mStringData.averageCellVolt/ 1000 * BP_NUMBER * CELL_NUMBER / 100;
					
					CopyAHKWHToEEPROMTemp(); 
					StringEEProm_Write(AHKWHEEPageNumber);		
				}	
			}
			
		}
		
		
		else if(mStringData.averageCellVolt <= 2500 && mStringData.averageCellVolt > 0 ) /**/
		{
			static uint16_t UnderVoltCount = 0;
			UnderVoltCount++;
			if(UnderVoltCount >= 500)
			{
				UnderVoltCount = 0;
				alreadyCalibrated = 1;
	
				mStringData.ah = 0;
				mStringData.kWh = 0;
				mStringData.soc = 0;
				CopyAHKWHToEEPROMTemp(); 
				StringEEProm_Write(AHKWHEEPageNumber);		
			}		
			
		}
	}		
	
}
void StringBP_Capacity_update(void)
{
	int32_t temp;

	static uint8_t OverVoltCount = 0;

 
/*BUG---需要保证读到highcellvolt 和 lowcellvolt*/	

	if(mStringData.highCellVolt >= 3600 && CalibrationFullAH_EEPROMDo_Once_Flag == 1)  /*校验值保存入EEPROM*/
	{		
/*
		for test
*/
		OverVoltCount++;
		if(OverVoltCount >= 100)
		{
			OverVoltCount = 0;
//			contactor1_off(); // turn off contactor
//			contactor2_off(); // turn off contactor
		
			CalibrationFullAH_EEPROMDo_Once_Flag = 0;
//			mStringData.DefaultCellAH = TotalAH;  /* AH 扩大100倍*/
//			mStringData.DefaultCellKWH = TotalKWH;  /*KWH 扩大 1000倍*/
//			mStringData.ah = TotalAH;
		
			//mStringData.kW = 36;                                                        /*暂时不写入EEPROM，因为关机时会将AH/KWH值写入EEPROM*/
//			mStringData.kWh = TotalKWH;
/*
		for test
*/	
//			CopyAHKWHToEEPROMTemp(); 
//			StringEEProm_Write(AHKWHEEPageNumber);
		}
		
	}
	

#if 0
	if(mStringData.lowCellVolt <= 2500 && CalibrationEmptyAH_EEPROMDo_Once_Flag == 1)  /*校验值保存入EEPROM*/
	{
		UnderVoltCount++;
		if(UnderVoltCount >= 100)
		{
			UnderVoltCount = 0;
			CalibrationEmptyAH_EEPROMDo_Once_Flag = 0;
			mStringData.DefaultCellAH = 0;
			mStringData.DefaultCellKWH = 0;
			mStringData.ah = 0;
			//mStringData.kW = 0;
			mStringData.kWh = 0;
			
			CopyAHKWHToEEPROMTemp(); 
			StringEEProm_Write(AHKWHEEPageNumber);		
		}
	}
#endif	
	
	if(mStringData.AHTimer >= 1000)
	{
		temp = mStringData.AHTimer;
		mStringData.AHTimer = temp - 1000;
		mStringData.CalAHTimer++;
	//	mStringData.
//		mStringData.stringCurrent = getStringCurrent(); 
 
		mStringData.AHTemp += mStringData.stringCurrent ;                /*100ma*/
		
		if(mStringData.stringCurrent >= 100)                /*计算KWH*/
			
		{
			mStringData.KWHTemp += mStringData.stringCurrent / 100 * mStringData.measureStringVoltage * 95 / 100; /* wh 0.93 为charge实际系数*/
			mStringData.kW = 60 * mStringData.measureStringVoltage ;
		}
		else
			
		{
			mStringData.KWHTemp += mStringData.stringCurrent / 100 * mStringData.measureStringVoltage; 
			mStringData.kW = 60 * mStringData.measureStringVoltage;
		}
		
//		mStringData.KWHTemp /= 1000;  /*wh->kwh*/
		
//		mStringData.kW = mStringData.KWHTemp ;   commnet by jason 20160601
		 
//		if(mStringData.AHTemp <= 0 || mStringData.KWHTemp <= 0 )
//		{
//			mStringData.AHTemp = 0;
//			mStringData.KWHTemp = 0;				
//		}	
		
		if(mStringData.CalAHTimer >= 10 && (mStringData.CalAHTimer % 10 == 0))
		{
			temp = mStringData.AHTemp ;
//			mStringData.ah += temp  / 3600 ;  // ah * 100 ，100mA
			{
				int AhTemp = mStringData.ah + temp  / 3600 ;
				if(AhTemp < 0)
					mStringData.ah = 0;
				else
					mStringData.ah = AhTemp;
				
			}
			mStringData.AHTemp = temp % 3600;
			temp = mStringData.KWHTemp;
//			mStringData.kWh += temp  / 3600 ; //wah * 100  100 mA
			{
				int32_t KhwTemp = mStringData.kWh + temp  / 3600 ;
				if(KhwTemp < 0)
					mStringData.kWh = 0;
				else
					mStringData.kWh = KhwTemp; 
				
			}
			mStringData.KWHTemp = temp % 3600;
		}
	}
	mStringData.soc = (u32)(mStringData.ah* 100) / TotalAH   ; /**/
	if(mStringData.soc >= 95)
		mStringData.soc = 95;
	 
}
 

