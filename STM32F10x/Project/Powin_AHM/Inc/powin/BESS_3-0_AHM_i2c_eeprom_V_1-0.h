/**
 ******************************************************************************
 * @file        BESS_3-0_AHM_i2c_eeprom_V_1-0.h
 * @author      Nystrom Engineering
 * @version     2.0.0
 * @date        2014-01-27
 * @copyright   Powin Energy
 * @brief       The include file for the I2C EEPROM datalogging memory.
 ******************************************************************************
*/

/*-----------------------------------------------------------------------------*/
/* Define to prevent recursive inclusion                                       */
/*-----------------------------------------------------------------------------*/
#ifndef I2C_EEPROM_H_
#define I2C_EEPROM_H_

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
#include "BESS_3-0_AHM_device_lib_V_1-0.h"

/** @addtogroup i2c_eeprom i2c_eeprom
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Exported Types                                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup i2c_eeprom_Exported_Types
  * @{
  */
struct EE_Save_Format {
    /**
     * @var int DefaultMark
     * @brief if == 0xaa55, memory will keep current values upon startup.
     * 			Otherwise, memory will be overwritten with default values upon startup.
     */
    uint16_t DefaultMark;							//Address 16
    /**
     * @var int ADTemperatureAdjust
     * @brief ppm as unit?
     */
    int16_t ADTemperatureAdjust;  // ppm as unit	//Address 18
    /**
     * @var int Current_Scaler
     * @brief ?????
     */
    int16_t Current_Scaler;							//Address 20
    /**
     * @var int Voltage1_Scaler
     * @brief ?????
     */
    int16_t Voltage1_Scaler;						//Address 22
    /**
     * @var int Voltage2_Scaler
     * @brief ?????
     */
    int16_t Voltage2_Scaler;						//Address 24
    /**
     * @var int Current_Offset
     * @brief ?????
     */
    int8_t Current_Offset;							//Address 26
    /**
     * @var int Voltage2_Offset
     * @brief ?????
     */
    int8_t Voltage1_Offset;							//Address 27
    /**
     * @var int Voltage2_Offset
     * @brief ?????
     */
    int8_t Voltage2_Offset;							//Address 28
    /**
     * @var int Current_Eleminated
     * @brief Minimum current level before registering as 0.
     */
    int8_t Current_Eleminated;						//Address 29
    /**
     * @var int Voltage_Eleminated
     * @brief Minimum voltage level before registering as 0.
     */
    int8_t Voltage_Eleminated;						//Address 30
    /**
     * @var unsigned char dev_id
     * @brief The device identifier.  Can be any unsiged char
     */
    uint8_t dev_id;  			// offset 15 (+16)	//Address 31
    /**
     * @var unsigned long Serial_Number
     * @brief serial number of the device.
     */
    uint32_t s_num;									//Address 32
    /**
     * @var char checksum
     * @brief Sum of all data before checksum
     */
    uint8_t checksum;								//Address 36
    /**
     * @var char resetMark
     * @brief ?????
     */
    uint8_t resetMark;								//Address 37
    /**
     * @var char AutoTickTime
     * @brief Number of ticks before autodata packets are sent.
     */
    uint8_t AutoTickTime;							//Address 38
    /**
     * @var char can_tx_block
     * @brief if == 0xbb, CAN messages will not be transmitted for 0xcc00000 cycles upon startup.
     * 			if == 0xff, CAN messages will transmit as normal.
     */
    uint8_t can_tx_block;							//Address 39
    /**
     * @var long Capacity_Count
     * @brief System capacity count.  Updated every 2 hours in memory.
     */
    int32_t Capacity_Count;							//Address 40
    /**
     * @var long CWattHour_Count
     * @brief System centiwatt-hour count.  Updated every 2 hours in memory.
     */
    int32_t CWattHour_Count;						//Address 44
    /**
     * @var unsigned short Current_Shunt_Value
     * @brief value of the current shunt.  4 bytes float, in ohms
     * it gets converted to siemens for use so it can be multiplied.
     */
    float Current_Shunt_Value;					//Address 48
    float _e_scale;                              // 52 what multiplies counts to make volts
    float _i_scale;                              // 56 what multiplies counts to make amps
    float _e_offs;                               // 60 offset in volts
    float _i_offs;							  	 // 64 offset in volts - really - cause we always measure volts

};
/**
  * Close the Doxygen i2c_eeprom__Exported_Types group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Constants                                                          */
/*-----------------------------------------------------------------------------*/
/** @defgroup i2c_eeprom_Exported_Constants
  * @{
  */
#define I2C_Speed            400000    //400KHz bus rate
#define I2C1_SLAVE_ADDRESS7    0xA0    //host address, disposable in this project
#define EEPROM_PageSize        16        //EEPROM page size, 16-bit
#define EEPROM_BlockSize    256        //EEPROM block size, to 24C16, each is 256-bit
#define EEPROM_Size            2048    // EEPROM size£¬24C16 2K-bit
#define I2C_Delay 4

// pjn test board after hack to fix i2c lines
#define SCL GPIOB,GPIO_Pin_10
#define SDA GPIOB,GPIO_Pin_11
// old boards with reversed lines
//#define SCL GPIOB,GPIO_Pin_11
//#define SDA GPIOB,GPIO_Pin_10

#define I2C_READ 1
#define I2C_WRITE 0

/**
  * Close the Doxygen i2c_eeprom_Exported_Constants group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Macros                                                             */
/*-----------------------------------------------------------------------------*/
/** @defgroup i2c_eeprom_Exported_Macros
  * @{
  */
#define EE_SAVE_START_ADDRESS 0x10															//Not sure why everything in memory
#define EEOFFSET(x) ((uint32_t)(&(((struct EE_Save_Format *)0)->x)))						// is offset by 16.  Kept this way
#define EEADDRESS(x) ((uint32_t)(&(((struct EE_Save_Format *)0)->x))+EE_SAVE_START_ADDRESS)	// to maintain compatibility.
/**
  * Close the Doxygen i2c_eeprom_Exported_Macros group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup i2c_eeprom_Exported_Variables
  * @{
  */
extern uint8_t SlaveAddr;
extern uint8_t ByteAddr;

/**
  * Close the Doxygen i2c_eeprom_Exported_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Function Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup i2c_eeprom_Exported_Functions
  * @{
  */

extern void I2C_EE_Initialize(void);
extern void I2C_START(void);
extern void I2C_STOP(void);
extern uint8_t I2C_SendByte(uint8_t Data);
extern uint8_t I2C_ReadByte(void);
extern void I2C_EE_ByteWrite(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite);
extern void I2C_EE_PageWrite(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite);
extern void I2C_EE_BufferWrite(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite);
extern void I2C_EE_BufferRead(uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead);
extern void I2C_EE_FillByte(uint8_t Byte,uint16_t StartAddr,uint16_t StopAddr);
extern void I2C_EE_BufferUpdate(uint8_t* pBuffer, uint16_t UpdateAddr,uint16_t NumByteToUpdate);
extern void I2C_EE_Test(void);
extern void Set_EEPRom_Default(void);
extern uint8_t Get_Check_Sum(void);
extern uint8_t EEPRom_Check_Sum(void);
extern void Update_EEPRom_Check_Sum(void);
void save_eeprom_datas(void);

/**
  * Close the Doxygen i2c_eeprom_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* End of the C bindings section for C++ compilers.                            */
/*-----------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* I2C_EEPROM_H_ */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen i2c_eeprom group.
  *    @}
*/



