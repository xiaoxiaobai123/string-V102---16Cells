/**
 ******************************************************************************
 * @file        i2c_eeprom.c
 * @author      Nystrom Engineering
 * @version     2.0.0
 * @date        2014-01-27
 * @copyright   Powin Energy
 * @brief       The file provides all of the I2C EEPROM data logging memory functions.
 * @page        I2C EEPROM Data Logger Memory Functions
 * @section     i2c_eeprom_intro Introduction
 * @par
 * The i2c_eeprom functions are used to interface an external EEPROM via an I2C bus. \n
 * The external EEPROM is used to log the battery pack measurements and performance. \n
 *
 ******************************************************************************
 *  @todo deadlock spin on I2C ACK (GACK!) is bad and could hang, a timer should be used to break the infinite loop.
 ******************************************************************************
*/

/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
#include "BESS_3-0_AHM_device_lib_V_1-0.h"
#include "BESS_3-0_AHM_can_V_1-0.h"
#include "BESS_3-0_AHM_main_V_1-0.h"

/** @addtogroup i2c_eeprom i2c_eeprom
  * @{
  */

/*-----------------------------------------------------------------------------*/
/* Private Types                                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup i2c_eeprom_Private_Types
  * @{
  */

/**
  * Close the Doxygen i2c_eeprom__Private_Types group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private_Constants                                                           */
/*-----------------------------------------------------------------------------*/
/** @defgroup i2c_eeprom_Private_Constants
  * @{
  */

/**
  * Close the Doxygen i2c_eeprom_Private_Constants group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private Variable Declarations                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup i2c_eeprom_Private_Variables
  * @{
  */

/**
  * Close the Doxygen i2c_eeprom_Private_Variables group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Private Function Declarations                                               */
/*-----------------------------------------------------------------------------*/
/** @defgroup i2c_eeprom_Private_Functions
  * @{
  */

/**
  * Close the Doxygen i2c_eeprom_Private_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/
/** @defgroup i2c_eeprom_Exported_Variables
  * @{
  */
uint8_t SlaveAddr;
uint8_t ByteAddr;

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


/*-----------------------------------------------------------------------------*/
/**
 * @brief  This function performs the I2C EEPROM Data Logger initialization.
 * @param  None
 * @retval None
 */
void I2C_EE_Initialize(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Configure I2C1 pins: SCL (PB10) and SDA (PB11) */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; // Open-drain output
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/*******************************************************************************
* Function Name  : I2C_START
* Description    : Modeling I2C bus starting bit
* Input          : None
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-23
* Modifier         : None
* Date           : None
*******************************************************************************/
void I2C_START(void)
{
    GPIO_SetBits(SDA);// set bits SDA
    Delay(I2C_Delay);
    GPIO_SetBits(SCL);// set bits SCL
    Delay(I2C_Delay*2);
    GPIO_ResetBits(SDA);//set bits SDA
    Delay(I2C_Delay*2);
    GPIO_ResetBits(SCL);//set bits SCL
    Delay(I2C_Delay*2);
}
/*******************************************************************************
* Function Name  : I2C_STOP
* Description    : Modeling I2C bus generation stop
* Input          : None
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-23
* Modifier         : None
* Date           : None
*******************************************************************************/
void I2C_STOP(void)
{
    GPIO_ResetBits(SDA);//set bits SDA
    Delay(I2C_Delay);
    GPIO_SetBits(SCL);//set bits SCL
    Delay(I2C_Delay*2);
    GPIO_SetBits(SDA);//set bits SDA
    Delay(I2C_Delay*2);
}
/*******************************************************************************
* Function Name  : uint8_t I2C_SendByte(uint8_t Data)
* Description    : Send bytes
* Input          : The data to be sent
* Output         : None
* Return         : Responding identifier
* Author         : Jeff Zhang
* Date             : 2010-12-24
* Modifier         : None
* Date           : None
*******************************************************************************/
uint8_t I2C_SendByte(uint8_t Data)
{
    uint8_t Datatemp = Data;
    uint8_t i;
    for(i=0; i<8; i++) {
        if(((Datatemp<<i)&0x80) == 0)//If the highest bit is low level
            GPIO_ResetBits(SDA);
        else
            GPIO_SetBits(SDA);
        Delay(I2C_Delay);//Effect delay
        GPIO_SetBits(SCL);// rise SCL
        Delay(I2C_Delay);// effect dely
        GPIO_ResetBits(SCL);
    }
    Delay(I2C_Delay);// effect dely
    GPIO_SetBits(SDA);//releaseSDA
    Delay(I2C_Delay);
    GPIO_SetBits(SCL);//
    Delay(I2C_Delay);
    i = GPIO_ReadInputDataBit(SDA);
    GPIO_ResetBits(SCL);
    Delay(I2C_Delay);
    return i;   // returns 0 on ACK received, because ACK is slave pulling SDA down pjn
}
/*******************************************************************************
* Function Name  : I2C_ReadByte(uint8_t addr ,uint8_t ReadorWrite)
* Description    : read byte
* Input          : None
* Output         : None
* Return         : The byte    read
* Author         : Jeff Zhang
* Date             : 2010-12-23
* Modifier         : None
* Date           : None
*******************************************************************************/
uint8_t I2C_ReadByte(void)
{
    uint8_t Data = 0;
    uint8_t i;
    GPIO_SetBits(SDA);
    for(i=0; i<8; i++) {
        Delay(I2C_Delay);
        GPIO_ResetBits(SCL);// lower the clock line
        Delay(I2C_Delay*2);
        GPIO_SetBits(SCL);// rise the clock line
        Delay(I2C_Delay);
        Data <<= 1;
        if(GPIO_ReadInputDataBit(SDA) == 1)//if the highest bit is low level
            Data |=0x01;
        Delay(I2C_Delay);//Delay
    }
    GPIO_ResetBits(SCL);// lower the clock line
    Delay(I2C_Delay);
    return(Data);
}
/*******************************************************************************
* Function Name  : I2C_Ack(bool ACK)
* Description    : Produce or non-produce ACK signal
* Input          : ACK״̬
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-23
* Modifier         : None
* Date           : None
*******************************************************************************/
void I2C_Ack(bool ACK)
{
    if(ACK == TRUE)
        GPIO_ResetBits(SDA);
    else
        GPIO_SetBits(SDA);
    Delay(I2C_Delay);
    GPIO_SetBits(SCL);
    Delay(I2C_Delay);
    GPIO_ResetBits(SCL);
    Delay(I2C_Delay);
}

/*******************************************************************************
* Function Name  : I2C_EE_ByteWrite
* Description    : I2C byte write operation
* Input          : The byte to be write, initial address, amount
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-24
* Modifier         : None
* Date           : None
*******************************************************************************/
void I2C_EE_ByteWrite(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite)
{
    uint8_t SlaveAddr,ByteAddr;
    uint8_t Ack;
    uint8_t gack;

    SlaveAddr = WriteAddr / EEPROM_BlockSize;//Calculating address
    SlaveAddr = (SlaveAddr <<1) + 0xa0;//
    ByteAddr =  WriteAddr % EEPROM_BlockSize;
    I2C_START();// Start signal
    Ack=I2C_SendByte(SlaveAddr);// send device address
    Ack=I2C_SendByte(ByteAddr);// send byte address
    Ack=I2C_SendByte(*pBuffer);// send data

    I2C_STOP();//send stop signal, EEPROM begin to write

    /** @TODO deadlock spin is bad and could hang, a timer should be used to break the infinite loop. */
    
    gack = Ack;
    while(gack) {
        I2C_START();
        gack = I2C_SendByte(SlaveAddr);

    }
}
/*******************************************************************************
* Function Name  : I2C_EE_PageWrite
* Description    : I2C page write operation
* Input          : The byte to be write, initialize address, amount to be writen
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-24
* Modifier         : None
* Date           : None
*******************************************************************************/
void I2C_EE_PageWrite(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite)
{
    uint8_t SlaveAddr,ByteAddr;
    uint16_t Count=0;
    volatile uint8_t Ack;
    SlaveAddr =(WriteAddr+Count) / EEPROM_BlockSize;// Calculating Address
    SlaveAddr = (SlaveAddr <<1) + 0xa0;//
    ByteAddr = (WriteAddr+Count) % EEPROM_BlockSize;
    I2C_START();//Start signal
    Ack=I2C_SendByte(SlaveAddr);// send device address
    Ack=I2C_SendByte(ByteAddr);// send byte address
    for(Count = 0; (Count <NumByteToWrite)&&(Count <EEPROM_PageSize); Count++)
        Ack=I2C_SendByte(*(pBuffer+ Count));//send data

    I2C_STOP();// send stop signal, EEPROM begin to write
    /** @TODO deadlock spin is bad and could hang, a timer should be used to break the infinite loop. */
    do {
        I2C_START();
        Ack = I2C_SendByte(SlaveAddr);
        I2C_STOP();
    }
    while (Ack);     // wait until slave ACKs (holds SDA low) control byte
}

/*******************************************************************************
* Function Name  : I2C_EE_BufferWrite
* Description    : I2C write operation
* Input          : The byte to be write, initialized address, amount to be writen
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-23
* Modifier         : None
* Date           : None
*******************************************************************************/
void I2C_EE_BufferWrite(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite)
{

    uint16_t Addr=WriteAddr;
    uint8_t *Buffer=pBuffer;
    uint16_t Count;
    uint16_t PageNum,SingleNum;

    PageNum =  NumByteToWrite / EEPROM_PageSize;
    SingleNum = NumByteToWrite % EEPROM_PageSize;

    if (PageNum) {
        for(Count = 0; Count < PageNum; Count++) {
            I2C_EE_PageWrite(Buffer,Addr,EEPROM_PageSize);
            Addr+=EEPROM_PageSize;
            Buffer+=EEPROM_PageSize;
        }
    }

    if(SingleNum != 0)
        I2C_EE_PageWrite(Buffer,Addr,SingleNum);
}


/*******************************************************************************
* Function Name  : I2C_EE_BufferRead
* Description    : Read the data from EEPROM
* Input          : Save pointer, the address to read, the number of byte to read
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-24
* Modifier         : None
* Date           : None
*******************************************************************************/
void I2C_EE_BufferRead(uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead)
{
    uint8_t SlaveAddr,ByteAddr;
    uint16_t Count=0;
    volatile uint8_t Ack, gack;
    SlaveAddr = ReadAddr / EEPROM_BlockSize;//Calculate the block size
    SlaveAddr = (SlaveAddr <<1) + 0xa0 ;//
    ByteAddr =  ReadAddr % EEPROM_BlockSize;
    I2C_START();// Start signal
    Ack=I2C_SendByte(SlaveAddr);// send device address
    Ack=I2C_SendByte(ByteAddr);// send device address

    I2C_START();// Start signal, restart
    SlaveAddr = SlaveAddr |0x01;// the read signal
    Ack=I2C_SendByte(SlaveAddr);//send device address
    gack = Ack;
    Ack = gack;
    for(Count=0 ; Count < NumByteToRead ; Count++) {

        *(pBuffer+Count)=I2C_ReadByte();// read byte
        if(Count == (NumByteToRead-1)) {
            I2C_Ack(FALSE);// If it is the last byte, generate stop signal
            I2C_STOP();// If it is the last bit, generate stop signal
        }
        else
            I2C_Ack(TRUE);// else generate responding signal
    }
}

/*******************************************************************************
* Function Name  : I2C_EE_FullByte
* Description    : Using byte to fulfill assigned EEPROM space
* Input          : The byte to be fulfilled
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-01
* Modifier         : None
* Date           : None
*******************************************************************************/
void I2C_EE_FillByte(uint8_t Byte,uint16_t StartAddr,uint16_t StopAddr)
{
    uint16_t Addr = StartAddr;
    uint8_t Buffer[EEPROM_PageSize];
    uint8_t Count;
    uint16_t PageNum,SingleNum;
    //volatile uint8_t Ack;

    PageNum  = ( StopAddr -StartAddr ) / EEPROM_PageSize;
    SingleNum =( StopAddr -StartAddr ) % EEPROM_PageSize;

    for(Count=0; Count<EEPROM_PageSize; Count++)
        Buffer[Count] = Byte;

    if(PageNum!=0)
        for(Count = 0; Count<PageNum; Count++) {
            I2C_EE_PageWrite(Buffer,Addr,EEPROM_PageSize);
            Count++;
            Addr = Addr + EEPROM_PageSize;
        }

    if(SingleNum != 0)
        I2C_EE_PageWrite(Buffer,Addr,SingleNum);
}



/*******************************************************************************
* Function Name  : I2C_EE_UPdate
* Description    : Read the EEPROM find difference, and save the location with differnce
* Input          : none
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date             : 2010-12-01
* Modifier         : None
* Date           : None
*******************************************************************************/
void I2C_EE_BufferUpdate(uint8_t* pBuffer, uint16_t UpdateAddr,uint16_t NumByteToUpdate)
{
    uint16_t i;
    uint8_t readbuffer;
    uint8_t writebuffer;
    uint16_t tryTimes;

    for (i=0; i<    NumByteToUpdate; i++) {

        tryTimes = 0;
        writebuffer = *pBuffer;
        do {
            tryTimes++;
            I2C_EE_BufferRead(&readbuffer, UpdateAddr, 1);
            if (readbuffer==writebuffer) tryTimes = 0xffff;
        }
        while ( tryTimes<4 );

        if (readbuffer!=writebuffer) I2C_EE_BufferWrite(pBuffer, UpdateAddr, 1);

        pBuffer++;
        UpdateAddr++;
    }
}

/*-----------------------------------------------------------------------------*/
/**
 * @fn       void I2C_EE_Test(void)
 * @brief    This function test the EEPROM subsystem.
 */
static uint8_t EETest[256];
void I2C_EE_Test(void)
{
    uint16_t a;
    uint16_t i, j;

    for (a = 0; a < sizeof(EETest); a++)
        EETest[a] = a;
    for (i = 0; i < 4; i++)
        I2C_EE_BufferWrite(EETest, i << 8, sizeof(EETest));
    for (i = 0; i < 4; i++) {
        I2C_EE_BufferRead(EETest, i << 8, sizeof(EETest));
        for (j = 0; j < sizeof(EETest); j++) {
            if (EETest[j] != j) {
                while (1);
            }
        }
    }

}

void Set_EEPRom_Default( )
{
    struct EE_Save_Format DefaultEE;
    uint8_t m[2];
    uint16_t Default_Mark;

    I2C_EE_BufferRead(m, EEADDRESS(DefaultMark), 2);
    Default_Mark = ( (m[1] << 8) | m[0] );
    if (Default_Mark != 0xffff) {
        if (Default_Mark == 0xaa55) {
//			startup_status|=EEPRom_Check_Sum();
        }
        return;
    }
    DefaultEE.DefaultMark = 0xaa55;
    DefaultEE.dev_id  = 1;
    DefaultEE.can_tx_block =0xff;
    DefaultEE.s_num = 0x00000201;
    DefaultEE.Current_Scaler = 5426; //50.0A full scale
    DefaultEE.Voltage1_Scaler = 12071; //700V full scale
    DefaultEE.Voltage2_Scaler = 5000; //50V full scale
    DefaultEE.Current_Offset= 0;
    DefaultEE.Current_Shunt_Value = 0x8000;
    DefaultEE.Current_Eleminated = 10; //10mA
    DefaultEE.Voltage_Eleminated = 5; //0.5V
    DefaultEE.Voltage1_Offset = 0;
    DefaultEE.Voltage2_Offset = 0;
    DefaultEE.AutoTickTime = 0x010;
    DefaultEE.ADTemperatureAdjust = 0;
    DefaultEE.checksum = 0;
    I2C_EE_BufferWrite((uint8_t *)(&DefaultEE), EE_SAVE_START_ADDRESS, sizeof(DefaultEE));
//	Update_EEPRom_Check_Sum();
}

#if 0
uint8_t Get_Check_Sum()
{
    unsigned char i;
    uint8_t Check_Sum=0;
    uint8_t acc;
    for (i=EE_SAVE_START_ADDRESS; i<EEADDRESS(checksum); i++) {
        I2C_EE_BufferRead(&acc, i, 1);
        Check_Sum += acc;
    }
    return Check_Sum;
}

uint8_t EEPRom_Check_Sum()
{
    uint8_t i,j;
    i=Get_Check_Sum();
    I2C_EE_BufferRead(&j, EEADDRESS(checksum), 1);
    if (i==j) return 0;
    return EEPROM_ERROR;
}

void Update_EEPRom_Check_Sum()
{
    uint8_t i;
    i=Get_Check_Sum();
    I2C_EE_BufferWrite(&i, EEADDRESS(checksum), 1);
}
#endif

void save_eeprom_datas(void)
{
    uint8_t m[4] = {0};
    
                   m[0] = capacity_count;
                    m[1] = (capacity_count >> 8);
                    m[2] = (capacity_count >> 16);
                    m[3] = (capacity_count >>24);
//    eep_write_len( EEADDRESS(Capacity_Count), (unsigned char *)(&capacity_count),sizeof(capacity_count));
//    eep_write_len( EEADDRESS(CWattHour_Count), (unsigned char *)(&cwatthour_count),sizeof(cwatthour_count));
    I2C_EE_BufferWrite(m, EEADDRESS(Capacity_Count), 4);
    m[0] = cwatthour_count;
    m[1] = (cwatthour_count >> 8);
    m[2] = (cwatthour_count >> 16);
    m[3] = (cwatthour_count >>24);
    I2C_EE_BufferWrite(m, EEADDRESS(CWattHour_Count), 4);
//	Update_EEPRom_Check_Sum();
}


/**
  * Close the Doxygen i2c_eeprom_Exported_Functions group.
  * @}
  */

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen i2c_eeprom group.
  *    @}
*/

/* End of i2c_eeprom.c */
