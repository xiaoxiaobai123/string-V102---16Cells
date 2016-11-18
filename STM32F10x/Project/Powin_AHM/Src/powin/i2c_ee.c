
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h" 
#include "stm32f10x.h"
#include "i2c_ee.h"
#include "BESS_3-0_AHM_stringdata_V_1-0.h"
#include "messageid.h"
#include "canupgrade.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
 
#define I2C1_SLAVE_ADDRESS7	0xA0  //host address, disposable in this project
#define EEPROM_PageSize		16	  //EEPROM page size, 16-bit
#define EEPROM_BlockSize	256	  //EEPROM block size, to 24C16, each is 256-bit
#define EEPROM_Size			2048// EEPROM size，24C16 2K-bit
 
//#define I2C_Delay 4
#define SCL_H         GPIOB->BSRR = GPIO_Pin_10
#define SCL_L         GPIOB->BRR  = GPIO_Pin_10 
    
#define SDA_H         GPIOB->BSRR = GPIO_Pin_11
#define SDA_L         GPIOB->BRR  = GPIO_Pin_11

#define SCL_read      GPIOB->IDR  & GPIO_Pin_10
#define SDA_read      GPIOB->IDR  & GPIO_Pin_11
 
u8 WriteTOEEBuffer[17] = {0};        //写入EEPROM的数据
 
uint8_t SlaveAddr , ByteAddr;
_EEPROM EEPROM_WriteFlag;
#define OSC     (32)                                 //32M  

#define OSC_D   ((OSC*144)/8) 

#pragma O3 

void Delay_us(unsigned int t) 
{ 
	int i; 
	for(i=0; i < OSC_D * t; i++);
}
void I2C_delay(void)
{        
    uint8_t i = 10;        /*最低可以到5*/
	while(i) 
    { 
		i--; 
    } 
 }

/*******************************************************************************
* Function Name  : I2C_START
* Description    : Modeling I2C bus starting bit
* Input          : None
* Output         : None
* Return         : None
* Author         : Jason Xu
* Date			 : 2015
* Modifier		 : None
* Date           : None
*******************************************************************************/
bool I2C_START(void)
{
	SDA_H;
    SCL_H;
    I2C_delay();
    if(!SDA_read)  return false;        
    SDA_L;
    I2C_delay();
    if(SDA_read) return false;         
    SDA_L;
    I2C_delay();
    return true;
}
/*******************************************************************************
* Function Name  : I2C_STOP
* Description    : Modeling I2C bus generation stop
* Input          : None
* Output         : None
* Return         : None
* Author         : Jeff Zhang
* Date		     : 2010-12-23
* Modifier	     : None
* Date           : None
*******************************************************************************/
void I2C_STOP(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay(); 
}
/*******************************************************************************
* Function Name  : uint8_t I2C_SendByte(uint8_t Data)
* Description    : Send bytes
* Input          : The data to be sent
* Output         : None
* Return         : Responding identifier
* Author         : Jason Xu
* Date			 : 2015-Feb
* Modifier		 : None
* Date           : None
*******************************************************************************/
void I2C_SendByte(uint8_t SendByte)
{
    uint8_t i=8;
     while(i--)
     {
		SCL_L;
        I2C_delay();
		if(SendByte & 0x80)
			SDA_H;  
		else 
			SDA_L;   
        SendByte<<=1;
        I2C_delay();
        SCL_H;
        I2C_delay();
     }
     SCL_L; 	
}
 
uint8_t I2C_ReadByte(void)
{
     uint8_t i=8;
     uint8_t ReceiveByte=0;
	 SDA_H;                                
     while(i--)
     {
		ReceiveByte<<=1;      
		SCL_L;
		I2C_delay();
        SCL_H;
		I2C_delay();        
		if(SDA_read)
		{
			ReceiveByte|=0x01;
		}
     }
     SCL_L;
     return ReceiveByte;	
}
 
void I2C_Ack(void)
{
   SCL_L;
   I2C_delay();
   SDA_L;
   I2C_delay();
   SCL_H;
   I2C_delay();
   SCL_L;
   I2C_delay();
}

void I2C_NoAck(void)
 {        
   SCL_L;
   I2C_delay();
   SDA_H;
   I2C_delay();
   SCL_H;
   I2C_delay();
   SCL_L;
   I2C_delay();
 }
 
bool I2C_WaitAck(void)          //???:=1?ACK,=0?ACK
{
	SCL_L;
	I2C_delay();
	SDA_H;                        
	I2C_delay();
	SCL_H;
	I2C_delay();
	if(SDA_read)
	{
		SCL_L;
		return false;
	}
	SCL_L;
	return true;
}
 
void I2C_EE_Init()
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	/* Configure I2C1 pins: PB10-SCL and PB11-SDA */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;// Open-drain output
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
 

/*EEPROM Write*/
uint8_t EE_I2C_PageWrite(uint8_t *pBuffer)                                
{
	uint8_t i = 0,readcount = 0;
	uint8_t pBuffer_Read[17] = {0};
	uint8_t crccal = 0;
//	uint8_t i2c_delay = I2C_Delay;
//	int Ack = 0;

 	uint8_t SlaveAddr =(*pBuffer - 1) * 16 / EEPROM_BlockSize;           
	uint8_t SlaveByte = (*pBuffer - 1) * 16 % EEPROM_BlockSize;
	pBuffer_Read[0] = *pBuffer;                            /*页号*/
	SlaveAddr = ((SlaveAddr << 1) & 0x0F) | 0xa0 ;               /*写*/
	while(readcount < 3)
	{
		if(!I2C_START()) return false;
		I2C_SendByte(SlaveAddr);                      /*发送control命令*/
		if(!I2C_WaitAck())
		{
			I2C_STOP();
			return false;
		}
		I2C_SendByte(SlaveByte);
		I2C_WaitAck();
		for(i = 1; i < EEPROM_PageSize + 1;i++)
		{
			I2C_SendByte(*(pBuffer + i));
			I2C_WaitAck();
		}
		I2C_STOP();
 		for(i = 0;i < 4;i++)
			Delay_us(10);         // 5 * 10us = 100us
		
		EE_I2C_PageRead(pBuffer_Read);
 		crccal = CRC107(&pBuffer_Read[1],15);
		if(crccal == pBuffer_Read[16])
		{
			return EE_SUCCESS;
		}
		readcount++;
    }
	return EE_ERROR;
}

/*EEPROM Read*/
bool EE_I2C_PageRead(uint8_t *pBuffer)                                
{
	uint8_t i = 0;
	
 	 
	uint8_t SlaveAddr = (*pBuffer - 1) * 16 / EEPROM_BlockSize;
	uint8_t SlaveByte = (*pBuffer - 1) * 16 % EEPROM_BlockSize;
	SlaveAddr = ((SlaveAddr << 1) + 0xa0);               /*读*/
    if(!I2C_START()) return false;
	I2C_SendByte(SlaveAddr);
	if(!I2C_WaitAck())
	{
		I2C_STOP();
		return false;
	}
	I2C_SendByte(SlaveByte);
	I2C_WaitAck();
	I2C_START();
	I2C_SendByte(SlaveAddr | 0x01);
	I2C_WaitAck();
 	for(i = 1; i < EEPROM_PageSize + 1;i++)
	{
		*(pBuffer + i) = I2C_ReadByte();
		if(i == EEPROM_PageSize)
		{
			I2C_NoAck();
			I2C_STOP();
		}
		else
			I2C_Ack();
	}
	return true;
	
}

uint8_t CRC107(unsigned char  *buff,  int len) /*????crc? */
{
    u16 q = 0, r = 0,crc = 0;
    u16 i = 0;				 
    while(q < len)
    {
        if(buff[q] & (0x80>>r))     
            crc |= 0x01;
		if(crc >= 0x100)		
            crc ^= 0x107;	      	 
		crc <<= 1;
		r++;
		if(r == 8)		     	 
		{
            r = 0;
			q++;
		}
    }
    for(i=0; i<8; i++)			 
    {
		if(crc >= 0x100)
            crc ^= 0x107;
		crc <<= 1;
    }
	crc >>= 1;
    return (uint8_t)crc;
}
uint8_t * EE_ProcessBeforeWrite(uint8_t *Message,uint8_t PageNumber)
{
	uint8_t *Src,*Dest;
	static uint8_t EE_Writebuffer[17] = {0};
	Src = Message;
	Dest = &EE_Writebuffer[1];
	MemCopy(Dest,Src,10);
	EE_Writebuffer[0] = PageNumber;
	EE_Writebuffer[16] = CRC107(&EE_Writebuffer[1],15);	 
	return EE_Writebuffer;
}
/*change by jason,2016年5月26日10:18:23 kwh增至 四个byte
  change by jason,2016年6月1日9:36:16 Kwh 的单位改成KWH，四个字节改成两个字节

*/

void CopyAHKWHToEEPROMTemp(void)
{
 	WriteTOEEBuffer[0] =  (u8)mStringData.ah;
	WriteTOEEBuffer[1] = (u8)(mStringData.ah >> 8);
	WriteTOEEBuffer[2] = (u8)mStringData.kWh;
	WriteTOEEBuffer[3] = (u8)(mStringData.kWh >> 8);
	WriteTOEEBuffer[4] = (u8)mStringData.soc;
	WriteTOEEBuffer[5] = (u8)(mStringData.soc >> 8);
	WriteTOEEBuffer[6] = (u8)mStringData.chargeefficiency;
	WriteTOEEBuffer[7] = (u8)(mStringData.chargeefficiency >> 8);
	WriteTOEEBuffer[8] = (u8)(Self_ID);	

 	WriteTOEEBuffer[9] = (u8)(mStringData.kWh >> 16);     /*change by jason,由 四个 byte 改成 2个byte*/
 	WriteTOEEBuffer[10] = (u8)(mStringData.kWh >> 24);
}

void CopyOverVolThresholdValueToEEPROM(void)  /*写 过压  Alarm/Warning*/
{
 	WriteTOEEBuffer[0] =  (u8)SetClrValue.SetCellOverVoltageAlarm;
	WriteTOEEBuffer[1] = (u8)(SetClrValue.SetCellOverVoltageAlarm >> 8);
	WriteTOEEBuffer[2] = (u8)SetClrValue.ClrCellOverVoltageAlarm;
	WriteTOEEBuffer[3] = (u8)(SetClrValue.ClrCellOverVoltageAlarm >> 8);
	WriteTOEEBuffer[4] = (u8)SetClrValue.SetCellOverVoltageWarning;
	WriteTOEEBuffer[5] = (u8)(SetClrValue.SetCellOverVoltageWarning >> 8);
	WriteTOEEBuffer[6] = (u8)SetClrValue.ClrCellOverVoltageWarning;
	WriteTOEEBuffer[7] = (u8)(SetClrValue.ClrCellOverVoltageWarning >> 8);
}

void CopyUnderVolThresholdValueToEEPROM(void) /*写 低压Alarm/Warning */
{
 	WriteTOEEBuffer[0] =  (u8)SetClrValue.SetCellUnderVoltageAlarm;
	WriteTOEEBuffer[1] = (u8)(SetClrValue.SetCellUnderVoltageAlarm >> 8);
	WriteTOEEBuffer[2] = (u8)SetClrValue.ClrCellUnderVoltageAlarm;
	WriteTOEEBuffer[3] = (u8)(SetClrValue.ClrCellUnderVoltageAlarm >> 8);
	WriteTOEEBuffer[4] = (u8)SetClrValue.SetCellUnderVoltageWarning;
	WriteTOEEBuffer[5] = (u8)(SetClrValue.SetCellUnderVoltageWarning >> 8);
	WriteTOEEBuffer[6] = (u8)SetClrValue.ClrCellUnderVoltageWarning;
	WriteTOEEBuffer[7] = (u8)(SetClrValue.ClrCellUnderVoltageWarning >> 8);	
}
void CopyOverTempThresholdValueToEEPROM(void) /*写过 温 Alarm/Warning*/
{
 	WriteTOEEBuffer[0] =  (u8)SetClrValue.SetCellOverTempAlarm;
	WriteTOEEBuffer[1] = (u8)(SetClrValue.SetCellOverTempAlarm >> 8);
	WriteTOEEBuffer[2] = (u8)SetClrValue.ClrCellOverTempAlarm;
	WriteTOEEBuffer[3] = (u8)(SetClrValue.ClrCellOverTempAlarm >> 8);
	WriteTOEEBuffer[4] = (u8)SetClrValue.SetCellOverTempWarning;
	WriteTOEEBuffer[5] = (u8)(SetClrValue.SetCellOverTempWarning >> 8);
	WriteTOEEBuffer[6] = (u8)SetClrValue.ClrCellOverTempWarning;
	WriteTOEEBuffer[7] = (u8)(SetClrValue.ClrCellOverTempWarning >> 8);	
}
void CopyUnderTempThresholdValueToEEPROM(void) /*写 低温 Alarm/Warning*/
{
 	WriteTOEEBuffer[0] =  (u8)SetClrValue.SetCellUnderTempAlarm;
	WriteTOEEBuffer[1] = (u8)(SetClrValue.SetCellUnderTempAlarm >> 8);
	WriteTOEEBuffer[2] = (u8)SetClrValue.ClrCellUnderTempAlarm;
	WriteTOEEBuffer[3] = (u8)(SetClrValue.ClrCellUnderTempAlarm >> 8);
	WriteTOEEBuffer[4] = (u8)SetClrValue.SetCellUnderTempWarning;
	WriteTOEEBuffer[5] = (u8)(SetClrValue.SetCellUnderTempWarning >> 8);
	WriteTOEEBuffer[6] = (u8)SetClrValue.ClrCellUnderTempWarning;
	WriteTOEEBuffer[7] = (u8)(SetClrValue.ClrCellUnderTempWarning >> 8);	
}
void CopyHighTempDeltaThresholdValueToEEPROM(void)
{
 	WriteTOEEBuffer[0] =  (u8)SetClrValue.SetHighCellTempDeltaAlarm;
	WriteTOEEBuffer[1] = (u8)(SetClrValue.SetHighCellTempDeltaAlarm >> 8);
	WriteTOEEBuffer[2] = (u8)SetClrValue.ClrHighCellTempDeltaAlarm;
	WriteTOEEBuffer[3] = (u8)(SetClrValue.ClrHighCellTempDeltaAlarm >> 8);
	WriteTOEEBuffer[4] = (u8)SetClrValue.SetHighCellTempDeltaWarning;
	WriteTOEEBuffer[5] = (u8)(SetClrValue.SetHighCellTempDeltaWarning >> 8);
	WriteTOEEBuffer[6] = (u8)SetClrValue.ClrHighCellTempDeltaWarning;
	WriteTOEEBuffer[7] = (u8)(SetClrValue.ClrHighCellTempDeltaWarning >> 8);	
}
void CopyHighTempRiseThresholdValueToEEPROM(void)
{
 	WriteTOEEBuffer[0] =  (u8)SetClrValue.SetHighCellTempRiseAlarm;
	WriteTOEEBuffer[1] = (u8)(SetClrValue.SetHighCellTempRiseAlarm >> 8);
	WriteTOEEBuffer[2] = (u8)SetClrValue.ClrHighCellTempRiseAlarm;
	WriteTOEEBuffer[3] = (u8)(SetClrValue.ClrHighCellTempRiseAlarm >> 8);
	WriteTOEEBuffer[4] = (u8)SetClrValue.SetHighCellTempRiseWarning;
	WriteTOEEBuffer[5] = (u8)(SetClrValue.SetHighCellTempRiseWarning >> 8);
	WriteTOEEBuffer[6] = (u8)SetClrValue.ClrHighCellTempRiseWarning;
	WriteTOEEBuffer[7] = (u8)(SetClrValue.ClrHighCellTempRiseWarning >> 8);	
}
void CopyChargeCurrentThresholdValueEEPROM(void)
{
 	WriteTOEEBuffer[0] =  (u8)SetClrValue.SetHighChargeRateAlarm;
	WriteTOEEBuffer[1] = (u8)(SetClrValue.SetHighChargeRateAlarm >> 8);
	WriteTOEEBuffer[2] = (u8)SetClrValue.ClrHighChargeRateAlarm;
	WriteTOEEBuffer[3] = (u8)(SetClrValue.ClrHighChargeRateAlarm >> 8);
	WriteTOEEBuffer[4] = (u8)SetClrValue.SetHighChargeRateWarning;
	WriteTOEEBuffer[5] = (u8)(SetClrValue.SetHighChargeRateWarning >> 8);
	WriteTOEEBuffer[6] = (u8)SetClrValue.ClrHighChargeRateWarning;
	WriteTOEEBuffer[7] = (u8)(SetClrValue.ClrHighChargeRateWarning >> 8);		
}
void CopyDischargeCurrentThresholdValueEEPROM(void)
{
 	WriteTOEEBuffer[0] =  (u8)SetClrValue.SetHighDisChargeRateAlarm;
	WriteTOEEBuffer[1] = (u8)(SetClrValue.SetHighDisChargeRateAlarm >> 8);
	WriteTOEEBuffer[2] = (u8)SetClrValue.ClrHighDisChargeRateAlarm;
	WriteTOEEBuffer[3] = (u8)(SetClrValue.ClrHighDisChargeRateAlarm >> 8);
	WriteTOEEBuffer[4] = (u8)SetClrValue.SetHighDisChargeRateWarning;
	WriteTOEEBuffer[5] = (u8)(SetClrValue.SetHighDisChargeRateWarning >> 8);
	WriteTOEEBuffer[6] = (u8)SetClrValue.ClrHighDisChargeRateWarning;
	WriteTOEEBuffer[7] = (u8)(SetClrValue.ClrHighDisChargeRateWarning >> 8);	
	
}
void CopyEEPROMWriteFlag(void)
{
 	WriteTOEEBuffer[0] =  EEPROM_WriteFlag.OverVoltageEEPROMFlag;
	WriteTOEEBuffer[1] =  EEPROM_WriteFlag.UnderVoltageEEPROMFlag;
	WriteTOEEBuffer[2] =  EEPROM_WriteFlag.OverTempEEPROMFlag;
	WriteTOEEBuffer[3] =  EEPROM_WriteFlag.UnderTempEEPROMFlag;
	WriteTOEEBuffer[4] =  EEPROM_WriteFlag.HighDeltaTempEEPROMFlag;
	WriteTOEEBuffer[5] =  EEPROM_WriteFlag.HighTempRiseEEPROMFlag;
	WriteTOEEBuffer[6] =  EEPROM_WriteFlag.HighChargeRateEERPOMFlag;
	WriteTOEEBuffer[7] =  EEPROM_WriteFlag.HighDisChargeRateEEPROMFlag;	
}
void StringEEProm_Write(u8 PageNumber)
{
	uint8_t *Write_Buffer = 0; 
	
	Write_Buffer = EE_ProcessBeforeWrite(WriteTOEEBuffer,PageNumber);
	EE_I2C_PageWrite(Write_Buffer);
							
}

u8 StringEEPromAHKWH_Read(void)        
{
	u8 EE_ReadBuffer[17] = {0};
	u8 CRCu8;
	EE_ReadBuffer[0] = AHKWHEEPageNumber;       /*获取页号*/
    EE_I2C_PageRead(EE_ReadBuffer);                       /*需要读3次*/
	CRCu8 = CRC107(&EE_ReadBuffer[1],15);
	if(CRCu8 == EE_ReadBuffer[16])
	{
		mStringData.ah = EE_ReadBuffer[1] | (EE_ReadBuffer[2] << 8);                     //AH
		mStringData.kWh = EE_ReadBuffer[3] | (EE_ReadBuffer[4] << 8) | (EE_ReadBuffer[10] << 16) | (EE_ReadBuffer[11] << 24);                     //kwh
//		mStringData.kWh = EE_ReadBuffer[3] | (EE_ReadBuffer[4] << 8);      /*change by jason ，only use 2 bytes*/
		mStringData.soc = EE_ReadBuffer[5] | (EE_ReadBuffer[6] << 8);                // capacity
		mStringData.chargeefficiency = EE_ReadBuffer[7] | (EE_ReadBuffer[8] << 8);          //chargeefficiency
//		Self_ID = EE_ReadBuffer[9];                                                       //通过软件 更改 iD号，暂时 不用 
	}
	else
	{
		return 0;
	}
	return 1;
}

u8 ReadEEPROM_Flag(void)
{
	u8 EE_ReadBuffer[17] = {0};
	u8 CRCu8;
	EE_ReadBuffer[0] = EEPROMFlagPageNumber;       /*获取页号*/
    EE_I2C_PageRead(EE_ReadBuffer);                       /*需要读3次*/
	CRCu8 = CRC107(&EE_ReadBuffer[1],15);
	if(CRCu8 == EE_ReadBuffer[16])
	{
		EEPROM_WriteFlag.OverVoltageEEPROMFlag = EE_ReadBuffer[1];
		EEPROM_WriteFlag.UnderVoltageEEPROMFlag = EE_ReadBuffer[2];
		EEPROM_WriteFlag.OverTempEEPROMFlag = EE_ReadBuffer[3];
		EEPROM_WriteFlag.UnderTempEEPROMFlag = EE_ReadBuffer[4];
		EEPROM_WriteFlag.HighDeltaTempEEPROMFlag = EE_ReadBuffer[5];
		EEPROM_WriteFlag.HighTempRiseEEPROMFlag = EE_ReadBuffer[6];
		EEPROM_WriteFlag.HighChargeRateEERPOMFlag = EE_ReadBuffer[7];
		EEPROM_WriteFlag.HighDisChargeRateEEPROMFlag = EE_ReadBuffer[8];
	}
	else
	{
		return 0;
	}
	return 1;	
}

u8 ReadEEPROMParameters()
{
	if(EEPROM_WriteFlag.OverVoltageEEPROMFlag == 1)
	{
		u8 EE_ReadBuffer[17] = {0};
		u8 CRCu8;
		EE_ReadBuffer[0] = OverVoltagePageNumber;       /*获取页号*/
		EE_I2C_PageRead(EE_ReadBuffer);                       /*需要读3次*/
		CRCu8 = CRC107(&EE_ReadBuffer[1],15);
		if(CRCu8 == EE_ReadBuffer[16])
		{
			SetClrValue.SetCellOverVoltageAlarm = EE_ReadBuffer[1] | (EE_ReadBuffer[2] << 8);            
			SetClrValue.ClrCellOverVoltageAlarm = EE_ReadBuffer[3] | (EE_ReadBuffer[4] << 8);            
			SetClrValue.SetCellOverVoltageWarning = EE_ReadBuffer[5] | (EE_ReadBuffer[6] << 8);          
			SetClrValue.ClrCellOverVoltageWarning = EE_ReadBuffer[7] | (EE_ReadBuffer[8] << 8);          		
		}
		else
		{
			return 0;
		}
		 
	}
	if(EEPROM_WriteFlag.UnderVoltageEEPROMFlag == 1)
	{
		u8 EE_ReadBuffer[17] = {0};
		u8 CRCu8;
		EE_ReadBuffer[0] = UnderVoltagePageNumber;       /*获取页号*/
		EE_I2C_PageRead(EE_ReadBuffer);                       /*需要读3次*/
		CRCu8 = CRC107(&EE_ReadBuffer[1],15);
		if(CRCu8 == EE_ReadBuffer[16])
		{
			SetClrValue.SetCellUnderVoltageAlarm = EE_ReadBuffer[1] | (EE_ReadBuffer[2] << 8);            
			SetClrValue.ClrCellUnderVoltageAlarm = EE_ReadBuffer[3] | (EE_ReadBuffer[4] << 8);            
			SetClrValue.SetCellUnderVoltageWarning = EE_ReadBuffer[5] | (EE_ReadBuffer[6] << 8);          
			SetClrValue.ClrCellUnderVoltageWarning = EE_ReadBuffer[7] | (EE_ReadBuffer[8] << 8);          		
		}
		else
		{
			return 0;
		}
		 
	}
	if(EEPROM_WriteFlag.OverTempEEPROMFlag == 1)
	{
		u8 EE_ReadBuffer[17] = {0};
		u8 CRCu8;
		EE_ReadBuffer[0] = OverTempPageNumber;       /*获取页号*/
		EE_I2C_PageRead(EE_ReadBuffer);                       /*需要读3次*/
		CRCu8 = CRC107(&EE_ReadBuffer[1],15);
		if(CRCu8 == EE_ReadBuffer[16])
		{
			SetClrValue.SetCellOverTempAlarm = EE_ReadBuffer[1] | (EE_ReadBuffer[2] << 8);            
			SetClrValue.ClrCellOverTempAlarm = EE_ReadBuffer[3] | (EE_ReadBuffer[4] << 8);            
			SetClrValue.SetCellOverTempWarning = EE_ReadBuffer[5] | (EE_ReadBuffer[6] << 8);          
			SetClrValue.ClrCellOverTempWarning = EE_ReadBuffer[7] | (EE_ReadBuffer[8] << 8);          		
		}
		else
		{
			return 0;
		}
		 
	}
	if(EEPROM_WriteFlag.UnderTempEEPROMFlag == 1)
	{
		u8 EE_ReadBuffer[17] = {0};
		u8 CRCu8;
		EE_ReadBuffer[0] = UnderTempPageNumber;       /*获取页号*/
		EE_I2C_PageRead(EE_ReadBuffer);                       /*需要读3次*/
		CRCu8 = CRC107(&EE_ReadBuffer[1],15);
		if(CRCu8 == EE_ReadBuffer[16])
		{
			SetClrValue.SetCellUnderTempAlarm = EE_ReadBuffer[1] | (EE_ReadBuffer[2] << 8);            
			SetClrValue.ClrCellUnderTempAlarm = EE_ReadBuffer[3] | (EE_ReadBuffer[4] << 8);            
			SetClrValue.SetCellUnderTempWarning = EE_ReadBuffer[5] | (EE_ReadBuffer[6] << 8);          
			SetClrValue.ClrCellUnderTempWarning = EE_ReadBuffer[7] | (EE_ReadBuffer[8] << 8);          		
		}
		else
		{
			return 0;
		}
		 
	}
	if(EEPROM_WriteFlag.HighDeltaTempEEPROMFlag == 1)
	{
		u8 EE_ReadBuffer[17] = {0};
		u8 CRCu8;
		EE_ReadBuffer[0] = HighTempDeltaPageNumber;       /*获取页号*/
		EE_I2C_PageRead(EE_ReadBuffer);                       /*需要读3次*/
		CRCu8 = CRC107(&EE_ReadBuffer[1],15);
		if(CRCu8 == EE_ReadBuffer[16])
		{
			SetClrValue.SetHighCellTempDeltaAlarm = EE_ReadBuffer[1] | (EE_ReadBuffer[2] << 8);            
			SetClrValue.ClrHighCellTempDeltaAlarm = EE_ReadBuffer[3] | (EE_ReadBuffer[4] << 8);            
			SetClrValue.SetHighCellTempDeltaWarning = EE_ReadBuffer[5] | (EE_ReadBuffer[6] << 8);          
			SetClrValue.ClrHighCellTempDeltaWarning = EE_ReadBuffer[7] | (EE_ReadBuffer[8] << 8);          		
		}
		else
		{
			return 0;
		}
		 
	}
	if(EEPROM_WriteFlag.HighTempRiseEEPROMFlag == 1)
	{
		u8 EE_ReadBuffer[17] = {0};
		u8 CRCu8;
		EE_ReadBuffer[0] = HighTempRisePageNumber;       /*获取页号*/
		EE_I2C_PageRead(EE_ReadBuffer);                       /*需要读3次*/
		CRCu8 = CRC107(&EE_ReadBuffer[1],15);
		if(CRCu8 == EE_ReadBuffer[16])
		{
			SetClrValue.SetHighCellTempRiseAlarm = EE_ReadBuffer[1] | (EE_ReadBuffer[2] << 8);            
			SetClrValue.ClrHighCellTempRiseAlarm = EE_ReadBuffer[3] | (EE_ReadBuffer[4] << 8);            
			SetClrValue.SetHighCellTempRiseWarning = EE_ReadBuffer[5] | (EE_ReadBuffer[6] << 8);          
			SetClrValue.ClrHighCellTempRiseWarning = EE_ReadBuffer[7] | (EE_ReadBuffer[8] << 8);          		
		}
		else
		{
			return 0;
		}
		 
	}
	if(EEPROM_WriteFlag.HighChargeRateEERPOMFlag == 1)
	{
		u8 EE_ReadBuffer[17] = {0};
		u8 CRCu8;
		EE_ReadBuffer[0] = HighChargeCurrentPageNumber;       /*获取页号*/
		EE_I2C_PageRead(EE_ReadBuffer);                       /*需要读3次*/
		CRCu8 = CRC107(&EE_ReadBuffer[1],15);
		if(CRCu8 == EE_ReadBuffer[16])
		{
			SetClrValue.SetHighChargeRateAlarm = EE_ReadBuffer[1] | (EE_ReadBuffer[2] << 8);            
			SetClrValue.ClrHighChargeRateAlarm = EE_ReadBuffer[3] | (EE_ReadBuffer[4] << 8);            
			SetClrValue.SetHighChargeRateWarning = EE_ReadBuffer[5] | (EE_ReadBuffer[6] << 8);          
			SetClrValue.ClrHighChargeRateWarning = EE_ReadBuffer[7] | (EE_ReadBuffer[8] << 8);          		
		}
		else
		{
			return 0;
		}
		 
	}
	if(EEPROM_WriteFlag.HighDisChargeRateEEPROMFlag == 1)
	{
		u8 EE_ReadBuffer[17] = {0};
		u8 CRCu8;
		EE_ReadBuffer[0] = HighDisChargeCurrentPageNumber;       /*获取页号*/
		EE_I2C_PageRead(EE_ReadBuffer);                       /*需要读3次*/
		CRCu8 = CRC107(&EE_ReadBuffer[1],15);
		if(CRCu8 == EE_ReadBuffer[16])
		{
			SetClrValue.SetHighDisChargeRateAlarm = EE_ReadBuffer[1] | (EE_ReadBuffer[2] << 8);            
			SetClrValue.ClrHighDisChargeRateAlarm = EE_ReadBuffer[3] | (EE_ReadBuffer[4] << 8);            
			SetClrValue.SetHighDisChargeRateWarning = EE_ReadBuffer[5] | (EE_ReadBuffer[6] << 8);          
			SetClrValue.ClrHighDisChargeRateWarning = EE_ReadBuffer[7] | (EE_ReadBuffer[8] << 8);          		
		}
		else
		{
			return 0;
		}
		 
	}
}
