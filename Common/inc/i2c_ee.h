
/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __I2C_EE_H
#define __I2C_EE_H
#include <stm32f10x.h> 
#include <stdbool.h>
/* Includes ------------------------------------------------------------------*/
typedef enum
{
	EE_SUCCESS = 0,
	EE_ERROR = 1 ,
	EE_TIMEOUT = 2
}EE_WriteStatus; 


 

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Delay(vu32 nCount);
void I2C_EE_Init(void);
_Bool I2C_START(void);
void I2C_STOP(void);
void I2C_SendByte(u8 Data);
u8 I2C_ReadByte(void);
void I2C_EE_ByteWrite(u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite);
void I2C_EE_PageWrite(u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite);
void I2C_EE_BufferWrite(u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite);
void I2C_EE_BufferRead(u8* pBuffer, u16 ReadAddr, u16 NumByteToRead);
void I2C_EE_FillByte(u8 Byte,u16 StartAddr,u16 StopAddr);
void I2C_EE_BufferUpdate(u8* pBuffer, u16 UpdateAddr,u16 NumByteToUpdate);
u8 EE_I2C_PageWrite(u8* pBuffer);
bool EE_I2C_PageRead(u8* pBuffer);
u8 EE_I2C_WriteCheck(u8 *pBuffer);
u8 CRC107(unsigned char  *buff,  int len);
u8 crc8(u8 *precdata,u8 recdata_long);
u8 * EE_ProcessBeforeWrite(u8 *Message,u8 PageNumber);
#endif /* __I2C_EE_H */



