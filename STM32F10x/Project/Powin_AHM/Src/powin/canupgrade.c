/**
*/ 
/* Includes ------------------------------------------------------------------*/
//#include "canprotocol.h"
/* Scheduler includes. */
//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
#ifdef _WINDOWS
#include "wincandriver.h"
#endif

#include "cancmd.h"


#include "canupgrade.h"


#define DEBUG_UPGRADE 0
#define DEBUG_UPGRADE_NORMAL 0



#if(SUPPORT_SERVER_UPGRADE)

#ifdef _WINDOWS
    uint8_t bp_id = 0x3;
    #include "canprotocol.h"
#else
    #include "string.h"
    #if(SERVER_UPGRADE_BP)
        #include "can.h"
        #include "can_rx.h"
        extern volatile uint8_t bp_id;  
    #endif
    
    #if(SERVER_UPGRADE_STRING)
        #include <stdio.h> 
        #include <stdlib.h>
        #include "BESS_3-0_AHM_can_V_1-0.h"
        #include "booterlib.h"
        #include "memoryconfig.h"
    #endif 

//#define INFINITE portMAX_DELAY

#endif


static uint8_t upgradeServerStatus = UPGRADE_STATUS_STANDBY;

static uint8_t upgradeBurnBuffer[PART_BURN_LEN];

static uint16_t upgradeBurnBufferLen = 0;
static uint32_t upgradeBurnTargetAddress = 0x0;
static uint16_t upgradeBurnTargetLen = 0;
static boolean  upgradeBypassBP = false;
#endif//#if(SUPPORT_SERVER_UPGRADE)

#if(SUPPORT_CLIENT_UPGRADE)
static uint32_t mClientMainProcessWaitTime = INFINITE;
static uint8_t upgradeClientStatus = UPGRADE_STATUS_NONE;
static uint8_t upgradeClientType = CLIENT_UPGRADE_TYPE_BP;
static CANUpgradeACKRecord canUpgradeACKRecord[UPGRADE_SERVER_SUPPORT_NUM];

static uint32_t upgradeFirmwareLocalAddress = 0x8007800;
static uint32_t upgradeFirmwareTargetBaseAddress = 0x0;
static uint32_t upgradeFirmwareTargetLen = 0x0;
static uint32_t upgradeFirmwareLenLeft = 0x0;

static uint32_t upgradeFirmwareTargetAddress = 0x0;
#endif//#if(SUPPORT_CLIENT_UPGRADE)


static uint8_t getCheckSum(uint32_t startPr, uint16_t len)
{
    uint8_t* pr = (uint8_t*)startPr;
    int i;
    uint8_t checkSum = 0;
    for (i = 0; i < len; i++)
    {
        checkSum = checkSum + pr[i];
    }
    return checkSum;
}

static char* getStatusString(uint8_t status)
{
    //printf("getStatusString:%d\r\n", status);
    switch (status)
    {
    case UPGRADE_STATUS_NONE:
        return " (0xff)STATUS_NONE ";
    case UPGRADE_STATUS_STANDBY:
        return " (0x00)STATUS_STANDBY ";
    case UPGRADE_STATUS_READY_TO_RECEIVE:
        return " (0x01)READY_TO_RECEIVE ";
    case UPGRADE_STATUS_SENDING_DATA:
        return " (0x02)SENDING_DATA ";
    case UPGRADE_STATUS_WAITING_TO_SEND:
        return " (0x03)WAITING_TO_SEND ";
    case UPGRADE_STATUS_WAITING_FOR_RECEIVED:
        return " (0x04)WAITING_FOR_RECEIVED ";
    case UPGRADE_STATUS_WAITING_FOR_BURNED:
        return " (0x05)WAITING_FOR_BURNED ";
    case UPGRADE_STATUS_WAITING_TO_BURN:
        return " (0x06)WAITING_TO_BURN ";
    case UPGRADE_STATUS_FINISH_TO_BURN:
        return " (0x07)FINISH_TO_BURN ";
    }
    return "CANT_GET_STRING";

}
#ifdef _WINDOWS
static void showCanMsg(CString title, uint8_t* data, uint8_t dataLen)
{
#if(DEBUG_UPGRADE)
    CString str, tmpstr;
    str.Format(CString("[%s, len=%d]:"), title, dataLen);
    for (int j = 0; j < dataLen; j++)
    {
        tmpstr.Format(CString("0x%02x "), data[j]);
        str += tmpstr;
    }
    str += "\r\n";
    GolbalUpdateMessage(str, false);
#endif
}
#endif

static boolean startBurnFunction(uint32_t address, uint8_t* data, uint16_t dataLen)
{
#ifdef _WINDOWS
    CString str, tmpstr;
#if(SERVER_UPGRADE_BP)
    str.Format(CString("==> startBurnFunction(BP): [address:0x%08x, len=%d]:\r\n"), address, dataLen);
#elif(SERVER_UPGRADE_STRING)
    str.Format(CString("==> startBurnFunction(String): [address:0x%08x, len=%d]:\r\n"), address, dataLen);
#endif
    
#if(0)
    for (int j = 0; j < dataLen; j++)
    {
        tmpstr.Format(CString("0x%02x "), data[j]);
        str += tmpstr;
        if ((j % 0x10) == 0xf)
            str += CString("\r\n");
    }
    //str += "\r\n";
#endif
    GolbalUpdateMessage(str, false);
    return true;
#else//#ifdef _WINDOWS
    
    int j;
#if(SERVER_UPGRADE_BP)
    printf("==> startBurnFunction(BP): [address:0x%08x, len=%d]:\r\n", address, dataLen);
        return true;
#elif(SERVER_UPGRADE_STRING)
    #if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
        #define FLASH_PAGE_SIZE    ((uint16_t)0x800)
    #else
        #define FLASH_PAGE_SIZE    ((uint16_t)0x400)
    #endif

    #define BANK1_WRITE_START_ADDR  ((uint32_t)address)
    #define BANK1_WRITE_END_ADDR    ((uint32_t)(address + dataLen))

    uint32_t EraseCounter = 0x00, Address = 0x00;
    uint32_t Data;// = (uint32_t*)data;// = 0x3210ABCD;
    uint32_t NbrOfPage = 0x00;
    volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
    volatile boolean MemoryProgramStatus = true;
    printf("==> startBurnFunction(String): [address:0x%08x, len=%d]:\r\n", address, dataLen);
    /* Porgram FLASH Bank1 ********************************************************/       
    /* Unlock the Flash Bank1 Program Erase controller */
    FLASH_UnlockBank1();

    /* Define the number of page to be erased */
    NbrOfPage = (BANK1_WRITE_END_ADDR - BANK1_WRITE_START_ADDR) / FLASH_PAGE_SIZE;

    /* Clear All pending flags */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

    /* Erase the FLASH pages */
    for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
    {
        FLASHStatus = FLASH_ErasePage(BANK1_WRITE_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter));
    }
  
    /* Program Flash Bank1 */
    Address = BANK1_WRITE_START_ADDR;

    while((Address < BANK1_WRITE_END_ADDR) && (FLASHStatus == FLASH_COMPLETE))
    {
        Data = *(uint32_t*)(data+Address-BANK1_WRITE_START_ADDR);
        //printf("W:0x%08x->0x%08x]:\r\n", Address, Data);
        FLASHStatus = FLASH_ProgramWord(Address, Data);
        Address = Address + 4;
    }

    FLASH_LockBank1();
  
    /* Check the correctness of written data */
    Address = BANK1_WRITE_START_ADDR;

    while((Address < BANK1_WRITE_END_ADDR) && (MemoryProgramStatus != false))
    {
         Data = *(uint32_t*)(data+Address-BANK1_WRITE_START_ADDR);
        if((*(__IO uint32_t*) Address) != Data)
        {
            MemoryProgramStatus = false;
        }
        Address += 4;
    }
    return MemoryProgramStatus;
#endif
    


#endif//#ifdef _WINDOWS


}



//function
#if(SUPPORT_SERVER_UPGRADE)
#ifdef _WINDOWS
uint8_t verFlag = 0xff;
uint8_t CanDriverGetLocalDeviceId(void)
{
    //#warning CanDriverGetLocalDeviceId....
    return bp_id;
}
uint8_t CanDriverFirwareVer(void)
{
    return 55;
}
uint8_t GetFirwareBootFlag(void)
{
    return verFlag;
}

uint8_t SetFirwareBootFlag(uint8_t flag)
{
    verFlag = flag;
    return verFlag;
}
#else
uint8_t verFlag = 0xff;
#if(SERVER_UPGRADE_BP)
boolean SendCanMessageBPToString(uint8_t groupId, uint8_t messageId, uint8_t deviceId, uint8_t* data, uint8_t dataLen)
{
    uint32_t ID = 0;
    ID = MAKE_ID(0,         
        MSG_GRP_ID_DATA,   
        DVC_TP_ID_BP,      
        STRING_ID_BROADCAST,
        deviceId, 
        messageId
        );

    can_make_send(data, 
        dataLen,  
        ID, 
        1   
        );
    return true;
}
uint8_t CanDriverGetLocalDeviceId(void)
{
    return bp_id;
}
#endif
    
#if(SERVER_UPGRADE_STRING)
boolean SendCanMessageStringToArray(uint8_t messageId, uint8_t* data, uint8_t dataLen)
{
    //void CAN_make_send_2_Array (uint8_t *buf, uint8_t length, uint8_t msg_id)
    CAN_make_send_2_Array(data, dataLen, messageId);
    return true;
}
boolean SendCanMessageStringToBP(uint8_t messageId, uint8_t* data, uint8_t dataLen)
{
    //void CAN_make_send_bypass_2_BP(uint8_t *buf, uint8_t length, uint8_t msg_id, uint8_t device_id)
    CAN_make_send_bypass_2_BP(data, dataLen, messageId, 0);
    return true;
}
uint8_t CanDriverGetLocalDeviceId(void)
{//沒用到, CAN_make_send_2_Array終究有ID了
    return 0;
}

#endif



uint8_t CanDriverFirwareVer(void)
{
    return STRING_VERSION;
	 
}
uint8_t GetFirwareBootFlag(void)
{    
    return GetBootConfig();
}

uint8_t SetFirwareBootFlag(uint8_t flag)
{
    SetBootConfig(flag);
    return GetBootConfig();
}

void SendFirmwareInfo(void)
{
    uint8_t data[8];
    uint16_t len;

    data[FIRMWARE_INFO_CMD_TYPE_INDEX] = FIRMWARE_INFO_CMD_TYPE_QUERY;
    len = FIRMWARE_INFO_CMD_TYPE_INDEX + 1;
    
    CanServerFirmwareInfoProcess(data, len);
}
#endif

static boolean SendCanMessageServerToClient(uint8_t groupId, uint8_t messageId, uint8_t deviceId, uint8_t* data, uint8_t dataLen)
{
    #if(SERVER_UPGRADE_BP)
        return SendCanMessageBPToString(groupId, messageId, deviceId, data, dataLen);
    #elif(SERVER_UPGRADE_STRING)
        return SendCanMessageStringToArray(messageId, data, dataLen);
    #endif
   
}
void CanServerSetBPBypassMode(boolean mode)
{
    upgradeBypassBP = mode;
}

boolean CanServerGetBPBypassMode(void)
{
    return upgradeBypassBP;
}

void CanServerCommandActionProcess(uint8_t* data, uint8_t dataLen)
{
    //uint8_t dataTmp[8] = { 0 }, len = 0;
#ifdef _WINDOWS
    CString str;
#endif
    switch (data[COMMAND_ACTION_CMD_TYPE_INDEX])
    {
    case COMMAND_ACTION_CMD_TYPE_REBOOT:
        
#ifdef _WINDOWS
        str.Format(CString("Local ID(%d) Reboot: %d\r\n"), CanDriverGetLocalDeviceId(), data[COMMAND_ACTION_CMD_TYPE_INDEX]);
        GolbalUpdateMessage(str, false);
#else
        printf("Local ID(%d) Reboot: %d\r\n", CanDriverGetLocalDeviceId(), data[COMMAND_ACTION_CMD_TYPE_INDEX]);
#endif 
        NVIC_SystemReset();        
        break;    

    case COMMAND_ACTION_CMD_TYPE_UPGRADE_BYPASS_BP:        
#ifdef _WINDOWS
        str.Format(CString("Local ID(%d):Set Upgrade Bypass BP\r\n"), CanDriverGetLocalDeviceId());
        GolbalUpdateMessage(str, false);
#else
        printf("Local ID(%d):Set Upgrade Bypass BP\r\n\r\n", CanDriverGetLocalDeviceId());
#endif
        CanServerSetBPBypassMode(true);        
        break;  
    
    case COMMAND_ACTION_CMD_TYPE_UPGRADE_JUST_STRING:        
#ifdef _WINDOWS
        str.Format(CString("Local ID(%d):Set Upgrade Just String\r\n"), CanDriverGetLocalDeviceId());
        GolbalUpdateMessage(str, false);
#else
        printf("Local ID(%d):Set Upgrade Just String\r\n\r\n", CanDriverGetLocalDeviceId());
#endif
        CanServerSetBPBypassMode(false);            
        break;  
    default:
        break;
    }
}
void CanServerFirmwareInfoProcess(uint8_t* data, uint8_t dataLen)
{
    uint8_t dataTmp[8] = { 0 }, len = 0;
#ifdef _WINDOWS
    CString str;
#endif
    switch (data[FIRMWARE_INFO_CMD_TYPE_INDEX])
    {
    case FIRMWARE_INFO_CMD_TYPE_QUERY:
        dataTmp[FIRMWARE_INFO_CMD_TYPE_INDEX] = FIRMWARE_INFO_CMD_TYPE_ACK;
        dataTmp[FIRMWARE_INFO_CMD_BOOT_FLAG_INDEX] = GetFirwareBootFlag();
        dataTmp[FIRMWARE_INFO_CMD_VER_INDEX] = CanDriverFirwareVer();
        len = FIRMWARE_INFO_CMD_VER_INDEX + 1;
        #ifdef _WINDOWS
        str.Format(CString("Server(%d) return Ver: %d, Boot Flag: %d ...\r\n"), CanDriverGetLocalDeviceId(), 
            dataTmp[FIRMWARE_INFO_CMD_VER_INDEX], dataTmp[FIRMWARE_INFO_CMD_BOOT_FLAG_INDEX]);
        GolbalUpdateMessage(str, false);
        #else
        printf("Server(%d) return Ver: %d, Boot Flag: %d ...\r\n", CanDriverGetLocalDeviceId(), 
                dataTmp[FIRMWARE_INFO_CMD_VER_INDEX], dataTmp[FIRMWARE_INFO_CMD_BOOT_FLAG_INDEX]);
        #endif
        SendCanMessageServerToClient(CMD_GROUP_ID_DATA, CMD_ID_SERVER_FIRMWARE_INFO, CanDriverGetLocalDeviceId(), dataTmp, len);
        break;
    case FIRMWARE_INFO_CMD_TYPE_SET:
        SetFirwareBootFlag(data[FIRMWARE_INFO_CMD_BOOT_FLAG_INDEX]);
        #ifdef _WINDOWS
        str.Format(CString("Server(%d) get Boot Flag: %d(%d) ...\r\n"), CanDriverGetLocalDeviceId(), 
            data[FIRMWARE_INFO_CMD_BOOT_FLAG_INDEX], GetFirwareBootFlag());
        GolbalUpdateMessage(str, false);
        #else
        printf("Server(%d) get Boot Flag: %d(%d) ...\r\n", CanDriverGetLocalDeviceId(), 
            data[FIRMWARE_INFO_CMD_BOOT_FLAG_INDEX], GetFirwareBootFlag());
        #endif
        break;
    default:
        break;
    }
}
void CanUpgradeServerProcess(uint8_t* data, uint8_t dataLen)
{
    uint8_t dataTmp[8] = { 0 }, len = 0;
#ifdef _WINDOWS
    CString str;
    showCanMsg(CString("-- Server Process[") + CString(getStatusString(upgradeServerStatus)) + CString("] --"), data, dataLen);
#else
#if(0)
    int j;
    printf("-- Server Process[%s] --", getStatusString(upgradeServerStatus));
    for (j = 0; j < dataLen; j++)
    {
        printf("0x%02x ", data[j]);

    }
    printf("\r\n");
#endif
#endif

    if (dataLen == (UPGRADE_CMD_READ_MEM_SIZE_INDEX_2 + 1))
    {
        switch (data[UPGRADE_CMD_FLAG_INDEX])
        {
        case UPGRADE_CMD_REQUEST:
        {
             uint32_t targetAddress = (uint32_t)data[UPGRADE_CMD_MEM_ADDRESS_BYTE_0_INDEX] |
                ((uint32_t)data[UPGRADE_CMD_MEM_ADDRESS_BYTE_1_INDEX] << 8) |
                ((uint32_t)data[UPGRADE_CMD_MEM_ADDRESS_BYTE_2_INDEX] << 16) |
                ((uint32_t)data[UPGRADE_CMD_MEM_ADDRESS_BYTE_3_INDEX] << 24);
            uint16_t readLen = data[UPGRADE_CMD_READ_MEM_SIZE_INDEX] | data[UPGRADE_CMD_READ_MEM_SIZE_INDEX_2] << 8;
            
            upgradeServerStatus = UPGRADE_STATUS_READY_TO_RECEIVE;           

            upgradeBurnTargetAddress = targetAddress;
            upgradeBurnTargetLen = readLen;

            memset(upgradeBurnBuffer, 0x0, sizeof(upgradeBurnBuffer));
            upgradeBurnBufferLen = 0;
#if(DEBUG_UPGRADE)
            str.Format(CString("Server get Request: address:0x%08x, len = %d ...\r\n"), upgradeBurnTargetAddress, upgradeBurnTargetLen);
            GolbalUpdateMessage(str, false);
#endif
            dataTmp[UPGRADE_CMD_FLAG_INDEX] = UPGRADE_CMD_ACK;
            dataTmp[UPGRADE_STATUS_ID_INDEX] = UPGRADE_STATUS_READY_TO_RECEIVE;
            len = UPGRADE_STATUS_ID_INDEX + 1;
            SendCanMessageServerToClient(CMD_GROUP_ID_DATA, CMD_ID_SERVER_UPGRADE_WRITE_MEMORY, CanDriverGetLocalDeviceId(), dataTmp, len);
        }
            break;
        default:
            #ifdef _WINDOWS
            str.Format(CString("*** Server Process not implement_1(data[UPGRADE_CMD_FLAG_INDEX]:0x%02x)...\r\n"), data[UPGRADE_CMD_FLAG_INDEX]);
            GolbalUpdateMessage(str, false);
            #else
            printf("*** Server Process not implement_1(data[0]:0x%02x)...\r\n", data[0]);
            #endif

            break;
        }
    }
    else if (dataLen == (UPGRADE_STATUS_ID_INDEX + 1))
    {
        switch (upgradeServerStatus)
        {    
        case UPGRADE_STATUS_WAITING_TO_BURN:
            switch (data[UPGRADE_CMD_FLAG_INDEX])
            {
            case UPGRADE_CMD_ACK:
#if(1)
#ifdef _WINDOWS
                str.Format(CString("\r\n==> ###  <Start burning... (0x%08x, %d)>  ###\r\n"), upgradeBurnTargetAddress, upgradeBurnTargetLen);
                GolbalUpdateMessage(str, false);
#else
                printf("\r\n==> ###  <Start burning... (0x%08x, %d)>  ###\r\n", upgradeBurnTargetAddress, upgradeBurnTargetLen);
#endif

#endif
                if (startBurnFunction(upgradeBurnTargetAddress, upgradeBurnBuffer, upgradeBurnTargetLen))
                {
#ifdef _WINDOWS
                    str.Format(CString("=======  Burn OK  ======\r\n"));
                    GolbalUpdateMessage(str, false);
#else
                    printf("=======  Burn OK  ======\r\n");
#endif

                    upgradeServerStatus = UPGRADE_STATUS_FINISH_TO_BURN;
                    dataTmp[UPGRADE_CMD_FLAG_INDEX] = UPGRADE_CMD_ACK;
                    dataTmp[UPGRADE_STATUS_ID_INDEX] = UPGRADE_STATUS_WAITING_TO_BURN;

                }
                else
                {
#ifdef _WINDOWS
                    str.Format(CString("=======  Burn Error  ======\r\n"));
                    GolbalUpdateMessage(str, false);
#else
                    printf("=======  Burn Error  ======\r\n");
#endif

                    upgradeServerStatus = UPGRADE_STATUS_NONE;
                    dataTmp[UPGRADE_CMD_FLAG_INDEX] = UPGRADE_CMD_NACK;
                    dataTmp[UPGRADE_STATUS_ID_INDEX] = UPGRADE_STATUS_NONE;
                }

                len = UPGRADE_STATUS_ID_INDEX + 1;
                SendCanMessageServerToClient(CMD_GROUP_ID_DATA, CMD_ID_SERVER_UPGRADE_WRITE_MEMORY, CanDriverGetLocalDeviceId(), dataTmp, len);
                break;
            default:
#ifdef _WINDOWS
                str.Format(CString("*** Server Process not implement_2(data[0]:0x%02x)...\r\n"), data[0]);
                GolbalUpdateMessage(str, false);
#else
                printf("*** Server Process not implement_2(data[0]:0x%02x)...\r\n", data[0]);
#endif
                break;

            }
            break;
        default:
#ifdef _WINDOWS
            str.Format(CString("* error Server Status(%s)...\r\n"), CString(getStatusString(upgradeServerStatus)));
            GolbalUpdateMessage(str, false);
#else
            printf("* error Server Status(%s)...\r\n", getStatusString(upgradeServerStatus));
#endif

            break;
        }
    }
    else if (dataLen == (UPGRADE_CHECKSUM_ID_INDEX + 1))
    {
        uint8_t checkSum;
        switch (upgradeServerStatus)
        {
        case UPGRADE_STATUS_READY_TO_RECEIVE:
            switch (data[UPGRADE_CMD_FLAG_INDEX])
            {
            case UPGRADE_CMD_ACK:
#if(DEBUG_UPGRADE)
#ifdef _WINDOWS
                str.Format(CString("==> get RAW Data finish (upgradeBurnBufferLen = %d(target len:%d)...\r\n")
                    , upgradeBurnBufferLen, upgradeBurnTargetLen);
                GolbalUpdateMessage(str, false);
#else
                printf("==> get RAW Data finish (upgradeBurnBufferLen = %d(target len:%d)...\r\n", upgradeBurnBufferLen, upgradeBurnTargetLen);
#endif

#endif
                checkSum = getCheckSum((uint32_t)upgradeBurnBuffer, upgradeBurnTargetLen);


                if((upgradeBurnBufferLen == upgradeBurnTargetLen) && (checkSum == data[UPGRADE_CHECKSUM_ID_INDEX]))
                {
//#if(DEBUG_UPGRADE)
#if(1)
#ifdef _WINDOWS
                    str.Format(CString("==> Receive OK, checkSum = 0x%02x(0x%02x), Len = %d(%d)...\r\n")
                        , checkSum, data[UPGRADE_CHECKSUM_ID_INDEX], upgradeBurnBufferLen, upgradeBurnTargetLen);
                    GolbalUpdateMessage(str, false);
#else
                    printf("==> Receive OK, checkSum= 0x%02x(0x%02x), Len = %d(%d)...\r\n", checkSum, data[UPGRADE_CHECKSUM_ID_INDEX], upgradeBurnBufferLen, upgradeBurnTargetLen);
#endif

#endif
                    upgradeServerStatus = UPGRADE_STATUS_WAITING_TO_BURN;
                    dataTmp[UPGRADE_CMD_FLAG_INDEX] = UPGRADE_CMD_ACK;
                    dataTmp[UPGRADE_STATUS_ID_INDEX] = UPGRADE_STATUS_WAITING_TO_BURN;

                }
                else
                {
                    //#if(DEBUG_UPGRADE)
#if(1)
#ifdef _WINDOWS
                    //str.Format(CString("==> Receive error...\r\n"));
                    GolbalUpdateMessage(str, false);
                    str.Format(CString(" > Receive error, checkSum = 0x%02x(0x%02x), Len = %d(%d)...\r\n")
                        , checkSum, data[UPGRADE_CHECKSUM_ID_INDEX], upgradeBurnBufferLen, upgradeBurnTargetLen);
                    GolbalUpdateMessage(str, false);
#else
                    //printf("==> Receive error...\r\n");
                    printf(" > Receive error, checkSum= 0x%02x(0x%02x), Len = %d(%d)...\r\n", checkSum, data[UPGRADE_CHECKSUM_ID_INDEX], upgradeBurnBufferLen, upgradeBurnTargetLen);
#endif

#endif

                    upgradeServerStatus = UPGRADE_STATUS_NONE;
                    dataTmp[UPGRADE_CMD_FLAG_INDEX] = UPGRADE_CMD_NACK;
                    dataTmp[UPGRADE_STATUS_ID_INDEX] = UPGRADE_STATUS_NONE;
                }

                len = UPGRADE_STATUS_ID_INDEX + 1;
                SendCanMessageServerToClient(CMD_GROUP_ID_DATA, CMD_ID_SERVER_UPGRADE_WRITE_MEMORY, CanDriverGetLocalDeviceId(), dataTmp, len);
                break;
            default:
#ifdef _WINDOWS
                str.Format(CString("*** Server Process not implement_2(data[UPGRADE_CMD_FLAG_INDEX]:0x%02x)...\r\n"), data[UPGRADE_CMD_FLAG_INDEX]);
                GolbalUpdateMessage(str, false);
#else
                printf("*** Server Process not implement_2(data[0]:0x%02x)...\r\n", data[0]);
#endif

                break;
            }
            break;
       
        default:
#ifdef _WINDOWS
            str.Format(CString("* error Server Status(%s)...\r\n"), CString(getStatusString(upgradeServerStatus)));
            GolbalUpdateMessage(str, false);
#else
            printf("* error Server Status(%s)...\r\n", getStatusString(upgradeServerStatus));
#endif

            break;
        }
    }
    else if (dataLen == 8)
    {
        switch (upgradeServerStatus)
        {
        case UPGRADE_STATUS_READY_TO_RECEIVE:
            memcpy(upgradeBurnBuffer + upgradeBurnBufferLen, data, dataLen);
            upgradeBurnBufferLen = upgradeBurnBufferLen + dataLen;
//#if(DEBUG_UPGRADE)
#if(DEBUG_UPGRADE)
            str.Format(CString("--> get RAW Data(upgradeBurnBufferLen = %d)...\r\n"), upgradeBurnBufferLen);
            GolbalUpdateMessage(str, false);
#endif
            break;
        default:
#ifdef _WINDOWS
            str.Format(CString("* error Server Status(%s)...\r\n"), CString(getStatusString(upgradeServerStatus)));
            GolbalUpdateMessage(str, false);
#else
            printf("* error Server Status(%s)...\r\n", getStatusString(upgradeServerStatus));
#endif

            break;
        }
    }
    else
    {
    }    
    
}
#endif

#if(SUPPORT_CLIENT_UPGRADE)
static void setWaitingTime(DWORD time)
{
    mClientMainProcessWaitTime = time;
    CanUpgradeSetEvent();
}
static void setPureWaitingTime(void)
{
    CanUpgradeSetEvent();
}
static void cancelWaitingTime(void)
{
    mClientMainProcessWaitTime = INFINITE;
    CanUpgradeSetEvent();
}
static void resetUpgradeACKRecordFlag(void)
{
    for (int i = 0; i<UPGRADE_SERVER_SUPPORT_NUM; i++)
    {
        canUpgradeACKRecord[i].ackFlag = false;
        //isInWantList = true;
    }
}
static boolean addUpgradeACKRecordFlag(uint8_t stringId, uint8_t deviceId)
{
    int i;
    uint8_t targertId = 0x0;
    boolean returnVal = true;
    boolean isInWantList = false;
    uint8_t totalNum = 0x0;

    canUpgradeACKRecord[deviceId-1].ackFlag = true;

    for (i = 0; i<UPGRADE_SERVER_SUPPORT_NUM; i++)
    {
        if (canUpgradeACKRecord[i].enable)
        {
            if (canUpgradeACKRecord[i].ackFlag == false)
            {
                return false;
            }
        }

    }
    return true;;
}
static boolean SendCanMessageClientToServer(uint8_t groupId, uint8_t messageId, uint8_t deviceId, uint8_t* data, uint8_t dataLen)
{
    switch (upgradeClientType)
    {
        case CLIENT_UPGRADE_TYPE_BP:
            return SendCanMessageStringToBP(groupId, messageId, deviceId, data, dataLen);
            break;
        case CLIENT_UPGRADE_TYPE_STRING:
            return SendCanMessageArrayToString(groupId, messageId, deviceId, data, dataLen);
            break;
        default:
            break;
    }
}

static uint32_t UpgradeMsgSendDataClient2Server(uint8_t targetId, uint32_t address, uint16_t length)
{
    uint8_t data[8] = { 0 }, len = 0;
    uint16_t readLen;
    int i, j;
    CString str;
    
    uint32_t targetAddress = upgradeFirmwareLocalAddress + address - upgradeFirmwareTargetBaseAddress;
#if(DEBUG_UPGRADE)
    str.Format(CString("Send Data:localAddress = 0x%08x, len = %d...\r\n"), targetAddress, length);
    GolbalUpdateMessage(str, false);
#else
    printf("<");
#endif
    //---
    readLen = length;  //0xff resprent 256
    for (j = 0; j< readLen / 8; j++)
    {
        for (i = 0; i< 8; i++)
        {
            uint8_t* reg8 = (uint8_t*)targetAddress + j * 8 + i;
            data[i] = *reg8;
#if(DEBUG_UPGRADE)
            str.Format(CString("- %d(%d) -: 0x%02x\r\n"), i, j * 8 + i, data[i]);
            GolbalUpdateMessage(str, false);
#endif
        }
        //TEMP
        //data[UPGRADE_CMD_FLAG_INDEX] = j;
        //
        len = 8;
        //SendCanMessageStringToBatteryPack(targetId, msgId, data, len, false);
        SendCanMessageClientToServer(CMD_GROUP_ID_DATA, CMD_ID_SERVER_UPGRADE_WRITE_MEMORY, targetId, data, len);
   

    }
    for (i = 0; i< readLen % 8; i++)
    {
        uint8_t* reg8 = (uint8_t*)targetAddress + (readLen / 8) * 8 + i;
        data[i] = *reg8;
#if(DEBUG_UPGRADE)
        str.Format(CString("= %d(%d) =: 0x%02x\r\n"), i, (readLen / 8) * 8 + i, data[i]);
        GolbalUpdateMessage(str, false);
#endif
    }
    len = readLen % 8;
    if (len > 0)
    {
        //SendCanMessageStringToBatteryPack(targetId, msgId, data, len, false);
        SendCanMessageClientToServer(CMD_GROUP_ID_DATA, CMD_ID_SERVER_UPGRADE_WRITE_MEMORY, targetId, data, len);
    }
    //---  
    return targetAddress;
  
}

void CanUpgradeStart(uint32_t srcAddress, uint32_t destAddress, uint32_t length)
{
    uint8_t data[8] = { 0 }, len = 0;
    boolean returnVal = true;
    CString str;
    str.Format(CString("[CanUpgradeStart] src = 0x%08x, dest = 0x%08x, length = %d\r\n"), 
                    srcAddress, destAddress, length);
    GolbalUpdateMessage(str, false);
    if ((length%PART_BURN_LEN) > 0)
        upgradeFirmwareLenLeft = (length / PART_BURN_LEN + 1)*PART_BURN_LEN;
    else
        upgradeFirmwareLenLeft = (length / PART_BURN_LEN)*PART_BURN_LEN;

    upgradeFirmwareLocalAddress = srcAddress;
    upgradeFirmwareTargetAddress = destAddress;
    upgradeFirmwareTargetBaseAddress = destAddress;
    upgradeFirmwareTargetLen = PART_BURN_LEN;

    upgradeClientStatus = UPGRADE_STATUS_STANDBY;

    setPureWaitingTime();//2000
}

void CanUpgradeStop()
{
    upgradeClientStatus = UPGRADE_STATUS_NONE;

    setPureWaitingTime();
}
void CanClientFirmwareInfoProcess(uint8_t* data, uint8_t dataLen, uint8_t targetDeviceId)
{
    //uint8_t dataTmp[8] = { 0 }, len = 0;
    CString str;
    switch (data[FIRMWARE_INFO_CMD_TYPE_INDEX])
    {
    case FIRMWARE_INFO_CMD_TYPE_ACK:
        str.Format(_T("Client get id:0x%02x Ver: %d, Flag: %d"), targetDeviceId, data[FIRMWARE_INFO_CMD_VER_INDEX], data[FIRMWARE_INFO_CMD_BOOT_FLAG_INDEX]);
        GolbalUpdateStatus(targetDeviceId, str);  
        GolbalUpdateMessage(str, false);
        break;
    
    default:
        str.Format(CString("*** Client Firmware InfoProcess(%s) not implement, (data[UPGRADE_CMD_FLAG_INDEX]:0x%02x)...\r\n"), CString(getStatusString(upgradeClientStatus)), data[UPGRADE_CMD_FLAG_INDEX]);
        GolbalUpdateMessage(str, false);
        break;
    }
}

void CanUpgradeClientProcess(uint8_t* data, uint8_t dataLen, uint8_t targetDeviceId)
{
    uint8_t dataTmp[8] = { 0 }, len = 0;
    CString str;
    showCanMsg(CString("-- Client Process[") + CString(getStatusString(upgradeClientStatus)) + CString("] --"), data, dataLen);
    switch (upgradeClientStatus)
    {
    case UPGRADE_STATUS_WAITING_TO_SEND://just ACK or NACK return
#if(DEBUG_UPGRADE)
        GolbalUpdateMessage(CString(" [current:UPGRADE_STATUS_WAITING_TO_SEND]\r\n"), false);
#endif
        switch (data[UPGRADE_CMD_FLAG_INDEX])
        {
        case UPGRADE_CMD_REQUEST:
            GolbalUpdateMessage(CString(" !!! [client_2] receive REQUEST, error cmd, set to STANDBY ...\r\n"), false);
            break;
        case UPGRADE_CMD_ACK:
#if(DEBUG_UPGRADE_NORMAL)                        
            str.Format(CString("  [\\@-@/(A)]  ACK from id:[0x%02x], status id:0x%02x(%s)...\r\n"), targetDeviceId, data[UPGRADE_STATUS_ID_INDEX], CString(getStatusString(data[UPGRADE_STATUS_ID_INDEX])));
            GolbalUpdateMessage(str, false);
#endif
            GolbalUpdateStatus(targetDeviceId, CString(getStatusString(data[UPGRADE_STATUS_ID_INDEX])));
            if (addUpgradeACKRecordFlag(0, targetDeviceId))//( check total ack)
            {
                cancelWaitingTime();// 觸發下一步  
#if(DEBUG_UPGRADE_NORMAL)
                str.Format(CString(" [client_2] receive total ACK, cancel timer, starting send data...\r\n"));
                GolbalUpdateMessage(str, false);
#endif
            }
            else
            {
#if(DEBUG_UPGRADE_NORMAL)
                str.Format(CString(" [client_2] receive ACK and wait for other server ACK back...\r\n"));
                GolbalUpdateMessage(str, false);
#else
                printf(".");
#endif
            }

            break;
        case UPGRADE_CMD_NACK:
            GolbalUpdateStatus(targetDeviceId, CString(getStatusString(data[UPGRADE_STATUS_ID_INDEX])));
            upgradeClientStatus = UPGRADE_STATUS_STANDBY;
            str.Format(CString("==> ### <retry (WAITING_TO_SEND receive NACK, device id = 0x%02x)...>  ###\r\n"), targetDeviceId);
            //
            cancelWaitingTime();
            //  
            break;
        }//switch (data[UPGRADE_CMD_FLAG_INDEX])

        break;

    case UPGRADE_STATUS_WAITING_FOR_RECEIVED://just ACK or NACK return
#if(DEBUG_UPGRADE)
        GolbalUpdateMessage(CString(" [current:UPGRADE_STATUS_WAITING_TO_SEND]\r\n"), false);
#endif
        switch (data[UPGRADE_CMD_FLAG_INDEX])
        {
        case UPGRADE_CMD_REQUEST:
            GolbalUpdateMessage(CString(" !!! [client_2] receive REQUEST, error cmd, set to STANDBY ...\r\n"), false);
            break;

        case UPGRADE_CMD_ACK:
#if(DEBUG_UPGRADE_NORMAL)                        
            str.Format(CString("  [\\@-@/(A)]  ACK from id:[0x%02x], status id:0x%02x(%s)...\r\n"), targetDeviceId, data[UPGRADE_STATUS_ID_INDEX], CString(getStatusString(data[UPGRADE_STATUS_ID_INDEX])));
            GolbalUpdateMessage(str, false);
#endif
            GolbalUpdateStatus(targetDeviceId, CString(getStatusString(data[UPGRADE_STATUS_ID_INDEX])));
            if (addUpgradeACKRecordFlag(0, targetDeviceId))//( check total ack)
            {
                cancelWaitingTime();// 觸發下一步  
#if(DEBUG_UPGRADE_NORMAL)
                str.Format(CString(" [client_3] receive total ACK, cancel timer, starting burn...\r\n"));
                GolbalUpdateMessage(str, false);
#endif
            }
            else
            {
#if(DEBUG_UPGRADE_NORMAL)
                str.Format(CString(" [client_3] receive ACK and wait for other server ACK back...\r\n"));
                GolbalUpdateMessage(str, false);
#else
                printf(".");
#endif
            }

            break;
        case UPGRADE_CMD_NACK:
            GolbalUpdateStatus(targetDeviceId, CString(getStatusString(data[UPGRADE_STATUS_ID_INDEX])));
            upgradeClientStatus = UPGRADE_STATUS_STANDBY;
            str.Format(CString("==> ### <retry (WAITING_TO_SEND receive NACK, device id = 0x%02x)...>  ###\r\n"), targetDeviceId);
            //
            cancelWaitingTime();
            //  
            break;
        }//switch (data[UPGRADE_CMD_FLAG_INDEX])

        break;

    case UPGRADE_STATUS_WAITING_FOR_BURNED://just ACK or NACK return
#if(DEBUG_UPGRADE)
        GolbalUpdateMessage(CString(" [current:UPGRADE_STATUS_WAITING_TO_SEND]\r\n"), false);
#endif
        switch (data[UPGRADE_CMD_FLAG_INDEX])
        {
        case UPGRADE_CMD_REQUEST:
            GolbalUpdateMessage(CString(" !!! [client_4] receive REQUEST, error cmd, set to STANDBY ...\r\n"), false);
            break;

        case UPGRADE_CMD_ACK:
            GolbalUpdateStatus(targetDeviceId, CString(getStatusString(data[UPGRADE_STATUS_ID_INDEX])));
#if(DEBUG_UPGRADE_NORMAL)                        
            str.Format(CString("  [\\@-@/(A)]  ACK from id:[0x%02x], status id:0x%02x(%s)...\r\n"), targetDeviceId, data[UPGRADE_STATUS_ID_INDEX], CString(getStatusString(data[UPGRADE_STATUS_ID_INDEX])));
            GolbalUpdateMessage(str, false);
#endif

            if (addUpgradeACKRecordFlag(0, targetDeviceId))//( check total ack)
            {
                cancelWaitingTime();// 觸發下一步  
#if(DEBUG_UPGRADE_NORMAL)
                str.Format(CString(" [client_4] Burn total ACK, cancel timer, starting burn...\r\n"));
                GolbalUpdateMessage(str, false);
#endif
            }
            else
            {
#if(DEBUG_UPGRADE_NORMAL)
                str.Format(CString(" [client_4] Burn ACK and wait for other server ACK back...\r\n"));
                GolbalUpdateMessage(str, false);
#else
                printf(".");
#endif
            }

            break;
        case UPGRADE_CMD_NACK:
            GolbalUpdateStatus(targetDeviceId, CString(getStatusString(data[UPGRADE_STATUS_ID_INDEX])));
            upgradeClientStatus = UPGRADE_STATUS_STANDBY;
            str.Format(CString("==> ### <retry (WAITING_TO_SEND receive NACK, device id = 0x%02x)...>  ###\r\n"), targetDeviceId);
            //
            cancelWaitingTime();
            //  
            break;
        }//switch (data[UPGRADE_CMD_FLAG_INDEX])

        break;
    case UPGRADE_STATUS_NONE://just QUERY
        GolbalUpdateStatus(targetDeviceId, CString(getStatusString(data[UPGRADE_STATUS_ID_INDEX])));
        break;
    default:
        str.Format(CString("*** Client Process(%s) not implement, (data[UPGRADE_CMD_FLAG_INDEX]:0x%02x)...\r\n"), CString(getStatusString(upgradeClientStatus)), data[UPGRADE_CMD_FLAG_INDEX]);
        GolbalUpdateMessage(str, false);
        break;
    }
}

void CanUpgradeClientMainProcess(void)
{
    uint8_t data[8] = { 0 }, len = 0;
    CString str;
    DWORD reVal;
    //str.Format(CString("-- Wait Client Main Process(wait time:%d) --\r\n"), mClientMainProcessWaitTime);
   // GolbalUpdateMessage(str, false);
    reVal = CanUpgradeWaitEvent(mClientMainProcessWaitTime);
    //str.Format(CString("-- Client Main Process(reVal = %d) --\r\n"), reVal);
    //GolbalUpdateMessage(str, false);
    switch (reVal)
    {
        case 0://object
            switch (upgradeClientStatus)
            {
                case UPGRADE_STATUS_NONE:
                    str.Format(CString("--> enter UPGRADE_STATUS_NONE, stop upgrade...\r\n"));
                    GolbalUpdateMessage(str, false);
                    mClientMainProcessWaitTime = INFINITE;
                    break;
                case UPGRADE_STATUS_STANDBY:
                    str.Format(CString("\r\n====> <Start Upgrade :address = 0x%08x, len = %d>\r\n"),
                        upgradeFirmwareTargetAddress, upgradeFirmwareTargetLen);
                    GolbalUpdateMessage(str, false);

                    resetUpgradeACKRecordFlag();

                    if (upgradeFirmwareLenLeft > PART_BURN_LEN)
                    {
                        upgradeFirmwareTargetLen = PART_BURN_LEN;
                    }
                    else
                    {
                        upgradeFirmwareTargetLen = upgradeFirmwareLenLeft;
                    }

                    data[UPGRADE_CMD_FLAG_INDEX] = UPGRADE_CMD_REQUEST;
                    data[UPGRADE_CMD_MEM_ADDRESS_BYTE_0_INDEX] = upgradeFirmwareTargetAddress & 0xff;
                    data[UPGRADE_CMD_MEM_ADDRESS_BYTE_1_INDEX] = (upgradeFirmwareTargetAddress >> 8) & 0xff;
                    data[UPGRADE_CMD_MEM_ADDRESS_BYTE_2_INDEX] = (upgradeFirmwareTargetAddress >> 16) & 0xff;
                    data[UPGRADE_CMD_MEM_ADDRESS_BYTE_3_INDEX] = (upgradeFirmwareTargetAddress >> 24) & 0xff;
                    data[UPGRADE_CMD_READ_MEM_SIZE_INDEX] = upgradeFirmwareTargetLen & 0xff;
                    data[UPGRADE_CMD_READ_MEM_SIZE_INDEX_2] = (upgradeFirmwareTargetLen >> 8) & 0xff;

                    len = UPGRADE_CMD_READ_MEM_SIZE_INDEX_2 + 1;

                    mClientMainProcessWaitTime = 3000;

                    str.Format(CString("Waiting for Server reponsed...\r\n"));
                    GolbalUpdateMessage(str, false);

                    upgradeClientStatus = UPGRADE_STATUS_WAITING_TO_SEND;



                    SendCanMessageClientToServer(CMD_GROUP_ID_DATA, CMD_ID_SERVER_UPGRADE_WRITE_MEMORY, 0, data, len);
                    break;
                case UPGRADE_STATUS_WAITING_TO_SEND:
                {
                    uint32_t targetAddress = upgradeFirmwareTargetAddress;
                    uint16_t readLen = upgradeFirmwareTargetLen;
                    uint32_t targetAddressTmp;

                    upgradeClientStatus = UPGRADE_STATUS_SENDING_DATA;

                    targetAddressTmp = UpgradeMsgSendDataClient2Server(0/*targetDeviceId*/, targetAddress, readLen);

                    upgradeClientStatus = UPGRADE_STATUS_WAITING_FOR_RECEIVED;
                    resetUpgradeACKRecordFlag();
                    mClientMainProcessWaitTime = 5000;//等待收到時間

                    str.Format(CString("Waiting for Server received...\r\n"));
                    GolbalUpdateMessage(str, false);
                    //
                    data[UPGRADE_CMD_FLAG_INDEX] = UPGRADE_CMD_ACK;
                    data[UPGRADE_STATUS_ID_INDEX] = UPGRADE_STATUS_WAITING_FOR_RECEIVED;
                    //checksum
                    data[UPGRADE_CHECKSUM_ID_INDEX] = getCheckSum(targetAddressTmp, readLen);
                    len = UPGRADE_CHECKSUM_ID_INDEX + 1;

                    SendCanMessageClientToServer(CMD_GROUP_ID_DATA, CMD_ID_SERVER_UPGRADE_WRITE_MEMORY, 0, data, len);
                    //

                    #if(DEBUG_UPGRADE_NORMAL)
                    str.Format(CString(" [client_2] send data ok, set FOR_RECEIVED, send ACK to server, set timer...\r\n"));
                    GolbalUpdateMessage(str, false);
                    #else
                    printf("|");
                    #endif 
                }
                    break;
                case UPGRADE_STATUS_WAITING_FOR_RECEIVED:
                    upgradeClientStatus = UPGRADE_STATUS_WAITING_FOR_BURNED;
                    resetUpgradeACKRecordFlag();
                    mClientMainProcessWaitTime = 5000;//等待燒錄時間

                    str.Format(CString("Waiting for Server burned...\r\n"));
                    GolbalUpdateMessage(str, false);
                    //
                    data[UPGRADE_CMD_FLAG_INDEX] = UPGRADE_CMD_ACK;
                    data[UPGRADE_STATUS_ID_INDEX] = UPGRADE_STATUS_WAITING_FOR_BURNED;
                    len = UPGRADE_STATUS_ID_INDEX + 1;

                    SendCanMessageClientToServer(CMD_GROUP_ID_DATA, CMD_ID_SERVER_UPGRADE_WRITE_MEMORY, 0, data, len);
                    //
                    #if(DEBUG_UPGRADE_NORMAL)
                    str.Format(CString(" [client_3] send data ok, set FOR_RECEIVED, send ACK to server, set timer...\r\n"));
                    GolbalUpdateMessage(str, false);
                    #endif 
                break;

                case UPGRADE_STATUS_WAITING_FOR_BURNED:
                    upgradeFirmwareLenLeft = upgradeFirmwareLenLeft - upgradeFirmwareTargetLen;
                    upgradeFirmwareTargetAddress = upgradeFirmwareTargetAddress + upgradeFirmwareTargetLen;
                    if (upgradeFirmwareLenLeft == 0)
                    {                       
                        str.Format(CString("\r\n==> ### <Stop burn success>  ###\r\n"));
                        GolbalUpdateMessage(str, false);
                        upgradeClientStatus = UPGRADE_STATUS_NONE;
                        
                    }
                    else
                    {                        
                        if (upgradeFirmwareLenLeft > PART_BURN_LEN)
                        {
                            upgradeFirmwareTargetLen = PART_BURN_LEN;
                        }
                        else
                        {
                            upgradeFirmwareTargetLen = upgradeFirmwareLenLeft;                            
                        }
                        upgradeClientStatus = UPGRADE_STATUS_STANDBY;   
                        //str.Format(CString("==> ### <do another burn success, 0x%08x>  ###\r\n", upgradeFirmwareTargetLen));
                        //GolbalUpdateMessage(str, false);
                        setPureWaitingTime();
                    }

                    break;

                default:
                    str.Format(CString("** not implement get object function(&d) \r\n"), upgradeClientStatus);
                    GolbalUpdateMessage(str, false);
                    break;
            }
            break;


        case 1://timeout
            str.Format(CString(" -!!!- {vUpgradeServerTask[ TIMEOUT ]} -- \r\n"));
            GolbalUpdateMessage(str, false);
            switch (upgradeClientStatus)
            {
                case UPGRADE_STATUS_NONE:
                    str.Format(CString("--> enter UPGRADE_STATUS_NONE, stop upgrade...\r\n"));
                    GolbalUpdateMessage(str, false);
                    mClientMainProcessWaitTime = INFINITE;
                    break;
                default:
                    upgradeClientStatus = UPGRADE_STATUS_STANDBY;     
                    setPureWaitingTime();
                    break;
            }
            
            break;
        case 2://other
            break;

    }
    
}

void CanUpgradeSend(uint32_t srcAddress, uint32_t destAddress, uint32_t length)
{
    //upgradeClientStatus = UPGRADE_STATUS_STANDBY;
}
void CanUpgradeSetEnable(uint8_t* data, uint8_t dataLen)
{
    CString str = CString("Set Enable:\r\n  ");
    for (int i = 0; i < min(dataLen, UPGRADE_SERVER_SUPPORT_NUM); i++)
    {
        canUpgradeACKRecord[i].enable = data[i];
        if (canUpgradeACKRecord[i].enable)
        {

            str = str + CString("O");
        }
        else
        {
            str = str + CString("X");
        }
        if (i % 5 == 4)
        {
            str = str + CString(" | ");
        }
        else
        {
            str = str + CString(", ");
        }
    }
    str = str + CString("\r\n");
    GolbalUpdateMessage(str, false);
}

void CanUpgradeSetClientType(int type)
{
    CString str = CString("ClientType:");
    if (type == CLIENT_UPGRADE_TYPE_BP)
    {
        str = str + CString(" TYPE_BP\r\n");
    }
    else if (type == CLIENT_UPGRADE_TYPE_STRING)
    {
        str = str + CString(" TYPE_STRING\r\n");
    }
    else
    {
        str = str + CString(" TYPE_ERROR\r\n");
    }
    upgradeClientType = type;
    GolbalUpdateMessage(str, false);
}

uint8_t CanUpgradeGetClientType(void)
{
    return upgradeClientType;
}


#endif
