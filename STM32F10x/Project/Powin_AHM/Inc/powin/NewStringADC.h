/*
 *�ļ�:adc.c
 *����:оƬ�Զ�adc�ɼ��¶ȵ���������
 *����:Alan
 *����:2015/09/21
 */


#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"

#include "NewStringGPIO.H"

#include "stdbool.h"

#ifndef CR2_EXTTRIG_SWSTART_Set
#define CR2_EXTTRIG_SWSTART_Set     ((uint32_t)0x00500000)
#endif
#ifndef CR2_EXTTRIG_SWSTART_Reset
#define CR2_EXTTRIG_SWSTART_Reset   ((uint32_t)0xFFAFFFFF)
#endif

#define ADC1_DR_Address    ((u32)0x4001244C)    //ADC1�����ݼĴ�����ַ,����DMA�������ַ


//����adc�ӿڶ�Ӧ��ͨ��
#define Voltage1_Channel        ADC_Channel_0
#define Voltage2_Channel        ADC_Channel_1
#define Voltage3_Channel        ADC_Channel_2
#define Current1_Channel        ADC_Channel_12
#define Current2_Channel        ADC_Channel_13
#define Temperature1_Channel    ADC_Channel_11
#define Temperature2_Channel    ADC_Channel_3

//����ͨ��ADC���ݻ�����ƫ����
#define RecvBuffAddrOffset_Current1         0
#define RecvBuffAddrOffset_Current2         1
#define RecvBuffAddrOffset_Voltage1         2
#define RecvBuffAddrOffset_Voltage2         3
#define RecvBuffAddrOffset_Voltage3         4
#define RecvBuffAddrOffset_Temperature1     5
#define RecvBuffAddrOffset_Temperature2     6

#define ADC_Buffer0_Updated  0
#define ADC_Buffer1_Updated  1
#define ADC_Buffer0_Using    0
#define ADC_Buffer1_Using    1


#define ADC_CHANNEL_SAMPLE_NUM  20  //ÿ��ͨ���������� ǰ2�β�ʹ��
#define ADC_DMA_RECV_NUM        ADC_CHANNEL_SAMPLE_NUM
#define ADC_SampleTime      ADC_SampleTime_239Cycles5

enum ADC_SAMPLE_STATE
{
    ADC_SAMPLE_NONE=0,
    ADC_SAMPLE_CURRENT1=1,
    ADC_SAMPLE_CURRENT2=2,
    ADC_SAMPLE_VOLTAGE1=3,
    ADC_SAMPLE_VOLTAGE2=4,
    ADC_SAMPLE_VOLTAGE3=5,
    ADC_SAMPLE_TEMPERATURE1=6,
    ADC_SAMPLE_TEMPERATURE2=7,
};


void ADC1_Configuration(void);
void ADC1_DMA_Config(void);
void ADC1_DMA_ConfigReceiveBuffer(u16 *dat, u32 len);
void ADC1_DMA_Channel_Enable(void);
void ADC1_DMA_Channel_Disable(void);
void ADC1_ContinuousCmd_Disable(void);
void ADC1_ContinuousCmd_Enable(void);
u16 GetAverageADC(u16 *dat);
u16 GetStringMeasuredVoltage(void);
u16 GetDCBusVoltage(void);
s16 ReadStringCurrent(void);
u16 GetDCBusCurrent(void);

bool GetGroundFaultState(void);



struct ADC_RawDataBuffer
{
    u16 Current1[ADC_CHANNEL_SAMPLE_NUM];
    u16 Current2[ADC_CHANNEL_SAMPLE_NUM];
    u16 Voltage1[ADC_CHANNEL_SAMPLE_NUM];
    u16 Voltage2[ADC_CHANNEL_SAMPLE_NUM];
    u16 Voltage3[ADC_CHANNEL_SAMPLE_NUM];
    u16 Temperature1[ADC_CHANNEL_SAMPLE_NUM];
    u16 Temperature2[ADC_CHANNEL_SAMPLE_NUM];
};
extern struct ADC_RawDataBuffer ADC_DMA_RecvBuffer0,ADC_DMA_RecvBuffer1, *ADC_DMA_RecvBuffer;

extern u8 ADC_BufferxIsUsing;
extern u8 ADC_BufferxDataUpdated;
extern u8 ADC_DataUpdatedFalg;



#endif /* __ADC_H */



