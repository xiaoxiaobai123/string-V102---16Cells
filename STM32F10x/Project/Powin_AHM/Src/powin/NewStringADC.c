/*
 *文件:adc.c
 *描述:STM32自带的ADC功能,adc初始化配置,用于电压,电流和温度的采样
 *作者:Alan
 *日期:2015/09/21
 */


#include "NewStringADC.h"
#include "stdbool.h"
#include "messageid.h"
enum ADC_SAMPLE_STATE ADC_Sample_State=ADC_SAMPLE_CURRENT1;//ADC采样步骤状态
u8 ADC_BufferxDataUpdated=0xFF;
u8 ADC_BufferxUsing=ADC_Buffer0_Using;//当前正在使用的缓冲区 默认从Buffer0开始存放
u8 ADC_DataUpdatedFalg=0;

struct ADC_RawDataBuffer ADC_DMA_RecvBuffer0={0},ADC_DMA_RecvBuffer1={0},*ADC_DMA_RecvBuffer=&ADC_DMA_RecvBuffer0;

/*
 *函数:ADC1_Configuration
 *描述:ADC1初始化函数
 *输入:无
 *输出:无
 *返回:无
 *作者:Alan
 *日期:2015/09/22
 */
void ADC1_Configuration(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    //使能资源时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    //配置电流传感器引脚为模拟输入功能
    GPIO_InitStructure.GPIO_Pin = ADC_Current1_PIN | ADC_Current2_PIN ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(ADC_Current_PORT, &GPIO_InitStructure);
    
    //配置电压传感器引脚为模拟输入功能
    GPIO_InitStructure.GPIO_Pin = ADC_Voltage1_PIN | ADC_Voltage2_PIN | ADC_Voltage3_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(ADC_Voltage_PORT, &GPIO_InitStructure);

    //配置温度传感器引脚为模拟输入功能
    GPIO_InitStructure.GPIO_Pin = Temp1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(Temp1_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = Temp1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(Temp1_PORT, &GPIO_InitStructure);
    
    //ADC1的DMA配置初始化
    ADC1_DMA_Config();
    
    //ADC1寄存器配置
    ADC_DeInit(ADC1);//复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent ; //ADC1和ADC2工作于独立模式
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;//扫描模式为单通道采集
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //禁止连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//软件启动转换而不是外部触发启动转换
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //ADC数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 1;//顺序进行规划转换的ADC通道数
    ADC_Init(ADC1, &ADC_InitStructure); //初始化ADC1寄存器
    
    //设置ADC分频因子9 72M/8=9M,ADC最大时间不能超过14M
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);   
    
    //配置ADC1的各个通道为239.5个采样周期，序列为1、2  通道采样时间t=(12.5+28.5)*Tq=(12.5+28.5)*(1/9)us=4.556us
    ADC_RegularChannelConfig(ADC1, Current1_Channel, 1, ADC_SampleTime);
    
    //使能ADC1的DMA
	ADC_DMACmd(ADC1, ENABLE);
	//使能ADC1
	ADC_Cmd(ADC1, ENABLE);
    //复位校准寄存器
    ADC_ResetCalibration(ADC1);
    //等待校准寄存器复位完成
    while(ADC_GetResetCalibrationStatus(ADC1));
    //启动ADC1校准
    ADC_StartCalibration(ADC1);
    //等待ADC1校准完成
    while(ADC_GetCalibrationStatus(ADC1));
    
    //使用软件触发ADC转换
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);    
}
/*
 *函数:ADC1_DMA_Config
 *描述:ADC1的DMA初始化函数
 *输入:无
 *输出:无
 *返回:无
 *作者:Alan
 *日期:2015/09/22
 */
void ADC1_DMA_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    //使能DMA传输
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
    
    //DMA1通道配置
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	 //DMA外设ADC1基地址
//	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(ADC_DMA_RecvBuffer[ADC_Sample_State]);//DMA内存基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(ADC_DMA_RecvBuffer0.Current1);//DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//外设为数据源
	DMA_InitStructure.DMA_BufferSize = ADC_DMA_RECV_NUM;//传输数量
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址固定
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址自增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//半字
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //半字
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //正常模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

/*
 *函数:ADC1_DMA_ConfigReceiveBuffer
 *描述:重新配置ADC1的DMA接收通道的接收缓存区和传输数量
 *输入:*dat:DMA接收缓冲区的数据指针
 *     len:传输数量
 *输出:无
 *返回:无
 *作者:Alan
 *日期:2015/09/22
 */
void ADC1_DMA_ConfigReceiveBuffer(u16 *dat, u32 len)//DMA启动接收配置
{
    DMA1_Channel1->CMAR = (u32)dat ;    //设置DMA存储器地址
    DMA1_Channel1->CNDTR = len ;    //传输数量为len
}
/*
 *函数:ADC1_DMA_Channel_Enable
 *描述:使能ADC1的DMA对应的接收通道
 *输入:无
 *输出:无
 *返回:无
 *作者:Alan
 *日期:2015/09/22
 */
void ADC1_DMA_Channel_Enable(void)
{
    ADC1->DR;//启动前 先读取一次寄存器值
    DMA1_Channel1->CCR |= (1 << 0);  //使能ADC1通道1
}
/*
 *函数:ADC1_DMA_Channel_Disable
 *描述:关闭ADC1的DMA对应的接收通道
 *输入:无
 *输出:无
 *返回:无
 *作者:Alan
 *日期:2015/09/22
 */
void ADC1_DMA_Channel_Disable(void)
{
    DMA1_Channel1->CCR &= ~( 1 << 0 );  //关闭ADC1通道1
}
/*
 *函数:ADC1_ContinuousCmd_Disable
 *描述:关闭adc连续转换
 *输入:无
 *输出:无
 *返回:无
 *作者:Alan
 *日期:2015/09/22
 */
void ADC1_ContinuousCmd_Disable(void)
{
    //Disable the selected ADC CONT
    ADC1->CR2 &=~(1 << 1);
    ADC1->CR2 &= CR2_EXTTRIG_SWSTART_Reset;
}
/*
 *函数:ADC1_ContinuousCmd_Enable
 *描述:启动adc连续转换
 *输入:无
 *输出:无
 *返回:无
 *作者:Alan
 *日期:2015/09/22
 */
void ADC1_ContinuousCmd_Enable(void)
{
    //Enable the selected ADC CONT
    ADC1->CR2 |= (1 << 1);
    ADC1->CR2 |= CR2_EXTTRIG_SWSTART_Set;
}

/*
 *函数:DMA1_Channel2_IRQHandler
 *描述:DMA1通道1中断服务程序,大约2.268ms中断一次
 *输入:无
 *输出:无
 *返回:无
 *作者:Alan
 *日期:2015/09/22
 */
void DMA1_Channel1_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC1))
    {
        ADC1_ContinuousCmd_Disable();//关闭ADC连续转换
        ADC1_DMA_Channel_Disable();//关闭ADC的DMA接收通道

        switch(ADC_Sample_State)
        {
            case ADC_SAMPLE_CURRENT1:
            {
                ADC_RegularChannelConfig(ADC1, Current2_Channel, 1, ADC_SampleTime);//配置下一通道规则
                ADC1_DMA_ConfigReceiveBuffer(ADC_DMA_RecvBuffer->Current2,ADC_DMA_RECV_NUM);
                ADC_Sample_State=ADC_SAMPLE_CURRENT2;
                break;
            }
            case ADC_SAMPLE_CURRENT2:
            {
                ADC_RegularChannelConfig(ADC1, Voltage1_Channel, 1, ADC_SampleTime);//配置下一通道规则
                ADC1_DMA_ConfigReceiveBuffer(ADC_DMA_RecvBuffer->Voltage1,ADC_DMA_RECV_NUM);
                ADC_Sample_State=ADC_SAMPLE_VOLTAGE1;
                break;
            }
            case ADC_SAMPLE_VOLTAGE1:
            {
                ADC_RegularChannelConfig(ADC1, Voltage2_Channel, 1, ADC_SampleTime);//配置下一通道规则
                ADC1_DMA_ConfigReceiveBuffer(ADC_DMA_RecvBuffer->Voltage2,ADC_DMA_RECV_NUM);
                ADC_Sample_State=ADC_SAMPLE_VOLTAGE2;
                break;
            }
            case ADC_SAMPLE_VOLTAGE2:
            {
                ADC_RegularChannelConfig(ADC1, Voltage3_Channel, 1, ADC_SampleTime);//配置下一通道规则
                ADC1_DMA_ConfigReceiveBuffer(ADC_DMA_RecvBuffer->Voltage3,ADC_DMA_RECV_NUM);
                ADC_Sample_State=ADC_SAMPLE_VOLTAGE3;
                break;
            }
            case ADC_SAMPLE_VOLTAGE3:
            {
                ADC_RegularChannelConfig(ADC1, Temperature1_Channel, 1, ADC_SampleTime);//配置下一通道规则
                ADC1_DMA_ConfigReceiveBuffer(ADC_DMA_RecvBuffer->Temperature1,ADC_DMA_RECV_NUM);
                ADC_Sample_State=ADC_SAMPLE_TEMPERATURE1;
                break;
            }
            case ADC_SAMPLE_TEMPERATURE1:
            {
                ADC_RegularChannelConfig(ADC1, Temperature2_Channel, 1, ADC_SampleTime);//配置下一通道规则
                ADC1_DMA_ConfigReceiveBuffer(ADC_DMA_RecvBuffer->Temperature2,ADC_DMA_RECV_NUM);
                ADC_Sample_State=ADC_SAMPLE_TEMPERATURE2;
                break;
            }
            case ADC_SAMPLE_TEMPERATURE2:
            {
                if(ADC_BufferxUsing==ADC_Buffer0_Using)
                {
                    ADC_BufferxDataUpdated=ADC_Buffer0_Updated;//ADC_DMA_RecvBuffer0数据最新
                    ADC_DMA_RecvBuffer=&ADC_DMA_RecvBuffer1;//改变数据保持到ADC_DMA_RecvBuffer1
                    ADC_BufferxUsing=ADC_Buffer1_Using;//设置下次将要使用缓冲区ADC_DMA_RecvBuffer1
                }
                else
                {
                    ADC_BufferxDataUpdated=ADC_Buffer1_Updated;//ADC_DMA_RecvBuffer1数据最新
                    ADC_DMA_RecvBuffer=&ADC_DMA_RecvBuffer0;//改变数据保持到ADC_DMA_RecvBuffer0
                    ADC_BufferxUsing=ADC_Buffer0_Using;//设置下次将要使用缓冲区ADC_DMA_RecvBuffer0
                }
                ADC_DataUpdatedFalg=1;//所有ADC数据已更新
                ADC_RegularChannelConfig(ADC1, Current1_Channel, 1, ADC_SampleTime);//配置下一通道规则
                ADC1_DMA_ConfigReceiveBuffer(ADC_DMA_RecvBuffer->Current1,ADC_DMA_RECV_NUM);
                ADC_Sample_State=ADC_SAMPLE_CURRENT1;
                break;
            }
            default:
            {
                break;
            }
        }
        
        ADC1_DMA_Channel_Enable();//启动DMA
        ADC1_ContinuousCmd_Enable();//启动ADC连续转换
        
        DMA_ClearITPendingBit(0x0F);
    }
}
bool GetGroundFaultState(void)
{
    u16 PortADC1=0,PortADC2=0;//??ADC
    u16 BPVolt1=0,BPVolt2=0,GroundFaultVolt=0;
    
    if(ADC_BufferxDataUpdated==ADC_Buffer0_Updated)
    {
        PortADC1=GetAverageADC(ADC_DMA_RecvBuffer0.Voltage1);
        PortADC2=GetAverageADC(ADC_DMA_RecvBuffer0.Voltage2);
    }
    else if(ADC_BufferxDataUpdated==ADC_Buffer1_Updated)
    {
        PortADC1=GetAverageADC(ADC_DMA_RecvBuffer1.Voltage1);
        PortADC2=GetAverageADC(ADC_DMA_RecvBuffer1.Voltage2);
    }
    BPVolt1=PortADC1*1.0/4095*1000;
    BPVolt2=PortADC2*1.0/4095*1000;
    if(BPVolt1>=BPVolt2)
    {
        GroundFaultVolt=BPVolt1-BPVolt2;
    }
    else
    {
        GroundFaultVolt=BPVolt2-BPVolt1;
    }
    if(GroundFaultVolt > LeakageVoltageThresholdALarm / 1000)
        return false;
    else
        return true;
}
/*
 *函数:GetStringMeasuredVoltage
 *描述:获取String整个电压信息
 *输入:无
 *输出:无
 *返回:(u32)返回String电压和GroundFault,低16Bit为String电压,高16Bit为GroundFault
 *作者:Alan
 *日期:2015/09/22
 */
u16 PortADC1=0,PortADC2=0;//??ADC
u16 GetStringMeasuredVoltage(void)
{

    u16 BPVolt1=0,BPVolt2=0,BPVolt=0;
    
    if(ADC_BufferxDataUpdated==ADC_Buffer0_Updated)
    {
        PortADC1=GetAverageADC(ADC_DMA_RecvBuffer0.Voltage1);
        PortADC2=GetAverageADC(ADC_DMA_RecvBuffer0.Voltage2);
    }
    else if(ADC_BufferxDataUpdated==ADC_Buffer1_Updated)
    {
        PortADC1=GetAverageADC(ADC_DMA_RecvBuffer1.Voltage1);
        PortADC2=GetAverageADC(ADC_DMA_RecvBuffer1.Voltage2);
    }
    BPVolt1=PortADC1*1.0/4095*1000;
    BPVolt2=PortADC2*1.0/4095*1000;
    BPVolt=BPVolt1+BPVolt2;//??V

    return BPVolt;
//	return BPVolt1;
}
/*
 *函数:GetDCBusVoltage
 *描述:获取DCBUS电压信息
 *输入:无
 *输出:无
 *返回:(u16)返回DCBus实际电压,V
 *作者:Alan
 *日期:2015/09/22
 */
u16 GetDCBusVoltage(void)
{
    u16 PortADC=0;//端口ADC
    u16 DCBusVolt=0;//实际DCBUS电压
    
    if(ADC_BufferxDataUpdated==ADC_Buffer0_Updated)
    {
        PortADC=GetAverageADC(ADC_DMA_RecvBuffer0.Voltage3);
    }
    else if(ADC_BufferxDataUpdated==ADC_Buffer1_Updated)
    {
        PortADC=GetAverageADC(ADC_DMA_RecvBuffer1.Voltage3);
    }
    DCBusVolt=PortADC*1.0/4095*1000;
    
    return DCBusVolt;
}

s16 ReadStringCurrent(void)
{
    u16 PortADC=0;
    s16 StringCurrent=0;
//    s16 StringCurrent1 = 0;
    if(ADC_BufferxDataUpdated==ADC_Buffer0_Updated)
    {
        PortADC=GetAverageADC(ADC_DMA_RecvBuffer0.Current1);
    }
    else if(ADC_BufferxDataUpdated==ADC_Buffer1_Updated)
    {
        PortADC=GetAverageADC(ADC_DMA_RecvBuffer1.Current1);
    }
//  StringCurrent = (float)(200000 / 4096) * PortADC - 100000 ;//单位100mA
	
//	StringCurrent1 = StringCurrent / 100;
 	StringCurrent =  (float)(4.88) * PortADC - 10000  ;//单位100mA 200000 / 4096
	
	return StringCurrent;
}

u16 GetDCBusCurrent(void)
{
    u16 PortADC=0;
    s16 DCBusCurrent=0;
    
    if(ADC_BufferxDataUpdated==ADC_Buffer0_Updated)
    {
        PortADC=GetAverageADC(ADC_DMA_RecvBuffer0.Current2);
    }
    else if(ADC_BufferxDataUpdated==ADC_Buffer1_Updated)
    {
        PortADC=GetAverageADC(ADC_DMA_RecvBuffer1.Current2);
    }
    DCBusCurrent = (u16)((float)(200000 / 4096) * PortADC - 100000)/10 ;//单位100mA
    return DCBusCurrent;
}
/*
 *函数:GetAverageADC
 *描述:获取adc通道的平均ad值
 *输入:*dat-输入的待处理的数据指针
 *输出:无
 *返回:(u16)平均ADC值
 *作者:Alan
 *日期:2015/09/22
 */
u16 GetAverageADC(u16 *dat) 
{
    u32 sum=0;
    u16 max=0,min=0,value=0;
    u8 i=2;
    
    max=min=*(dat+2);//最前面两字节不使用
    
    for(i=2;i<ADC_CHANNEL_SAMPLE_NUM;i++)
    {
        value=*(dat+i);
        if(value>max)
        {
            max=value;
        }
        if(value<min)
        {
            min=value;
        }
        sum+=value;
    }
    sum=(sum-max-min)/(ADC_CHANNEL_SAMPLE_NUM-2-2);
    return (u16)sum;
}

