/*
 *�ļ�:adc.c
 *����:STM32�Դ���ADC����,adc��ʼ������,���ڵ�ѹ,�������¶ȵĲ���
 *����:Alan
 *����:2015/09/21
 */


#include "NewStringADC.h"
#include "stdbool.h"
#include "messageid.h"
enum ADC_SAMPLE_STATE ADC_Sample_State=ADC_SAMPLE_CURRENT1;//ADC��������״̬
u8 ADC_BufferxDataUpdated=0xFF;
u8 ADC_BufferxUsing=ADC_Buffer0_Using;//��ǰ����ʹ�õĻ����� Ĭ�ϴ�Buffer0��ʼ���
u8 ADC_DataUpdatedFalg=0;

struct ADC_RawDataBuffer ADC_DMA_RecvBuffer0={0},ADC_DMA_RecvBuffer1={0},*ADC_DMA_RecvBuffer=&ADC_DMA_RecvBuffer0;

/*
 *����:ADC1_Configuration
 *����:ADC1��ʼ������
 *����:��
 *���:��
 *����:��
 *����:Alan
 *����:2015/09/22
 */
void ADC1_Configuration(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    //ʹ����Դʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    //���õ�������������Ϊģ�����빦��
    GPIO_InitStructure.GPIO_Pin = ADC_Current1_PIN | ADC_Current2_PIN ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(ADC_Current_PORT, &GPIO_InitStructure);
    
    //���õ�ѹ����������Ϊģ�����빦��
    GPIO_InitStructure.GPIO_Pin = ADC_Voltage1_PIN | ADC_Voltage2_PIN | ADC_Voltage3_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(ADC_Voltage_PORT, &GPIO_InitStructure);

    //�����¶ȴ���������Ϊģ�����빦��
    GPIO_InitStructure.GPIO_Pin = Temp1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(Temp1_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = Temp1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(Temp1_PORT, &GPIO_InitStructure);
    
    //ADC1��DMA���ó�ʼ��
    ADC1_DMA_Config();
    
    //ADC1�Ĵ�������
    ADC_DeInit(ADC1);//��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent ; //ADC1��ADC2�����ڶ���ģʽ
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;//ɨ��ģʽΪ��ͨ���ɼ�
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //��ֹ����ת��ģʽ
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//�������ת���������ⲿ��������ת��
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //ADC�����Ҷ���
    ADC_InitStructure.ADC_NbrOfChannel = 1;//˳����й滮ת����ADCͨ����
    ADC_Init(ADC1, &ADC_InitStructure); //��ʼ��ADC1�Ĵ���
    
    //����ADC��Ƶ����9 72M/8=9M,ADC���ʱ�䲻�ܳ���14M
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);   
    
    //����ADC1�ĸ���ͨ��Ϊ239.5���������ڣ�����Ϊ1��2  ͨ������ʱ��t=(12.5+28.5)*Tq=(12.5+28.5)*(1/9)us=4.556us
    ADC_RegularChannelConfig(ADC1, Current1_Channel, 1, ADC_SampleTime);
    
    //ʹ��ADC1��DMA
	ADC_DMACmd(ADC1, ENABLE);
	//ʹ��ADC1
	ADC_Cmd(ADC1, ENABLE);
    //��λУ׼�Ĵ���
    ADC_ResetCalibration(ADC1);
    //�ȴ�У׼�Ĵ�����λ���
    while(ADC_GetResetCalibrationStatus(ADC1));
    //����ADC1У׼
    ADC_StartCalibration(ADC1);
    //�ȴ�ADC1У׼���
    while(ADC_GetCalibrationStatus(ADC1));
    
    //ʹ���������ADCת��
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);    
}
/*
 *����:ADC1_DMA_Config
 *����:ADC1��DMA��ʼ������
 *����:��
 *���:��
 *����:��
 *����:Alan
 *����:2015/09/22
 */
void ADC1_DMA_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    //ʹ��DMA����
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
    
    //DMA1ͨ������
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	 //DMA����ADC1����ַ
//	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(ADC_DMA_RecvBuffer[ADC_Sample_State]);//DMA�ڴ����ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(ADC_DMA_RecvBuffer0.Current1);//DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//����Ϊ����Դ
	DMA_InitStructure.DMA_BufferSize = ADC_DMA_RECV_NUM;//��������
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ�̶�
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//����
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //����
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //����ģʽ
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
 *����:ADC1_DMA_ConfigReceiveBuffer
 *����:��������ADC1��DMA����ͨ���Ľ��ջ������ʹ�������
 *����:*dat:DMA���ջ�����������ָ��
 *     len:��������
 *���:��
 *����:��
 *����:Alan
 *����:2015/09/22
 */
void ADC1_DMA_ConfigReceiveBuffer(u16 *dat, u32 len)//DMA������������
{
    DMA1_Channel1->CMAR = (u32)dat ;    //����DMA�洢����ַ
    DMA1_Channel1->CNDTR = len ;    //��������Ϊlen
}
/*
 *����:ADC1_DMA_Channel_Enable
 *����:ʹ��ADC1��DMA��Ӧ�Ľ���ͨ��
 *����:��
 *���:��
 *����:��
 *����:Alan
 *����:2015/09/22
 */
void ADC1_DMA_Channel_Enable(void)
{
    ADC1->DR;//����ǰ �ȶ�ȡһ�μĴ���ֵ
    DMA1_Channel1->CCR |= (1 << 0);  //ʹ��ADC1ͨ��1
}
/*
 *����:ADC1_DMA_Channel_Disable
 *����:�ر�ADC1��DMA��Ӧ�Ľ���ͨ��
 *����:��
 *���:��
 *����:��
 *����:Alan
 *����:2015/09/22
 */
void ADC1_DMA_Channel_Disable(void)
{
    DMA1_Channel1->CCR &= ~( 1 << 0 );  //�ر�ADC1ͨ��1
}
/*
 *����:ADC1_ContinuousCmd_Disable
 *����:�ر�adc����ת��
 *����:��
 *���:��
 *����:��
 *����:Alan
 *����:2015/09/22
 */
void ADC1_ContinuousCmd_Disable(void)
{
    //Disable the selected ADC CONT
    ADC1->CR2 &=~(1 << 1);
    ADC1->CR2 &= CR2_EXTTRIG_SWSTART_Reset;
}
/*
 *����:ADC1_ContinuousCmd_Enable
 *����:����adc����ת��
 *����:��
 *���:��
 *����:��
 *����:Alan
 *����:2015/09/22
 */
void ADC1_ContinuousCmd_Enable(void)
{
    //Enable the selected ADC CONT
    ADC1->CR2 |= (1 << 1);
    ADC1->CR2 |= CR2_EXTTRIG_SWSTART_Set;
}

/*
 *����:DMA1_Channel2_IRQHandler
 *����:DMA1ͨ��1�жϷ������,��Լ2.268ms�ж�һ��
 *����:��
 *���:��
 *����:��
 *����:Alan
 *����:2015/09/22
 */
void DMA1_Channel1_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC1))
    {
        ADC1_ContinuousCmd_Disable();//�ر�ADC����ת��
        ADC1_DMA_Channel_Disable();//�ر�ADC��DMA����ͨ��

        switch(ADC_Sample_State)
        {
            case ADC_SAMPLE_CURRENT1:
            {
                ADC_RegularChannelConfig(ADC1, Current2_Channel, 1, ADC_SampleTime);//������һͨ������
                ADC1_DMA_ConfigReceiveBuffer(ADC_DMA_RecvBuffer->Current2,ADC_DMA_RECV_NUM);
                ADC_Sample_State=ADC_SAMPLE_CURRENT2;
                break;
            }
            case ADC_SAMPLE_CURRENT2:
            {
                ADC_RegularChannelConfig(ADC1, Voltage1_Channel, 1, ADC_SampleTime);//������һͨ������
                ADC1_DMA_ConfigReceiveBuffer(ADC_DMA_RecvBuffer->Voltage1,ADC_DMA_RECV_NUM);
                ADC_Sample_State=ADC_SAMPLE_VOLTAGE1;
                break;
            }
            case ADC_SAMPLE_VOLTAGE1:
            {
                ADC_RegularChannelConfig(ADC1, Voltage2_Channel, 1, ADC_SampleTime);//������һͨ������
                ADC1_DMA_ConfigReceiveBuffer(ADC_DMA_RecvBuffer->Voltage2,ADC_DMA_RECV_NUM);
                ADC_Sample_State=ADC_SAMPLE_VOLTAGE2;
                break;
            }
            case ADC_SAMPLE_VOLTAGE2:
            {
                ADC_RegularChannelConfig(ADC1, Voltage3_Channel, 1, ADC_SampleTime);//������һͨ������
                ADC1_DMA_ConfigReceiveBuffer(ADC_DMA_RecvBuffer->Voltage3,ADC_DMA_RECV_NUM);
                ADC_Sample_State=ADC_SAMPLE_VOLTAGE3;
                break;
            }
            case ADC_SAMPLE_VOLTAGE3:
            {
                ADC_RegularChannelConfig(ADC1, Temperature1_Channel, 1, ADC_SampleTime);//������һͨ������
                ADC1_DMA_ConfigReceiveBuffer(ADC_DMA_RecvBuffer->Temperature1,ADC_DMA_RECV_NUM);
                ADC_Sample_State=ADC_SAMPLE_TEMPERATURE1;
                break;
            }
            case ADC_SAMPLE_TEMPERATURE1:
            {
                ADC_RegularChannelConfig(ADC1, Temperature2_Channel, 1, ADC_SampleTime);//������һͨ������
                ADC1_DMA_ConfigReceiveBuffer(ADC_DMA_RecvBuffer->Temperature2,ADC_DMA_RECV_NUM);
                ADC_Sample_State=ADC_SAMPLE_TEMPERATURE2;
                break;
            }
            case ADC_SAMPLE_TEMPERATURE2:
            {
                if(ADC_BufferxUsing==ADC_Buffer0_Using)
                {
                    ADC_BufferxDataUpdated=ADC_Buffer0_Updated;//ADC_DMA_RecvBuffer0��������
                    ADC_DMA_RecvBuffer=&ADC_DMA_RecvBuffer1;//�ı����ݱ��ֵ�ADC_DMA_RecvBuffer1
                    ADC_BufferxUsing=ADC_Buffer1_Using;//�����´ν�Ҫʹ�û�����ADC_DMA_RecvBuffer1
                }
                else
                {
                    ADC_BufferxDataUpdated=ADC_Buffer1_Updated;//ADC_DMA_RecvBuffer1��������
                    ADC_DMA_RecvBuffer=&ADC_DMA_RecvBuffer0;//�ı����ݱ��ֵ�ADC_DMA_RecvBuffer0
                    ADC_BufferxUsing=ADC_Buffer0_Using;//�����´ν�Ҫʹ�û�����ADC_DMA_RecvBuffer0
                }
                ADC_DataUpdatedFalg=1;//����ADC�����Ѹ���
                ADC_RegularChannelConfig(ADC1, Current1_Channel, 1, ADC_SampleTime);//������һͨ������
                ADC1_DMA_ConfigReceiveBuffer(ADC_DMA_RecvBuffer->Current1,ADC_DMA_RECV_NUM);
                ADC_Sample_State=ADC_SAMPLE_CURRENT1;
                break;
            }
            default:
            {
                break;
            }
        }
        
        ADC1_DMA_Channel_Enable();//����DMA
        ADC1_ContinuousCmd_Enable();//����ADC����ת��
        
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
 *����:GetStringMeasuredVoltage
 *����:��ȡString������ѹ��Ϣ
 *����:��
 *���:��
 *����:(u32)����String��ѹ��GroundFault,��16BitΪString��ѹ,��16BitΪGroundFault
 *����:Alan
 *����:2015/09/22
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
 *����:GetDCBusVoltage
 *����:��ȡDCBUS��ѹ��Ϣ
 *����:��
 *���:��
 *����:(u16)����DCBusʵ�ʵ�ѹ,V
 *����:Alan
 *����:2015/09/22
 */
u16 GetDCBusVoltage(void)
{
    u16 PortADC=0;//�˿�ADC
    u16 DCBusVolt=0;//ʵ��DCBUS��ѹ
    
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
//  StringCurrent = (float)(200000 / 4096) * PortADC - 100000 ;//��λ100mA
	
//	StringCurrent1 = StringCurrent / 100;
 	StringCurrent =  (float)(4.88) * PortADC - 10000  ;//��λ100mA 200000 / 4096
	
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
    DCBusCurrent = (u16)((float)(200000 / 4096) * PortADC - 100000)/10 ;//��λ100mA
    return DCBusCurrent;
}
/*
 *����:GetAverageADC
 *����:��ȡadcͨ����ƽ��adֵ
 *����:*dat-����Ĵ����������ָ��
 *���:��
 *����:(u16)ƽ��ADCֵ
 *����:Alan
 *����:2015/09/22
 */
u16 GetAverageADC(u16 *dat) 
{
    u32 sum=0;
    u16 max=0,min=0,value=0;
    u8 i=2;
    
    max=min=*(dat+2);//��ǰ�����ֽڲ�ʹ��
    
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

