#include "adc.h"
#include "pinmap.h"

/*-----------------------------------------------------------------------------*/
/**
 * @fn       void ADC_Initialize(void)
 * @brief    Function to initialize the ADC converter subsystem.
 *           Voltage and temperature sampling using ADC1.
 *           Voltage and temperature are converted independently.
 *           Currently ADC2 is not utilized; ADC2 could do simultaneous sampling.
 */
void adc_init(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//GPIOA Peripheral clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); /**< Enable ADC1 Clock */
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); /**<  ADC clock divide by 4 APB2 Clock */

    /**< Pack Cell Voltage Inputs to the ADC */
    GPIO_InitStructure.GPIO_Pin =
        IAMP1_PIN |		/**< PA0 Current Amp 1 input*/
        IAMP2_PIN |		/**< PA1 Current Amp 2 input*/
        VAMP_PIN  |     /**< PA2 Voltage Amp input*/
        0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(IVAMP_PORT, &GPIO_InitStructure);

    /**< ADC1 configuration */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent ; /**<  Synchronization mode = Independent */
    ADC_InitStructure.ADC_ScanConvMode = DISABLE; /**<  Single channel convert mode */
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; /**<  Single Sample conversion mode */
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; /**< No External trigger */
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; /**<  Data align right */
    ADC_InitStructure.ADC_NbrOfChannel = 1; /**< Channel convert number */

    ADC_Init(ADC1, &ADC_InitStructure); /**< Set ADC1 configuration */
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));

    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC1);
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));

//    VoltSampDelay = PackPara.SlowVoltCheckTime; /**<  Voltage sampling interval time  */
//    TempSampDelay = PackPara.TempCheckTime;     /**<  Temperature sampling interval time */
}
#if 1
/*-----------------------------------------------------------------------------*/
/**
 * @fn       uint16_t Get_ADCResult(uint8_t ADC_channel)
 * @brief    Get the ADC result from the specified channel.
 * @param    ADC_channel - The ADC channel to sample from.
 * 							Can be: ADC_CHANNEL_IAMP1
 * 									ADC_CHANNEL_IAMP2
 * 									ADC_CHANNEL_VAMP
 * @return   The function returns the raw ADC counts from channel ADC_channel.
 */
uint16_t adc_getresult(uint8_t ADC_channel)
{
//    uint32_t acc = 0;

    ADC_RegularChannelConfig(ADC1, ADC_channel, 1, ADC_SampleTime_28Cycles5);
//    ADC_Cmd(ADC1, ENABLE);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(((ADC1->SR)&0x02) == 0) {
        ; /**< Wait for SAR ADC conversion operation to be completed. */
    }
    return ADC1->DR;  /**< return the current ADC sample.  A read of this register automatically resets the EOC bit of the ADC_SR register. */
}
#endif

#if 0
/*-----------------------------------------------------------------------------*/
/**
 * @fn       uint16_t Get_ADCResult(uint8_t ADC_channel)
 * @brief    Get the ADC result from the specified channel.
 * @param    ADC_channel - The ADC channel to sample from.
 * 	 Can be: ADC_CHANNEL_IAMP1
 * 	 ADC_CHANNEL_IAMP2
 * 	 ADC_CHANNEL_VAMP
 * @return   The function returns the raw ADC counts from channel ADC_channel.
 */
#define ADC_SAMPLE_TIMES  10
uint16_t adcTemp[ADC_SAMPLE_TIMES];
uint16_t adcMax, adcMin, adcValue;
uint32_t adcSum;
uint16_t adc_getresult(uint8_t ADC_channel)
{
    int i = 0;
    adcSum = 0;
    adcMax = 0;
    adcMin = 0;
    
		for(i = 0; i<ADC_SAMPLE_TIMES; i++)
		{
			ADC_RegularChannelConfig(ADC1, ADC_channel, 1, ADC_SampleTime_28Cycles5);
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);
      while(((ADC1->SR)&0x02) == 0) {
            ; /**< Wait for SAR ADC conversion operation to be completed. */
        }
      adcTemp[i] = ADC1->DR;
        if(i == 0)
        {
            adcMax = adcTemp[i];
            adcMin = adcTemp[i];
        }
        else
        {
            if(adcTemp[i]>adcMax)
            {
                adcMax = adcTemp[i];
            }
            if(adcTemp[i]<adcMin)
            {
                adcMin = adcTemp[i];
            }
        }
        adcSum = adcSum + adcTemp[i];
        
    }
    adcValue = (adcSum - adcMax - adcMin)/(ADC_SAMPLE_TIMES-2);
    /*
    if(ADC_channel == ADC_CHANNEL_IAMP1)
    {
        for(i = 0; i<ADC_SAMPLE_TIMES; i++)
        {
            printf("{%02d}:%d\r\n", i, adcTemp[i]);
        }
        printf("{Max}:%d, {Min}:%d, {adcValue}:%d\r\n", adcMax, adcMin, adcValue);
    }
    */
    return adcValue;  /**< return the current ADC sample.  A read of this register automatically resets the EOC bit of the ADC_SR register. */
}
#endif
