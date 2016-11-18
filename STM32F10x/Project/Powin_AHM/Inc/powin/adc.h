#ifndef ADC_H_
#define ADC_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f10x_adc.h"

#define ADC_CHANNEL_IAMP1		ADC_Channel_0
#define ADC_CHANNEL_IAMP2		ADC_Channel_1
#define ADC_CHANNEL_VAMP		ADC_Channel_2

void adc_init(void);
uint16_t adc_getresult(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H_ */

