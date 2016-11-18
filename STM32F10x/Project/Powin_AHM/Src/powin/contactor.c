/*
 * contactor.c
 *
 *  Created on: Jul 13, 2015
 *      Author: Yosh
 */

#include "pinmap.h"
#include "stm32f10x_gpio.h"
#include "contactor.h"
#include "NewStringGPIO.h"
void contactor_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);    //GPIOD Peripheral clock enable

	GPIO_InitStructure.GPIO_Pin = CONT1_PIN | CONT2_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(Contactor_Port , &GPIO_InitStructure);
    GPIO_ResetBits(Contactor_Port ,CONT1_PIN | CONT2_PIN);
    
    GPIO_InitStructure.GPIO_Pin = CONT1_SNSn_PIN | CONT2_SNSn_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(Contactor_Port, &GPIO_InitStructure);	
	
	
    GPIO_InitStructure.GPIO_Pin = Relay1_Ctrl_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(Relay1_Ctrl_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(Relay1_Ctrl_PORT,Relay1_Ctrl_PIN);
    
    GPIO_InitStructure.GPIO_Pin = Relay2_Ctrl_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(Relay2_Ctrl_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(Relay2_Ctrl_PORT,Relay2_Ctrl_PIN);
	
    contactor1_off();
    contactor2_off();


}

void contactor1_on(void)
{
    GPIO_SetBits(CONT1_PORT, CONT1_PIN);
}
void contactor2_on(void)
{
    GPIO_SetBits(CONT2_PORT, CONT2_PIN);
}

void contactor1_off(void)
{
    GPIO_ResetBits(CONT1_PORT, CONT1_PIN);
}

void contactor2_off(void)
{
    GPIO_ResetBits(CONT2_PORT, CONT2_PIN);
}

void ContactorAllTurnOn(void)
{
#if POWIN_AP
	contactor1_on();
	contactor2_on();
#endif
}

void ContactorAllTurnOff(void)
{
#if POWIN_AP
	contactor1_off();
	contactor2_off();		
#endif
}
uint8_t is_contactor_1_closed(void)
{
    if (GPIO_ReadInputDataBit(CONT1_SNSn_PORT, CONT1_SNSn_PIN) == 0)
        return 1;
    else
        return 0;
}

uint8_t is_contactor_2_closed(void)
{
    if (GPIO_ReadInputDataBit(CONT2_SNSn_PORT, CONT2_SNSn_PIN) == 0)
        return 1;
    else
        return 0;
}
