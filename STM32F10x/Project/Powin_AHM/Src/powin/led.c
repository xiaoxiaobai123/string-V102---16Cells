//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

#include "led.h"
#include "pinmap.h"
#include "NewStringGPIO.h"
// ----------------------------------------------------------------------------
static int greenGreenStatus = 0;
static int greenLedStatus = 0, redLedStatus = 0, yellowLedStatus = 0;

void led_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = LED1_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED1_PORT, &GPIO_InitStructure);    // implicit that led1 and led2 are on the same port
    
    GPIO_InitStructure.GPIO_Pin = STATUS_LED_GREEN_PIN;    
    GPIO_Init(STATUS_LED_GREEN_PORT, &GPIO_InitStructure);    // 
    
    GPIO_InitStructure.GPIO_Pin = STATUS_LED_RED_PIN;    
    GPIO_Init(STATUS_LED_RED_PORT, &GPIO_InitStructure);    // 


 //   led1_ctl(0);
 //   led2_ctl(0);
    GPIO_SetBits(FRONT_LED_RED_PORT, FRONT_LED_RED_PIN);
	GPIO_SetBits(FRONT_LED_GRN_PORT, FRONT_LED_GRN_PIN);
//    statusLedGreen_ctl(0);
//    statusLedRed_ctl(1);
	
}

void led1_ctl(uint8_t on_)
{
    // active high led
    if (on_) 
        GPIO_SetBits(LED1_PORT, LED1_PIN);
    else 
        GPIO_ResetBits(LED1_PORT, LED1_PIN);
}
 
void statusLedGreen_ctl(uint8_t on_)
{
    // active high led
    if (on_) 
    {
        GPIO_SetBits(STATUS_LED_GREEN_PORT, STATUS_LED_GREEN_PIN);
    }
    else 
    {
        GPIO_ResetBits(STATUS_LED_GREEN_PORT, STATUS_LED_GREEN_PIN);
    }
    greenGreenStatus = on_;
}
void statusLedRed_ctl(uint8_t on_)
{
    // active high led
    if (on_) 
    {
        GPIO_SetBits(STATUS_LED_RED_PORT, STATUS_LED_RED_PIN);
    }
    else 
    {
        GPIO_ResetBits(STATUS_LED_RED_PORT, STATUS_LED_RED_PIN);
    }
    redLedStatus = on_;
}


void statusLedGreen_blink(void)
{
    if(greenGreenStatus == 0)
    {
        greenGreenStatus = 1;
    }
    else
    {
        greenGreenStatus = 0;
    }
    statusLedGreen_ctl(greenGreenStatus);
    statusLedRed_ctl(1);
}

void statusLedRed_blink(void)
{
     if(redLedStatus == 0)
    {
        redLedStatus = 1;
    }
    else
    {
        redLedStatus = 0;
    }
    statusLedGreen_ctl(1);
    statusLedRed_ctl(redLedStatus);    
}

void statusLedYellow_blink(void)
{
    if(yellowLedStatus == 0)
    {
        yellowLedStatus = 1;
    }
    else
    {
        yellowLedStatus = 0;
    }
    
    statusLedRed_ctl(yellowLedStatus);
    statusLedGreen_ctl(yellowLedStatus);
    
}

void RS485_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = RS485_TX_LED_PIN | RS485_RX_LED_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(RS485_LED_PORT, &GPIO_InitStructure);
    GPIO_SetBits(RS485_LED_PORT, RS485_TX_LED_PIN | RS485_RX_LED_PIN);
}
