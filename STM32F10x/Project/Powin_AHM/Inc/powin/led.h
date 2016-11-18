//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

#ifndef LED_H_
#define LED_H_

#include "stm32f10x.h"


void led_init(void);
void led1_ctl(uint8_t on_);
void led2_ctl(uint8_t on_);
void statusLedGreen_ctl(uint8_t on_);
void statusLedRed_ctl(uint8_t on_);

void statusLedGreen_blink(void);
void statusLedRed_blink(void);
void statusLedYellow_blink(void);
// ----------------------------------------------------------------------------
uint8_t getSelfID(void);
#endif // BLINKLED_H_
