#pragma once
#ifndef INIT_H
#define INIT_H

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_tim.h"

#define TIIME_FACTOR 100
#define PWM_FACTOR 100

extern volatile char buffer[80];

TIM_OCInitTypeDef timerPWM1, timerPWM2;
uint16_t PWM1, PWM2;

char init_();
void gpio_init();
void rcc_init();
void uart_init();
void i2c_init();
void tim4_init();
void tim3_init();

#endif // INIT_H
