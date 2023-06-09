#ifndef __LED_H
#define __LED_H

#include "bsp.h"                  // Device header

#define     GPIOx                       GPIOA
#define     RCC_APB2Periph_GPIOx        RCC_APB2Periph_GPIOA
#define     PIN                         GPIO_Pin_15
#define     GPIOMode                    GPIO_Mode_Out_PP


void LED_Init(void);
void Led_Flash(u16 time);
    
#endif
