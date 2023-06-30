#ifndef __LED_H
#define __LED_H

#include "bsp.h"                  // Device header

#define     GPIOx                       GPIOA
#define     RCC_APB2Periph_GPIOx        RCC_APB2Periph_GPIOA
#define     PIN                         GPIO_Pin_8
#define     GPIOMode                    GPIO_Mode_Out_PP


void LED_Init(void);
void LED_toggle(void);
    
    
#endif
