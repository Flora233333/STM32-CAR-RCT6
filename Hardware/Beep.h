#ifndef __BEEP_H
#define __BEEP_H

#include "bsp.h"                  // Device header

#define     BEEP_GPIOx                       GPIOA
#define     BEEP_RCC_APB2Periph_GPIOx        RCC_APB2Periph_GPIOA
#define     BEEP_PIN                         GPIO_Pin_12


void BEEP_Init(void);
void BEEP_toggle(void);
void Beep_Run(void);
    
#endif
