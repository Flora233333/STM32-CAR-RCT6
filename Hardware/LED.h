#ifndef __LED_H
#define __LED_H

#include "bsp.h"                  // Device header

#define     GPIOx                       GPIOB
#define     RCC_APB2Periph_GPIOx        RCC_APB2Periph_GPIOB
#define     PIN                         GPIO_Pin_5
#define     GPIOMode                    GPIO_Mode_Out_PP


#define     GPIOB_ODR_Addr              (GPIOB_BASE+12) //0x40010C0C 
#define     BITBAND(addr, bitnum)       ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define     MEM_ADDR(addr)              *((volatile unsigned long  *)(addr)) 
#define     BIT_ADDR(addr, bitnum)      MEM_ADDR(BITBAND(addr, bitnum)) 
#define     PBout(n)                    BIT_ADDR(GPIOB_ODR_Addr,n)  //Êä³ö
#define     LED PBout(13) 
void LED_Init(void);
void Led_Flash(u16 time);
    
#endif
