#include "LED.h"

void LED_Init(void) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOx, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;    
 	GPIO_InitStructure.GPIO_Mode = GPIOMode;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = PIN;
 	GPIO_Init(GPIOx, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOx, PIN);
}

void LED_toggle(void) {
    GPIOx->ODR ^= PIN;
}
