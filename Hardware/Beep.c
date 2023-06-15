#include "Beep.h"

void BEEP_Init(void) {
	RCC_APB2PeriphClockCmd(BEEP_RCC_APB2Periph_GPIOx, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;    
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = BEEP_PIN;
 	GPIO_Init(BEEP_GPIOx, &GPIO_InitStructure);
	
	GPIO_ResetBits(BEEP_GPIOx, BEEP_PIN);
}

void BEEP_toggle(void) {
    BEEP_GPIOx->ODR ^= BEEP_PIN;
}
