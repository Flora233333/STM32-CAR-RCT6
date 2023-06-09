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


//void Led_Flash(u16 time)
//{
//	  static int temp;
//	  if(0==time) LED=0;
//	  else		if(++temp==time)	LED=~LED,temp=0;
//}
