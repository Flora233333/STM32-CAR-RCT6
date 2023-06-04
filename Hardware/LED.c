#include "LED.h"

void LED_Init(void) {
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOx, ENABLE);
//	
//	GPIO_InitTypeDef GPIO_InitStructure;    
// 	GPIO_InitStructure.GPIO_Mode = GPIOMode;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Pin = PIN;
// 	GPIO_Init(GPIOx, &GPIO_InitStructure);
//	
//	GPIO_ResetBits(GPIOx, PIN);
    RCC->APB2ENR|=1<<3; //使能 PORT 时钟  
    GPIOB->CRH&=0XFF0FFFFF;
    GPIOB->CRH|=0X00300000;//推挽输出
    GPIOB->ODR|=1<<13; // 输出高
}


void Led_Flash(u16 time)
{
	  static int temp;
	  if(0==time) LED=0;
	  else		if(++temp==time)	LED=~LED,temp=0;
}
