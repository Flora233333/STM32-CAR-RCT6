#include "timer.h"                  // Device header

void Timer1_InternalClock_Init() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //使能时钟
	
	TIM_InternalClockConfig(TIM1); //选择晶振源
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct; //时基单元配置
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 1000 - 1;   //10ms
	TIM_TimeBaseInitStruct.TIM_Prescaler = 720 - 1; 
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);
										  
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); 
	
	NVIC_InitTypeDef NVIC_InitStruct; //NVIC配置
	NVIC_InitStruct.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
	
	TIM_Cmd(TIM1, ENABLE); //开启时钟计数器
}
