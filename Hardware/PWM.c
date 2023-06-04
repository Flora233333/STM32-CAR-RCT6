#include "PWM.h"              


void PWM_Init() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//配置引脚复用时钟
	
	  
    //GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE);//配置重映射模式为部分重映射
    
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	TIM_InternalClockConfig(TIM2);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 3600 - 1; //PWM频率一定要快, 20000HZ
	TIM_TimeBaseInitStruct.TIM_Prescaler = 1 - 1; //1分频
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode  = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 	  // 极易写错成TIM_OCNPolarity（多个N），后面的也会是TIM_OCNPolarity_High
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 这里不是写ENABLE，极易写错成OutputNState（多个N）后面的也会是TIM_OutputNState_Enable
	TIM_OCInitStructure.TIM_Pulse = 0;
    
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);

	TIM_Cmd(TIM2, ENABLE);
}

int PWM_restrict(int Motor, int max) {
	//if(*Motor_1 < 100) *Motor_1 = 100;
	//if(*Motor_2 < 100) *Motor_2 = 100;
    
    if(Motor > max) Motor = max;
    if(Motor < -max) Motor = -max;

    return Motor;
}

void PWM_stop(void) {
    Motor1_SetDirct(0,0);
    Motor2_SetDirct(0,0);
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;
}

//char str3[100];
void PWM_updata_Motor1(int Motor_1) {
     
    if(Motor_1 > 0) {
		Motor1_SetDirct(1, 0);
    }
	else if (Motor_1 < 0) {
		Motor1_SetDirct(0, 1);
		Motor_1 = -1 * Motor_1;
	}
    //Motor1_SetDirct(0, 1);
    if(Motor_1 == 0) {
        TIM2->CCR4 = 0;
        //TIM_SetCompare3(TIM2, 0);
    } 
    else {
        Motor_1 = Motor_1 + Dead_Voltage_1;
        Motor_1 = PWM_restrict(Motor_1, PWM_Max);
        TIM2->CCR4 = Motor_1;
        //printf("%d",Motor_1)
        //printf("Moto1=%d\r\n",Motor_1);
        //TIM_SetCompare3(TIM2, Motor_1);
    }
} 

void PWM_updata_Motor2(int Motor_2) {
    if(Motor_2 > 0) {
		Motor2_SetDirct(1, 0);
    }
	else if (Motor_2 < 0) {
		Motor2_SetDirct(0, 1);
		Motor_2 = -1 * Motor_2;
	}
    
    if(Motor_2 == 0) {
        TIM2->CCR3 = 0;
        //TIM_SetCompare4(TIM2, 0);
    }
    else {
        Motor_2 = Motor_2 + Dead_Voltage_2;
        Motor_2 = PWM_restrict(Motor_2, PWM_Max);
        TIM2->CCR3 = Motor_2;
        //TIM_SetCompare4(TIM2, Motor_2);
    }
} 

void PWM_updata(int Motor_1, int Motor_2) {
	if(Motor_1 > 0) {
		Motor1_SetDirct(1, 0);
    }
	else if (Motor_1 < 0) {
		Motor1_SetDirct(0, 1);
		Motor_1 = -1 * Motor_1;
	}
	
	if(Motor_2 > 0) {
		Motor2_SetDirct(1, 0);
    }
	else if (Motor_2 < 0) {
		Motor2_SetDirct(0, 1);
		Motor_2 = -1 * Motor_2;
	}
    
	if(Motor_1 == 0) {
        TIM2->CCR3 = 0;
        //TIM_SetCompare3(TIM2, 0);
    } 
    else {
        TIM2->CCR3 = Motor_1 + Dead_Voltage_1;
        //TIM_SetCompare3(TIM2, Motor_1);
    }
        
    if(Motor_2 == 0) {
        TIM2->CCR4 = 0;
        //TIM_SetCompare4(TIM2, 0);
    }
    else {
        TIM2->CCR4 = Motor_2 + Dead_Voltage_2;
        //TIM_SetCompare4(TIM2, Motor_2);
    }
}
