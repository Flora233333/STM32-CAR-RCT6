#include "encoder.h"                

void Motor_Init() {
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 ;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15 ;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Encoder_Init() {
    //使能GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    //使能定时器
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    //因为编码器模式使用的就是通道1和通道2，这个是固定的，通道3和通道4不可配置为编码器模式，硬件设计时要注意。   
    GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);		

	//时基单元配置
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;    
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 65535;  //配置0xFFFF是技巧
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0x00; //不要分频
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	TIM_SetCounter(TIM3, 0);
	TIM_SetCounter(TIM4, 0);
	//开启时钟
	TIM_Cmd(TIM3, ENABLE); 
    TIM_Cmd(TIM4, ENABLE); 
}

void Motor1_SetDirct(uint16_t A1, uint16_t A2) {
    GPIO_WriteBit(GPIOB, GPIO_Pin_0, (BitAction)A1);
	GPIO_WriteBit(GPIOB, GPIO_Pin_1, (BitAction)A2);
}

void Motor2_SetDirct(uint16_t B1, uint16_t B2) {
    GPIO_WriteBit(GPIOB, GPIO_Pin_14, (BitAction)B1);
	GPIO_WriteBit(GPIOB, GPIO_Pin_15, (BitAction)B2);
}

int Read_Encoder(uint8_t TIMX)
{
    int Encoder_cnt;    
    switch(TIMX)
    {
        case 3:  Encoder_cnt= (short)(TIM3 -> CNT); TIM3 -> CNT=0; break;
        case 4:  Encoder_cnt= (short)(TIM4 -> CNT); TIM4 -> CNT=0; break;		
        default: Encoder_cnt=0;
    }
    return Encoder_cnt;
}

/**************************************************************************
功    能: 计算实际转速
输    入: encoder_cnt：脉冲数；ppr：码盘数；ratio：减速比；cnt_time：计数时间(ms)
返回  值: 车轮转速 rpm
**************************************************************************/
float Moto_Speed(int encoder_cnt,uint16_t ppr,uint16_t ratio,uint16_t cnt_time)
{
    encoder_cnt = abs(encoder_cnt);  
    return (encoder_cnt/4/ppr/ratio)*(1000/cnt_time)*60;    /* 4倍频 */   
}

/**************************************************************************
功    能: 计算转数对应编码器脉冲数
输    入: num：转数；ppr：码盘数；ratio：减速比
返 回 值: 电机脉冲数 
**************************************************************************/
long Num_Encoder_Cnt(float num,uint16_t ppr,float ratio)
{
    return (num*ratio*ppr*4);                               /* 4倍频 */       
}

/**************************************************************************
功    能: 计算转速对应编码器脉冲数 rpm单位: min/圈
输    入: rpm：转速；ppr：码盘数；ratio：减速比；cnt_time：计数时间(ms)
返 回 值: 电机脉冲数 
**************************************************************************/
long Rpm_Encoder_Cnt(float rpm,uint16_t ppr,uint16_t ratio,uint16_t cnt_time)
{
    return (rpm*ratio*ppr*4)/(60*1000/cnt_time);            /* 4倍频 */       
}


void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != 0)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) != 0)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}
