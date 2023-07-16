#include "BLU.h"

// __IO uint8_t BLU_rxdata[4];
Usart_Data Usart3_Data;


void BLU_Uart3_Init(void) {
    
    GPIO_InitTypeDef GPIO_InitStructure;                         
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE); 
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;               
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB , &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;             
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB , &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = 115200;               
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;              
    
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;         
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         
    NVIC_Init(&NVIC_InitStructure);         

    USART_Init(USART3,&USART_InitStructure);                        
    
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);  

    USART_Cmd(USART3,ENABLE);
}


#ifdef USART3_DMA

void BLU_Uart3_DMA_Init(void) 
{    
    GPIO_InitTypeDef GPIO_InitStructure;                         
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE); 
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;               
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB , &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;             
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB , &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = 115200;               
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;              
    
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;         
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         
    NVIC_Init(&NVIC_InitStructure);         

    USART_Init(USART3,&USART_InitStructure);                        
    
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);  

    USART_Cmd(USART3,ENABLE);

    // 开启DMA时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
	DMA_InitTypeDef DMA_InitStructure;
    // 设置DMA源地址：串口数据寄存器地址*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
    // 内存地址(要传输的变量的指针)
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Usart3_Data.rece_buff;
    // 方向：从外设到内存	
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    // 传输大小	
    DMA_InitStructure.DMA_BufferSize = Usart_RxData_Len;
    // 外设地址不增	    
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // 内存地址自增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    // 外设数据单位	
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    // 内存数据单位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 
    // DMA模式，一次或者循环模式
//		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	
    // 优先级：中	
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 
    // 禁止内存到内存的传输
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    // 配置DMA通道		   
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);		
    // 使能DMA
    DMA_Cmd (DMA1_Channel3,ENABLE);

    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
}

#endif


void BLU_SendDataPack(u8 * Data, u16 SIZE) {
	u16 i = 0;

    USART_SendData(USART3, 0xFF);
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET); //发送完了标志位

	for(i = 0; i < SIZE; i++)
	{
		USART_SendData(USART3,Data[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET); //发送完了标志位
	}

    USART_SendData(USART3, 0xFE);
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET); //发送完了标志位
}

void BLU_SendSingleData(u8 Data) {

    USART_SendData(USART3, 0xFF);
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET); //发送完了标志位

	USART_SendData(USART3, Data);
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET); //发送完了标志位

    USART_SendData(USART3, 0xFE);
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET); //发送完了标志位
}








