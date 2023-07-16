#include "BLU.h"

// __IO uint8_t BLU_rxdata[4];


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


void BLU_Init(void) {

    BLU_Uart3_Init();

}






