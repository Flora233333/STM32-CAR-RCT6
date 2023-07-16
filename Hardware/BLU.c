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

    // ����DMAʱ��
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
	DMA_InitTypeDef DMA_InitStructure;
    // ����DMAԴ��ַ���������ݼĴ�����ַ*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
    // �ڴ��ַ(Ҫ����ı�����ָ��)
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Usart3_Data.rece_buff;
    // ���򣺴����赽�ڴ�	
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    // �����С	
    DMA_InitStructure.DMA_BufferSize = Usart_RxData_Len;
    // �����ַ����	    
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // �ڴ��ַ����
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    // �������ݵ�λ	
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    // �ڴ����ݵ�λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 
    // DMAģʽ��һ�λ���ѭ��ģʽ
//		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	
    // ���ȼ�����	
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 
    // ��ֹ�ڴ浽�ڴ�Ĵ���
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    // ����DMAͨ��		   
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);		
    // ʹ��DMA
    DMA_Cmd (DMA1_Channel3,ENABLE);

    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
}

#endif


void BLU_SendDataPack(u8 * Data, u16 SIZE) {
	u16 i = 0;

    USART_SendData(USART3, 0xFF);
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET); //�������˱�־λ

	for(i = 0; i < SIZE; i++)
	{
		USART_SendData(USART3,Data[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET); //�������˱�־λ
	}

    USART_SendData(USART3, 0xFE);
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET); //�������˱�־λ
}

void BLU_SendSingleData(u8 Data) {

    USART_SendData(USART3, 0xFF);
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET); //�������˱�־λ

	USART_SendData(USART3, Data);
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET); //�������˱�־λ

    USART_SendData(USART3, 0xFE);
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET); //�������˱�־λ
}








