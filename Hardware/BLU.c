#include "BLU.h"

__IO uint8_t BLU_rxdata[4];

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


void USART3_IRQHandler(void)
{ 
    uint8_t re = 0;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
	{
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);

        static uint8_t Count = 0;
        static uint8_t rxbuf[3];

        re = USART_ReceiveData(USART3);
        
        rxbuf[Count] = re;

        if (rxbuf[Count] == 0xFF || Count > 0)
            Count++;
        else
            Count = 0;
        
        if (Count == 3) {

            Count = 0;

            if (rxbuf[2] == 0xFE)
            {
                BLU_rxdata[0] = rxbuf[0];
                BLU_rxdata[1] = rxbuf[1];
                BLU_rxdata[2] = rxbuf[2];
                //OLED_ShowNum(1,1,BLU_rxdata[1],2);
            }
            
        }
        
	}
}


void BLU_Init(void) {

    BLU_Uart3_Init();

}






