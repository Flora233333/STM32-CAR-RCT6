
#include "bsp_usart.h"

// ����DMA�ṹ��
Usart_Data Usart1_Data;



// __IO int capture_data[4] = {middle_loc, 0, 0, 0};
// __IO uint16_t distant = 0; //��λ : mm


int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ����� */
		USART_SendData(USART1, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}


///�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
		/* �ȴ������������� */
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(DEBUG_USARTx);
}


void USART1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

	// �򿪴���GPIO��ʱ��
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
	
	// �򿪴��������ʱ��
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 2 ;
	//Subpriority //�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQͨ��ʹ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���	
	NVIC_Init(&NVIC_InitStructure);	
    
	// ���ô��ڵĹ�������
	// ���ò�����
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	// ���� �������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// ����У��λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// ����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
    
    USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
	// ʹ�ܴ���
	USART_Cmd(DEBUG_USARTx, ENABLE);	    
}

#ifdef USART1_DMA

void USART1_DMA_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

	// �򿪴���GPIO��ʱ��
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
	
	// �򿪴��������ʱ��
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 2;
	//Subpriority //�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQͨ��ʹ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���	
	NVIC_Init(&NVIC_InitStructure);	
    
	// ���ô��ڵĹ�������
	// ���ò�����
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	// ���� �������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// ����У��λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// ����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
    
    USART_ITConfig(DEBUG_USARTx, USART_IT_IDLE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
	// ʹ�ܴ���
	USART_Cmd(DEBUG_USARTx, ENABLE);	

    // ����DMAʱ��
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
	DMA_InitTypeDef DMA_InitStructure;
    // ����DMAԴ��ַ���������ݼĴ�����ַ*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
    // �ڴ��ַ(Ҫ����ı�����ָ��)
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Usart1_Data.rece_buff;
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
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);		
    // ʹ��DMA
    DMA_Cmd (DMA1_Channel5,ENABLE);

    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
}

#endif


void UART4_Init(void)    //��ʼ��
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure; 
 
	NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);  //USART4ʱ������
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 	  
 
	USART_DeInit(UART4);  
	//USART4_TX	PC.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);				  //��ʼ��PC10
	//USART4_RX	PC.11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;			 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //��������
	GPIO_Init(GPIOC, &GPIO_InitStructure);							  //��ʼ��PC11
 
	//USART4��ʼ��
	USART_InitStructure.USART_BaudRate = 115200; 
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ����λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1λֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ��������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //�շ�ģʽ
	USART_Init(UART4, &USART_InitStructure); //��ʼ������
 
	//Usart4 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	  //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		  //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure); //����ָ���Ĳ�����ʼ��VIC�Ĵ���
 
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//�����ж�
	USART_Cmd(UART4, ENABLE);					  //ʹ�ܴ���
	USART_ClearFlag(UART4, USART_FLAG_TXE);	   /* �巢����ɱ�־��Transmission Complete flag */   
}
 

