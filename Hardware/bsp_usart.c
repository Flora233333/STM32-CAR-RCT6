
#include "bsp_usart.h"

__IO int capture_data[4] = {middle_loc, 0, 0, 0};
__IO uint16_t distant = 0; //��λ : mm

 /**
  * @brief  USART GPIO ����,������������
  * @param  ��
  * @retval ��
  */
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
    
    USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
	// ʹ�ܴ���
	USART_Cmd(DEBUG_USARTx, ENABLE);	

    //TODO ��������һ�����������û�б�Ҫ��
    //USART_ClearFlag(DEBUG_USARTx, USART_FLAG_TXE);    
}


int USART1_IRQHandler(void)
{	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
	{
        USART_ClearITPendingBit(USART1,USART_IT_RXNE); //����жϱ�־
        
        u8 Usart_Receive;
		static u8 Count = 0;
		static u8 rxbuf[3] = {0,0,0};
		u8 error = 0;

		Usart_Receive = USART_ReceiveData(USART1);
        
        rxbuf[Count] = Usart_Receive;
        //OLED_ShowNum(2, 1, Count, 1);
        //printf("%d ", rxbuf[Count]);
		//ȷ�������һ������Ϊ0xFF
        if(rxbuf[Count] == 0xFF || Count > 0) 
			Count++; 
		else 
			Count = 0;
        //printf("%d ", Count);
		if (Count == 3) //Verify the length of the packet //��֤���ݰ��ĳ���
		{   
    
            Count = 0; //Ϊ����������������������׼��

            if(rxbuf[2] == 0xFE) //Verify the frame tail of the packet //��֤���ݰ���֡β
            {			
                if(error == 0)	 
                {		
                    capture_data[0] = rxbuf[1] * 2;
//                    rxarr[1] = rxbuf[2];
//                    rxarr[2] = rxbuf[3];
//                    rxarr[3] = rxbuf[4];
                }
            }
		}
	}
	return 0;	
}


///�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
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
 
 
 // TODO �����ڽ����ж�д�ĸ�³����ǿЩ
void UART4_IRQHandler(void) //�жϽ��մ�������
{
	
    if(USART_GetITStatus(UART4,USART_IT_RXNE)!=RESET)
    {
        USART_ClearITPendingBit(UART4,USART_IT_RXNE);
                
        u8 data;
        static u8 flag = 0;
		static u8 Count = 0;
		static u8 rxbuf[4] = {0, 0, 0, 0};
		//static u8 error = 0;
        //����ģ�����ݸ�ʽ��d:  226 mm\r\nState;0 , Range Valid
		data = USART_ReceiveData(UART4);

        if(data == ':') {
            flag = 1;
            Count = 0;
            return;
            //error = 0;
        }
        if(flag == 1 && data != ' ') {
            if(data < '0' || data > '9') {
                if(data == 'm') {
                    uint8_t i = 0;
                    uint16_t sum = 0;
                    for(i = 0; i < Count; i++) {
                        sum += (rxbuf[i] - '0') * pow(10, Count - i - 1);
                    }
                    distant = sum;
                }
                else {
                    //error = 1;
                }
                flag = 0;
                Count = 0;
            }
            else
                rxbuf[Count++] = data;
        }
	}
}
 


