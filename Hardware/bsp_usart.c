
#include "bsp_usart.h"

__IO int capture_data[4] = {middle_loc, 0, 0, 0};
__IO uint16_t distant = 0; //单位 : mm

 /**
  * @brief  USART GPIO 配置,工作参数配置
  * @param  无
  * @retval 无
  */
void USART1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

	// 打开串口GPIO的时钟
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
	
	// 打开串口外设的时钟
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

	// 将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  // 将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 2;
	//Subpriority //子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQ通道使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器	
	NVIC_Init(&NVIC_InitStructure);	
    
	// 配置串口的工作参数
	// 配置波特率
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 配置停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
    
    USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
	// 使能串口
	USART_Cmd(DEBUG_USARTx, ENABLE);	

    //TODO 调试明白一下这个到底有没有必要加
    //USART_ClearFlag(DEBUG_USARTx, USART_FLAG_TXE);    
}


int USART1_IRQHandler(void)
{	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //Check if data is received //判断是否接收到数据
	{
        USART_ClearITPendingBit(USART1,USART_IT_RXNE); //清除中断标志
        
        u8 Usart_Receive;
		static u8 Count = 0;
		static u8 rxbuf[3] = {0,0,0};
		u8 error = 0;

		Usart_Receive = USART_ReceiveData(USART1);
        
        rxbuf[Count] = Usart_Receive;
        //OLED_ShowNum(2, 1, Count, 1);
        //printf("%d ", rxbuf[Count]);
		//确保数组第一个数据为0xFF
        if(rxbuf[Count] == 0xFF || Count > 0) 
			Count++; 
		else 
			Count = 0;
        //printf("%d ", Count);
		if (Count == 3) //Verify the length of the packet //验证数据包的长度
		{   
    
            Count = 0; //为串口数据重新填入数组做准备

            if(rxbuf[2] == 0xFE) //Verify the frame tail of the packet //验证数据包的帧尾
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


///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(USART1, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		/* 等待串口输入数据 */
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(DEBUG_USARTx);
}


void UART4_Init(void)    //初始化
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure; 
 
	NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);  //USART4时钟设置
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 	  
 
	USART_DeInit(UART4);  
	//USART4_TX	PC.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_Init(GPIOC, &GPIO_InitStructure);				  //初始化PC10
	//USART4_RX	PC.11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;			 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
	GPIO_Init(GPIOC, &GPIO_InitStructure);							  //初始化PC11
 
	//USART4初始化
	USART_InitStructure.USART_BaudRate = 115200; 
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1位停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
	USART_Init(UART4, &USART_InitStructure); //初始化串口
 
	//Usart4 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	  //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		  //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器
 
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启中断
	USART_Cmd(UART4, ENABLE);					  //使能串口
	USART_ClearFlag(UART4, USART_FLAG_TXE);	   /* 清发送完成标志，Transmission Complete flag */   
}
 
 
 // TODO 将串口接收中断写的更鲁棒性强些
void UART4_IRQHandler(void) //中断接收处理函数；
{
	
    if(USART_GetITStatus(UART4,USART_IT_RXNE)!=RESET)
    {
        USART_ClearITPendingBit(UART4,USART_IT_RXNE);
                
        u8 data;
        static u8 flag = 0;
		static u8 Count = 0;
		static u8 rxbuf[4] = {0, 0, 0, 0};
		//static u8 error = 0;
        //激光模块数据格式：d:  226 mm\r\nState;0 , Range Valid
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
 


