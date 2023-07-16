#include "protocol_process.h"


UART_Packet_t USART1_packet;
UART_Packet_t USART3_packet;
UART_Packet_t USART4_packet;
UART_Packet_t UART5_packet;


__IO uint8_t BLU_rxdata[4];
__IO int capture_data[4];
__IO uint16_t distant;


void UART_Packet_Init(UART_Packet_t* UART_packet, uint8_t max) {
    UART_packet->UART_MAX_SIZE = max;
    UART_packet->rxIndex = 0;
    UART_packet->rxState = STATE_IDLE;
}


void ALL_Packet_Init(void) {
    UART_Packet_Init(&USART1_packet, USART1_BUFFER_SIZE);
    UART_Packet_Init(&USART3_packet, USART3_BUFFER_SIZE);
    UART_Packet_Init(&USART4_packet, USART4_BUFFER_SIZE);
    UART_Packet_Init(&UART5_packet, UART5_BUFFER_SIZE);
}


void processPacket(uint8_t* packet, uint8_t length) {
    uint8_t equipment = packet[1];
    for (uint8_t i = 0; i < length; i++) {
        printf("%c", packet[i]);
    }
    printf("\r\n");
    switch (equipment) {
        case 0x01:
            // Process data from equipment 1
            break;
        case 0x02:
            // Process data from equipment 2
            break;
        default:
            // Unknown equipment
            break;
    }
}

#ifdef USART1_DMA


#else

    int USART1_IRQHandler(void) {
    if(USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        uint8_t data = USART_ReceiveData(USART1);
                
        switch (USART1_packet.rxState) 
        {
                case STATE_IDLE:
                    if (data == USART1_HEADER_BYTE) {
                        USART1_packet.rxState = STATE_HEADER_RECEIVED;
                        USART1_packet.rxIndex = 0;
                        USART1_packet.rxBuffer[USART1_packet.rxIndex++] = data;
                    }
                    break;

                case STATE_HEADER_RECEIVED:
                    if (USART1_packet.rxIndex < USART1_BUFFER_SIZE) {
                        USART1_packet.rxBuffer[USART1_packet.rxIndex++] = data;

                        if (data == USART1_FOOTER_BYTE) {
                            USART1_packet.rxState = STATE_DATA_RECEIVED;
                            // Process received data packet
                            processPacket(USART1_packet.rxBuffer, USART1_packet.rxIndex);
                        }
                    } 
                    else {
                        USART1_packet.rxState = STATE_IDLE;
                        USART1_packet.rxIndex = 0;
                    }
                    break;

                case STATE_DATA_RECEIVED:
                    // Discard any additional data until next header byte is received
                    if (data == USART1_HEADER_BYTE) {
                        USART1_packet.rxState = STATE_HEADER_RECEIVED;
                        USART1_packet.rxIndex = 0;
                        USART1_packet.rxBuffer[USART1_packet.rxIndex++] = data;
                    }
                    break;
        }
        }
        return 0;
    }
    
#endif

// int USART1_IRQHandler(void)
// {	
//  	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //Check if data is received //判断是否接收到数据
//  	{
//         USART_ClearITPendingBit(USART1,USART_IT_RXNE); //清除中断标志
         
//         u8 Usart_Receive;
// 		static u8 Count = 0;
// 		static u8 rxbuf[3] = {0,0,0};
// 		u8 error = 0;

// 		Usart_Receive = USART_ReceiveData(USART1);
//        printf("%d\r\n", Usart_Receive);
//         rxbuf[Count] = Usart_Receive;
//         //OLED_ShowNum(2, 1, Count, 1);
//         //printf("%d ", rxbuf[Count]);
// 		//确保数组第一个数据为0xFF
//         if(rxbuf[Count] == 0xFF || Count > 0) 
// 			Count++; 
// 		else 
// 			Count = 0;
//         //printf("%d ", Count);
// 		if (Count == 3) //Verify the length of the packet //验证数据包的长度
// 		{   
//             Count = 0; //为串口数据重新填入数组做准备

//             if(rxbuf[2] == 0xFE) //Verify the frame tail of the packet //验证数据包的帧尾
//             {			
//                 if(error == 0)	 
//                 {		
//                     capture_data[0] = rxbuf[1] * 2;
// //                    rxarr[1] = rxbuf[2];
// //                    rxarr[2] = rxbuf[3];
// //                    rxarr[3] = rxbuf[4];
//                 }
//             }
// 		}
//  	}
//  	return 0;	
// }


void USART3_IRQHandler(void)
{ 
    uint8_t re = 0;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
	{
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);

        static uint8_t Count = 0;
        static uint8_t rxbuf[3] = {0, 0, 0};

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
                BLU_rxdata[0] = rxbuf[1];
                //BLU_rxdata[1] = rxbuf[1];
                //BLU_rxdata[2] = rxbuf[2];
                //OLED_ShowNum(1,1,BLU_rxdata[1],2);
            }
        } 
	}
}


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








