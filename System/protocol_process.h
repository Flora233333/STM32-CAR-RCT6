#ifndef __PROTOCOL_PROCESS_H
#define __PROTOCOL_PROCESS_H

#include "bsp.h"                  // Device header

#define     MAX_BUFFER_SIZE            64

#define     USART1_BUFFER_SIZE         32
#define     USART1_HEADER_BYTE         '<'
#define     USART1_FOOTER_BYTE         '>'

#define     USART3_BUFFER_SIZE         32
#define     USART3_HEADER_BYTE         '<'
#define     USART3_FOOTER_BYTE         '>'

#define     USART4_BUFFER_SIZE         32
#define     USART4_HEADER_BYTE         '<'
#define     USART4_FOOTER_BYTE         '>'

#define     UART5_BUFFER_SIZE          32
#define     UART5_HEADER_BYTE          '<'
#define     UART5_FOOTER_BYTE          '>'


// #define     USART1_DMA


typedef enum {
    STATE_IDLE,
    STATE_HEADER_RECEIVED,
    STATE_DATA_RECEIVED,
} ReceiveState_t;


typedef struct {
    uint8_t UART_MAX_SIZE;
    uint8_t rxBuffer[MAX_BUFFER_SIZE];
    uint8_t rxIndex;
    ReceiveState_t rxState;
} UART_Packet_t;








    
#endif

