#ifndef __BLU_H
#define __BLU_H

#include "bsp.h"                  // Device header


// extern __IO uint8_t BLU_rxdata[4];


// #define        USART3_DMA



void BLU_Uart3_Init(void);
void BLU_Uart3_DMA_Init(void);
void BLU_SendDataPack(u8 * Data, u16 SIZE);
void BLU_SendSingleData(u8 Data);
    

#endif
