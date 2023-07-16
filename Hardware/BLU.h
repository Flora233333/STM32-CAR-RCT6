#ifndef __BLU_H
#define __BLU_H

#include "bsp.h"                  // Device header


// extern __IO uint8_t BLU_rxdata[4];

void BLU_Init(void);
void BLU_SendDataPack(u8 * Data, u16 SIZE);
void BLU_SendSingleData(u8 Data);
    
#endif
