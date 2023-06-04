#ifndef __ENCODER_H
#define __ENCODER_H

#include "bsp.h"       

void Motor_Init(void);
void Encoder_Init(void);
void Motor1_SetDirct(uint16_t A1, uint16_t B1);
void Motor2_SetDirct(uint16_t A2, uint16_t B2);
int Read_Encoder(uint8_t TIMX);
long Rpm_Encoder_Cnt(float rpm,uint16_t ppr,uint16_t ratio,uint16_t cnt_time);
long Num_Encoder_Cnt(float num,uint16_t ppr,float ratio);

#endif
