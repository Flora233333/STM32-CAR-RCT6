#ifndef __PWM_H
#define __PWM_H

#include "bsp.h"

void PWM_Init(void);
int PWM_restrict(int Motor, int max);
void PWM_updata_Motor1(int Motor_1);
void PWM_updata_Motor2(int Motor_2);
void PWM_updata(int Motor_1, int Motor_2);
void PWM_stop(void);

#endif
