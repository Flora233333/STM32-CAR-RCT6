#ifndef __CONTROL_H
#define __CONTROL_H

#include "bsp.h"


extern int Target_Velocity_1, Reality_Velocity_1;   /* Ŀ���ٶȣ�ʵ���ٶ� */
extern int Target_Position_1, Reality_Position_1;   /* Ŀ��λ�ã�ʵ��λ�� */

extern int Target_Velocity_2, Reality_Velocity_2;   /* Ŀ���ٶȣ�ʵ���ٶ� */
extern int Target_Position_2, Reality_Position_2;   /* Ŀ��λ�ã�ʵ��λ�� */

extern int Target_angle;
extern uint8_t start_flag;

int forwardfeedback(float in);
int Position_PID_left(int reality,int target);
int Position_PID_right(int reality,int target);
int Incremental_PID_left(int reality,int target);       
int Incremental_PID_right(int reality,int target);  
int Trace_PID(int reality,int target);

#endif
