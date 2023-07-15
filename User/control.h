#ifndef __CONTROL_H
#define __CONTROL_H

#include "bsp.h"


extern __IO int Target_Velocity_1, Reality_Velocity_1;   /* Ŀ���ٶȣ�ʵ���ٶ� */
extern __IO int Target_Position_1, Reality_Position_1;   /* Ŀ��λ�ã�ʵ��λ�� */

extern __IO int Target_Velocity_2, Reality_Velocity_2;   /* Ŀ���ٶȣ�ʵ���ٶ� */
extern __IO int Target_Position_2, Reality_Position_2;   /* Ŀ��λ�ã�ʵ��λ�� */

extern int Target_angle;
extern uint8_t start_flag;
extern uint8_t stop_flag;
extern uint8_t task_finish;


void Mode_Select(void);
int forwardfeedback(float in);
int Position_PID_left(int reality,int target);
int Position_PID_right(int reality,int target);
int Incremental_PID_left(int reality,int target);       
int Incremental_PID_right(int reality,int target);  
int Trace_PID(int reality,int target);
int Angle_PID(int reality,int target);
int Gray_PID(float reality,int target);
int Distant_PID(int reality,int target);
void Task_Update(void);

#endif
