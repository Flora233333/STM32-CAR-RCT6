#ifndef __CONTROL_H
#define __CONTROL_H

#include "bsp.h"

#define  Amplitude     3550     /* PWM������3600 */
#define  Dead_Voltage  2070     /* ������ѹ */
#define  Rpm_Max       250      /* ���ת�� */


//#define  PWM  TIM1->CCR4        /* PA11 */

//#define  AIN1(x)  do{x?HAL_GPIO_WritePin(GPIOB,AIN1_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOB,AIN1_Pin,GPIO_PIN_RESET);}while(0) /* AIN1��ת */                 
//#define  AIN2(x)  do{x?HAL_GPIO_WritePin(GPIOB,AIN2_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOB,AIN2_Pin,GPIO_PIN_RESET);}while(0) /* AIN2��ת */
                  

//extern int Target_Velocity,Reality_Velocity;   
//extern int Target_Position,Reality_Position;   

//void Set_Pwm(int moto);
//void Moto_Stop(void);
//int Xianfu(int data,int max);

extern int Target_Velocity_1, Reality_Velocity_1;   /* Ŀ���ٶȣ�ʵ���ٶ� */
extern int Target_Position_1, Reality_Position_1;   /* Ŀ��λ�ã�ʵ��λ�� */

extern int Target_Velocity_2, Reality_Velocity_2;   /* Ŀ���ٶȣ�ʵ���ٶ� */
extern int Target_Position_2, Reality_Position_2;   /* Ŀ��λ�ã�ʵ��λ�� */


int Position_PID_left(int reality,int target);
int Position_PID_right(int reality,int target);
int Incremental_PID_left(int reality,int target);       
int Incremental_PID_right(int reality,int target);  

#endif
