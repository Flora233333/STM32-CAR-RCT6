#ifndef __CONTROL_H
#define __CONTROL_H

#include "bsp.h"

#define  Amplitude     3550     /* PWM满幅是3600 */
#define  Dead_Voltage  2070     /* 死区电压 */
#define  Rpm_Max       250      /* 最大转速 */


//#define  PWM  TIM1->CCR4        /* PA11 */

//#define  AIN1(x)  do{x?HAL_GPIO_WritePin(GPIOB,AIN1_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOB,AIN1_Pin,GPIO_PIN_RESET);}while(0) /* AIN1翻转 */                 
//#define  AIN2(x)  do{x?HAL_GPIO_WritePin(GPIOB,AIN2_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOB,AIN2_Pin,GPIO_PIN_RESET);}while(0) /* AIN2翻转 */
                  

//extern int Target_Velocity,Reality_Velocity;   
//extern int Target_Position,Reality_Position;   

//void Set_Pwm(int moto);
//void Moto_Stop(void);
//int Xianfu(int data,int max);

extern int Target_Velocity_1, Reality_Velocity_1;   /* 目标速度，实际速度 */
extern int Target_Position_1, Reality_Position_1;   /* 目标位置，实际位置 */

extern int Target_Velocity_2, Reality_Velocity_2;   /* 目标速度，实际速度 */
extern int Target_Position_2, Reality_Position_2;   /* 目标位置，实际位置 */


int Position_PID_left(int reality,int target);
int Position_PID_right(int reality,int target);
int Incremental_PID_left(int reality,int target);       
int Incremental_PID_right(int reality,int target);  

#endif
