#ifndef __BSP_H
#define __BSP_H

#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "oled.h"
//#include "PID.h"
#include "PWM.h"
//#include "Timer.h"
#include "led.h"
#include "Beep.h"
#include "encoder.h"  
#include "bsp_usart.h"
#include "bsp_key.h"  
#include "atk_ms901m.h"
#include "timer.h" 
#include "control.h"
#include "MPU6050.h"
#include "task.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//断言（这里是自定义的模拟断言，但没有调用，用 configASSERT(x)来判断，vAssertCalled(char,int)来输出错误信息）
#define     vAssertCalled(err, char, int)       printf("Error:%s on %s,%d\r\n",err, char, int)
#define     configASSERT(err)                   vAssertCalled(err, __FILE__, __LINE__)
#define     abs(x)                              (int)(x > 0 ? x:-x)


#define     True                1
#define     False               0


#define     Dead_Voltage_1      0
#define     Dead_Voltage_2      0
#define     PWM_Max             3550        // pwm满幅3600
#define     Rpm_Max             250         // 最大转速
#define     middle_loc          160         // 摄像头循迹的中间位置
#define     I_restrict          3200


// 任务系统配置界面

#define     Max_TASK_NUM        10




// 按键功能定义(左边为自定义功能名称)

#define             KEY_DOWN_K1		        KEY_1_DOWN
#define             KEY_UP_K1		        KEY_1_UP
#define             KEY_LONG_K1		        KEY_1_LONG

#define             KEY_DOWN_K2		        KEY_2_DOWN
#define             KEY_UP_K2		        KEY_2_UP
#define             KEY_LONG_K2		        KEY_2_LONG

#define             KEY_DOWN_K3		        KEY_3_DOWN
#define             KEY_UP_K3		        KEY_3_UP
#define             KEY_LONG_K3		        KEY_3_LONG

#define             KEY_DOWN_K4		        KEY_4_DOWN
#define             KEY_UP_K4		        KEY_4_UP
#define             KEY_LONG_K4		        KEY_4_LONG

#define             KEY_DOWN_K5             KEY_5_DOWN
#define             KEY_UP_K5	            KEY_5_UP
#define             KEY_LONG_K5             KEY_5_LONG

                        
                        


// 板级支持包初始化
void bsp_init(void);
void info_print(void);
void Key_map(uint16_t KeyCode);



#endif
