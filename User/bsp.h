#ifndef __BSP_H
#define __BSP_H

#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "oled.h"
//#include "PID.h"
#include "PWM.h"
//#include "Timer.h"
#include "led.h"
#include "encoder.h"  
#include "bsp_usart.h"
#include "bsp_key.h"  
#include "atk_ms901m.h"
#include "timer.h" 
#include "control.h"
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
#define     PWM_Max             3500        // pwm满幅3600
#define     Rpm_Max             250         // 最大转速
#define     middle_loc          160         // 摄像头循迹的中间位置
#define     I_restrict          3200


// 任务系统配置界面

#define     Max_TASK_NUM        10




#endif
