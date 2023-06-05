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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#define     Dead_Voltage_1      0
#define     Dead_Voltage_2      0
#define     PWM_Max             3500        // pwm满幅3600
#define     Rpm_Max             250         // 最大转速
#define     middle_loc          160         // 摄像头循迹的中间位置
#define     I_restrict          3200



#endif
