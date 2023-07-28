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
#include "gray_i2c.h"
#include "timer.h" 
#include "control.h"
#include "MPU6050.h"
#include "BLU.h"
#include "task.h"
#include "servo.h"
#include "protocol_process.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>


//断言（这里是自定义的模拟断言，但没有调用，用 configASSERT(x)来判断，vAssertCalled(char,int)来输出错误信息）
#define     vAssertCalled(err, char, int)       printf("Error:%s on %s,%d\r\n",err, char, int)
#define     configASSERT(err)                   vAssertCalled(err, __FILE__, __LINE__)
#define     abs(x)                              (int)(x > 0 ? x:-x)


#define     True                1
#define     False               0


// 电机参数
#define     Dead_Voltage_1      0
#define     Dead_Voltage_2      0
#define     PWM_Max             3550        // pwm满幅3600
#define     Rpm_Max             300         // 最大转速
#define     middle_loc          170         // 摄像头循迹的中间位置
#define     I_restrict          3200
#define     Kc                  0.5         // 抗积分饱和系数


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

// 按键标志位

extern uint8_t user_key_num;                     





// 控制标志位

extern __IO int Target_Velocity_1, Reality_Velocity_1;   /* 目标速度，实际速度 */
extern __IO int Target_Position_1, Reality_Position_1;   /* 目标位置，实际位置 */

extern __IO int Target_Velocity_2, Reality_Velocity_2;   /* 目标速度，实际速度 */
extern __IO int Target_Position_2, Reality_Position_2;   /* 目标位置，实际位置 */

extern int Target_angle;
extern uint8_t start_flag;
extern uint8_t stop_flag;
extern uint8_t task_finish;



// 灰度标志位

extern uint8_t passby_cross_num;





//串口开启DMA控制

//#define             USART1_DMA
//#define             USART3_DMA

#define             Usart_TxData_Len                        128
#define             Usart_RxData_Len                        128

typedef struct {
    uint8_t send_buff[Usart_TxData_Len];
    __IO uint8_t rece_buff[Usart_RxData_Len];
    uint32_t send_len;
    uint32_t rece_len;
    uint8_t send_flag;
    uint8_t rece_flag;
    uint8_t send_dma_flag;
    uint8_t rece_dma_flag;
} Usart_Data;

extern Usart_Data Usart1_Data;
extern Usart_Data Usart3_Data;





// 通信协议相关标志位

extern __IO uint8_t BLU_rxdata[4];
extern __IO int capture_data[4];
extern __IO uint16_t distant;




//其他标志位

extern __IO uint8_t Beep_flag;                             /* 蜂鸣器标志位 */


// 陀螺仪相关标志位

// ATKM901
extern float ANGLE_STATIC_BIAS;                     /* 角度静差 */
extern float Now_Angle;                             /* 当前角度 */
extern uint8_t Gyro_OK;                             /* 陀螺仪校准完成标志 */
                            
// MPU6050
typedef struct {
    float pitch;
    float roll;
    float yaw;
} MPU;

extern MPU MPU_Data;                                /* 陀螺仪数据 */
extern uint8_t MPU_FLAG;                            /* 陀螺仪校准完成标志 */












// 板级支持包初始化
void bsp_init(void);
void info_print(void);
void Key_map(uint16_t KeyCode);
void Test_Encoder(void);


#endif
