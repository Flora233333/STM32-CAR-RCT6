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


//���ԣ��������Զ����ģ����ԣ���û�е��ã��� configASSERT(x)���жϣ�vAssertCalled(char,int)�����������Ϣ��
#define     vAssertCalled(err, char, int)       printf("Error:%s on %s,%d\r\n",err, char, int)
#define     configASSERT(err)                   vAssertCalled(err, __FILE__, __LINE__)
#define     abs(x)                              (int)(x > 0 ? x:-x)


#define     True                1
#define     False               0


// �������
#define     Dead_Voltage_1      0
#define     Dead_Voltage_2      0
#define     PWM_Max             3550        // pwm����3600
#define     Rpm_Max             300         // ���ת��
#define     middle_loc          170         // ����ͷѭ�����м�λ��
#define     I_restrict          3200
#define     Kc                  0.5         // �����ֱ���ϵ��


// ����ϵͳ���ý���

#define     Max_TASK_NUM        10



// �������ܶ���(���Ϊ�Զ��幦������)

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

// ������־λ

extern uint8_t user_key_num;                     





// ���Ʊ�־λ

extern __IO int Target_Velocity_1, Reality_Velocity_1;   /* Ŀ���ٶȣ�ʵ���ٶ� */
extern __IO int Target_Position_1, Reality_Position_1;   /* Ŀ��λ�ã�ʵ��λ�� */

extern __IO int Target_Velocity_2, Reality_Velocity_2;   /* Ŀ���ٶȣ�ʵ���ٶ� */
extern __IO int Target_Position_2, Reality_Position_2;   /* Ŀ��λ�ã�ʵ��λ�� */

extern int Target_angle;
extern uint8_t start_flag;
extern uint8_t stop_flag;
extern uint8_t task_finish;



// �Ҷȱ�־λ

extern uint8_t passby_cross_num;





//���ڿ���DMA����

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





// ͨ��Э����ر�־λ

extern __IO uint8_t BLU_rxdata[4];
extern __IO int capture_data[4];
extern __IO uint16_t distant;




//������־λ

extern __IO uint8_t Beep_flag;                             /* ��������־λ */


// ��������ر�־λ

// ATKM901
extern float ANGLE_STATIC_BIAS;                     /* �ǶȾ��� */
extern float Now_Angle;                             /* ��ǰ�Ƕ� */
extern uint8_t Gyro_OK;                             /* ������У׼��ɱ�־ */
                            
// MPU6050
typedef struct {
    float pitch;
    float roll;
    float yaw;
} MPU;

extern MPU MPU_Data;                                /* ���������� */
extern uint8_t MPU_FLAG;                            /* ������У׼��ɱ�־ */












// �弶֧�ְ���ʼ��
void bsp_init(void);
void info_print(void);
void Key_map(uint16_t KeyCode);
void Test_Encoder(void);


#endif
