#ifndef __TASK_H
#define __TASK_H

#include "bsp.h"


#define         _TASK_RUN_SUCCESS_                       0
#define         _TASK_CREATE_SUCCESS_                    0
#define         _TASK_SLEEP_SUCCESS_                     0
#define         _TASK_RESUME_SUCCESS_                    0
#define         _TASK_DELAY_SUCCESS_                     0


#define         _TRANSACTION_TRIGGERING_                 0


#define         _Transaction_NULL_                      -1
#define         _TRANSACTION_FUNC_NULL_                 -2


#define         _TASK_MEMORY_ALLOCATED_FAILED_          -1
#define         _TASK_NUM_EXCESSIVE_                    -2
#define         _TASK_FUNC_NULL_                        -3
#define         _TASK_CREATE_FAILED_                    -4
#define         _TASK_HANDLER_NULL_                     -5
#define         _UNKNOWN_TASK_MODE_                     -6


extern __IO int64_t global_time;


// �¼�����
// ���Ǵ���һ���жϺ���ָ�룬�ں����ڲ��ж��Ƿ񴥷��¼�����������¼����ͷ���0�����򷵻�-1
typedef struct {
    int8_t(*Transaction_func)(void);
} Transaction_handle;


typedef struct {
	void(*task_func)(void);
	int64_t last_run; // ���Ա�ʾΪstart_time, ������һ��ִ�����ʱ��
    uint16_t cycle; //mode=0 ����ʱ��, mode=1 ��������ִ��ʱ��
    uint8_t is_live; //�Ƿ���
    uint8_t mode; //mode=0 ����ģʽ, mode=1 ���γ�������ģʽ, mode=2 ����ģʽ, mode=3 ��������ģʽ(��ʱ���ִֻ��һ��)
    Transaction_handle transaction;
} Task_handle;


/***********************************������***********************************/

extern Task_handle Task_1ms_handler;
extern Task_handle Task_5ms_handler;
extern Task_handle Task_10ms_handler;
extern Task_handle Task_100ms_handler;
extern Task_handle Task_500ms_handler;

extern Task_handle Get_Angle_Static_Bias_handler;
extern Task_handle Detect_Special_GrayData_handler;
extern Task_handle Beep_ON_handler;
/*****************************************************************************/


int64_t Get_nowtime(void);

int8_t Task_Init(void);
int8_t Task_Run(void);
int8_t Task_Suspend(Task_handle * task_handler);
int8_t Task_Resume(Task_handle * task_handler);
int8_t Task_Delay(Task_handle * task_handler, uint32_t time);

#endif
