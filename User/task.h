#ifndef __TASK_H
#define __TASK_H

#include "bsp.h"


#define         _TASK_CREATE_SUCCESS_                    0
#define         _TASK_MEMORY_ALLOCATED_FAILED_          -1
#define         _TASK_NUM_EXCESSIVE_                    -2
#define         _TASK_FUNC_NULL_                        -3
#define         _TASK_CREATE_FAILED_                    -4

extern __IO unsigned long long global_time;

typedef struct {
	void(*task_func)(void);
	uint16_t cycle;
	uint64_t last_run;
    uint8_t is_live;
    uint8_t mode; //mode=0 周期模式，mode=1 任务模式
} Task_handle;

int8_t Task_init(void);
uint64_t Get_nowtime(void);
void Task_run(void);

#endif
