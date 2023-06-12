#include "task.h"

__IO unsigned long long global_time = 0;

uint8_t Task_num = 0;

static Task_handle* sched_tasks[Max_TASK_NUM];


/***********************************任务句柄***********************************/

Task_handle * Task_1ms_handler = NULL;
Task_handle * Task_5ms_handler = NULL;
Task_handle * Task_10ms_handler = NULL;
Task_handle * Task_100ms_handler = NULL;
Task_handle * Task_500ms_handler = NULL;

/*****************************************************************************/


uint64_t Get_nowtime(void) {
	return global_time;
}
char str12[1000];
void Task_1ms(void) {
    //printf("1ms\r\n");
}

void Task_5ms(void) {

    //printf("5ms\r\n");

}

void Task_10ms(void) {
    bsp_KeyScan10ms();
    //printf("10ms\r\n");

}

void Task_100ms(void) {

    //printf("100ms\r\n");
    
}

void Task_500ms(void) {

    //printf("500ms\r\n");
}

void TIM_Standard_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 1000 - 1; // 1ms
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = DISABLE;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

    TIM_ClearITPendingBit(TIM6,TIM_IT_Update); //清空中断状态
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure); 

    TIM_Cmd(TIM6, ENABLE);
}

void TIM6_IRQHandler(void) {
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        //printf("TIM6_IRQHandler\n");
        global_time++;
    }
}

int8_t Create_Task(void(*task_func)(void), uint16_t cycle, uint32_t last_run, uint8_t is_live, uint8_t mode, Task_handle * user_handler) {
    Task_handle * task_handler = NULL;

    if (task_func == NULL) {
        configASSERT("_TASK_FUNC_NULL_");
        return _TASK_FUNC_NULL_;
    }

    if (Task_num == Max_TASK_NUM) {
        configASSERT("_TASK_NUM_EXCESSIVE_");
        return _TASK_NUM_EXCESSIVE_;
    }

    task_handler = (Task_handle *)malloc(sizeof(Task_handle));

    if (task_handler == NULL) {
        free(task_handler);     
        task_handler = NULL;
        configASSERT("_TASK_MEMORY_ALLOCATED_FAILED_");
        //printf("MEMORY ALLOCATED FAILED!\n");
        return _TASK_MEMORY_ALLOCATED_FAILED_;
    }

    task_handler->task_func = task_func;
    task_handler->cycle = cycle;
    task_handler->last_run = last_run;
    task_handler->is_live = is_live;
    task_handler->mode = mode;
    
    user_handler = task_handler;
    sched_tasks[Task_num++] = user_handler;
    
    return _TASK_CREATE_SUCCESS_;
}

int8_t Task_init(void) {
    uint8_t err1 = 0, err2 = 0, err3 = 0, err4 = 0, err5 = 0;
    
    err1 = Create_Task(Task_1ms, 1, 0, True, 0, Task_1ms_handler);
    err2 = Create_Task(Task_5ms, 5, 0, True, 0, Task_5ms_handler);
    err3 = Create_Task(Task_10ms, 10, 0, True, 0, Task_10ms_handler);
    err4 = Create_Task(Task_100ms, 100, 0, True, 0, Task_100ms_handler);
    err5 = Create_Task(Task_500ms, 500, 0, True, 0, Task_500ms_handler);

    if (err1 || err2 || err3 || err4 || err5) {
        configASSERT("_TASK_CREATE_FAILED_");
        return _TASK_CREATE_FAILED_;
    }

    TIM_Standard_Init();

    return _TASK_CREATE_SUCCESS_;
}

void Task_run() {
    uint8_t i = 0;

    for(i = 0; i < Task_num; i++) {
        
        if(sched_tasks[i]->is_live == True && sched_tasks[i]->mode == 0) { // 周期模式

            if(global_time - sched_tasks[i]->last_run >= sched_tasks[i]->cycle) {
                sched_tasks[i]->last_run = global_time;
                sched_tasks[i]->task_func();
            } 
        }
        else if(sched_tasks[i]->is_live == True && sched_tasks[i]->mode == 1) { // 单次任务模式

            if(global_time - sched_tasks[i]->last_run >= sched_tasks[i]->cycle) {
                sched_tasks[i]->last_run = global_time;
                sched_tasks[i]->is_live = False;
            } 
            else {
                sched_tasks[i]->task_func();
            }
        }
    }
}
