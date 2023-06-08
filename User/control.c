#include "control.h"


float Position_KP=0.295,Position_KI=0,Position_KD=0.68;         //位置环PID
float Incremental_KP=80,Incremental_KI=10,Incremental_KD=0;     //速度环PID
float trace_KP = 10, trace_KI = 0, trace_KD = 30;               //巡线环PID
float angle_KP = 0, angle_KI = 0, angle_KD = 0;                 //角度环PID 

// dead=0 float Position_KP=0.295,Position_KI=0,Position_KD=0.68;float Incremental_KP=80,Incremental_KI=10,Incremental_KD=0;   /* 增量式PID系数 */

int Target_Velocity_1 = 0, Reality_Velocity_1 = 0;   /* 目标速度，实际速度 */
int Target_Position_1 = 0, Reality_Position_1 = 0;   /* 目标位置，实际位置 */

int Target_Velocity_2 = 0, Reality_Velocity_2 = 0;   /* 目标速度，实际速度 */
int Target_Position_2 = 0, Reality_Position_2 = 0;   /* 目标位置，实际位置 */

int Target_angle = 0;                                /* 目标角度 */

uint8_t start_flag = 0;

int task1_time = 0, task2_time = 0;

// 10ms定时器更新中断回调函数
void TIM1_UP_IRQHandler() {
    
    static int Moto1 = 0, Moto2 = 0;
    
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {
        global_time++;
        if(start_flag == 1) {
            int bias = 0;
            
            task1_time = global_time;

            Reality_Velocity_1 = Read_Encoder(3);                       /* 获取实际脉冲数 */         
            Reality_Position_1 += Reality_Velocity_1;                   /* 实际位置脉冲数 */
            
            Reality_Velocity_2 = -Read_Encoder(4);                       /* 获取实际脉冲数 */         
            Reality_Position_2 += Reality_Velocity_2;                   /* 实际位置脉冲数 */
            
            Moto1 = -1500;
            
            Moto2 = -1500;
            
            
            bias = Trace_PID(rxarr[0], middle_loc);
            
            
            //Moto1 = Position_PID_left(Reality_Position_1,Target_Position_1);  /* 位置式位置控制 */    
            //Moto2 = Position_PID_right(Reality_Position_2,Target_Position_2);  /* 位置式位置控制 */
            
            //Moto1 = PWM_restrict(Moto1,Target_Velocity_1);                    /* 位置环输出限幅 */
            //Moto2 = PWM_restrict(Moto2,Target_Velocity_2);                    /* 位置环输出限幅 */
            
    //        if(t1 <= 100 && abs(Reality_Position_1-Target_Position_1) <= 3)             /* 滤除部分干扰 */
    //        {
    //            //PWM_updata_Motor1(0);
    //            if(t1++ == 100){
    ////                Target_Velocity_1 = Rpm_Encoder_Cnt(150,13,30,10);   /* 将转速转化为10ms的脉冲数，目标速度 */
    ////                Target_Position_1 = Num_Encoder_Cnt(0,13,30);      /* 将圈数转化为目标脉冲数，目标位置 */
    //            }
    //            else {
    //                PWM_updata_Motor1(0);
    //            }
    //        }
    //        else


            {
                //Moto1 = Incremental_PID_left(Moto1, Target_Velocity_1);             /* 增量式速度控制 */
                //printf("d:%d, %d\n",Reality_Velocity_1, Target_Velocity_1);

                Moto1 -= bias;                             /* 轨迹控制 */
                
                PWM_updata_Motor1(Moto1);                                               /* 赋值 */
            }
            
            
            
    //        if(t2 <= 100 && abs(Reality_Position_2-Target_Position_2) <= 3)             /* 滤除部分干扰 */
    //        {
    //            
    //            if(t2++ == 100){
    ////                Target_Velocity_2 = Rpm_Encoder_Cnt(150,13,30,10);
    ////                Target_Position_2 = Num_Encoder_Cnt(0,13,30);    
    //            }
    //            else {
    //                PWM_updata_Motor2(0);
    //            }
    //            
    //        }
    //        else
            
            {
                //Moto2 = Incremental_PID_right(Moto2, Target_Velocity_2);      /* 增量式速度控制 */
                
                Moto2 += bias;                                                  /* 轨迹控制 */
                
                PWM_updata_Motor2(Moto2);                                   /* 赋值 */
            }
        }
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

/**************************************************************************
函数功能：位置式PID控制器
入口参数：实际位置，目标位置
返回  值：电机PWM
**************************************************************************/
int Position_PID_left(int reality,int target)
{ 	
    static float Bias,pwm,Last_Bias,Integral_bias=0;
    
    Bias=target-reality;                            /* 计算偏差 */
    Integral_bias+=Bias;	                        /* 偏差累积 */
    
    if(Integral_bias> I_restrict) Integral_bias = I_restrict;   /* 积分限幅 */
    if(Integral_bias< -I_restrict) Integral_bias = -I_restrict;
    
    pwm = (Position_KP*Bias)                        /* 比例环节 */
         +(Position_KI*Integral_bias)               /* 积分环节 */
         +(Position_KD*(Bias-Last_Bias));           /* 微分环节 */
    
    Last_Bias=Bias;                                 /* 保存上次偏差 */
    return pwm;                                     /* 输出结果 */
}


int Position_PID_right(int reality,int target)
{ 	
    static float Bias,pwm,Last_Bias,Integral_bias=0;
    
    Bias=target-reality;                            /* 计算偏差 */
    Integral_bias+=Bias;	                        /* 偏差累积 */
    
    if(Integral_bias> I_restrict) Integral_bias = I_restrict;   /* 积分限幅 */
    if(Integral_bias< -I_restrict) Integral_bias = -I_restrict;
    
    pwm = (Position_KP*Bias)                        /* 比例环节 */
         +(Position_KI*Integral_bias)               /* 积分环节 */
         +(Position_KD*(Bias-Last_Bias));           /* 微分环节 */
    
    Last_Bias=Bias;                                 /* 保存上次偏差 */
    return pwm;                                     /* 输出结果 */
}

/**************************************************************************
函数功能：增量PID控制器
入口参数：实际值，目标值
返回  值：电机PWM
**************************************************************************/
int Incremental_PID_left(int reality,int target)  
{ 	
	 static float Bias,pwm,Last_bias=0,Prev_bias=0;
    
	 Bias=target-reality;                                   /* 计算偏差 */
    
	 pwm += (Incremental_KP*(Bias-Last_bias))               /* 比例环节 */
           +(Incremental_KI*Bias)                           /* 积分环节 */
           +(Incremental_KD*(Bias-2*Last_bias+Prev_bias));  /* 微分环节 */ 
    
     Prev_bias=Last_bias;                                   /* 保存上上次偏差 */
	 Last_bias=Bias;	                                    /* 保存上一次偏差 */
    
	 return pwm;                                            /* 输出结果 */
}

int Incremental_PID_right(int reality,int target)
{ 	
	 static float Bias,pwm,Last_bias=0,Prev_bias=0;
    
	 Bias=target-reality;                                   /* 计算偏差 */
    
	 pwm += (Incremental_KP*(Bias-Last_bias))               /* 比例环节 */
           +(Incremental_KI*Bias)                           /* 积分环节 */
           +(Incremental_KD*(Bias-2*Last_bias+Prev_bias));  /* 微分环节 */ 
    
     Prev_bias=Last_bias;                                   /* 保存上上次偏差 */
	 Last_bias=Bias;	                                    /* 保存上一次偏差 */
    
	 return pwm;                                            /* 输出结果 */
}

/**************************************************************************
函数功能：位置式PID控制器，巡线PID
入口参数：实际位置，目标位置
返回  值：电机PWM
**************************************************************************/
int Trace_PID(int reality,int target)
{ 	
    static float Bias,pwm,Last_Bias,Integral_bias=0;
    
    Bias=target-reality;                            /* 计算偏差 */
    Integral_bias+=Bias;	                        /* 偏差累积 */
    
    if(Integral_bias > I_restrict) Integral_bias = I_restrict;   /* 积分限幅 */
    if(Integral_bias < -I_restrict) Integral_bias = -I_restrict;
    
    pwm = (trace_KP*Bias)                        /* 比例环节 */
         +(trace_KI*Integral_bias)               /* 积分环节 */
         +(trace_KD*(Bias-Last_Bias));           /* 微分环节 */
    
    Last_Bias=Bias;                                 /* 保存上次偏差 */
    return pwm;                                     /* 输出结果 */
}


/**************************************************************************
函数功能：位置式PID控制器，角度环
入口参数：实际位置，目标位置
返回  值：电机PWM
**************************************************************************/
int Angle_PID(int reality,int target)
{ 	
    static float Bias,pwm,Last_Bias,Integral_bias=0;
    
    Bias=target-reality;                            /* 计算偏差 */
    Integral_bias+=Bias;	                        /* 偏差累积 */
    
    if(Integral_bias > I_restrict) Integral_bias = I_restrict;   /* 积分限幅 */
    if(Integral_bias < -I_restrict) Integral_bias = -I_restrict;
    
    pwm = (angle_KP*Bias)                        /* 比例环节 */
         +(angle_KI*Integral_bias)               /* 积分环节 */
         +(angle_KD*(Bias-Last_Bias));           /* 微分环节 */
    
    Last_Bias=Bias;                                 /* 保存上次偏差 */
    return pwm;                                     /* 输出结果 */
}


