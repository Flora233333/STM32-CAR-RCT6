#include "control.h"



/*************************** 编码器位置环和速度环PID ******************************************/

float Position_KP=0.295,Position_KI=0,Position_KD=0.68;                 //位置环PID

float Incremental_KP_1=82,Incremental_KI_1=6,Incremental_KD_1=2;        //left  速度环PID
float Incremental_KP_2=60,Incremental_KI_2=8,Incremental_KD_2=2;        //right 速度环PID

/* left */
int Target_Velocity_1 = 0, Reality_Velocity_1 = 0;   /* 目标速度，实际速度 */
int Target_Position_1 = 0, Reality_Position_1 = 0;   /* 目标位置，实际位置 */

/* right */
int Target_Velocity_2 = 0, Reality_Velocity_2 = 0;   /* 目标速度，实际速度 */
int Target_Position_2 = 0, Reality_Position_2 = 0;   /* 目标位置，实际位置 */

/**********************************************************************************************/




/*************************** 摄像头轨迹环PID **************************************************/

float trace_KP = 10, trace_KI = 0, trace_KD = 30;                       //摄像头巡线环PID

/**********************************************************************************************/



/***************************灰度循迹环**********************************************************/


float gray_KP = 0, gray_KI = 0, gray_KD = 0;  



/**********************************************************************************************/




/*************************** 角度环PID ********************************************************/

//float angle_KP = 3, angle_KI = 0, angle_KD = 7;                 //叠加速度内环的角度环PID  when set_v = 100, Moto = 0
float angle_KP = 4.5, angle_KI = 0.46, angle_KD = 12;             //叠加速度内环的角度环PID  when set_v = 100, Moto = Target_Velocity (经测试v=150时感觉也行)
//float angle_KP = 100, angle_KI = 0, angle_KD = 80;              //纯角度环PID

//float angle_KP = 6, angle_KI = 0, angle_KD = 10;                //叠加速度内环的角度环PID  when set_v = 100, Moto = Target_Velocity

int Target_angle = 0;                                             //目标角度
float Angle_Dead_Space = 6.0f;                                    //角度死区

/***********************************************************************************************/                                


uint8_t start_flag = 0;

int task1_time = 0, task2_time = 0;

// 10ms定时器更新中断回调函数
void TIM1_UP_IRQHandler() {
    
    static int Moto1 = 0, Moto2 = 0;
    
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {
        
        if(start_flag == 1) {
            //int capture_bias = 0;
            int angle_bias = 0;
            
            task1_time = global_time;

            Reality_Velocity_1 = Read_Encoder(3);                       /* 获取实际脉冲数 */         
            Reality_Position_1 += Reality_Velocity_1;                   /* 实际位置脉冲数 */
            
            Reality_Velocity_2 = -Read_Encoder(4);                       /* 获取实际脉冲数 */         
            Reality_Position_2 += Reality_Velocity_2;                   /* 实际位置脉冲数 */

            //printf("d:%d, %d, %d\n",Reality_Velocity_1, Reality_Velocity_2, Target_Velocity_1);
            
            Moto1 = Target_Velocity_1; 
            
            Moto2 = Target_Velocity_2; 


            /* 摄像头轨迹环 */
            {
                //capture_bias = Trace_PID(rxarr[0], middle_loc);

                //Moto1 -= angle_bias;
                //Moto2 += angle_bias;
            }

            /* 角度环 */
            {
                angle_bias = Angle_PID(Now_Angle, Target_angle);

                Moto1 -= angle_bias;
                Moto2 += angle_bias;
            }

            /* 位置环 */
            {
                //Moto1 = Position_PID_left(Reality_Position_1,Target_Position_1);  
                //Moto2 = Position_PID_right(Reality_Position_2,Target_Position_2);  
            }
            
            Moto1 = PWM_restrict(Moto1,Target_Velocity_1);                    /* 位置环输出限幅 */
            Moto2 = PWM_restrict(Moto2,Target_Velocity_2);                    /* 位置环输出限幅 */
            
            /* Motor1速度环 */
            {
                //Moto1 = Incremental_PID_left(Moto1, Target_Velocity_1);             
                Moto1 = Incremental_PID_left(Reality_Velocity_1, Moto1);

                
                PWM_updata_Motor1(Moto1);                                               
            }
            
            /* Motor2速度环 */
            {
                //Moto2 = Incremental_PID_right(Moto2, Target_Velocity_2);      
                Moto2 = Incremental_PID_right(Reality_Velocity_2, Moto2);

                
                PWM_updata_Motor2(Moto2);                                   
            }

        }
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}


int forwardfeedback(float in) {
    static float last_in = 0, Ta = 0.3, Tb = 0.3;
    float out = 0;
    out = Ta * (in - last_in) / 0.01 + Tb * in; // 对时间求导，0.01为采样时间 PD前馈(Ta*s + Tb)
    // 这里是因为我Ta, Tb都是1(因为不知道怎么调)，所以直接相加了
    last_in = in;
    return out;
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

	pwm += (Incremental_KP_1*(Bias-Last_bias))               /* 比例环节 */
          +(Incremental_KI_1*Bias)                           /* 积分环节 */
          +(Incremental_KD_1*(Bias-2*Last_bias+Prev_bias));  /* 微分环节 */ 

    Prev_bias=Last_bias;                                   /* 保存上上次偏差 */
	Last_bias=Bias;	                                    /* 保存上一次偏差 */

	return pwm;                                            /* 输出结果 */
}

int Incremental_PID_right(int reality,int target)
{ 	
	static float Bias,pwm,Last_bias=0,Prev_bias=0;

	Bias=target-reality;                                   /* 计算偏差 */

	pwm += (Incremental_KP_2*(Bias-Last_bias))               /* 比例环节 */
          +(Incremental_KI_2*Bias)                           /* 积分环节 */
          +(Incremental_KD_2*(Bias-2*Last_bias+Prev_bias));  /* 微分环节 */ 

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
函数功能：位置式PID控制器，角度环，积分分离
入口参数：实际位置，目标位置
返回  值：电机PWM
**************************************************************************/
int Angle_PID(int reality,int target)
{ 	
    static float Bias, pwm, Last_Bias, Integral_bias = 0;
    
    Bias = target - reality;                            /* 计算偏差 */

    if(Bias < 0.05 && Bias > -0.05) 
        Bias = 0;
    
    if(Bias < Angle_Dead_Space && Bias > -Angle_Dead_Space) /* 积分分离 */
        Integral_bias += Bias;	                        
    else
        Integral_bias = 0;
    
    if(Integral_bias >  I_restrict) Integral_bias =  I_restrict;   /* 积分限幅 */
    if(Integral_bias < -I_restrict) Integral_bias = -I_restrict;
    
    pwm = (angle_KP*Bias)                        /* 比例环节 */
         +(angle_KI*Integral_bias)               /* 积分环节 */
         +(angle_KD*(Bias-Last_Bias));           /* 微分环节 */
    
    Last_Bias = Bias;                               /* 保存上次偏差 */
    return pwm;                                     /* 输出结果 */
}



/**************************************************************************
函数功能：位置式PID控制器，灰度环
入口参数：实际位置，目标位置
返回  值：电机PWM
**************************************************************************/
int Gray_PID(int reality,int target)
{ 	
    static float Bias,pwm,Last_Bias,Integral_bias=0;
    
    Bias=target-reality;                            /* 计算偏差 */
    Integral_bias+=Bias;	                        /* 偏差累积 */
    
    if(Integral_bias> I_restrict) Integral_bias = I_restrict;   /* 积分限幅 */
    if(Integral_bias< -I_restrict) Integral_bias = -I_restrict;
    
    pwm = (gray_KP*Bias)                        /* 比例环节 */
         +(gray_KI*Integral_bias)               /* 积分环节 */
         +(gray_KD*(Bias-Last_Bias));           /* 微分环节 */
    
    Last_Bias=Bias;                                 /* 保存上次偏差 */
    return pwm;                                     /* 输出结果 */
}
