#include "control.h"


float Position_KP=0.295,Position_KI=0,Position_KD=0.68;        //位置环PID
float Incremental_KP=80,Incremental_KI=10,Incremental_KD=0;   //速度环PID
float trace_KP = 0, trace_KD = 0;


// dead=0 float Position_KP=0.295,Position_KI=0,Position_KD=0.68;float Incremental_KP=80,Incremental_KI=10,Incremental_KD=0;   /* 增量式PID系数 */

int Target_Velocity_1 = 0, Reality_Velocity_1 = 0;   /* 目标速度，实际速度 */
int Target_Position_1 = 0, Reality_Position_1 = 0;   /* 目标位置，实际位置 */

int Target_Velocity_2 = 0, Reality_Velocity_2 = 0;   /* 目标速度，实际速度 */
int Target_Position_2 = 0, Reality_Position_2 = 0;   /* 目标位置，实际位置 */

int t1 = 0, t2 = 0;

// 10ms定时器更新中断回调函数
void TIM1_UP_IRQHandler() {
    
    static int Moto1 = 0, Moto2 = 0;
    
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {
		Reality_Velocity_1 = Read_Encoder(3);                       /* 获取实际脉冲数 */         
        Reality_Position_1 += Reality_Velocity_1;                   /* 实际位置脉冲数 */
        
        Reality_Velocity_2 = -Read_Encoder(4);                       /* 获取实际脉冲数 */         
        Reality_Position_2 += Reality_Velocity_2;                   /* 实际位置脉冲数 */
        
        Moto1 = Reality_Velocity_1;
        
        Moto2 = Reality_Velocity_2;
        
        
        
        
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
            Moto1 = Incremental_PID_left(Moto1, Target_Velocity_1);      /* 增量式速度控制 */
            printf("d:%d, %d\n",Reality_Velocity_1, Target_Velocity_1);
            
            PWM_updata_Motor1(Moto1);                                      /* 赋值 */
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
            Moto2 = Incremental_PID_right(Moto2, Target_Velocity_2);      /* 增量式速度控制 */
            
            
            
            PWM_updata_Motor2(Moto2);                                /* 赋值 */
        }
        
        
        
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
//void Set_Pwm(int moto)
//{
//    if(moto>0)      /* 正转 */
//    {
//        AIN1(1);
//        AIN2(0);
//    }
//    else           /* 反转 */
//    {
//        AIN1(0);
//        AIN2(1);
//    }
//    
//    if(moto)        /* 控制器有输出 */        
//    {  
//        moto = abs(moto) + Dead_Voltage;/* 取绝对值 + 死区电压*/
//        PWM = Xianfu(moto,Amplitude);   /* 限幅 */
//    }
//    else            /* 无输出，直接关闭驱动 */       
//    {
//        PWM = 0;
//    }
//    
//   
//}

/**************************************************************************
函数功能：电机停止
入口参数：无
返回  值：无
**************************************************************************/
//void Moto_Stop(void)
//{
//    PWM = 0;
//    AIN1(0);
//    AIN2(0);     
//}

/**************************************************************************
函数功能：限幅 
入口参数：电机PWM值
返回  值：限制后的值
**************************************************************************/
//int Xianfu(int data,int max)
//{	
//    if(data<-max) data=-max;	
//    if(data> max) data= max;	
//    return data;
//}

/**************************************************************************
函数功能：位置式PID控制器
入口参数：实际位置，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,...,k;
pwm代表输出
**************************************************************************/
int Position_PID_left(int reality,int target)
{ 	
    static float Bias,pwm,Last_Bias,Integral_bias=0;
    
    Bias=target-reality;                            /* 计算偏差 */
    Integral_bias+=Bias;	                        /* 偏差累积 */
    
    if(Integral_bias> 5000) Integral_bias = 5000;   /* 积分限幅 */
    if(Integral_bias<-5000) Integral_bias =-5000;
    
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
    
    if(Integral_bias> 5000) Integral_bias = 5000;   /* 积分限幅 */
    if(Integral_bias<-5000) Integral_bias =-5000;
    
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
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
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
