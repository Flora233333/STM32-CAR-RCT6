#include "control.h"


float Position_KP=0.295,Position_KI=0,Position_KD=0.68;         //λ�û�PID
float Incremental_KP=80,Incremental_KI=10,Incremental_KD=0;     //�ٶȻ�PID
float trace_KP = 10, trace_KI = 0, trace_KD = 30;               //Ѳ�߻�PID
float angle_KP = 0, angle_KI = 0, angle_KD = 0;                 //�ǶȻ�PID 

// dead=0 float Position_KP=0.295,Position_KI=0,Position_KD=0.68;float Incremental_KP=80,Incremental_KI=10,Incremental_KD=0;   /* ����ʽPIDϵ�� */

int Target_Velocity_1 = 0, Reality_Velocity_1 = 0;   /* Ŀ���ٶȣ�ʵ���ٶ� */
int Target_Position_1 = 0, Reality_Position_1 = 0;   /* Ŀ��λ�ã�ʵ��λ�� */

int Target_Velocity_2 = 0, Reality_Velocity_2 = 0;   /* Ŀ���ٶȣ�ʵ���ٶ� */
int Target_Position_2 = 0, Reality_Position_2 = 0;   /* Ŀ��λ�ã�ʵ��λ�� */

int Target_angle = 0;                                /* Ŀ��Ƕ� */

uint8_t start_flag = 0;

int task1_time = 0, task2_time = 0;

// 10ms��ʱ�������жϻص�����
void TIM1_UP_IRQHandler() {
    
    static int Moto1 = 0, Moto2 = 0;
    
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {
        global_time++;
        if(start_flag == 1) {
            int bias = 0;
            
            task1_time = global_time;

            Reality_Velocity_1 = Read_Encoder(3);                       /* ��ȡʵ�������� */         
            Reality_Position_1 += Reality_Velocity_1;                   /* ʵ��λ�������� */
            
            Reality_Velocity_2 = -Read_Encoder(4);                       /* ��ȡʵ�������� */         
            Reality_Position_2 += Reality_Velocity_2;                   /* ʵ��λ�������� */
            
            Moto1 = -1500;
            
            Moto2 = -1500;
            
            
            bias = Trace_PID(rxarr[0], middle_loc);
            
            
            //Moto1 = Position_PID_left(Reality_Position_1,Target_Position_1);  /* λ��ʽλ�ÿ��� */    
            //Moto2 = Position_PID_right(Reality_Position_2,Target_Position_2);  /* λ��ʽλ�ÿ��� */
            
            //Moto1 = PWM_restrict(Moto1,Target_Velocity_1);                    /* λ�û�����޷� */
            //Moto2 = PWM_restrict(Moto2,Target_Velocity_2);                    /* λ�û�����޷� */
            
    //        if(t1 <= 100 && abs(Reality_Position_1-Target_Position_1) <= 3)             /* �˳����ָ��� */
    //        {
    //            //PWM_updata_Motor1(0);
    //            if(t1++ == 100){
    ////                Target_Velocity_1 = Rpm_Encoder_Cnt(150,13,30,10);   /* ��ת��ת��Ϊ10ms����������Ŀ���ٶ� */
    ////                Target_Position_1 = Num_Encoder_Cnt(0,13,30);      /* ��Ȧ��ת��ΪĿ����������Ŀ��λ�� */
    //            }
    //            else {
    //                PWM_updata_Motor1(0);
    //            }
    //        }
    //        else


            {
                //Moto1 = Incremental_PID_left(Moto1, Target_Velocity_1);             /* ����ʽ�ٶȿ��� */
                //printf("d:%d, %d\n",Reality_Velocity_1, Target_Velocity_1);

                Moto1 -= bias;                             /* �켣���� */
                
                PWM_updata_Motor1(Moto1);                                               /* ��ֵ */
            }
            
            
            
    //        if(t2 <= 100 && abs(Reality_Position_2-Target_Position_2) <= 3)             /* �˳����ָ��� */
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
                //Moto2 = Incremental_PID_right(Moto2, Target_Velocity_2);      /* ����ʽ�ٶȿ��� */
                
                Moto2 += bias;                                                  /* �켣���� */
                
                PWM_updata_Motor2(Moto2);                                   /* ��ֵ */
            }
        }
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ�����ʵ��λ�ã�Ŀ��λ��
����  ֵ�����PWM
**************************************************************************/
int Position_PID_left(int reality,int target)
{ 	
    static float Bias,pwm,Last_Bias,Integral_bias=0;
    
    Bias=target-reality;                            /* ����ƫ�� */
    Integral_bias+=Bias;	                        /* ƫ���ۻ� */
    
    if(Integral_bias> I_restrict) Integral_bias = I_restrict;   /* �����޷� */
    if(Integral_bias< -I_restrict) Integral_bias = -I_restrict;
    
    pwm = (Position_KP*Bias)                        /* �������� */
         +(Position_KI*Integral_bias)               /* ���ֻ��� */
         +(Position_KD*(Bias-Last_Bias));           /* ΢�ֻ��� */
    
    Last_Bias=Bias;                                 /* �����ϴ�ƫ�� */
    return pwm;                                     /* ������ */
}


int Position_PID_right(int reality,int target)
{ 	
    static float Bias,pwm,Last_Bias,Integral_bias=0;
    
    Bias=target-reality;                            /* ����ƫ�� */
    Integral_bias+=Bias;	                        /* ƫ���ۻ� */
    
    if(Integral_bias> I_restrict) Integral_bias = I_restrict;   /* �����޷� */
    if(Integral_bias< -I_restrict) Integral_bias = -I_restrict;
    
    pwm = (Position_KP*Bias)                        /* �������� */
         +(Position_KI*Integral_bias)               /* ���ֻ��� */
         +(Position_KD*(Bias-Last_Bias));           /* ΢�ֻ��� */
    
    Last_Bias=Bias;                                 /* �����ϴ�ƫ�� */
    return pwm;                                     /* ������ */
}

/**************************************************************************
�������ܣ�����PID������
��ڲ�����ʵ��ֵ��Ŀ��ֵ
����  ֵ�����PWM
**************************************************************************/
int Incremental_PID_left(int reality,int target)  
{ 	
	 static float Bias,pwm,Last_bias=0,Prev_bias=0;
    
	 Bias=target-reality;                                   /* ����ƫ�� */
    
	 pwm += (Incremental_KP*(Bias-Last_bias))               /* �������� */
           +(Incremental_KI*Bias)                           /* ���ֻ��� */
           +(Incremental_KD*(Bias-2*Last_bias+Prev_bias));  /* ΢�ֻ��� */ 
    
     Prev_bias=Last_bias;                                   /* �������ϴ�ƫ�� */
	 Last_bias=Bias;	                                    /* ������һ��ƫ�� */
    
	 return pwm;                                            /* ������ */
}

int Incremental_PID_right(int reality,int target)
{ 	
	 static float Bias,pwm,Last_bias=0,Prev_bias=0;
    
	 Bias=target-reality;                                   /* ����ƫ�� */
    
	 pwm += (Incremental_KP*(Bias-Last_bias))               /* �������� */
           +(Incremental_KI*Bias)                           /* ���ֻ��� */
           +(Incremental_KD*(Bias-2*Last_bias+Prev_bias));  /* ΢�ֻ��� */ 
    
     Prev_bias=Last_bias;                                   /* �������ϴ�ƫ�� */
	 Last_bias=Bias;	                                    /* ������һ��ƫ�� */
    
	 return pwm;                                            /* ������ */
}

/**************************************************************************
�������ܣ�λ��ʽPID��������Ѳ��PID
��ڲ�����ʵ��λ�ã�Ŀ��λ��
����  ֵ�����PWM
**************************************************************************/
int Trace_PID(int reality,int target)
{ 	
    static float Bias,pwm,Last_Bias,Integral_bias=0;
    
    Bias=target-reality;                            /* ����ƫ�� */
    Integral_bias+=Bias;	                        /* ƫ���ۻ� */
    
    if(Integral_bias > I_restrict) Integral_bias = I_restrict;   /* �����޷� */
    if(Integral_bias < -I_restrict) Integral_bias = -I_restrict;
    
    pwm = (trace_KP*Bias)                        /* �������� */
         +(trace_KI*Integral_bias)               /* ���ֻ��� */
         +(trace_KD*(Bias-Last_Bias));           /* ΢�ֻ��� */
    
    Last_Bias=Bias;                                 /* �����ϴ�ƫ�� */
    return pwm;                                     /* ������ */
}


/**************************************************************************
�������ܣ�λ��ʽPID���������ǶȻ�
��ڲ�����ʵ��λ�ã�Ŀ��λ��
����  ֵ�����PWM
**************************************************************************/
int Angle_PID(int reality,int target)
{ 	
    static float Bias,pwm,Last_Bias,Integral_bias=0;
    
    Bias=target-reality;                            /* ����ƫ�� */
    Integral_bias+=Bias;	                        /* ƫ���ۻ� */
    
    if(Integral_bias > I_restrict) Integral_bias = I_restrict;   /* �����޷� */
    if(Integral_bias < -I_restrict) Integral_bias = -I_restrict;
    
    pwm = (angle_KP*Bias)                        /* �������� */
         +(angle_KI*Integral_bias)               /* ���ֻ��� */
         +(angle_KD*(Bias-Last_Bias));           /* ΢�ֻ��� */
    
    Last_Bias=Bias;                                 /* �����ϴ�ƫ�� */
    return pwm;                                     /* ������ */
}


