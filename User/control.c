#include "control.h"



/*************************** ������λ�û����ٶȻ�PID ******************************************/

float Position_KP=0.295,Position_KI=0,Position_KD=0.68;                 //λ�û�PID

float Incremental_KP_1=82,Incremental_KI_1=6,Incremental_KD_1=2;        //left  �ٶȻ�PID
float Incremental_KP_2=60,Incremental_KI_2=8,Incremental_KD_2=2;        //right �ٶȻ�PID

/* left */
int Target_Velocity_1 = 0, Reality_Velocity_1 = 0;   /* Ŀ���ٶȣ�ʵ���ٶ� */
int Target_Position_1 = 0, Reality_Position_1 = 0;   /* Ŀ��λ�ã�ʵ��λ�� */

/* right */
int Target_Velocity_2 = 0, Reality_Velocity_2 = 0;   /* Ŀ���ٶȣ�ʵ���ٶ� */
int Target_Position_2 = 0, Reality_Position_2 = 0;   /* Ŀ��λ�ã�ʵ��λ�� */

/**********************************************************************************************/




/*************************** ����ͷ�켣��PID **************************************************/

float trace_KP = 10, trace_KI = 0, trace_KD = 30;                       //����ͷѲ�߻�PID

/**********************************************************************************************/



/***************************�Ҷ�ѭ����**********************************************************/


float gray_KP = 0, gray_KI = 0, gray_KD = 0;  



/**********************************************************************************************/




/*************************** �ǶȻ�PID ********************************************************/

//float angle_KP = 3, angle_KI = 0, angle_KD = 7;                 //�����ٶ��ڻ��ĽǶȻ�PID  when set_v = 100, Moto = 0
float angle_KP = 4.5, angle_KI = 0.46, angle_KD = 12;             //�����ٶ��ڻ��ĽǶȻ�PID  when set_v = 100, Moto = Target_Velocity (������v=150ʱ�о�Ҳ��)
//float angle_KP = 100, angle_KI = 0, angle_KD = 80;              //���ǶȻ�PID

//float angle_KP = 6, angle_KI = 0, angle_KD = 10;                //�����ٶ��ڻ��ĽǶȻ�PID  when set_v = 100, Moto = Target_Velocity

int Target_angle = 0;                                             //Ŀ��Ƕ�
float Angle_Dead_Space = 6.0f;                                    //�Ƕ�����

/***********************************************************************************************/                                


uint8_t start_flag = 0;

int task1_time = 0, task2_time = 0;

// 10ms��ʱ�������жϻص�����
void TIM1_UP_IRQHandler() {
    
    static int Moto1 = 0, Moto2 = 0;
    
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {
        
        if(start_flag == 1) {
            //int capture_bias = 0;
            int angle_bias = 0;
            
            task1_time = global_time;

            Reality_Velocity_1 = Read_Encoder(3);                       /* ��ȡʵ�������� */         
            Reality_Position_1 += Reality_Velocity_1;                   /* ʵ��λ�������� */
            
            Reality_Velocity_2 = -Read_Encoder(4);                       /* ��ȡʵ�������� */         
            Reality_Position_2 += Reality_Velocity_2;                   /* ʵ��λ�������� */

            //printf("d:%d, %d, %d\n",Reality_Velocity_1, Reality_Velocity_2, Target_Velocity_1);
            
            Moto1 = Target_Velocity_1; 
            
            Moto2 = Target_Velocity_2; 


            /* ����ͷ�켣�� */
            {
                //capture_bias = Trace_PID(rxarr[0], middle_loc);

                //Moto1 -= angle_bias;
                //Moto2 += angle_bias;
            }

            /* �ǶȻ� */
            {
                angle_bias = Angle_PID(Now_Angle, Target_angle);

                Moto1 -= angle_bias;
                Moto2 += angle_bias;
            }

            /* λ�û� */
            {
                //Moto1 = Position_PID_left(Reality_Position_1,Target_Position_1);  
                //Moto2 = Position_PID_right(Reality_Position_2,Target_Position_2);  
            }
            
            Moto1 = PWM_restrict(Moto1,Target_Velocity_1);                    /* λ�û�����޷� */
            Moto2 = PWM_restrict(Moto2,Target_Velocity_2);                    /* λ�û�����޷� */
            
            /* Motor1�ٶȻ� */
            {
                //Moto1 = Incremental_PID_left(Moto1, Target_Velocity_1);             
                Moto1 = Incremental_PID_left(Reality_Velocity_1, Moto1);

                
                PWM_updata_Motor1(Moto1);                                               
            }
            
            /* Motor2�ٶȻ� */
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
    out = Ta * (in - last_in) / 0.01 + Tb * in; // ��ʱ���󵼣�0.01Ϊ����ʱ�� PDǰ��(Ta*s + Tb)
    // ��������Ϊ��Ta, Tb����1(��Ϊ��֪����ô��)������ֱ�������
    last_in = in;
    return out;
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

	pwm += (Incremental_KP_1*(Bias-Last_bias))               /* �������� */
          +(Incremental_KI_1*Bias)                           /* ���ֻ��� */
          +(Incremental_KD_1*(Bias-2*Last_bias+Prev_bias));  /* ΢�ֻ��� */ 

    Prev_bias=Last_bias;                                   /* �������ϴ�ƫ�� */
	Last_bias=Bias;	                                    /* ������һ��ƫ�� */

	return pwm;                                            /* ������ */
}

int Incremental_PID_right(int reality,int target)
{ 	
	static float Bias,pwm,Last_bias=0,Prev_bias=0;

	Bias=target-reality;                                   /* ����ƫ�� */

	pwm += (Incremental_KP_2*(Bias-Last_bias))               /* �������� */
          +(Incremental_KI_2*Bias)                           /* ���ֻ��� */
          +(Incremental_KD_2*(Bias-2*Last_bias+Prev_bias));  /* ΢�ֻ��� */ 

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
�������ܣ�λ��ʽPID���������ǶȻ������ַ���
��ڲ�����ʵ��λ�ã�Ŀ��λ��
����  ֵ�����PWM
**************************************************************************/
int Angle_PID(int reality,int target)
{ 	
    static float Bias, pwm, Last_Bias, Integral_bias = 0;
    
    Bias = target - reality;                            /* ����ƫ�� */

    if(Bias < 0.05 && Bias > -0.05) 
        Bias = 0;
    
    if(Bias < Angle_Dead_Space && Bias > -Angle_Dead_Space) /* ���ַ��� */
        Integral_bias += Bias;	                        
    else
        Integral_bias = 0;
    
    if(Integral_bias >  I_restrict) Integral_bias =  I_restrict;   /* �����޷� */
    if(Integral_bias < -I_restrict) Integral_bias = -I_restrict;
    
    pwm = (angle_KP*Bias)                        /* �������� */
         +(angle_KI*Integral_bias)               /* ���ֻ��� */
         +(angle_KD*(Bias-Last_Bias));           /* ΢�ֻ��� */
    
    Last_Bias = Bias;                               /* �����ϴ�ƫ�� */
    return pwm;                                     /* ������ */
}



/**************************************************************************
�������ܣ�λ��ʽPID���������ҶȻ�
��ڲ�����ʵ��λ�ã�Ŀ��λ��
����  ֵ�����PWM
**************************************************************************/
int Gray_PID(int reality,int target)
{ 	
    static float Bias,pwm,Last_Bias,Integral_bias=0;
    
    Bias=target-reality;                            /* ����ƫ�� */
    Integral_bias+=Bias;	                        /* ƫ���ۻ� */
    
    if(Integral_bias> I_restrict) Integral_bias = I_restrict;   /* �����޷� */
    if(Integral_bias< -I_restrict) Integral_bias = -I_restrict;
    
    pwm = (gray_KP*Bias)                        /* �������� */
         +(gray_KI*Integral_bias)               /* ���ֻ��� */
         +(gray_KD*(Bias-Last_Bias));           /* ΢�ֻ��� */
    
    Last_Bias=Bias;                                 /* �����ϴ�ƫ�� */
    return pwm;                                     /* ������ */
}
