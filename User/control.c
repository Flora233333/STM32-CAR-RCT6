#include "control.h"



/*************************** ������λ�û����ٶȻ�PID ******************************************/

float Position_KP=0.295,Position_KI=0,Position_KD=0.68;                 //λ�û�PID

//float Incremental_KP_1=82,Incremental_KI_1=6,Incremental_KD_1=2;        //left  �ٶȻ�PID
//float Incremental_KP_2=60,Incremental_KI_2=8,Incremental_KD_2=2;        //right �ٶȻ�PID
float Incremental_KP_1=150,Incremental_KI_1=15,Incremental_KD_1=52;        //left  �ٶȻ�PID
float Incremental_KP_2=150,Incremental_KI_2=15,Incremental_KD_2=47;        //right �ٶȻ�PID
/* left */
__IO int Target_Velocity_1 = 0, Reality_Velocity_1 = 0;   /* Ŀ���ٶȣ�ʵ���ٶ� */
__IO int Target_Position_1 = 0, Reality_Position_1 = 0;   /* Ŀ��λ�ã�ʵ��λ�� */

/* right */
__IO int Target_Velocity_2 = 0, Reality_Velocity_2 = 0;   /* Ŀ���ٶȣ�ʵ���ٶ� */
__IO int Target_Position_2 = 0, Reality_Position_2 = 0;   /* Ŀ��λ�ã�ʵ��λ�� */

/**********************************************************************************************/




/*************************** ����ͷ�켣��PID **************************************************/

//float trace_KP = 6, trace_KI = 0, trace_KD = 30;  //pwm=1500                     //����ͷѲ�߻�PID
//float trace_KP = 0.25, trace_KI = 0, trace_KD = 0.6; //ת��=100
float trace_KP = 0.45, trace_KI = 0, trace_KD = 0.8; //ת��=250

/**********************************************************************************************/



/***************************�Ҷ�ѭ����**********************************************************/


//float gray_KP = 5.2, gray_KI = 0, gray_KD = 10;  //ת��=100
//float gray_KP = 125, gray_KI = 1, gray_KD = 240; //pwm = 2000
float gray_KP = 145, gray_KI = 0, gray_KD = 360; // pwm = 1500

/**********************************************************************************************/




/*************************** �ǶȻ�PID ********************************************************/

//float angle_KP = 3, angle_KI = 0, angle_KD = 7;                 //�����ٶ��ڻ��ĽǶȻ�PID  when set_v = 100, Moto = 0
float angle_KP = 4.5, angle_KI = 0.46, angle_KD = 12;             //�����ٶ��ڻ��ĽǶȻ�PID  when set_v = 100, Moto = Target_Velocity (������v=150ʱ�о�Ҳ��)
//float angle_KP = 100, angle_KI = 0, angle_KD = 80;              //���ǶȻ�PID

//float angle_KP = 6, angle_KI = 0, angle_KD = 10;                //�����ٶ��ڻ��ĽǶȻ�PID  when set_v = 100, Moto = Target_Velocity

int Target_angle = 0;                                             //Ŀ��Ƕ�
float Angle_Dead_Space = 6.0f;                                    //�Ƕ�����

/***********************************************************************************************/  




/*************************** ���뻷PID ********************************************************/

float distant_KP = 0.5, distant_KI = 0, distant_KD = 0;          //���뻷PID

/***********************************************************************************************/ 


uint8_t start_flag = 0;
uint8_t task_finish = 0;
int64_t wait_time = 0;
uint8_t stop_flag = 0;


/********** ��ʱ�����־ **********/
uint8_t task_1 = 0;


uint8_t task_2 = 0;


uint8_t task_3 = 0;
int64_t task3_nowtime = 0;


uint8_t task_4 = 0;
int64_t task4_nowtime = 0;
/********************************/


// 10ms��ʱ�������жϻص�����
void TIM1_UP_IRQHandler() {
    
    static int Moto1 = 0, Moto2 = 0;
    
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {
        
        wait_time = Get_nowtime();

        Task_Update();
        // if (user_key_num == 1 && stop_flag >= 3) {
        //     Target_Velocity_1 = 0;
        //     Target_Velocity_2 = 0;
        // }

        // if (task_4 == 0 && user_key_num == 4 && stop_flag == 2) {
        //     Target_Velocity_1 = 0;
        //     Target_Velocity_2 = 0;
        //     task_4 = 1;
        //     task4_nowtime = Get_nowtime();
        //     Detect_Special_GrayData_handler.last_run = task4_nowtime + 4600;
        // }

        // if (task_4 == 1 && task4_nowtime + 5000 <= wait_time) {
        //     Target_Velocity_1 = Rpm_Encoder_Cnt(150,13,30,10); 
        //     Target_Velocity_2 = Rpm_Encoder_Cnt(150,13,30,10);
        //     if(stop_flag >= 3) {
        //         Target_Velocity_1 = 0;
        //         Target_Velocity_2 = 0;
        //     }
        // }
        

        if(start_flag == 1) {
            

            Reality_Velocity_1 = Read_Encoder(3);                       /* ��ȡʵ�������� */         
            Reality_Position_1 += Reality_Velocity_1;                   /* ʵ��λ�������� */
            
            Reality_Velocity_2 = -Read_Encoder(4);                       /* ��ȡʵ�������� */         
            Reality_Position_2 += Reality_Velocity_2;                   /* ʵ��λ�������� */
            
            Moto1 = Target_Velocity_1; 
            
            Moto2 = Target_Velocity_2; 
            //printf("%d\r\n",Moto1);

            /* �ҶȻ� */
            if(task_3 != 1)
            {
                int gray_bias = Gray_PID(Detect_GraySensor_Bias(), 0);

                Moto1 += gray_bias;
                Moto2 -= gray_bias;
            }

            /* ����ͷ�켣�� */
            {
                //if(first_stop_flag == 0) {
                // int capture_bias = Trace_PID(capture_data[0], middle_loc);

                // Moto1 -= capture_bias;
                // Moto2 += capture_bias;
                //}

            }

            /* �ǶȻ� */
            {
                // int angle_bias = Angle_PID(Now_Angle, Target_angle);

                // Moto1 -= angle_bias;
                // Moto2 += angle_bias;
            }

            /* λ�û� */
            {
                //Moto1 = Position_PID_left(Reality_Position_1,Target_Position_1);  
                //Moto2 = Position_PID_right(Reality_Position_2,Target_Position_2);  
            }

            /* ���뻷 */
            {
                //int distant_bias = Distant_PID(distant, 200);

                //Moto1 += distant_bias;
                //Moto2 += distant_bias;

            }

            //Moto1 = forwardfeedback(Reality_Velocity_1);
            //Moto2 = forwardfeedback(Reality_Velocity_2);

            Moto1 = PWM_restrict(Moto1,Target_Velocity_1);                    /* λ�û�����޷� */
            Moto2 = PWM_restrict(Moto2,Target_Velocity_2);                    /* λ�û�����޷� */
            //printf("%d\r\n",Moto1);

            if(task_finish == 1) {
                Moto1 = 0; 
                Moto2 = 0; 
            }


            /* Motor1�ٶȻ� */
            {
                //Moto1 = Incremental_PID_left(Moto1, Target_Velocity_1);             
                Moto1 = Incremental_PID_left(Reality_Velocity_1, Moto1);

                //Moto1 += forwardfeedback(Reality_Velocity_1) * -1;
                PWM_updata_Motor1(Moto1);                                               
            }
            
            /* Motor2�ٶȻ� */
            {
                //Moto2 = Incremental_PID_right(Moto2, Target_Velocity_2);      
                Moto2 = Incremental_PID_right(Reality_Velocity_2, Moto2);

                //Moto2 += forwardfeedback(Reality_Velocity_2);
                PWM_updata_Motor2(Moto2);                                   
            }

        }
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

void Mode_Select(void) {
    uint8_t send_data = 0;
    // PID��������Ĭ��ֵ��û������ѡ����д���ʹ�����Ĭ��ֵ��
    if(start_flag == 0) {

        switch (user_key_num)
        {
        case 1:
            TIM3 -> CNT = 0; //���ϴ������ۼ�
            TIM4 -> CNT = 0;

            Target_Velocity_1 = Rpm_Encoder_Cnt(Cal_Speed2Rpm(0.3, 6.5), 13, 30, 10);
            Target_Velocity_2 = Rpm_Encoder_Cnt(Cal_Speed2Rpm(0.3, 6.5), 13, 30, 10);

            trace_KP = 0.25; trace_KI = 0; trace_KD = 0.6;
            gray_KP = 3.6, gray_KI = 0, gray_KD = 6.9;  //ת��=100

            send_data = 1;
            start_flag = 1;
            break;

        case 2:
            TIM3 -> CNT = 0; //���ϴ������ۼ�
            TIM4 -> CNT = 0;

            Target_Velocity_1 = Rpm_Encoder_Cnt(Cal_Speed2Rpm(0.5, 6.5), 13, 30, 10);
            Target_Velocity_2 = Rpm_Encoder_Cnt(Cal_Speed2Rpm(0.5, 6.5), 13, 30, 10);

            trace_KP = 0.25; trace_KI = 0; trace_KD = 0.6;
            gray_KP = 6.3 , gray_KI = 0, gray_KD = 13.7;  //ת��=100
            
            send_data = 2;
            start_flag = 1;
            break;

        case 3:
            TIM3 -> CNT = 0; //���ϴ������ۼ�
            TIM4 -> CNT = 0;

            Target_Velocity_1 = Rpm_Encoder_Cnt(Cal_Speed2Rpm(0.5, 6.5), 13, 30, 10);
            Target_Velocity_2 = Rpm_Encoder_Cnt(Cal_Speed2Rpm(0.5, 6.5), 13, 30, 10);

            gray_KP = 6.3 , gray_KI = 0, gray_KD = 13.7;  //ת��=100

            send_data = 3;
            start_flag = 1;
            break;

        case 4:
            TIM3 -> CNT = 0; //���ϴ������ۼ�
            TIM4 -> CNT = 0;

            Target_Velocity_1 = Rpm_Encoder_Cnt(Cal_Speed2Rpm(0.6, 6.5), 13, 30, 10);
            Target_Velocity_2 = Rpm_Encoder_Cnt(Cal_Speed2Rpm(0.6, 6.5), 13, 30, 10);

            gray_KP = 9.3 , gray_KI = 0, gray_KD = 20.5;  //ת��=100gray_KP = 8.2 , gray_KI = 0, gray_KD = 17.7;

            send_data = 4;
            start_flag = 1;
            break;
        }
        BLU_SendSingleData(send_data);
    }
}

// ֻ�ǹ�����Ŀ�����񣬲��ǵ������񣬿������Ϊ״̬��
void Task_Update(void) {

    //static uint8_t add_speed = 35; // = 10 ��ʵ�ֱ�� task_3���ٶ�

    if(task_finish == 0) {

        switch (user_key_num)
        {
        case 1:
            if(task_1 == 0 && stop_flag == 2) {
                task_1 = 1;
                Target_Velocity_1 = 0;
                Target_Velocity_2 = 0;
                task_finish = 1;
                BLU_SendSingleData(0);
            }
            break;

        case 2:
            if(task_2 == 0 && stop_flag == 3) {
                task_2 = 1;
                Target_Velocity_1 = 0;
                Target_Velocity_2 = 0;
                task_finish = 1;
                BLU_SendSingleData(0);
            }
            break;

        case 3:
            if(task_3 == 0 && passby_cross_num == 3) { //������
                task_3 = 1;
                //Target_Velocity_1 += 3;
                Target_Velocity_2 += 38;
                task3_nowtime = Get_nowtime() + 500;
                BLU_SendSingleData(5);
            }

            if (task_3 == 1 && Get_nowtime() > task3_nowtime) { //���ڻ�����
                task_3 = 2;
                Target_Velocity_1 += 0; // ��ʱ���ڻ���˫�����������
                Target_Velocity_2 = Target_Velocity_1;
                gray_KP = 10.8 , gray_KI = 0, gray_KD = 22.7;  //ת��=100
                task3_nowtime = Get_nowtime() + 3000;
            }

            if(task_3 == 2 && Get_nowtime() > task3_nowtime) { //���������
                task_3 = 3;
                gray_KP = 6.3 , gray_KI = 0, gray_KD = 13.7;  //ת��=100s
                Target_Velocity_1 -= 0;
                Target_Velocity_2 -= 0;
            }

            if (task_3 == 3 && stop_flag == 4) { //ͣ��
                task_3 = 4;
                Target_Velocity_1 = 0;
                Target_Velocity_2 = 0;
                task_finish = 1;
                BLU_SendSingleData(0);
            }

            break;

        case 4:
            if(task_4 == 0 && stop_flag == 2) {
                task_4 = 1;
                Target_Velocity_1 = 0;
                Target_Velocity_2 = 0;
                BLU_SendSingleData(0);
                // TIM3 -> CNT = 0; //���ϴ������ۼ�
                // TIM4 -> CNT = 0;
                task4_nowtime = Get_nowtime() + 5000;
                Detect_Special_GrayData_handler.last_run = Get_nowtime() + 5500; // ԭ�ȣ�Detect_Special_GrayData_handler.last_run = task4_nowtime + 5500;
            }

            if(task_4 == 1 && Get_nowtime() > task4_nowtime) {
                task_4 = 2;
                Target_Velocity_1 = Rpm_Encoder_Cnt(Cal_Speed2Rpm(0.6, 6.5), 13, 30, 10);
                Target_Velocity_2 = Rpm_Encoder_Cnt(Cal_Speed2Rpm(0.6, 6.5), 13, 30, 10);
                BLU_SendSingleData(4);
            }

            if(task_4 == 2 && stop_flag == 3) {
                task_4 = 3;
                Target_Velocity_1 = 0;
                Target_Velocity_2 = 0;
                task_finish = 1;
                BLU_SendSingleData(0);
            }

            break;
        }
    }
}


int forwardfeedback(float in) {
    static float last_in = 0, Ta = 0.3, Tb = 0.1;
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
int Gray_PID(float reality,int target)
{ 	
    static float Bias,pwm,Last_Bias,Integral_bias=0;
    
    Bias=(float)target-reality;                            /* ����ƫ�� */
    
    
    // if(Integral_bias> I_restrict) Integral_bias = I_restrict;   /* �����޷� */
    // if(Integral_bias< -I_restrict) Integral_bias = -I_restrict;
    
    pwm = (gray_KP*Bias)                        /* �������� */
         +(gray_KI*Integral_bias)               /* ���ֻ��� */
         +(gray_KD*(Bias-Last_Bias));           /* ΢�ֻ��� */
    
    Last_Bias=Bias;                                 /* �����ϴ�ƫ�� */
    
    return (int)pwm;                                     /* ������ */
}

/**************************************************************************
�������ܣ�λ��ʽPID�����������뻷
��ڲ�����ʵ��λ�ã�Ŀ��λ��
����  ֵ�����PWM
**************************************************************************/
int Distant_PID(int reality,int target)
{ 	
    static float Bias,pwm,Last_Bias,Integral_bias=0;

    if(reality > 1500 || reality < 50) {
        return 0;
    }
    
    Bias=target-reality;                            /* ����ƫ�� */
    Integral_bias+=Bias;	                        /* ƫ���ۻ� */
    
    if(Integral_bias > I_restrict) Integral_bias = I_restrict;   /* �����޷� */
    if(Integral_bias < -I_restrict) Integral_bias = -I_restrict;
    
    pwm = (distant_KP*Bias)                        /* �������� */
         +(distant_KI*Integral_bias)               /* ���ֻ��� */
         +(distant_KD*(Bias-Last_Bias));           /* ΢�ֻ��� */
    
    Last_Bias=Bias;                                 /* �����ϴ�ƫ�� */

    return pwm;                                     /* ������ */
}

