#include "bsp.h"                  // Device header


char str1[100];
char str2[100];

int main() {
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //NVIC中断分组
    
    delay_init(72);                 //=====延时初始化
    LED_Init();
    USART_Config();
    OLED_Init();
    Motor_Init();
    PWM_Init();
	Encoder_Init();
    
//    int ret = atk_ms901m_init(115200);
//    if (ret != 0)
//    {
//        OLED_ShowString(1, 1,"IMU err");
//        while (1)
//        {
//            delay_ms(200);
//        }
//    }   



    Target_Velocity_1 = Rpm_Encoder_Cnt(100,13,30,10); 
    
    //Target_Position_1 = Num_Encoder_Cnt(10,13,30);      
    
    Target_Velocity_2 = Rpm_Encoder_Cnt(100,13,30,10); 
    
    //Target_Position_2 = Num_Encoder_Cnt(10,13,30); 
    
    Timer1_InternalClock_Init();
    

    
    
    //int cnt = 0;
    //int flag = 0;
    //int ram = 0;
    //printf("1");
    
    
	while(1) {
       /*
        TIM2->CCR3 = cnt;
        TIM2->CCR4 = cnt;       
        delay_ms(10);
        
        if (cnt >= 3600) {
            flag = 1;
        }
        else if (cnt <= 0) {
            flag = 0;
        }
        
        if (flag == 1) {
            cnt -= 10;
        } else {
            cnt += 10;
        }
        
        */
        
        //printf("1");
        
        //ram = (short)(TIM3 -> CNT);
        
        //printf("1");
        //printf("target1=%d, %d\r\n", Target_Position_1, Reality_Position_1);
        //printf("target2=%d, %d\r\n", Target_Position_2, Reality_Position_2);
        //sprintf(str1, "A = %d", Target_Position_1);
        
        //sprintf(str2, "B = %d", ram);
        
        //OLED_ShowString(2, 1, str1);
        
        //OLED_ShowString(3, 1, str2);
        //OLED_Clear();
        
        //OLED_Clear();
        //Led_Flash(100000);
//        if(a++ == 10000000){
//            a=0;
//            if(ram < 300)
//                ram+=50;
//            Target_Velocity_1 = Rpm_Encoder_Cnt(ram,13,30,10);   /* 将转速转化为10ms的脉冲数，目标速度 */
//        }
        //demo_key0_fun();
		
	}
}
