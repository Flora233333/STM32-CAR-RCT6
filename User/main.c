#include "bsp.h"                  // Device header

uint16_t KeyCode = 0;

int main() {
    
    bsp_init();

    OLED_ShowString(1, 1, "Gray: ");
    OLED_ShowString(2, 1, "dis : ");
    OLED_ShowString(3, 1, "Stop: ");
    OLED_ShowString(4, 1, "Cap : ");
    //Target_Velocity_1 = Rpm_Encoder_Cnt(100,13,30,10); 
    
    //Target_Position_1 = Num_Encoder_Cnt(10,13,30);      
    
    //Target_Velocity_2 = Rpm_Encoder_Cnt(100,13,30,10); 
    
    //Target_Position_2 = Num_Encoder_Cnt(10,13,30); 
    
    Timer1_InternalClock_Init();
    

    //int cnt = 0;
    //int flag = 0;
    //printf("1");
    //uint8_t num = 0;
    //__IO uint8_t a;
    //sw_i2c_write_byte(&i2c_interface, 0x4C << 1, 0xB4);
	while(1) {
        Task_Run();
        Get_Angle();
        Get_GrayData();
        OLED_ShowNum(2, 7, distant, 4);
        OLED_ShowNum(3, 7, stop_flag, 2);
        OLED_ShowNum(4, 7, capture_data[0], 3);
        //sw_i2c_write_byte(&i2c_interface, 0x4C << 1, 0xB4);
        //sw_i2c_read_byte(&i2c_interface, 0x4C << 1, &a);
        //printf("d:%d, %d, %d\n",Reality_Velocity_1, Reality_Velocity_2, Target_Velocity_1);
        //printf("d:%.2f, %d\n",Now_Angle, Target_angle);
        //printf("%d\r\n",a);
        
        //Test_Encoder();
        //OLED_ShowNum(1,1,rxdata[0],3);
        
        KeyCode = bsp_GetKey();
        Key_map(KeyCode);

        
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
        
        
        //printf("1");
        //printf("target1=%d, %d\r\n", Target_Position_1, Reality_Position_1);
        //printf("target2=%d, %d\r\n", Target_Position_2, Reality_Position_2);
        //sprintf(str1, "A = %d", Target_Position_1);

        
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

        
	}
}
