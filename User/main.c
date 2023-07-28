#include "bsp.h"                  // Device header

uint16_t KeyCode = 0;

int main() {
    
    bsp_init();

    // OLED_ShowString(1, 1, "Gray: ");
    // OLED_ShowString(2, 1, "Yaw : ");
    // OLED_ShowString(3, 1, "Stop: ");
    // OLED_ShowString(4, 1, "Pass: ");

    //Target_Velocity_1 = Rpm_Encoder_Cnt(100,13,30,10); 
    
    //Target_Position_1 = Num_Encoder_Cnt(10,13,30);      
    
    //Target_Velocity_2 = Rpm_Encoder_Cnt(100,13,30,10); 
    
    //Target_Position_2 = Num_Encoder_Cnt(10,13,30); 
    
    //Timer1_InternalClock_Init();

    //int cnt = 0;
    //int flag = 0;
    //printf("1");
    //uint8_t num = 0;
    //__IO uint8_t a;
    //sw_i2c_write_byte(&i2c_interface, 0x4C << 1, 0xB4);
    //uint8_t send_data[4] = {2,3,4,5};
	while(1) {
        //Task_Run();
        Servo_test();
        KeyCode = bsp_GetKey();
        Key_map(KeyCode);
        //Test_Encoder();
        //Get_GrayData();
        //Get_Angle();
        //OLED_ShowNum(2, 7, distant, 4);
        //OLED_ShowNum(3, 7, stop_flag, 2);
        //OLED_ShowNum(4, 7, passby_cross_num, 3);
        //printf("1");

        
	}
}
