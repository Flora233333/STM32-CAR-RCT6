#include "bsp.h"

void bsp_init(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //NVIC�жϷ���
    
    delay_init(72);                                 //��ʱ��ʼ��
    LED_Init();
    USART1_Config();
    OLED_Init();
    Motor_Init();
    PWM_Init();
	Encoder_Init();
    bsp_key_init();
    BLU_Init();
//    int ret = atk_ms901m_init(115200);
//    if (ret != 0)
//    {
//        OLED_ShowString(1, 1,"IMU err");
//        while (1)
//        {
//            delay_ms(200);
//        }
//    }  
    
//    MPU_Init();
//    DMP_Init();

    Task_Init();
}

void info_print(void) {
    printf("MCU:STM32F103RCT6\r\n");
    printf("System Clock:%dMHz\r\n", SystemCoreClock / 1000000);
    printf("Author:Flora\r\n");
    printf("Date:2023-6-12\r\n");
    printf("Version:1.0\r\n");
    printf("SCHOOL:XTU\r\n");
}

void Key_map(uint16_t KeyCode) {
    if (KeyCode != KEY_NONE)
    {
        //printf("ucKeyCode = %d\r\n", ucKeyCode);
        switch (KeyCode)
        {
            case KEY_DOWN_K1:			/* K1������ */
                start_flag = 1;
                printf("K1������\r\n");
                break;

            case KEY_UP_K1:				/* K1������ */
                printf("K1������\r\n");
                break;
            
            case KEY_LONG_K1:			/* K1������ */
                printf("K1������\r\n");
                break;

            case KEY_DOWN_K2:			/* K2������ */
                printf("K2������\r\n");
                break;

            case KEY_UP_K2:				/* K2������ */
                printf("K2������\r\n");
                break;

            case KEY_LONG_K2:	        /* K2������ */
                printf("K2������\r\n");
                break;

            case KEY_DOWN_K3:			/* K3������ */
                printf("K3������\r\n");
                break;

            case KEY_UP_K3:				/* K3������ */
                printf("K3������\r\n");
                break;

            case KEY_LONG_K3:	        /* K3������ */
                printf("K3������\r\n");
                break;
            
             case KEY_DOWN_K4:			
                printf("K4������\r\n");
                break;

            case KEY_UP_K4:				
                printf("K4������\r\n");
                break;

            case KEY_LONG_K4:	        
                printf("K4������\r\n");
                break;

            case KEY_DOWN_K5:			
                printf("K5������\r\n");
                break;

            case KEY_UP_K5:				
                printf("K5������\r\n");
                break;

            case KEY_LONG_K5:	        
                printf("K5������\r\n");
                break;
                
            default:
                /* �����ļ�ֵ������ */
                break;
        }
    }
}

void Test_Encoder(void) {
    char str1[20], str2[20];
    static int ram1 = 0, ram2 = 0;
    ram1 = (short)(TIM3 -> CNT);
    ram2 = (short)(TIM4 -> CNT);
    sprintf(str1, "A = %d          ", ram1);
    sprintf(str2, "B = %d          ", ram2);
    OLED_ShowString(1, 1, str1);
    OLED_ShowString(2, 1, str2);
}
