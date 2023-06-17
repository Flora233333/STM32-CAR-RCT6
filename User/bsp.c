#include "bsp.h"

void bsp_init(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //NVIC中断分组
    
    delay_init(72);                                 //延时初始化
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
            case KEY_DOWN_K1:			/* K1键按下 */
                start_flag = 1;
                printf("K1键按下\r\n");
                break;

            case KEY_UP_K1:				/* K1键弹起 */
                printf("K1键弹起\r\n");
                break;
            
            case KEY_LONG_K1:			/* K1键长按 */
                printf("K1键长按\r\n");
                break;

            case KEY_DOWN_K2:			/* K2键按下 */
                printf("K2键按下\r\n");
                break;

            case KEY_UP_K2:				/* K2键弹起 */
                printf("K2键弹起\r\n");
                break;

            case KEY_LONG_K2:	        /* K2键长按 */
                printf("K2键长按\r\n");
                break;

            case KEY_DOWN_K3:			/* K3键按下 */
                printf("K3键按下\r\n");
                break;

            case KEY_UP_K3:				/* K3键弹起 */
                printf("K3键弹起\r\n");
                break;

            case KEY_LONG_K3:	        /* K3键长按 */
                printf("K3键长按\r\n");
                break;
            
             case KEY_DOWN_K4:			
                printf("K4键按下\r\n");
                break;

            case KEY_UP_K4:				
                printf("K4键弹起\r\n");
                break;

            case KEY_LONG_K4:	        
                printf("K4键长按\r\n");
                break;

            case KEY_DOWN_K5:			
                printf("K5键按下\r\n");
                break;

            case KEY_UP_K5:				
                printf("K5键弹起\r\n");
                break;

            case KEY_LONG_K5:	        
                printf("K5键长按\r\n");
                break;
                
            default:
                /* 其它的键值不处理 */
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
