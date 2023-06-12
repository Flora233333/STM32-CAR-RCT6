#include "bsp.h"

void bsp_init(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //NVIC�жϷ���
    
    delay_init(72);                                 //��ʱ��ʼ��
    LED_Init();
    USART_Config();
    OLED_Init();
    Motor_Init();
    PWM_Init();
	Encoder_Init();
    bsp_key_init();

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
                
            default:
                /* �����ļ�ֵ������ */
                break;
        }
    }
}
