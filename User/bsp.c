#include "bsp.h"

void bsp_init(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //NVIC中断分组
    
    delay_init(72);                                 //延时初始化
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
            case KEY_DOWN_K1:			/* K1键按下 */
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
                
            default:
                /* 其它的键值不处理 */
                break;
        }
    }
}
