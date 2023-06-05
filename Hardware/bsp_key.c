/**
  ******************************************************************************
  * @file    bsp_key.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   按键应用bsp（扫描模式）
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 F103-指南者 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
  
#include "bsp_key.h"  

/**
  * @brief  配置按键用到的I/O口
  * @param  无
  * @retval 无
  */
void Key_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*开启按键端口的时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	//选择按键的引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
	// 设置按键的引脚为浮空输入
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	//使用结构体初始化按键
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//选择按键的引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
	//设置按键的引脚为浮空输入
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	//使用结构体初始化按键
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

 /*
 * 函数名：Key_Scan
 * 描述  ：检测是否有按键按下
 * 输入  ：GPIOx：x 可以是 A，B，C，D或者 E
 *		     GPIO_Pin：待读取的端口位 	
 * 输出  ：KEY_OFF(没按下按键)、KEY_ON（按下按键）
 */
uint8_t Key_Scan(void)
{			
	/*检测是否有按键按下 */
    
//	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2) == KEY_ON )  
//	{	 
//		/*等待按键释放 */
//        delay_ms(20);
//		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2) == KEY_ON);   
//        
//		return 	2;	 
//	}
    
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5) == KEY_ON )  
	{	 
		/*等待按键释放 */
        delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5) == KEY_OFF);   
        
		return 	5;	 
	}
    
    return 0;
}
/*********************************************END OF FILE**********************/
