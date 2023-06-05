/**
  ******************************************************************************
  * @file    bsp_key.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   ����Ӧ��bsp��ɨ��ģʽ��
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� F103-ָ���� STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
  
#include "bsp_key.h"  

/**
  * @brief  ���ð����õ���I/O��
  * @param  ��
  * @retval ��
  */
void Key_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*���������˿ڵ�ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	//ѡ�񰴼�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
	// ���ð���������Ϊ��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	//ʹ�ýṹ���ʼ������
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//ѡ�񰴼�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
	//���ð���������Ϊ��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	//ʹ�ýṹ���ʼ������
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

 /*
 * ��������Key_Scan
 * ����  ������Ƿ��а�������
 * ����  ��GPIOx��x ������ A��B��C��D���� E
 *		     GPIO_Pin������ȡ�Ķ˿�λ 	
 * ���  ��KEY_OFF(û���°���)��KEY_ON�����°�����
 */
uint8_t Key_Scan(void)
{			
	/*����Ƿ��а������� */
    
//	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2) == KEY_ON )  
//	{	 
//		/*�ȴ������ͷ� */
//        delay_ms(20);
//		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2) == KEY_ON);   
//        
//		return 	2;	 
//	}
    
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5) == KEY_ON )  
	{	 
		/*�ȴ������ͷ� */
        delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5) == KEY_OFF);   
        
		return 	5;	 
	}
    
    return 0;
}
/*********************************************END OF FILE**********************/
