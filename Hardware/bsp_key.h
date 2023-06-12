#ifndef __KEY_H
#define	__KEY_H


#include "bsp.h"

//  引脚定义
#define                 KEY1_GPIO_CLK                   RCC_APB2Periph_GPIOA
#define                 KEY1_GPIO_PORT                  GPIOA			   
#define                 KEY1_GPIO_PIN		            GPIO_Pin_0

#define                 KEY2_GPIO_CLK                   RCC_APB2Periph_GPIOC
#define                 KEY2_GPIO_PORT                  GPIOC		   
#define                 KEY2_GPIO_PIN		            GPIO_Pin_13

#define                 HARD_KEY_NUM                    3	                                        /* 实际硬件按键的个数 */
#define                 KEY_COUNT                       (sizeof(s_gpio_list) / sizeof(X_GPIO_T))	/* 配置按键的个数(包括组合按键) */
#define                 KEY_FILTER_TIME                 5
#define                 KEY_LONG_TIME                   100			                                /* 单位10ms， 持续1秒，认为长按事件 */


typedef enum
{
	KID_K1 = 0,
	KID_K2,
	KID_K3
} KEY_ID_E;

typedef enum
{
	KEY_NONE = 0,			/* 0 表示按键事件 */

	KEY_1_DOWN,				/* 1键按下 */
	KEY_1_UP,				/* 1键弹起 */
	KEY_1_LONG,				/* 1键长按 */

	KEY_2_DOWN,				/* 2键按下 */
	KEY_2_UP,				/* 2键弹起 */
	KEY_2_LONG,				/* 2键长按 */

	KEY_3_DOWN,				/* 3键按下 */
	KEY_3_UP,				/* 3键弹起 */
	KEY_3_LONG,				/* 3键长按 */

    KEY_4_DOWN,				/* 4键按下 */
	KEY_4_UP,				/* 4键弹起 */
	KEY_4_LONG,				/* 4键长按 */

} KEY_ENUM;


/* 依次定义GPIO */      
typedef struct
{
	GPIO_TypeDef* gpio;
	uint16_t pin;
	uint8_t ActiveLevel;	/* 按下时的电平(激活电平,高或低) 这个一定要注意否则一堆bug*/
} X_GPIO_T;


typedef struct
{
	/* 下面是一个函数指针，指向判断按键手否按下的函数 */
	uint8_t (*IsKeyDownFunc)(void); /* 按键按下的判断函数,1表示按下 */

	uint8_t  Count;			/* 滤波器计数器 */
	uint16_t LongCount;		/* 长按计数器 */
	uint16_t LongTime;		/* 按键按下持续时间, 0表示不检测长按 */
	uint8_t  State;			/* 按键当前状态（按下还是弹起） */
	uint8_t  RepeatSpeed;	/* 连续按键周期 */
	uint8_t  RepeatCount;	/* 连续按键计数器 */
} KEY_T;


#define KEY_FIFO_SIZE	10

typedef struct
{
	uint8_t Buf[KEY_FIFO_SIZE];		/* 键值缓冲区 */
	uint8_t Read;					/* 缓冲区读指针1 */
	uint8_t Write;					/* 缓冲区写指针 */
	uint8_t Read2;					/* 缓冲区读指针2 */
} KEY_FIFO_T;



void bsp_key_init(void);
uint8_t bsp_GetKey(void);
void bsp_KeyScan10ms(void);


#endif /* __KEY_H */

