#ifndef __KEY_H
#define	__KEY_H


#include "bsp.h"

//  ���Ŷ���
#define                 KEY1_GPIO_CLK                   RCC_APB2Periph_GPIOA
#define                 KEY1_GPIO_PORT                  GPIOA			   
#define                 KEY1_GPIO_PIN		            GPIO_Pin_0

#define                 KEY2_GPIO_CLK                   RCC_APB2Periph_GPIOC
#define                 KEY2_GPIO_PORT                  GPIOC		   
#define                 KEY2_GPIO_PIN		            GPIO_Pin_13

#define                 HARD_KEY_NUM                    3	                                        /* ʵ��Ӳ�������ĸ��� */
#define                 KEY_COUNT                       (sizeof(s_gpio_list) / sizeof(X_GPIO_T))	/* ���ð����ĸ���(������ϰ���) */
#define                 KEY_FILTER_TIME                 5
#define                 KEY_LONG_TIME                   100			                                /* ��λ10ms�� ����1�룬��Ϊ�����¼� */


typedef enum
{
	KID_K1 = 0,
	KID_K2,
	KID_K3
} KEY_ID_E;

typedef enum
{
	KEY_NONE = 0,			/* 0 ��ʾ�����¼� */

	KEY_1_DOWN,				/* 1������ */
	KEY_1_UP,				/* 1������ */
	KEY_1_LONG,				/* 1������ */

	KEY_2_DOWN,				/* 2������ */
	KEY_2_UP,				/* 2������ */
	KEY_2_LONG,				/* 2������ */

	KEY_3_DOWN,				/* 3������ */
	KEY_3_UP,				/* 3������ */
	KEY_3_LONG,				/* 3������ */

    KEY_4_DOWN,				/* 4������ */
	KEY_4_UP,				/* 4������ */
	KEY_4_LONG,				/* 4������ */

} KEY_ENUM;


/* ���ζ���GPIO */      
typedef struct
{
	GPIO_TypeDef* gpio;
	uint16_t pin;
	uint8_t ActiveLevel;	/* ����ʱ�ĵ�ƽ(�����ƽ,�߻��) ���һ��Ҫע�����һ��bug*/
} X_GPIO_T;


typedef struct
{
	/* ������һ������ָ�룬ָ���жϰ����ַ��µĺ��� */
	uint8_t (*IsKeyDownFunc)(void); /* �������µ��жϺ���,1��ʾ���� */

	uint8_t  Count;			/* �˲��������� */
	uint16_t LongCount;		/* ���������� */
	uint16_t LongTime;		/* �������³���ʱ��, 0��ʾ����ⳤ�� */
	uint8_t  State;			/* ������ǰ״̬�����»��ǵ��� */
	uint8_t  RepeatSpeed;	/* ������������ */
	uint8_t  RepeatCount;	/* �������������� */
} KEY_T;


#define KEY_FIFO_SIZE	10

typedef struct
{
	uint8_t Buf[KEY_FIFO_SIZE];		/* ��ֵ������ */
	uint8_t Read;					/* ��������ָ��1 */
	uint8_t Write;					/* ������дָ�� */
	uint8_t Read2;					/* ��������ָ��2 */
} KEY_FIFO_T;



void bsp_key_init(void);
uint8_t bsp_GetKey(void);
void bsp_KeyScan10ms(void);


#endif /* __KEY_H */

