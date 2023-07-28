
#include "bsp_key.h"  


static const X_GPIO_T s_gpio_list[HARD_KEY_NUM] = {
	{GPIOA, GPIO_Pin_0, 0},		/* K1 */
	{GPIOA, GPIO_Pin_4, 0},	    /* K2 */
    {GPIOA, GPIO_Pin_5, 0},     /* K3 */
    {GPIOA, GPIO_Pin_11, 0},
    {GPIOC, GPIO_Pin_4, 0}
};	

static KEY_T s_tBtn[KEY_COUNT] = {0};
static KEY_FIFO_T s_tKey;		/* ����FIFO����,�ṹ�� */

void bsp_InitKeyHard(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	uint8_t i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    
	/* ��2�����������еİ���GPIOΪ��������ģʽ(ʵ����CPU��λ���������״̬) */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   			/* �������� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;           /* GPIO�ٶȵȼ� */
	
	for (i = 0; i < HARD_KEY_NUM; i++)
	{
		GPIO_InitStructure.GPIO_Pin = s_gpio_list[i].pin;
		GPIO_Init(s_gpio_list[i].gpio, &GPIO_InitStructure);	
	}
    
}


void bsp_InitKeyVar(void)
{
	uint8_t i;

	/* �԰���FIFO��дָ������ */
	s_tKey.Read = 0;
	s_tKey.Write = 0;
	s_tKey.Read2 = 0;

	/* ��ÿ�������ṹ���Ա������һ��ȱʡֵ */
	for (i = 0; i < KEY_COUNT; i++)
	{
		s_tBtn[i].LongTime = KEY_LONG_TIME;			/* ����ʱ�� 0 ��ʾ����ⳤ�����¼� */
		s_tBtn[i].Count = KEY_FILTER_TIME / 2;		/* ����������Ϊ�˲�ʱ���һ�� */
		s_tBtn[i].State = 0;							/* ����ȱʡ״̬��0Ϊδ���� */
		s_tBtn[i].RepeatSpeed = 0;						/* �����������ٶȣ�0��ʾ��֧������ */
		s_tBtn[i].RepeatCount = 0;						/* ���������� */
	}
}

void bsp_key_init(void) {
    bsp_InitKeyHard();
    bsp_InitKeyVar();
}


uint8_t bsp_GetKey(void)
{
	uint8_t ret;

	if (s_tKey.Read == s_tKey.Write)
	{
		return KEY_NONE;
	}
	else
	{
		ret = s_tKey.Buf[s_tKey.Read];

		if (++s_tKey.Read >= KEY_FIFO_SIZE)
		{
			s_tKey.Read = 0;
		}
		return ret;
	}
}

void bsp_PutKey(uint8_t _KeyCode)
{
	s_tKey.Buf[s_tKey.Write] = _KeyCode;

	if (++s_tKey.Write  >= KEY_FIFO_SIZE)
	{
		s_tKey.Write = 0;
	}
}

static uint8_t KeyPinActive(uint8_t _id)
{
	uint8_t level;
	
	if ((s_gpio_list[_id].gpio->IDR & s_gpio_list[_id].pin) == 0)
	{
		level = 0;
	}
	else
	{
		level = 1;
	}

	
	if (level == s_gpio_list[_id].ActiveLevel)
    {
		return 1;
	}
	else
	{
		return 0;
	}
}


static uint8_t IsKeyDownFunc(uint8_t _id)
{
	/* ʵ�嵥�� */
	if (_id < HARD_KEY_NUM)
	{
		uint8_t i;
		uint8_t count = 0;
		uint8_t save = 255;
		
		/* �ж��м��������� */
		for (i = 0; i < HARD_KEY_NUM; i++)
		{
			if (KeyPinActive(i)) 
			{
				count++;
				save = i;
			}
		}
		
		if (count == 1 && save == _id)
		{
			return 1;	/* ֻ��1��������ʱ����Ч */
		}		

		return 0;
	}

	return 0;
}


static void bsp_DetectKey(uint8_t i)
{
	KEY_T *pBtn;

	pBtn = &s_tBtn[i];
	if (IsKeyDownFunc(i))
	{
		if (pBtn->Count < KEY_FILTER_TIME)
		{
			pBtn->Count = KEY_FILTER_TIME;
		}
		else if(pBtn->Count < 2 * KEY_FILTER_TIME)
		{
			pBtn->Count++;
		}
		else
		{
			if (pBtn->State == 0)
			{
				pBtn->State = 1;

				/* ���Ͱ�ť���µ���Ϣ */
				bsp_PutKey((uint8_t)(3 * i + 1)); //��iӳ�䵽KEY_ENUM��ö�����Ͷ�Ӧ�ı�����ȥ
			}

			if (pBtn->LongTime > 0)
			{
				if (pBtn->LongCount < pBtn->LongTime)
				{
					/* ���Ͱ�ť�������µ���Ϣ */
					if (++pBtn->LongCount == pBtn->LongTime)
					{
						/* ��ֵ���밴��FIFO */
						bsp_PutKey((uint8_t)(3 * i + 3)); //��iӳ�䵽KEY_ENUM��ö�����Ͷ�Ӧ�ı�����ȥ
					}
				}
				else
				{
					if (pBtn->RepeatSpeed > 0)
					{
						if (++pBtn->RepeatCount >= pBtn->RepeatSpeed) //pBtn->RepeatCount��++
						{
							pBtn->RepeatCount = 0;
							/* ��������ÿ��10ms����1������ */
							bsp_PutKey((uint8_t)(3 * i + 1));
						}
					}
				}
			}
		}
	}
	else
	{
		if(pBtn->Count > KEY_FILTER_TIME)
		{
			pBtn->Count = KEY_FILTER_TIME;
		}
		else if(pBtn->Count != 0) //����ִ�KEY_FILTER_TIME����0���������ɿ��������˲�
		{
			pBtn->Count--;
		}
		else
		{
			if (pBtn->State == 1)
			{
				pBtn->State = 0;

				/* ���Ͱ�ť�������Ϣ */
				bsp_PutKey((uint8_t)(3 * i + 2));
			}
		}

		pBtn->LongCount = 0;
		pBtn->RepeatCount = 0;
	}
}

void bsp_KeyScan10ms(void)
{
	uint8_t i;

	for (i = 0; i < KEY_COUNT; i++)
	{
		bsp_DetectKey(i);
	}
}
