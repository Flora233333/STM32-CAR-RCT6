/*
 * Copyright (c) 2023 ��Ϊ���ܿƼ�(����)
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 */

#ifndef __GRAY_I2C_H_GUARD
#define __GRAY_I2C_H_GUARD

#include "bsp.h"

typedef struct {
	void (*sda_out)(uint8_t bit, void *user_data);
	uint8_t (*sda_in)(void *user_data);
	void (*scl_out)(uint8_t bit, void *user_data);
	void *user_data;
} sw_i2c_interface_t;


/**
 * @brief ��ʼ��IIC����
 */
#define     SW_RCC                  RCC_APB2Periph_GPIOC
#define     SW_GPIOX                GPIOC
#define     SW_I2C1_PIN_SCL         GPIO_Pin_6
#define     SW_I2C1_PIN_SDA         GPIO_Pin_7


extern sw_i2c_interface_t i2c_interface; // ͨ�Žӿھ��
extern __IO uint8_t gray_sensor[8];
extern __IO uint8_t digital_data;


/**
 * @brief ��IIC�����ϵ��豸��ȡ����ֽ�
 * @param i2c_interface
 * @param dev_addr ���豸��ַ
 * @param[out] data ��ȡ�����ֽ�����
 * @param data_length ��ȡ��С(�ֽ�)
 * @return 0:�ɹ�, 1:����
 */
int8_t sw_i2c_read(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, __IO uint8_t *data, uint8_t data_length);
int8_t sw_i2c_write(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, const uint8_t *data, uint8_t data_length);

/**
 * @brief ��IIC�����ϵ��豸��ȡһ���ֽ�
 * @param i2c_interface
 * @param dev_addr ���豸��ַ
 * @param[out] data ��ȡ�����ֽ�
 * @return 0:�ɹ�, 1:����
 */
int8_t sw_i2c_read_byte(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, __IO uint8_t *data);
int8_t sw_i2c_write_byte(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, const uint8_t data);

/**
 * @brief ��ȡIIC���ߴ��豸�ļĴ�������. ����д��Ĵ�����ַ,����ֹλ,��������ȡ��������
 * @param i2c_interface
 * @param dev_addr ���豸��ַ
 * @param mem_addr �Ĵ�����ַ
 * @param[out] data ��ȡ�����ֽ�����
 * @param data_length ��ȡ��С(�ֽ�),�������Ĵ�����ַ����
 * @return 0:�ɹ�, 1:����
 */
int8_t sw_i2c_mem_read(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, uint8_t data_length);

/**
 * @brief д��IIC���ߴ��豸�ļĴ���. ����д��Ĵ�����ַ,������д�������е�����
 * @param i2c_interface
 * @param dev_addr ���豸��ַ
 * @param mem_addr �Ĵ�����ַ
 * @param[out] data ����д�������
 * @param data_length ��д����ֽڴ�С,�������Ĵ�����ַ����
 * @return 0:�ɹ�, 1:����
 */
int8_t sw_i2c_mem_write(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t mem_addr, const uint8_t *data, uint8_t data_length);


void Gray_Init(void);
void Get_GrayData(void);


#endif //SW_I2C_H_GUARD
