/*
 * Copyright (c) 2023 ��Ϊ���ܿƼ�(����)
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 */

#include "gray_i2c.h"
#include "gray_sensor.h"

#define ACK 0x0 // acknowledge (SDA LOW)
#define NACK 0x1 // not acknowledge (SDA HIGH)

#define LOW 0x0
#define HIGH 0x1

#define I2C_READ 0x1
#define I2C_WRITE 0x0

static void sw_i2c_hal_start(sw_i2c_interface_t *i2c_interface);
static void sw_i2c_hal_stop(sw_i2c_interface_t *i2c_interface);

static void sw_i2c_hal_write_bit(sw_i2c_interface_t *i2c_interface, uint8_t bit);
static uint8_t sw_i2c_hal_read_bit(sw_i2c_interface_t *i2c_interface);

static uint8_t sw_i2c_hal_write_byte(sw_i2c_interface_t *i2c_interface, uint8_t byte);
static uint8_t sw_i2c_hal_read_byte(sw_i2c_interface_t *i2c_interface, uint8_t ack);


sw_i2c_interface_t i2c_interface; // ͨ�Žӿھ��
__IO uint8_t gray_sensor[8];      // 8·�Ҷ�����
__IO uint8_t digital_data = 0X00;


void sw_i2c_init()
{
	RCC_APB2PeriphClockCmd(SW_RCC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = SW_I2C1_PIN_SCL | SW_I2C1_PIN_SDA;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SW_GPIOX, &GPIO_InitStructure);
	
	GPIO_SetBits(SW_GPIOX, SW_I2C1_PIN_SCL | SW_I2C1_PIN_SDA);
}

/* ����sda������� bit=0Ϊ�͵�ƽ bit=1Ϊ�ߵ�ƽ */
void sda_out(uint8_t bit, void *user_data)
{
	GPIO_WriteBit(GPIOB, SW_I2C1_PIN_SDA, (BitAction)bit);
	
	/* IIC����ӳ� */
	delay_us(10);
}

/* ����sda��ȡ���� bit Ϊ���صĵ�ƽֵ */
uint8_t sda_in(void *user_data)
{
	uint8_t bit;
	bit = (uint8_t)GPIO_ReadInputDataBit(GPIOB, SW_I2C1_PIN_SDA);
	
	/* IIC����ӳ� */
	delay_us(10);
	return bit;
}

/* ����sclʱ��������� bit=0Ϊ�͵�ƽ bit=1Ϊ�ߵ�ƽ */
void scl_out(uint8_t bit, void *user_data)
{
	GPIO_WriteBit(GPIOB, SW_I2C1_PIN_SCL, (BitAction)bit);
	
	/* IIC����ӳ� */
	delay_us(10);
}

void Gray_Init(void) {
	uint8_t ping_response;
	
	/* ��ʼ��IIC */
	sw_i2c_init();
	/* ps: ���IIC��ʼ����ʱ������һ��IIC start���ᵼ�µ�һ��IICͨѶ��ʧ�� */
	
	/* �������IIC���� */
	i2c_interface.sda_in = sda_in;
    i2c_interface.scl_out = scl_out;
    i2c_interface.sda_out = sda_out;
    i2c_interface.user_data = 0;            //�û����ݣ������������������õ�

    /* ��һ��IICͨѶ��ʧ�ܣ���Ϊ���IIC������start�����ֶ�����stopҲ������ */
	sw_i2c_mem_read(&i2c_interface, 0x4C << 1, GW_GRAY_PING, &ping_response, 1);
	/* ����IICͨѶ�������� */
	sw_i2c_mem_read(&i2c_interface, 0x4C << 1, GW_GRAY_PING, &ping_response, 1);
    
    /* ��ַ��֤ */
    if(ping_response == 0x66)
        OLED_ShowString(1, 1, "Gray Sensor Init OK!");    
    else 
        OLED_ShowString(1, 1, "Gray Sensor Failed");

    delay_ms(1000);
	
	/* �򿪿���������ģʽ */
	sw_i2c_write_byte(&i2c_interface, 0x4C << 1, GW_GRAY_DIGITAL_MODE);
    /* ��һ��8�������������� */
    sw_i2c_read_byte(&i2c_interface, 0x4C << 1, &digital_data); // digital_data ��1~8��̽ͷ��������
}

void Get_GrayData(void) {
    /* ��ȡ���������� */
	sw_i2c_read_byte(&i2c_interface, 0x4C << 1, &digital_data); // digital_data ��1~8��̽ͷ��������

    SEP_ALL_BIT8(digital_data, 
        gray_sensor[0], //̽ͷ1
        gray_sensor[1], //̽ͷ2
        gray_sensor[2], //̽ͷ3
        gray_sensor[3], //̽ͷ4
        gray_sensor[4], //̽ͷ5
        gray_sensor[5], //̽ͷ6
        gray_sensor[6], //̽ͷ7
        gray_sensor[7]  //̽ͷ8
    );
}



int8_t sw_i2c_read(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, __IO uint8_t *data, uint8_t data_length)
{
	uint8_t i;
	uint8_t ack_bit;

	/* ��ʼλ */
	sw_i2c_hal_start(i2c_interface);

	/* ��ַ+��дλ */
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_READ);
	if (ack_bit) {
		/* ���豸û�лظ�ACK,ֱ���˳� */
		sw_i2c_hal_stop(i2c_interface);
		return 1;
	}

	/* ������ȡN-1������ ��ACK */
	for (i = 0; i < data_length - 1; ++i) {
		data[i] = sw_i2c_hal_read_byte(i2c_interface, ACK);
	}

	/* ���һ�����ݸ� NACK */
	data[i] = sw_i2c_hal_read_byte(i2c_interface, NACK);

	/* ֹͣλ */
	sw_i2c_hal_stop(i2c_interface);
	return 0;
}


int8_t sw_i2c_write(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, const uint8_t *data, uint8_t data_length)
{
	uint8_t i;
	uint8_t ack_bit;

	/* ��ʼλ */
	sw_i2c_hal_start(i2c_interface);

	/* ��ַ+��дλ */
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_WRITE);
	if (ack_bit) {
		/* ���豸û�лظ�ACK,ֱ���˳� */
		sw_i2c_hal_stop(i2c_interface);
		return 1;
	}

	/* ����д��N������, ÿ�ζ�ȡ1 bit�� ACK */
	for (i = 0; i < data_length; ++i) {
		 ack_bit = sw_i2c_hal_write_byte(i2c_interface, data[i]);
	}

	/* ֹͣλ */
	sw_i2c_hal_stop(i2c_interface);
	return 0;
}


int8_t sw_i2c_read_byte(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, __IO uint8_t *data)
{
	return sw_i2c_read(i2c_interface, dev_addr, data, 1);
}


int8_t sw_i2c_write_byte(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, const uint8_t data)
{
	return sw_i2c_write(i2c_interface, dev_addr, &data, 1);
}


int8_t sw_i2c_mem_read(sw_i2c_interface_t *i2c_interface,
					   uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, uint8_t data_length)
{
	uint8_t ack_bit;
	sw_i2c_hal_start(i2c_interface);
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_WRITE);
	if (ack_bit) {
		sw_i2c_hal_stop(i2c_interface);
		return 1;
	}
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, mem_addr);

	return sw_i2c_read(i2c_interface, dev_addr, data, data_length);
}


int8_t sw_i2c_mem_write(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t mem_addr, const uint8_t *data,
                        uint8_t data_length)
{
	uint8_t ack_bit;
	sw_i2c_hal_start(i2c_interface);
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_WRITE);
	if (ack_bit) {
		sw_i2c_hal_stop(i2c_interface);
		return 1;
	}
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, mem_addr);

	return sw_i2c_write(i2c_interface, dev_addr, data, data_length);
}


/***************************
 * ���A���������
 **************************/

/**
 * @brief send start bit by driving sda and scl LOW
 * @param i2c_interface
 */
static void sw_i2c_hal_start(sw_i2c_interface_t *i2c_interface)
{
	i2c_interface->sda_out(HIGH, i2c_interface->user_data);
	i2c_interface->scl_out(HIGH, i2c_interface->user_data);
	i2c_interface->sda_out(LOW, i2c_interface->user_data);
	i2c_interface->scl_out(LOW, i2c_interface->user_data);
}

/**
 * @brief send stop bit
 * @param i2c_interface
 */
static void sw_i2c_hal_stop(sw_i2c_interface_t *i2c_interface)
{
	i2c_interface->sda_out(LOW, i2c_interface->user_data);
	i2c_interface->scl_out(HIGH, i2c_interface->user_data);
	i2c_interface->sda_out(HIGH, i2c_interface->user_data);
}

/**
 * @brief ��� sda ��ƽ,Ȼ�� scl ���һ��ʱ��
 * @param i2c_interface
 * @param bit bit level to send, 0:LOW, 1:HIGH
 */
static void sw_i2c_hal_write_bit(sw_i2c_interface_t *i2c_interface, uint8_t bit)
{
	i2c_interface->sda_out(bit, i2c_interface->user_data);
	i2c_interface->scl_out(HIGH, i2c_interface->user_data);
	i2c_interface->scl_out(LOW, i2c_interface->user_data);
}

/**
 * @brief �� sda ��ƽֵ,Ȼ�� scl ���һ��ʱ��
 * @param i2c_interface
 * @return ���� SDA ��ƽֵ, 0:LOW, 1:HIGH
 */
static uint8_t sw_i2c_hal_read_bit(sw_i2c_interface_t *i2c_interface)
{
	uint8_t bit;
	i2c_interface->sda_out(HIGH, i2c_interface->user_data);
	i2c_interface->scl_out(HIGH, i2c_interface->user_data);
	bit = i2c_interface->sda_in(i2c_interface->user_data);
	i2c_interface->scl_out(LOW, i2c_interface->user_data);
	return bit;
}

/**
 * @brief ��IIC���һ���ֽ�
 * @param i2c_interface
 * @param byte
 * @return ���豸������ ACK ��ƽֵ
 */
static uint8_t sw_i2c_hal_write_byte(sw_i2c_interface_t *i2c_interface, uint8_t byte)
{
	uint8_t i;
	uint8_t ack;

	for (i = 0; i < 8; ++i) {
		sw_i2c_hal_write_bit(i2c_interface, byte & (0x80 >> i));
	}

	ack = sw_i2c_hal_read_bit(i2c_interface);
	return ack;
}

/**
 * @brief ��IIC�����϶�ȡһ���ֽ�
 * @param i2c_interface
 * @param ack ����豸���� ACK ���� NACK
 * @return ��ȡ�����ֽ�
 */
static uint8_t sw_i2c_hal_read_byte(sw_i2c_interface_t *i2c_interface, uint8_t ack)
{
	uint8_t byte = 0;
	uint8_t i;

	i2c_interface->sda_out(HIGH, i2c_interface->user_data);
	for (i = 0; i < 8; ++i) {
		i2c_interface->scl_out(HIGH, i2c_interface->user_data);
		byte <<= 1;
		byte |= i2c_interface->sda_in(i2c_interface->user_data);
		i2c_interface->scl_out(LOW, i2c_interface->user_data);
	}

	sw_i2c_hal_write_bit(i2c_interface, ack);
	return byte;
}
