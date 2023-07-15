/*
 * Copyright (c) 2023 感为智能科技(济南)
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


sw_i2c_interface_t i2c_interface; // 通信接口句柄
__IO uint8_t gray_sensor[8];      // 8路灰度数据
__IO uint8_t digital_data = 0X00;
uint8_t Gray_OK = 0;

void sw_i2c_init()
{
	RCC_APB2PeriphClockCmd(SW_RCC, ENABLE);

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    //GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = SW_I2C1_PIN_SCL | SW_I2C1_PIN_SDA;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SW_GPIOX, &GPIO_InitStructure);
	
	GPIO_SetBits(SW_GPIOX, SW_I2C1_PIN_SCL | SW_I2C1_PIN_SDA);
}

/* 定义sda输出函数 bit=0为低电平 bit=1为高电平 */
void sda_out(uint8_t bit, void *user_data)
{
	GPIO_WriteBit(SW_GPIOX, SW_I2C1_PIN_SDA, (BitAction)bit);
	
	/* IIC软件延迟 */
	delay_us(10);
}

/* 定义sda读取函数 bit 为返回的电平值 */
uint8_t sda_in(void *user_data)
{
	uint8_t bit;
	bit = (uint8_t)GPIO_ReadInputDataBit(SW_GPIOX, SW_I2C1_PIN_SDA);
	
	/* IIC软件延迟 */
	delay_us(10);
	return bit;
}

/* 定义scl时钟输出函数 bit=0为低电平 bit=1为高电平 */
void scl_out(uint8_t bit, void *user_data)
{
	GPIO_WriteBit(SW_GPIOX, SW_I2C1_PIN_SCL, (BitAction)bit);
	
	/* IIC软件延迟 */
	delay_us(10);
}

void Gray_Init(void) {
	uint8_t ping_response;
	
	/* 初始化IIC */
	sw_i2c_init();
	/* ps: 软件IIC初始化的时候会出发一次IIC start，会导致第一次IIC通讯会失败 */
	
	/* 设置软件IIC驱动 */
	i2c_interface.sda_in = sda_in;
    i2c_interface.scl_out = scl_out;
    i2c_interface.sda_out = sda_out;
    i2c_interface.user_data = 0;            //用户数据，可在输入输出函数里得到

    /* 第一次IIC通讯会失败（因为软件IIC触发了start），手动发个stop也能消除 */
	sw_i2c_mem_read(&i2c_interface, 0x4C << 1, GW_GRAY_PING, &ping_response, 1);
	/* 后面IIC通讯是正常的 */
	sw_i2c_mem_read(&i2c_interface, 0x4C << 1, GW_GRAY_PING, &ping_response, 1);
    
    /* 地址验证 */
    if(ping_response == GW_GRAY_PING_OK) {
        OLED_ShowString(1, 1, "Gray Init OK!");  
        Gray_OK = 1;  
        delay_ms(1000);
        OLED_Clear();
    }
    else {
        OLED_ShowString(1, 1, "Gray  Failed");
    }

	/* 打开开关量数据模式 */
	sw_i2c_write_byte(&i2c_interface, 0x4C << 1, GW_GRAY_DIGITAL_MODE);
    /* 读一次8个传感器的数据 */
    sw_i2c_read_byte(&i2c_interface, 0x4C << 1, &digital_data); // digital_data 有1~8号探头开关数据
}

char gray_str[20];

void Get_GrayData(void) {
    if (Gray_OK) {
        /* 读取开关量数据 */
	    sw_i2c_read_byte(&i2c_interface, 0x4C << 1, &digital_data); // digital_data 有1~8号探头开关数据

        SEP_ALL_BIT8(digital_data, 
            gray_sensor[0], //探头1
            gray_sensor[1], //探头2
            gray_sensor[2], //探头3
            gray_sensor[3], //探头4
            gray_sensor[4], //探头5
            gray_sensor[5], //探头6
            gray_sensor[6], //探头7
            gray_sensor[7]  //探头8
        );

        sprintf(gray_str, "%d%d%d%d%d%d%d%d", 
                          gray_sensor[7], 
                          gray_sensor[6], 
                          gray_sensor[5], 
                          gray_sensor[4], 
                          gray_sensor[3], 
                          gray_sensor[2], 
                          gray_sensor[1], 
                          gray_sensor[0] 
                );

        OLED_ShowString(1, 7, gray_str);
    }
}

uint8_t passby_cross_num = 0; // 通过十字路口的次数

/* 检测特殊灰度数据 */
void Detect_Special_GrayData(void) {

    if(Gray_OK == 1 && task_finish == 0) {

        // if( gray_sensor[0] == 0 && gray_sensor[1] == 0 && gray_sensor[2] == 0 && 
        //     gray_sensor[3] == 0 && gray_sensor[4] == 1 && 
        //     gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
        //     Target_angle = -90; // 左转
        //     //左转程序flag = 1;
        // }


        //停止检测
        // if(gray_sensor[1] == 0 && gray_sensor[2] == 0 && 
        //     gray_sensor[3] == 0 && gray_sensor[4] == 0 && 
        //     gray_sensor[5] == 0 && gray_sensor[6] == 0 ) {

        //     if(user_key_num == 1) {
        //         stop_flag ++;
            
        //     } 
        //     // else if(user_key_num == 2) {
        //     //     pass_line_num ++;
        //     // }
        //     // else if(user_key_num == 3) {
        //     //     pass_line_num ++;
        //     // }
        //     else if(user_key_num == 4) {
        //         stop_flag ++;
                
        //     }
        //     Detect_Special_GrayData_handler.last_run = Get_nowtime() + 500;
        // }

        // 停止检测
        if( gray_sensor[0] == 0 && gray_sensor[1] == 0 &&
            gray_sensor[2] == 0 && gray_sensor[3] == 0 && 
            gray_sensor[4] == 0 && gray_sensor[5] == 0 &&
            gray_sensor[6] == 0 && gray_sensor[7] == 0 ) {
            if(user_key_num == 1) {
                stop_flag ++;
            } 
            else if(user_key_num == 2) {
                stop_flag ++;
            }
            else if(user_key_num == 3) {
                stop_flag ++;
            }
            else if(user_key_num == 4) {
                stop_flag ++;
            }

            if(user_key_num != 0) {
                Beep_ON_handler.is_live = True;
                Beep_ON_handler.last_run = Get_nowtime() + 5;
                BEEP_toggle();
                Detect_Special_GrayData_handler.last_run = Get_nowtime() + 600;
            }
        }

        if(user_key_num != 0 && (MPU_Data.yaw < 80 && MPU_Data.yaw > -80)) {
        // 三叉路口检测
            if( gray_sensor[0] == 0 && gray_sensor[1] == 1 && 
                gray_sensor[2] == 1 && gray_sensor[3] == 0 &&
                gray_sensor[4] == 1 && gray_sensor[5] == 1 && 
                gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
                passby_cross_num ++;
                Beep_ON_handler.is_live = True;
                Beep_ON_handler.last_run = Get_nowtime() + 5;
                BEEP_toggle();
                Detect_Special_GrayData_handler.last_run = Get_nowtime() + 1000;
            }
            if( gray_sensor[0] == 1 && gray_sensor[1] == 0 && 
                gray_sensor[2] == 1 && gray_sensor[3] == 0 &&
                gray_sensor[4] == 1 && gray_sensor[5] == 1 && 
                gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
                passby_cross_num ++;
                Beep_ON_handler.is_live = True;
                Beep_ON_handler.last_run = Get_nowtime() + 5;
                BEEP_toggle();
                Detect_Special_GrayData_handler.last_run = Get_nowtime() + 1000;
            }

            if( gray_sensor[0] == 1 && gray_sensor[1] == 0 && 
                gray_sensor[2] == 0 && gray_sensor[3] == 1 &&
                gray_sensor[4] == 0 && gray_sensor[5] == 1 && 
                gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
                passby_cross_num ++;
                Beep_ON_handler.is_live = True;
                Beep_ON_handler.last_run = Get_nowtime() + 5;
                BEEP_toggle();
                Detect_Special_GrayData_handler.last_run = Get_nowtime() + 1000;
            }

            if( gray_sensor[0] == 1 && gray_sensor[1] == 0 && 
                gray_sensor[2] == 1 && gray_sensor[3] == 1 &&
                gray_sensor[4] == 0 && gray_sensor[5] == 1 && 
                gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
                passby_cross_num ++;
                Beep_ON_handler.is_live = True;
                Beep_ON_handler.last_run = Get_nowtime() + 5;
                BEEP_toggle();
                Detect_Special_GrayData_handler.last_run = Get_nowtime() + 1000;
            }

            if( gray_sensor[0] == 0 && gray_sensor[1] == 0 && 
                gray_sensor[2] == 1 && gray_sensor[3] == 1 &&
                gray_sensor[4] == 0 && gray_sensor[5] == 1 && 
                gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
                passby_cross_num ++;
                Beep_ON_handler.is_live = True;
                Beep_ON_handler.last_run = Get_nowtime() + 5;
                BEEP_toggle();
                Detect_Special_GrayData_handler.last_run = Get_nowtime() + 1000;
            }

            if( gray_sensor[0] == 0 && gray_sensor[1] == 0 && 
                gray_sensor[2] == 1 && gray_sensor[3] == 0 &&
                gray_sensor[4] == 0 && gray_sensor[5] == 1 && 
                gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
                passby_cross_num ++;
                Beep_ON_handler.is_live = True;
                Beep_ON_handler.last_run = Get_nowtime() + 5;
                BEEP_toggle();
                Detect_Special_GrayData_handler.last_run = Get_nowtime() + 1000;
            }

            // if( gray_sensor[0] == 1 && gray_sensor[1] == 0 && 
            //     gray_sensor[2] == 0 && gray_sensor[3] == 0 &&
            //     gray_sensor[4] == 0 && gray_sensor[5] == 1 && 
            //     gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
            //     passby_cross_num ++;
            //     Beep_ON_handler.is_live = True;
            //     Beep_ON_handler.last_run = Get_nowtime() + 5;
            //     BEEP_toggle();
            //     Detect_Special_GrayData_handler.last_run = Get_nowtime() + 1000;
            // }

            if( gray_sensor[0] == 0 && gray_sensor[1] == 0 && 
                gray_sensor[2] == 0 && gray_sensor[3] == 0 &&
                gray_sensor[4] == 0 && gray_sensor[5] == 1 && 
                gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
                passby_cross_num ++;
                Beep_ON_handler.is_live = True;
                Beep_ON_handler.last_run = Get_nowtime() + 5;
                BEEP_toggle();
                Detect_Special_GrayData_handler.last_run = Get_nowtime() + 1000;
            }

            // if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && 
            //     gray_sensor[2] == 0 && gray_sensor[3] == 0 &&
            //     gray_sensor[4] == 0 && gray_sensor[5] == 1 && 
            //     gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
            //     passby_cross_num ++;
            //     Beep_ON_handler.is_live = True;
            //     Beep_ON_handler.last_run = Get_nowtime() + 5;
            //     BEEP_toggle();
            //     Detect_Special_GrayData_handler.last_run = Get_nowtime() + 1000;
            // }

            if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && 
                gray_sensor[2] == 0 && gray_sensor[3] == 1 &&
                gray_sensor[4] == 0 && gray_sensor[5] == 1 && 
                gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
                passby_cross_num ++;
                Beep_ON_handler.is_live = True;
                Beep_ON_handler.last_run = Get_nowtime() + 5;
                BEEP_toggle();
                Detect_Special_GrayData_handler.last_run = Get_nowtime() + 1000;
            }

            if( gray_sensor[0] == 0 && gray_sensor[1] == 0 && 
                gray_sensor[2] == 1 && gray_sensor[3] == 0 &&
                gray_sensor[4] == 1 && gray_sensor[5] == 1 && 
                gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
                passby_cross_num ++;
                Beep_ON_handler.is_live = True;
                Beep_ON_handler.last_run = Get_nowtime() + 5;
                BEEP_toggle();
                Detect_Special_GrayData_handler.last_run = Get_nowtime() + 1000;
            }

            
        }
    }
    //......
}

extern uint8_t task_3;
/* 检测灰度传感器偏差 */
float Detect_GraySensor_Bias(void) {
    float bias = 0;
    static float last_bias = 0;

    if(Gray_OK == 1) {

        
        // if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
        //     gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
        //     gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
        //     bias = last_bias * 0.5; // 飞了
        //     return bias;
        // }
        // if(task_3 == 2) {
        //     if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 0 && 
        //         gray_sensor[3] == 1 && gray_sensor[4] == 0 && 
        //         gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
        //         bias = 5; // 4右偏
        //     }
        //     else if( gray_sensor[0] == 0 && gray_sensor[1] == 0 && gray_sensor[2] == 1 && 
        //         gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
        //         gray_sensor[5] == 0 && gray_sensor[6] == 0 && gray_sensor[7] == 1 ) {
        //         bias = 5; // 4左偏
        //     }

        //     else if( gray_sensor[0] == 1 && gray_sensor[1] == 0 && gray_sensor[2] == 1 && 
        //         gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
        //         gray_sensor[5] == 1 && gray_sensor[6] == 0 && gray_sensor[7] == 0 ) {
        //         bias = 5; // 4右偏
        //     }
        //     else if( gray_sensor[0] == 0 && gray_sensor[1] == 0 && gray_sensor[2] == 1 && 
        //         gray_sensor[3] == 1 && gray_sensor[4] == 0 && 
        //         gray_sensor[5] == 0 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
        //         bias = 5; // 4左偏
        //     }
        //     else if( gray_sensor[0] == 0 && gray_sensor[1] == 0 && gray_sensor[2] == 0 && 
        //         gray_sensor[3] == 0 && gray_sensor[4] == 0 && 
        //         gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
        //         bias = 5; // 4左偏
        //     }
        //     else if( gray_sensor[0] == 0 && gray_sensor[1] == 0 && gray_sensor[2] == 0 && 
        //         gray_sensor[3] == 0 && gray_sensor[4] == 0 && 
        //         gray_sensor[5] == 1 && gray_sensor[6] == 0 && gray_sensor[7] == 1 ) {
        //         bias = 5; // 4左偏
        //     }
        //     else if( gray_sensor[0] == 0 && gray_sensor[1] == 0 && gray_sensor[2] == 0 && 
        //         gray_sensor[3] == 0 && gray_sensor[4] == 0 && 
        //         gray_sensor[5] == 1 && gray_sensor[6] == 0 && gray_sensor[7] == 1 ) {
        //         bias = 5; // 4左偏
        //     }
        //     else if( gray_sensor[0] == 0 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
        //         gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
        //         gray_sensor[5] == 0 && gray_sensor[6] == 0 && gray_sensor[7] == 0 ) {
        //         bias = 5; // 4左偏
        //     }
        //     else if( gray_sensor[0] == 0 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
        //         gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
        //         gray_sensor[5] == 1 && gray_sensor[6] == 0 && gray_sensor[7] == 0 ) {
        //         bias = 5; // 4左偏
        //     }
        //     else if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
        //         gray_sensor[3] == 1 && gray_sensor[4] == 0 && 
        //         gray_sensor[5] == 0 && gray_sensor[6] == 0 && gray_sensor[7] == 1 ) {
        //         bias = 5; // 4左偏
        //     }
        //     else if( gray_sensor[0] == 1 && gray_sensor[1] == 0 && gray_sensor[2] == 1 && 
        //         gray_sensor[3] == 1 && gray_sensor[4] == 0 && 
        //         gray_sensor[5] == 0 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
        //         bias = 5; // 4左偏
        //     }
        //     else if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
        //         gray_sensor[3] == 0 && gray_sensor[4] == 0 && 
        //         gray_sensor[5] == 0 && gray_sensor[6] == 0 && gray_sensor[7] == 1 ) {
        //         bias = 5; // 4左偏
        //     }
        //     if(bias != 0)
        //         return bias;
        // }

        if(task_3 == 2) {
            // if(gray_sensor[0] == 0 || gray_sensor[1] == 0 || 
            //    gray_sensor[2] == 0 || gray_sensor[3] == 0) {
            //     bias = 5;
            // }

            if( gray_sensor[0] == 0 && gray_sensor[1] == 0 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 0 ) {
                bias = 5; 
            }

            else if( gray_sensor[0] == 0 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 0 ) {
                bias = 5; 
            }

            else if( gray_sensor[0] == 0 || gray_sensor[1] == 0 ) {
                bias = 3;
            }
            
            if(bias != 0)
                return bias;
        }


        if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 0 && gray_sensor[4] == 0 && 
            gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
            bias = 0; // 无偏差
        }
        else if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 0 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
            bias = 0.5; // 右小偏
        }
        else if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 0 && 
            gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
            bias = -0.5; // 左小偏
        }
        else if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 0 && 
            gray_sensor[3] == 0 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
            bias = 1; // 右中偏
        }
        else if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 0 && 
            gray_sensor[5] == 0 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
            bias = -1; // 左中偏
        }
        else if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 0 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
            bias = 1.5; // 右中大偏
        }
        else if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 0 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
            bias = -1.5; // 左中大偏
        }

        else if( gray_sensor[0] == 1 && gray_sensor[1] == 0 && gray_sensor[2] == 0 && 
            gray_sensor[3] == 0 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
            bias = 2; // 3右偏
        }
        else if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 0 && 
            gray_sensor[5] == 0 && gray_sensor[6] == 0 && gray_sensor[7] == 1 ) {
            bias = -2; // 3左偏
        }


        else if( gray_sensor[0] == 1 && gray_sensor[1] == 0 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 0 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
            bias = 5; // 4右偏
        }
        else if( gray_sensor[0] == 1 && gray_sensor[1] == 0 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 0 && 
            gray_sensor[5] == 0 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
            bias = 5; // 4左偏
        }

        else if( gray_sensor[0] == 1 && gray_sensor[1] == 0 && gray_sensor[2] == 0 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
            bias = 2.5; // 右大偏
        }
        else if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 0 && gray_sensor[6] == 0 && gray_sensor[7] == 1 ) {
            bias = -2.5; // 左大偏
        }
        else if( gray_sensor[0] == 1 && gray_sensor[1] == 0 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
            bias = 3; // 右大大偏
        }
        else if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 1 && gray_sensor[6] == 0 && gray_sensor[7] == 1 ) {
            bias = -3; // 左大大偏
        }

        // else if( gray_sensor[0] == 1 && gray_sensor[1] == 0 && gray_sensor[2] == 1 && 
        //     gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
        //     gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
        //     bias = 3; // 右大大偏
        // }
        // else if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
        //     gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
        //     gray_sensor[5] == 1 && gray_sensor[6] == 0 && gray_sensor[7] == 1 ) {
        //     bias = -3; // 左大大偏
        // }

        // else if( gray_sensor[0] == 1 && gray_sensor[1] == 0 && gray_sensor[2] == 0 && 
        //     gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
        //     gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
        //     bias = 3.5; // 3右偏
        // }
        // else if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
        //     gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
        //     gray_sensor[5] == 0 && gray_sensor[6] == 0 && gray_sensor[7] == 1 ) {
        //     bias = -3.5; // 3左偏
        // }


        else if( gray_sensor[0] == 0 && gray_sensor[1] == 0 && gray_sensor[2] == 0 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
            bias = 3.5; // 2右偏
        }
        else if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 0 && gray_sensor[6] == 0 && gray_sensor[7] == 0 ) {
            bias = -3.5; // 2左偏
        }

        else if( gray_sensor[0] == 0 && gray_sensor[1] == 0 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
            bias = 4; // 完全右偏
        }
        else if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 1 && gray_sensor[6] == 0 && gray_sensor[7] == 0 ) {
            bias = -4; // 完全左偏
        }

        else if( gray_sensor[0] == 0 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 1 ) {
            bias = 4.5; // 1右偏
        }
        else if( gray_sensor[0] == 1 && gray_sensor[1] == 1 && gray_sensor[2] == 1 && 
            gray_sensor[3] == 1 && gray_sensor[4] == 1 && 
            gray_sensor[5] == 1 && gray_sensor[6] == 1 && gray_sensor[7] == 0 ) {
            bias = -4.5; // 1左偏
        }
        else {
            bias = 0; // 无偏差
        }
        bias = last_bias * 0.2 + bias * 0.8; //小插个值
        last_bias = bias;
    }
    
    return bias;
}



int8_t sw_i2c_read(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, __IO uint8_t *data, uint8_t data_length)
{
	uint8_t i;
	uint8_t ack_bit;

	/* 起始位 */
	sw_i2c_hal_start(i2c_interface);

	/* 地址+读写位 */
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_READ);
	if (ack_bit) {
		/* 从设备没有回复ACK,直接退出 */
		sw_i2c_hal_stop(i2c_interface);
		return 1;
	}

	/* 连续读取N-1个数据 给ACK */
	for (i = 0; i < data_length - 1; ++i) {
		data[i] = sw_i2c_hal_read_byte(i2c_interface, ACK);
	}

	/* 最后一个数据给 NACK */
	data[i] = sw_i2c_hal_read_byte(i2c_interface, NACK);

	/* 停止位 */
	sw_i2c_hal_stop(i2c_interface);
	return 0;
}


int8_t sw_i2c_write(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, const uint8_t *data, uint8_t data_length)
{
	uint8_t i;
	uint8_t ack_bit;

	/* 起始位 */
	sw_i2c_hal_start(i2c_interface);

	/* 地址+读写位 */
	ack_bit = sw_i2c_hal_write_byte(i2c_interface, dev_addr | I2C_WRITE);
	if (ack_bit) {
		/* 从设备没有回复ACK,直接退出 */
		sw_i2c_hal_stop(i2c_interface);
		return 1;
	}

	/* 连续写入N个数据, 每次读取1 bit的 ACK */
	for (i = 0; i < data_length; ++i) {
		 ack_bit = sw_i2c_hal_write_byte(i2c_interface, data[i]);
	}

	/* 停止位 */
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
 * 基礎操作抽象层
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
 * @brief 输出 sda 电平,然后 scl 输出一个时钟
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
 * @brief 读 sda 电平值,然后 scl 输出一个时钟
 * @param i2c_interface
 * @return 返回 SDA 电平值, 0:LOW, 1:HIGH
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
 * @brief 向IIC输出一个字节
 * @param i2c_interface
 * @param byte
 * @return 从设备反馈的 ACK 电平值
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
 * @brief 从IIC总线上读取一个字节
 * @param i2c_interface
 * @param ack 向从设备反馈 ACK 或者 NACK
 * @return 读取到的字节
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
