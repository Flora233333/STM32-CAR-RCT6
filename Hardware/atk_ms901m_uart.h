/**
 ****************************************************************************************************
 * @file        atk_ms901m_uart.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS901Mģ��UART�ӿ���������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� MiniSTM32 V4������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#ifndef __ATK_MS901M_UART_H
#define __ATK_MS901M_UART_H

//#include "./SYSTEM/sys/sys.h"
#include "stm32f10x.h"                  // Device header
#include <stdio.h>

/* ���Ŷ��� */

/*#define ATK_MS901M_UART_TX_GPIO_PORT            GPIOC
#define ATK_MS901M_UART_TX_GPIO_PIN             GPIO_PIN_12
#define ATK_MS901M_UART_TX_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0) 

#define ATK_MS901M_UART_RX_GPIO_PORT            GPIOD
#define ATK_MS901M_UART_RX_GPIO_PIN             GPIO_PIN_2
#define ATK_MS901M_UART_RX_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)  */ 

#define ATK_MS901M_UART_INTERFACE               UART5
#define ATK_MS901M_UART_IRQn                    UART5_IRQn
#define ATK_MS901M_UART_IRQHandler              UART5_IRQHandler
// #define ATK_MS901M_UART_CLK_ENABLE()            do{ __HAL_RCC_UART5_CLK_ENABLE(); }while(0) /* UART5 ʱ��ʹ�� */

/* UART����FIFO�����С */
#define ATK_MS901M_UART_RX_FIFO_BUF_SIZE        128

/* �������� */
uint8_t atk_ms901m_uart_rx_fifo_write(uint8_t *dat, uint16_t len);  /* ATK-MS901M UART����FIFOд������ */
uint16_t atk_ms901m_uart_rx_fifo_read(uint8_t *dat, uint16_t len);  /* ATK-MS901M UART����FIFO��ȡ���� */
void atk_ms901m_rx_fifo_flush(void);                                /* ATK-MS901M UART����FIFO��� */
void atk_ms901m_uart_send(uint8_t *dat, uint8_t len);               /* ATK-MS901M UART�������� */
void atk_ms901m_uart_init(uint32_t baudrate);                       /* ATK-MS901M UART��ʼ�� */

#endif
