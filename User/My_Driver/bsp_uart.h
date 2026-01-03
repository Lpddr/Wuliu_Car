/**
 ******************************************************************************
 * @file    bsp_uart.h
 * @author  lingxing
 * @brief   串口底层驱动 (DMA+空闲中断方式)
 ******************************************************************************
 */

#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"

/**
 * @usage 使用说明:
 * 1. 初始化: 调用 BSP_UART_Init(&uart2_imu)
 * 2. 发送:   调用 BSP_UART_Send(&uart2_imu, data, len)
 * 3. 接收:   数据自动进入 rx_buffer，通过 rx_flag 判断新数据
 */

#define UART_RX_BUF_SIZE 128

/* 串口控制结构体 */
typedef struct
{
    UART_HandleTypeDef *huart; /* HAL 串口句柄 */

    uint8_t rx_buffer[UART_RX_BUF_SIZE]; /* 接收缓冲区 */
    uint16_t rx_len;                     /* 最近一次接收长度 */
    uint8_t rx_flag;                     /* 接收完成标志 */
} UART_t;

/* 声明外部可用串口实例 */
extern UART_t uart1_qr;     /* 串口 1: 二维码识别摄像头 */
extern UART_t uart2_imu;    /* 串口 2: IMU 陀螺仪 */
extern UART_t uart6_vision; /* 串口 6: 物料识别摄像头 */

/* 函数接口 */
void BSP_UART_Init(UART_t *uart);
void BSP_UART_Send(UART_t *uart, uint8_t *data, uint16_t len);
void BSP_UART_printf(UART_t *uart, const char *format, ...);

#endif /* __BSP_UART_H */
