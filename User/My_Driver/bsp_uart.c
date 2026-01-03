/**
 ******************************************************************************
 * @file    bsp_uart.c
 * @author  lingxing
 * @brief   串口底层驱动 (DMA+空闲中断方式)
 ******************************************************************************
 */

#include "bsp_uart.h"
#include "../../cubemx/Inc/main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "../Components/imu_wit.h"
#include "../My_App/app_imu_proc.h"
#include "../My_App/app_vision_proc.h"
#include "../My_App/app_qr_proc.h"

/*
 * [保姆级修复]:
 * 由于 RT-Thread Studio 编译 CubeMX 文件夹时可能会漏掉变量定义，
 * 我们在这里强行手动定义这三个串口句柄，确保链接器能找到它们。
 */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* 实例化串口 1 (二维码) */
UART_t uart1_qr = {
    .huart = &huart1,
    .rx_flag = 0,
    .rx_len = 0};

/* 实例化串口 2 (IMU) */
UART_t uart2_imu = {
    .huart = &huart2,
    .rx_flag = 0,
    .rx_len = 0};

/* 实例化串口 6 (物料识别) */
UART_t uart6_vision = {
    .huart = &huart6,
    .rx_flag = 0,
    .rx_len = 0};

/**
 * @brief  初始化串口 DMA 接收及空闲中断
 */
void BSP_UART_Init(UART_t *uart)
{
    /* 开启空闲中断 */
    __HAL_UART_ENABLE_IT(uart->huart, UART_IT_IDLE);

    /* 开启 DMA 循环接收 */
    HAL_UART_Receive_DMA(uart->huart, uart->rx_buffer, UART_RX_BUF_SIZE);
}

/**
 * @brief  串口发送数据
 */
void BSP_UART_Send(UART_t *uart, uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(uart->huart, data, len, 100);
}

/**
 * @brief  格式化打印
 */
void BSP_UART_printf(UART_t *uart, const char *format, ...)
{
    va_list args;
    static char buf[256];
    uint16_t len;

    va_start(args, format);
    len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    BSP_UART_Send(uart, (uint8_t *)buf, len);
}

/**
 * @brief  串口空闲中断回调
 */
void BSP_UART_IdleCallback(UART_t *uart)
{
    if (__HAL_UART_GET_FLAG(uart->huart, UART_FLAG_IDLE) != RESET)
    {
        /* 1. 清除空闲中断标志 (HAL 要求的特定序列：读状态再读数据) */
        __HAL_UART_CLEAR_IDLEFLAG(uart->huart);

        /* 2. 停止 DMA 接收，计算长度 */
        HAL_UART_DMAStop(uart->huart);

        /* 3. 计算接收到的字节数 = 总长度 - 剩余传输计数 */
        uart->rx_len = UART_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(uart->huart->hdmarx);

        /* 4. 【核心路由】通过消息队列通知 App 线程 (生产者模式) */
        uint32_t msg = uart->rx_len; /* 携带本次接收的数据长度 */
        if (uart->huart == &huart2)
        {
            rt_mq_send(imu_mq, &msg, sizeof(msg));
        }
        else if (uart->huart == &huart1)
        {
            rt_mq_send(qr_mq, &msg, sizeof(msg)); /* 二维码数据就绪 */
        }
        else if (uart->huart == &huart6)
        {
            rt_mq_send(vision_mq, &msg, sizeof(msg));
        }

        /* 5. 设置标志位给应用层 */
        uart->rx_flag = 1;

        /* 5. 重新开启 DMA 接收 */
        HAL_UART_Receive_DMA(uart->huart, uart->rx_buffer, UART_RX_BUF_SIZE);
    }
}
