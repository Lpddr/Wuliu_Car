/**
 ******************************************************************************
 * @file    bsp_uart.c
 * @author  lingxing
 * @brief   串口底层驱动 (DMA+空闲中断方式)
 ******************************************************************************
 */

#include "bsp_uart.h"
#include <rtthread.h>
#include "../../cubemx/Inc/main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "../Components/imu_wit.h"
#include "../My_App/app_imu_proc.h"
#include "../My_App/app_vision_proc.h"
#include "../My_App/app_qr_proc.h"

#define DBG_TAG "bsp.uart"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/*
 * 定义串口句柄，防止编译优化问题
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
    if (uart == RT_NULL || uart->huart == RT_NULL)
    {
        return;
    }

    /* 开启空闲中断 */
    __HAL_UART_ENABLE_IT(uart->huart, UART_IT_IDLE);

    /* 开启 DMA 循环接收 */
    HAL_UART_Receive_DMA(uart->huart, uart->rx_buffer, UART_RX_BUF_SIZE);

    /* CubeMX 生成的 HAL_UART_MspInit() 只给 USART1 / USART3 / USART6 使能了 NVIC，
     * USART2(IMU) 分支漏了 HAL_NVIC_EnableIRQ(USART2_IRQn)，导致空闲中断永远进不来。
     * 放在这里而不是 MspInit 里，是为了避免被 CubeMX 重新生成代码时覆盖掉。 */
    if (uart->huart->Instance == USART2)
    {
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
}

/**
 * @brief  一次性初始化全部三路串口（自动初始化入口）
 * @note   必须晚于 rt_hw_board_init() 中的 MX_USARTx_UART_Init()，
 *         因为 HAL_UART_Receive_DMA() 依赖 MspInit 里 __HAL_LINKDMA 绑定的 hdmarx。
 */
int BSP_UART_InitAll(void)
{
    BSP_UART_Init(&uart1_qr);     /* USART1 - 二维码识别摄像头 */
    BSP_UART_Init(&uart2_imu);    /* USART2 - IMU (HWT101)   */
    BSP_UART_Init(&uart6_vision); /* USART6 - 物料识别摄像头  */

    LOG_I("UART DMA+IDLE RX started on USART1 / USART2 / USART6.");
    return RT_EOK;
}

/* 导出为设备级自动初始化：原代码里 BSP_UART_Init() 从未被任何地方调用，
 * 三路串口的空闲中断与 DMA 接收实际上从未启动过。 */
INIT_DEVICE_EXPORT(BSP_UART_InitAll);

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

        /* 4. 发送消息到队列（生产者模式）
         *    本函数运行在中断上下文。BSP_UART_InitAll() 属于 INIT_DEVICE_EXPORT，
         *    早于 App_IMU_Init / App_QR_Init / App_Vision_Init 的 INIT_APP_EXPORT，
         *    也就是说 DMA 接收刚开启时，三个队列可能还没有被创建 —— 必须判空，
         *    否则 rt_mq_send(NULL,...) 在 RT_DEBUG 关闭时会直接野指针 HardFault。 */
        uint32_t msg = uart->rx_len; /* 携带本次接收的数据长度 */
        if (uart->huart == &huart2)
        {
            if (imu_mq != RT_NULL)
            {
                rt_mq_send(imu_mq, &msg, sizeof(msg));
            }
        }
        else if (uart->huart == &huart1)
        {
            if (qr_mq != RT_NULL)
            {
                rt_mq_send(qr_mq, &msg, sizeof(msg)); /* 二维码数据就绪 */
            }
        }
        else if (uart->huart == &huart6)
        {
            if (vision_mq != RT_NULL)
            {
                rt_mq_send(vision_mq, &msg, sizeof(msg));
            }
        }

        /* 5. 设置标志位给应用层 */
        uart->rx_flag = 1;

        /* 6. 重新开启 DMA 接收 */
        HAL_UART_Receive_DMA(uart->huart, uart->rx_buffer, UART_RX_BUF_SIZE);
    }
}
