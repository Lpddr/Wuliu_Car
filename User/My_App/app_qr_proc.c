/**
 ******************************************************************************
 * @file    app_qr_proc.c
 * @author  lingxing
 * @brief   二维码识别处理任务 (Pure MQ 模式)
 ******************************************************************************
 */

#include "app_qr_proc.h"
#include "app_task_proc.h"
#include "../My_Driver/bsp_uart.h"
#include <string.h>

#define QR_STACK_SIZE 1024
#define QR_PRIORITY 12
#define QR_TICK 5

/* 核心通信资源 */
rt_mq_t qr_mq = RT_NULL;            /* 内部同步：中断 -> 线程 */
rt_mq_t qr_result_mq = RT_NULL;     /* 任务直投：线程 -> 大脑 (Pure MQ) */
rt_mutex_t qr_data_mutex = RT_NULL; // 暂时保留句柄以防其他处引用，但不再初始化/使用

static rt_thread_t qr_thread = RT_NULL;

/**
 * @brief  二维码数据打包与投递 (Pure MQ 模式)
 */
static void QR_Parse(uint8_t *data, uint16_t len)
{
    if (len >= 3)
    {
        QR_Task_Msg_t msg;
        rt_memset(msg.content, 0, sizeof(msg.content));

        /* 数据脱壳：将原始字节拷贝入包裹 */
        rt_memcpy(msg.content, data, (len < 15) ? len : 15);

        /* 核心动作：投递包裹到大脑信箱 (不加锁，直接寄信) */
        if (rt_mq_send(qr_result_mq, &msg, sizeof(msg)) == RT_EOK)
        {
            /* 发送成功，大脑端的 mq_recv 会立刻被唤醒并拿到数据副本 */
        }
    }
}

/**
 * @brief 二维码处理线程入口
 */
static void qr_proc(void *parameter)
{
    while (1)
    {
        uint32_t rx_len;
        /* 1. 等待串口接收信号 */
        if (rt_mq_recv(qr_mq, &rx_len, sizeof(rx_len), RT_WAITING_FOREVER) == RT_EOK)
        {
            QR_Parse(uart1_qr.rx_buffer, (uint16_t)rx_len);
        }
    }
}

/**
 * @brief 初始化二维码识别任务
 */
int App_QR_Init(void)
{
    /* 1. 创建内部唤醒队列 */
    qr_mq = rt_mq_create("mq_qr", sizeof(uint32_t), 5, RT_IPC_FLAG_FIFO);

    /* 2. 创建大脑结果投递队列 (存2个包，循环覆盖) */
    qr_result_mq = rt_mq_create("mq_res", sizeof(QR_Task_Msg_t), 2, RT_IPC_FLAG_FIFO);

    /* 3. 创建处理线程 */
    qr_thread = rt_thread_create("qr_proc",
                                 qr_proc,
                                 RT_NULL,
                                 QR_STACK_SIZE,
                                 QR_PRIORITY,
                                 QR_TICK);

    if (qr_thread != RT_NULL && qr_mq != RT_NULL && qr_result_mq != RT_NULL)
    {
        rt_thread_startup(qr_thread);
        return 0;
    }
    return -1;
}

INIT_APP_EXPORT(App_QR_Init);
