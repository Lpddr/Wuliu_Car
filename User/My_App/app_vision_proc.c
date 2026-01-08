/**
 ******************************************************************************
 * @file    app_vision_proc.c
 * @author  lingxing
 * @brief   视觉识别处理任务 (物料/颜色识别)
 ******************************************************************************
 */

#include "app_vision_proc.h"
#include "../My_Driver/bsp_uart.h"
#include <string.h>

#define VISION_STACK_SIZE 1024
#define VISION_PRIORITY 11
#define VISION_TICK 5

volatile App_Vision_Data_t vision_app_data = {0};
rt_mq_t vision_mq = RT_NULL;

static rt_thread_t vision_thread = RT_NULL;

/**
 * @brief  视觉数据解析
 * @format 报文格式：'a' + ID + XXX + YYY + 'c'
 * @param  data: 原始缓冲区
 * @param  len: 数据长度
 */
static void Vision_Parse(uint8_t *data, uint16_t len)
{
    /* 遍历查找帧头 'a' */
    for (int i = 0; i < len; i++)
    {
        /* 检查剩余长度是否足够包含一个完整帧 (至少 9 字节: a + ID + 3位X + 3位Y + c) */
        if (data[i] == 'a' && (i + 8) < len)
        {
            if (data[i + 8] == 'c')
            {
                /* 提取 ID 并将数字字符转换为数值 */
                uint8_t id = data[i + 1] - '0';

                /* 解析 X 坐标 (3位数字) */
                uint16_t x = (data[i + 2] - '0') * 100 +
                             (data[i + 3] - '0') * 10 +
                             (data[i + 4] - '0');

                /* 解析 Y 坐标 (3位数字) */
                uint16_t y = (data[i + 5] - '0') * 100 +
                             (data[i + 6] - '0') * 10 +
                             (data[i + 7] - '0');
                /*在 32位系统下读取 16位数据是原子的，且闭环控制允许微小误差，故移除互斥锁以提升性能 */
                vision_app_data.target_id = id;
                vision_app_data.target_x = x;
                vision_app_data.target_y = y;
                vision_app_data.is_found = RT_TRUE;
                vision_app_data.last_update = rt_tick_get();

                // rt_kprintf("[Vision] Found ID:%d at (%d, %d)\n", id, x, y);
                return;
            }
        }
    }
}

/**
 * @brief 视觉处理线程入口
 */
static void vision_proc(void *parameter)
{
    while (1)
    {
        uint32_t rx_len;
        /* 等待串口接收完成信号 (生产者-消费者模型) */
        if (rt_mq_recv(vision_mq, &rx_len, sizeof(rx_len), RT_WAITING_FOREVER) == RT_EOK)
        {
            Vision_Parse(uart6_vision.rx_buffer, (uint16_t)rx_len);
        }
    }
}

/**
 * @brief 初始化视觉识别
 */
int App_Vision_Init(void)
{
    // 创建一个名字叫 "mq_vis" 的消息队列
    vision_mq = rt_mq_create("mq_vis", sizeof(uint32_t), 10, RT_IPC_FLAG_FIFO);

    vision_thread = rt_thread_create("vision_proc",
                                     vision_proc,
                                     RT_NULL,
                                     VISION_STACK_SIZE,
                                     VISION_PRIORITY,
                                     VISION_TICK);

    if (vision_thread != RT_NULL && vision_mq != RT_NULL)
    {
        rt_thread_startup(vision_thread);
        return 0;
    }
    return -1;
}

INIT_APP_EXPORT(App_Vision_Init);
