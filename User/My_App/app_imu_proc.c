/**
 * @file    app_imu.c
 * @brief   IMU (HWT101) 处理 App
 *
 * [专业架构思路]:
 * 1. 异步化：由串口中断通过信号量唤醒，避开 while(1) 盲目轮询造成的资源消费。
 * 2. 独立化：采样与解析独立成线程，确保姿态数据不因业务繁重而丢失或跳变。
 *
 */

#include "app_imu_proc.h"
#include "../Components/imu_wit.h"
#include "../My_Driver/bsp_uart.h"

#define IMU_STACK_SIZE 2048
#define IMU_PRIORITY 10
#define IMU_TICK 5

App_IMU_Data_t imu_app_data = {0};   /* 全局共享姿态数据 */
rt_mq_t imu_mq = RT_NULL;            /* 消息队列：数据解耦的中枢 */
rt_mutex_t imu_data_mutex = RT_NULL; /* 互斥锁：保护全局姿态数据 */

static rt_thread_t imu_thread = RT_NULL;

/**
 * @brief  IMU 处理线程入口 (Proc)
 */
static void imu_proc(void *parameter)
{
    IMU_Init(); /* 组件层初始化 */

    while (1)
    {
        uint32_t rx_len;
        /* 1. 等待消息队列：只有串口收完一帧数据，此线程才会被唤醒 (消费者模式) */
        if (rt_mq_recv(imu_mq, &rx_len, sizeof(rx_len), RT_WAITING_FOREVER) == RT_EOK)
        {
            /* 2. 在线程环境中执行复杂的包解析 */
            IMU_ParsePacket(uart2_imu.rx_buffer, (uint16_t)rx_len);

            /* 3. 使用互斥锁保护共享数据更新 */
            rt_mutex_take(imu_data_mutex, RT_WAITING_FOREVER);
            imu_app_data.yaw = g_imu_data.yaw;
            imu_app_data.yaw_total = g_imu_data.yaw_continuous;
            rt_mutex_release(imu_data_mutex);
        }
    }
}

/**
 * @brief  初始化 IMU 任务 (Init)
 */
int App_IMU_Init(void)
{
    /* [避坑]: 必须先创建通信对象 (MQ + Mutex) */
    imu_mq = rt_mq_create("mq_imu", sizeof(uint32_t), 10, RT_IPC_FLAG_FIFO);
    imu_data_mutex = rt_mutex_create("mux_imu", RT_IPC_FLAG_FIFO);

    imu_thread = rt_thread_create("imu_proc",
                                  imu_proc,
                                  RT_NULL,
                                  IMU_STACK_SIZE,
                                  IMU_PRIORITY,
                                  IMU_TICK);

    if (imu_thread != RT_NULL && imu_mq != RT_NULL && imu_data_mutex != RT_NULL)
    {
        rt_thread_startup(imu_thread);
        return 0;
    }

    return -1;
}

/* 自动化启动：系统启动时自动调用 App_IMU_Init */
INIT_APP_EXPORT(App_IMU_Init);
