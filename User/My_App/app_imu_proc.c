/**
 ******************************************************************************
 * @file    app_imu_proc.c
 * @author  lingxing
 * @brief   IMU (HWT101) 处理 App
 ******************************************************************************
 */

*/

#include "app_imu_proc.h"
#include "../Components/imu_wit.h"
#include "../My_Driver/bsp_uart.h"

#define IMU_STACK_SIZE 2048
#define IMU_PRIORITY 10
#define IMU_TICK 5

App_IMU_Data_t imu_app_data = {0}; /* 全局共享姿态数据 */
rt_mq_t imu_mq = RT_NULL;              /* imu 队列 */
rt_mutex_t imu_data_mutex = RT_NULL;   /* imu 互斥锁 */

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
        /* 1. 等待消息队列 ：只有串口收完一帧数据，此线程才会被唤醒 (消费者模式)*/
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
    /* 创建通信对象 */
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
