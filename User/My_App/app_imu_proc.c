/**
 ******************************************************************************
 * @file    app_imu_proc.c
 * @author  lingxing
 * @brief   IMU (HWT101) 处理 App
 ******************************************************************************
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

            /* 3. 使用互斥锁保护共享数据更新
             *    临界区只做「赋值 + 盖章」，尽量短。
             *    move_proc(优先级 8) 高于本线程(10)，这里依赖互斥锁的
             *    优先级继承特性来避免优先级反转。 */
            rt_mutex_take(imu_data_mutex, RT_WAITING_FOREVER);
            imu_app_data.pitch = g_imu_data.pitch;
            imu_app_data.roll = g_imu_data.roll;
            imu_app_data.yaw = g_imu_data.yaw;
            imu_app_data.yaw_total = g_imu_data.yaw_continuous;
            imu_app_data.timestamp = rt_tick_get(); /* 盖章：本帧数据入应用的时刻 */
            imu_app_data.seq++;                     /* 更新序号自增，供消费者判断新旧 */
            rt_mutex_release(imu_data_mutex);
        }
    }
}

/**
 * @brief  [API] 原子地取出一份 IMU 一致快照
 * @note   这是消费者侧的唯一推荐入口，替代直接访问 imu_app_data。
 *         在临界区内做结构体整体拷贝，保证 4 个字段 + 时间戳同源；
 *         取出后再判断新鲜度，避免"持锁做业务判断"。
 */
rt_err_t App_IMU_GetData(App_IMU_Data_t *out, rt_tick_t max_age)
{
    rt_tick_t stamp;

    if (out == RT_NULL || imu_data_mutex == RT_NULL)
    {
        return -RT_ERROR;
    }

    /* 临界区：一次性整体拷贝，拿到自洽快照（不会出现新 yaw + 旧 yaw_total） */
    rt_mutex_take(imu_data_mutex, RT_WAITING_FOREVER);
    *out = imu_app_data;
    rt_mutex_release(imu_data_mutex);

    /* 无符号相减，天然正确处理 rt_tick 回绕 */
    stamp = out->timestamp;
    if (max_age != 0 && (rt_tick_get() - stamp) > max_age)
    {
        return -RT_ETIMEOUT; /* 数据过期：IMU 掉线或长时间无帧 */
    }

    return RT_EOK;
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
