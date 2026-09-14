/**
 ******************************************************************************
 * @file    app_imu_proc.h
 * @author  lingxing
 * @brief   IMU 数据处理任务
 ******************************************************************************
 */

#ifndef __APP_IMU_H
#define __APP_IMU_H

#include <rtthread.h>

/**
 * @brief IMU 应用层数据结构
 *
 * @note 这是「状态型」共享数据（永远只有最新一份姿态），不是「队列型」。
 *       消费者关心的是「这个值有多新」，而不是「积压了多少条」，所以这里
 *       用 时间戳 + 更新序号 来表达新鲜度；不用 empty/full 信号量配对——
 *       那会变成队列语义，IMU 100~200Hz 产、move_proc 50Hz 消，生产者很快
 *       就会在 empty 信号量上被背压卡住，进而把 imu_mq 撑满、整帧丢弃。
 */
typedef struct
{
    float pitch;     /* 俯仰角 */
    float roll;      /* 横滚角 */
    float yaw;       /* 相对航向角 (归零后) */
    float yaw_total; /* 连续航向角 (不归零) */

    rt_tick_t timestamp; /* 生产者盖章：本帧数据的产生时刻 (rt_tick_get) */
    rt_uint32_t seq;     /* 更新序号，只增不减；消费者据此判断「是否拿到新数据」 */
} App_IMU_Data_t;

/*
 * IMU 数据有效期 (ms)。
 * 超过该时长仍未更新即判定 IMU 掉线 / 链路异常。
 * 取值原则：≥ 5 倍 IMU 输出周期。HWT101 典型 100Hz → 10ms/帧，
 * 这里放到 100ms，既能容忍偶发丢帧，又能在真正掉线时快速发现。
 * 注意：move_proc 控制周期是 20ms，即 5 个控制周期内必须见到新数据。
 */
#define IMU_DATA_MAX_AGE_MS 100

extern App_IMU_Data_t imu_app_data;
extern rt_mq_t imu_mq;            /* imu 队列 */
extern rt_mutex_t imu_data_mutex; /* imu 互斥锁 */

/**
 * @brief  [API] 初始化 IMU 处理任务
 * @return 0: 成功, -1: 失败
 * @note   调用后会启动 IMU 采样线程，并通过消息队列异步更新全局变量 imu_app_data。
 */
int App_IMU_Init(void);

/**
 * @brief  [API] 原子地取出一份 IMU 一致快照（消费者侧唯一推荐入口）
 *
 * @param  out:     输出缓冲。函数内部在临界区里做「结构体整体拷贝」，
 *                  因此 yaw / yaw_total / timestamp 一定来自同一次更新，
 *                  不会出现「读到新 yaw、旧 yaw_total」的混合态。
 * @param  max_age: 允许的最大数据龄期 (tick)。传 0 表示跳过新鲜度检查。
 *
 * @return RT_EOK       数据有效且新鲜
 *         -RT_ETIMEOUT 数据已过期（IMU 掉线 / 长时间无帧）
 *         -RT_ERROR    入参或互斥锁对象非法
 *
 * @note   典型用法：
 *             App_IMU_Data_t imu;
 *             if (App_IMU_GetData(&imu, rt_tick_from_millisecond(IMU_DATA_MAX_AGE_MS)) == RT_EOK)
 *                 use(imu.yaw);
 *             else
 *                 emergency_stop();
 */
rt_err_t App_IMU_GetData(App_IMU_Data_t *out, rt_tick_t max_age);

#endif /* __APP_IMU_H */
