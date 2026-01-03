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
 */
typedef struct
{
    float pitch;     /* 俯仰角 */
    float roll;      /* 横滚角 */
    float yaw;       /* 相对航向角 (归零后) */
    float yaw_total; /* 连续航向角 (不归零) */
} App_IMU_Data_t;

extern App_IMU_Data_t imu_app_data;
extern rt_mq_t imu_mq;            /* imu 队列 */
extern rt_mutex_t imu_data_mutex; /* imu 互斥锁 */

/**
 * @brief  [API] 初始化 IMU 处理任务
 * @return 0: 成功, -1: 失败
 * @note   调用后会启动 IMU 采样线程，并通过消息队列异步更新全局变量 imu_app_data。
 */
int App_IMU_Init(void);

#endif /* __APP_IMU_H */
