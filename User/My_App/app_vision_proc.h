/**
 ******************************************************************************
 * @file    app_vision_proc.h
 * @author  lingxing
 * @brief   视觉识别处理任务 (物料/颜色识别)
 ******************************************************************************
 */

#ifndef __APP_VISION_PROC_H
#define __APP_VISION_PROC_H

#include <rtthread.h>

/**
 * @brief 视觉识别应用层数据结构
 */
typedef struct
{
    uint8_t target_id;     /* 目标 ID/类型 */
    uint16_t target_x;     /* 目标中心 X (0-320) */
    uint16_t target_y;     /* 目标中心 Y (0-240) */
    rt_bool_t is_found;    /* 是否找到目标 */
    rt_tick_t last_update; /* 最后更新系统时间 */
} App_Vision_Data_t;

extern volatile App_Vision_Data_t vision_app_data;
extern rt_mq_t vision_mq; /* 消息队列：对接 MaixCam 的异步解析中枢 */

/**
 * @brief  [API] 初始化视觉识别任务
 * @return 0: 成功, -1: 失败
 * @note   用于对接 MaixCam 等外部视觉计算单元。启动后会监听消息队列，异步更新目标物料 ID。
 */
int App_Vision_Init(void);

#endif /* __APP_VISION_PROC_H */
