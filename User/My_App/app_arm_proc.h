/**
 ******************************************************************************
 * @file    app_arm_proc.h
 * @author  lingxing
 * @brief   机械臂动作组应用层逻辑
 ******************************************************************************
 */

#ifndef __APP_ARM_PROC_H
#define __APP_ARM_PROC_H

#include <rtthread.h>
#include "app_param.h"

/* 机械臂单个动作允许的最长时间；应根据实机最慢动作继续标定。 */
#define ARM_ACTION_TIMEOUT_MS 30000U

/**
 * @brief 机械臂高度枚举位 (统一命名规范)
 */
typedef enum
{
    HEIGHT_PLATE = 0, /* 原料区货架 (PLATE) */
    HEIGHT_CAR,       /* 车内转盘 (CAR) */
    HEIGHT_FLOOR,     /* 地面/加工区/暂存区一层 (FLOOR) */
    HEIGHT_STACK,     /* 码垛二层 (STACK) */
    HEIGHT_HOME       /* 安全抬升高度 */
} Arm_Height_t;

/**
 * @brief  [API] 从原料区抓取 (PLATE)
 */
rt_err_t Arm_Pick_From_Raw(void);

/**
 * @brief  [API] 从地面抓取 (FLOOR - 用于转运)
 */
rt_err_t Arm_Pick_From_Floor(void);

/**
 * @brief  [API] 放置到车内转盘 (CAR)
 * @param  tray_num: 1, 2, 3 号位
 */
rt_err_t Arm_Place_To_Car(uint8_t tray_num);

/**
 * @brief  [API] 从车内转盘抓取 (CAR)
 * @param  tray_num: 1, 2, 3 号位
 */
rt_err_t Arm_Pick_From_Car(uint8_t tray_num);

/**
 * @brief  [API] 放置到地面 (FLOOR)
 */
rt_err_t Arm_Place_To_Floor(void);

/**
 * @brief  [API] 放置到码垛二层 (STACK)
 */
rt_err_t Arm_Place_To_Stack(void);

/**
 * @brief  [API] 视觉定位前调整机械臂安全姿态
 */
rt_err_t Arm_Prepare(void);

/**
 * @brief  [API] 完赛时将机械臂底座收回车体中心
 */
rt_err_t Arm_Return_Center(void);

/**
 * @brief  [API] 机械臂系统复位
 */
rt_err_t Arm_Reset_Pos(void);

/**
 * @brief  [API] 查询机械臂任务是否健康
 * @note   空闲时视为健康；忙碌超过 ARM_ACTION_TIMEOUT_MS 或执行失败时返回 RT_FALSE。
 */
rt_bool_t App_Arm_IsHealthy(void);

/**
 * @brief  [API] 机械臂紧急停止并锁存故障
 */
void App_Arm_EmergencyStop(void);

/**
 * @brief  初始化机械臂消息队列和独立线程
 */
int App_Arm_Init(void);

#endif /* __APP_ARM_PROC_H */
