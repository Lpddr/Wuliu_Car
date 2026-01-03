/**
 ******************************************************************************
 * @file    app_arm_proc.h
 * @author  lingxing
 * @brief   机械臂动作组应用层逻辑
 ******************************************************************************
 */

#ifndef __APP_ARM_PROC_H
#define __APP_ARM_PROC_H

#include "app_param.h"

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
void Arm_Pick_From_Raw(void);

/**
 * @brief  [API] 从地面抓取 (FLOOR - 用于转运)
 */
void Arm_Pick_From_Floor(void);

/**
 * @brief  [API] 放置到车内转盘 (CAR)
 * @param  tray_num: 1, 2, 3 号位
 */
void Arm_Place_To_Car(uint8_t tray_num);

/**
 * @brief  [API] 从车内转盘抓取 (CAR)
 * @param  tray_num: 1, 2, 3 号位
 */
void Arm_Pick_From_Car(uint8_t tray_num);

/**
 * @brief  [API] 放置到地面 (FLOOR)
 */
void Arm_Place_To_Floor(void);

/**
 * @brief  [API] 放置到码垛二层 (STACK)
 */
void Arm_Place_To_Stack(void);

/**
 * @brief  [API] 机械臂系统复位
 */
void Arm_Reset_Pos(void);

#endif /* __APP_ARM_PROC_H */
