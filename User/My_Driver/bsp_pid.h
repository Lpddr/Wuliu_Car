/**
 ******************************************************************************
 * @file    bsp_pid.h
 * @author  lingxing
 * @brief   高级 PID 控制算法底层驱动 (支持位置式/增量式)
 ******************************************************************************
 */

#ifndef __BSP_PID_H
#define __BSP_PID_H

#include "main.h"

/**
 * @usage 多路复用示例 :
 * 1. 声明多个实例 (相当于创建两个独立的脑子):
 *    PID_t pid_bal;   // 用于平衡
 *    PID_t pid_speed; // 用于速度
 *
 * 2. 分别初始化 (各自拥有独立的参数):
 *    BSP_PID_Init(&pid_bal,   12.0, 0, 1.5, 0, 10000); // 平衡参数
 *    BSP_PID_Init(&pid_speed, 5.0,  1, 0.5, 0, 5000);  // 速度参数
 *
 * 3. 独立计算:
 *    out1 = BSP_PID_CalcPositional(&pid_bal, ang);    // 这里用平衡的参数算
 *    out2 = BSP_PID_CalcIncremental(&pid_speed, rpm); // 这里用速度的参数算
 */

/* PID 控制器结构体 */
typedef struct
{
    /* 核心系数 */
    float kp;
    float ki;
    float kd;

    /* 目标与反馈 */
    float target;
    float current;

    /* 误差历史 */
    float error;
    float last_error;
    float last2_error; /* 用于增量式 PID */

    /* 状态与输出 */
    float integral;            /* 积分项累加 (用于位置式) */
    float p_out, i_out, d_out; /* 三项分量输出 (方便调试) */
    float output;
    float output_limit; /* 输出限幅 */
} PID_t;

/* --- 用户 API 接口 --- */

/**
 * @brief  PID 初始化
 * @param  kp, ki, kd: PID 系数
 * @param  target: 初始目标值
 * @param  limit: 输出限幅值
 */
void BSP_PID_Init(PID_t *pid, float kp, float ki, float kd, float target, float limit);

/**
 * @brief  设置控制参数
 */
void BSP_PID_SetTarget(PID_t *pid, float target);
void BSP_PID_SetParams(PID_t *pid, float kp, float ki, float kd);
void BSP_PID_SetLimit(PID_t *pid, float limit);
void BSP_PID_Reset(PID_t *pid);

/**
 * @brief  PID 计算核心 (位置式)
 * @param  current: 当前物理量测量值
 * @return 计算后的控制量输出
 */
float BSP_PID_CalcPositional(PID_t *pid, float current);

/**
 * @brief  PID 计算核心 (增量式)
 * @param  current: 当前物理量测量值
 * @return 计算后的控制量【增量】
 */
float BSP_PID_CalcIncremental(PID_t *pid, float current);

#endif /* __BSP_PID_H */
