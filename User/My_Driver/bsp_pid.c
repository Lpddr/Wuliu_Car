/**
 ******************************************************************************
 * @file    bsp_pid.c
 * @author  lingxing
 * @brief   高级 PID 控制算法底层驱动
 ******************************************************************************
 */

#ifndef NULL
#define NULL 0
#endif

#include "bsp_pid.h"

/* --- 内部私有函数 --- */
static void _BSP_PID_Limit(PID_t *pid);

/**
 * @brief  PID 参数初始化
 */
void BSP_PID_Init(PID_t *pid, float kp, float ki, float kd, float target, float limit)
{
    if (pid == NULL)
        return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = target;
    pid->output_limit = limit;

    BSP_PID_Reset(pid);
}

/**
 * @brief  动态设置目标值
 */
void BSP_PID_SetTarget(PID_t *pid, float target)
{
    if (pid)
        pid->target = target;
}

/**
 * @brief  动态设置 PID 参数
 */
void BSP_PID_SetParams(PID_t *pid, float kp, float ki, float kd)
{
    if (pid)
    {
        pid->kp = kp;
        pid->ki = ki;
        pid->kd = kd;
    }
}

/**
 * @brief  动态设置限幅
 */
void BSP_PID_SetLimit(PID_t *pid, float limit)
{
    if (pid)
        pid->output_limit = limit;
}

/**
 * @brief  位置式 PID 计算
 * @note   常用场景: 平衡偏角控制、舵机角度控制
 */
float BSP_PID_CalcPositional(PID_t *pid, float current)
{
    if (pid == NULL)
        return 0.0f;

    pid->current = current;
    pid->error = pid->target - pid->current;

    /* 1. 积分累加 */
    pid->integral += pid->error;

    /* 积分抗饱和处理 (限幅) */
    float i_limit = pid->output_limit * 0.8f; /* 默认积分项最大贡献 80% */
    if (pid->integral > i_limit)
        pid->integral = i_limit;
    if (pid->integral < -i_limit)
        pid->integral = -i_limit;

    /* 2. 计算三项分量 */
    pid->p_out = pid->kp * pid->error;
    pid->i_out = pid->ki * pid->integral;
    pid->d_out = pid->kd * (pid->error - pid->last_error);

    /* 3. 合成输出 */
    pid->output = pid->p_out + pid->i_out + pid->d_out;

    /* 4. 更新误差记录 */
    pid->last_error = pid->error;

    /* 5. 输出限幅 */
    _BSP_PID_Limit(pid);

    return pid->output;
}

/**
 * @brief  增量式 PID 计算
 * @note   常用场景: 电机转速控制、大惯性系统
 * @return 返回的是【输出增加量】，需加到原有的 PWM 指令上
 */
float BSP_PID_CalcIncremental(PID_t *pid, float current)
{
    if (pid == NULL)
        return 0.0f;

    pid->current = current;
    pid->error = pid->target - pid->current;

    /* 1. 增量公式: ΔOut = Kp*(e - e_last) + Ki*e + Kd*(e - 2*e_last + e_last2) */
    pid->p_out = pid->kp * (pid->error - pid->last_error);
    pid->i_out = pid->ki * pid->error;
    pid->d_out = pid->kd * (pid->error - 2.0f * pid->last_error + pid->last2_error);

    /* 2. 合成增量 */
    pid->output += (pid->p_out + pid->i_out + pid->d_out);

    /* 3. 更新误差历史 */
    pid->last2_error = pid->last_error;
    pid->last_error = pid->error;

    /* 4. 输出限幅 */
    _BSP_PID_Limit(pid);

    return pid->output;
}

/**
 * @brief  复位 PID 数据
 */
void BSP_PID_Reset(PID_t *pid)
{
    if (pid == NULL)
        return;

    pid->current = 0;
    pid->error = 0;
    pid->last_error = 0;
    pid->last2_error = 0;
    pid->integral = 0;
    pid->p_out = pid->i_out = pid->d_out = 0;
    pid->output = 0;
}

/**
 * @brief  私有限幅处理
 */
static void _BSP_PID_Limit(PID_t *pid)
{
    if (pid->output > pid->output_limit)
        pid->output = pid->output_limit;
    if (pid->output < -pid->output_limit)
        pid->output = -pid->output_limit;
}
