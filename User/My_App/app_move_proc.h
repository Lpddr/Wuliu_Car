/**
 ******************************************************************************
 * @file    app_move_proc.h
 * @author  lingxing
 * @brief   运动控制层 (肌肉) - 接口定义
 ******************************************************************************
 */

#ifndef __APP_MOVE_PROC_H
#define __APP_MOVE_PROC_H

#include <rtthread.h>

/**
 * @brief 初始化运动控制
 */
/** 运动模式枚举 (下位机底层支柱) */
typedef enum
{
    MOVE_STOP = 0,    /* 强制停止 */
    MOVE_FORWARD,     /* 前进 (纠偏模式) */
    MOVE_BACKWARD,    /* 后退 (纠偏模式) */
    MOVE_SLIDE_LEFT,  /* 左平移 */
    MOVE_SLIDE_RIGHT, /* 右平移 */
    MOVE_TURN_LEFT,   /* 原地左转 (开环速度控制) */
    MOVE_TURN_RIGHT,  /* 原地右转 (开环速度控制) */
    MOVE_TURN_ABS     /* 绝对角度旋转 (PID 闭环控制) */
} Move_Mode_t;

/**
 * @brief  [API] 全方向移动控制接口
 * @param  mode: 运动模式枚举 (@see Move_Mode_t)。
 *               前进: MOVE_FORWARD, 后退: MOVE_BACKWARD,
 *               左平移: MOVE_SLIDE_LEFT, 右平移: MOVE_SLIDE_RIGHT
 * @param  speed_mm_s: 期望运行的目标速度。单位：mm/s (建议范围: 50~350)。
 * @param  distance_mm: 计划运行的位移距离。单位：mm。
 *                     - 若传入 500.0f : 小车走 50cm 后利用 T 型曲线自动刹停。
 *                     - 若传入 0.0f   : 小车进入巡航模式，持续行驶。
 * @note   线程安全：本函数会把 mode/speed/pulse/yaw 四个字段连同里程计复位
 *         作为一次原子发布（内部互斥锁保护），供 move_proc 线程整组采样。
 *         因此可以在任意线程上下文调用，不会让控制周期看到"半更新"的命令。
 */
void Move_Now(Move_Mode_t mode, float speed_mm_s, float distance_mm);

/**
 * @brief  [API] 精准绝对角度旋转接口
 * @param  abs_angle: 目标绝对角度。范围：0.0 ~ 360.0 (度)。
 *                   - 0.0 : 发车时的初始正方向
 *                   - 90.0: 向左转 90 度
 *                   - 270.0 (或 -90.0): 向右转 90 度
 * @note   线程安全：命令字段整组原子发布（内部互斥锁保护）。
 */
void Move_Turn_Abs(float abs_angle);

/**
 * @brief  [API] 紧急主动停止
 * @note   强制清空所有运动状态，清空里程计目标，使底盘电机制动停止。
 * @note   线程安全：本函数既会被 brain 线程调用（视觉超时保护），也会被
 *         move_proc 线程自己调用（到位停车 / IMU 掉线停车），因此命令字段
 *         的写入加了互斥锁；电机停机在锁外执行。
 */
void Move_Stop(void);

#endif /* __APP_MOVE_PROC_H */
