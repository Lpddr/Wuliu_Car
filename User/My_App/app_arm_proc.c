/**
 * @file    app_arm_proc.c
 * @brief   机械臂动作组实现 (小脑层：组合动作)
 */

#include "app_arm_proc.h"
#include "app_task_proc.h"
#include "../My_Driver/bsp_servo.h"
#include "../My_Driver/bsp_motor.h"

#define DBG_TAG "app.arm"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>
#include <stdlib.h>

/*
 * =======================================================================
 * 机械臂下降距离参数 (用户可调 - 脉冲数)
 * =======================================================================
 * 这里的数值代表从最高点(HOME)向下运行的距离
 */
#define DIST_PLATE 8000  /* 到原料区(货架)下降距离 */
#define DIST_CAR 4000    /* 到车内转盘下降距离 */
#define DIST_STACK 10000 /* 到暂存区二层(码垛)下降距离 */
#define DIST_FLOOR 15000 /* 到地面(加工区/暂存一层)下降距离 */

/* 爪子角度参数已移至 app_param.h 统一管理 */

extern Motor_t motor_5; /* 升降步进电机 */

/**
 * @brief  [Internal] 控制升降电机运行指定距离 (脉冲)
 * @param  dist: 移动距离 (脉冲数)
 * @param  is_down: 1 为向下, 0 为向上
 */
static void Arm_Move_Dist(int32_t dist, uint8_t is_down)
{
    if (dist <= 0)
        return;

    /* 重置步数统计，实现相对移动 */
    BSP_Motor_ResetSteps(&motor_5);

    /* 启动电机：正向速度向下，反向速度向上 */
    BSP_Motor_SetSpeed(&motor_5, is_down ? 5000 : -5000);

    /* 等待走完指定脉冲数 */
    while (abs(BSP_Motor_GetSteps(&motor_5)) < dist)
    {
        rt_thread_mdelay(10);
    }

    BSP_Motor_Stop(&motor_5);
}

/**
 * @brief  从原料区抓取 (PLATE)
 */
void Arm_Pick_From_Raw(void)
{
    LOG_I("Action: [Raw] Picking...");
    Servo_SetAngle(SERVO_BASE, 0); // 面向原料区
    Servo_SetAngle(SERVO_ARM, CLAW_OPEN);
    rt_thread_mdelay(300);

    Arm_Move_Dist(DIST_PLATE, 1); // 下降
    rt_thread_mdelay(200);

    Servo_SetAngle(SERVO_ARM, CLAW_CLOSE);
    rt_thread_mdelay(500);

    Arm_Move_Dist(DIST_PLATE, 0); // 原路返回最高点
    rt_event_send(&mission_event, EV_ARM_FINISHED);
}

/**
 * @brief  放置到车内 (CAR)
 */
void Arm_Place_To_Car(uint8_t tray_num)
{
    float tray_angles[] = {0.0, PLATE_RED, PLATE_GREEN, PLATE_BLUE};
    float target_angle = (tray_num <= 3) ? tray_angles[tray_num] : 17.0;

    LOG_I("Action: [Car] Placing to Tray %d...", tray_num);
    Servo_SetAngle(SERVO_BASE, 98); // 面向车内
    Servo_SetAngle(SERVO_PLATE, target_angle);
    rt_thread_mdelay(500);

    Arm_Move_Dist(DIST_CAR, 1); // 下降
    rt_thread_mdelay(200);

    Servo_SetAngle(SERVO_ARM, CLAW_OPEN);
    rt_thread_mdelay(400);

    Arm_Move_Dist(DIST_CAR, 0); // 返回最高点
    rt_event_send(&mission_event, EV_ARM_FINISHED);
}

/**
 * @brief  从车内转盘抓取 (CAR)
 */
void Arm_Pick_From_Car(uint8_t tray_num)
{
    float tray_angles[] = {0.0, PLATE_RED, PLATE_GREEN, PLATE_BLUE};
    float target_angle = (tray_num <= 3) ? tray_angles[tray_num] : 17.0;

    // LOG_I("Action: [Car] Picking from Tray %d...", tray_num);
    Servo_SetAngle(SERVO_BASE, 98); // 面向车内
    Servo_SetAngle(SERVO_PLATE, target_angle);
    Servo_SetAngle(SERVO_ARM, CLAW_OPEN);
    rt_thread_mdelay(500);

    Arm_Move_Dist(DIST_CAR, 1); // 下降
    rt_thread_mdelay(200);

    Servo_SetAngle(SERVO_ARM, CLAW_CLOSE);
    rt_thread_mdelay(500);

    Arm_Move_Dist(DIST_CAR, 0); // 返回最高点
    rt_event_send(&mission_event, EV_ARM_FINISHED);
}

/**
 * @brief  放置到地面 (FLOOR)
 */
void Arm_Place_To_Floor(void)
{
    LOG_I("Action: [Floor] Unloading...");
    Servo_SetAngle(SERVO_BASE, 0); // 回归正前方 (对标原厂 0 度)
    rt_thread_mdelay(400);

    Arm_Move_Dist(DIST_FLOOR, 1); // 下降
    rt_thread_mdelay(200);

    Servo_SetAngle(SERVO_ARM, CLAW_OPEN);
    rt_thread_mdelay(400);

    Arm_Move_Dist(DIST_FLOOR, 0); // 返回最高点
    rt_event_send(&mission_event, EV_ARM_FINISHED);
}

/**
 * @brief  从地面抓取 (FLOOR)
 */
void Arm_Pick_From_Floor(void)
{
    LOG_I("Action: [Floor] Picking from Ground...");
    Servo_SetAngle(SERVO_BASE, 0); // 确保面向前方
    Servo_SetAngle(SERVO_ARM, CLAW_OPEN);
    rt_thread_mdelay(300);

    Arm_Move_Dist(DIST_FLOOR, 1); // 下降
    rt_thread_mdelay(200);

    Servo_SetAngle(SERVO_ARM, CLAW_CLOSE);
    rt_thread_mdelay(500);

    Arm_Move_Dist(DIST_FLOOR, 0); // 返回最高点
    rt_event_send(&mission_event, EV_ARM_FINISHED);
}

/**
 * @brief  放置到二层码垛 (STACK)
 */
void Arm_Place_To_Stack(void)
{
    LOG_I("Action: [Stack] Stacking...");
    Servo_SetAngle(SERVO_BASE, 0); // 面向前方码放区
    rt_thread_mdelay(400);

    Arm_Move_Dist(DIST_STACK, 1); // 下降到二层高度
    rt_thread_mdelay(200);

    Servo_SetAngle(SERVO_ARM, CLAW_OPEN);
    rt_thread_mdelay(400);

    Arm_Move_Dist(DIST_STACK, 0); // 返回最高点
    rt_event_send(&mission_event, EV_ARM_FINISHED);
}

/**
 * @brief  一键复位至最高点安全位
 */
void Arm_Reset_Pos(void)
{
    Servo_SetAngle(SERVO_ARM, CLAW_OPEN);
    Servo_SetAngle(SERVO_BASE, 0);
    Servo_SetAngle(SERVO_PLATE, PLATE_RED);
    /*
     * 注意：复位时由于不知道当前确切位置，建议手动将手臂抬到最高。
     * 或者此处逻辑可以改为持续向上跑直到撞到限位开关（如果有）。
     */
    BSP_Motor_Stop(&motor_5);
    LOG_I("Arm Hardware Reset (Manual reset to HOME recommended).");
}
