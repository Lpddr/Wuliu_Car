/**
 * @file    bsp_servo.h
 * @brief   机械臂舵机控制驱动 (PA0, PE5, PE6)
 */

#ifndef __BSP_SERVO_H
#define __BSP_SERVO_H

#include <rtthread.h>

/*
 * 舵机 ID 定义 (直接对应原工程 duoji 1, 3, 4)
 * 1. SERVO_ARM:   爪子 (PA0)
 * 3. SERVO_BASE:  底座 (PE5)
 * 4. SERVO_PLATE: 物料盘 (PE6)
 */
#define SERVO_ARM 1
#define SERVO_BASE 3
#define SERVO_PLATE 4

/**
 * @brief  [API] 设置舵机角度 (0-180度)
 * @param  servo_id: 舵机 ID (1, 3, 4)
 * @param  angle: 角度值 (0-180)
 */
void Servo_SetAngle(uint8_t servo_id, float angle);

/**
 * @brief  [API] 初始化舵机 PWM 设备
 */
int BSP_Servo_Init(void);

#endif /* __BSP_SERVO_H */
