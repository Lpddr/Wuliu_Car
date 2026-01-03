/**
 * @file    bsp_servo.c
 * @brief   机械臂舵机控制实现 (基于 HAL 库)
 */

#include "bsp_servo.h"
#include "../../cubemx/Inc/main.h"

/* 这里的 htim5 和 htim9 是在 CubeMX 生成的 tim.c 中定义的 */
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;

#define DBG_TAG "bsp.servo"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/**
 * @brief  [私有] 角度转计数值
 * @note   映射关系: 0deg -> 250, 180deg -> 1250
 */
static uint32_t angle_to_pulse(float angle)
{
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;

    return (uint32_t)(250.0f + (angle * (1000.0f / 180.0f)));
}

/**
 * @brief 设置舵机角度
 */
void Servo_SetAngle(uint8_t servo_id, float angle)
{
    uint32_t pulse = angle_to_pulse(angle);

    switch (servo_id)
    {
    case SERVO_ARM:
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pulse);
        break;
    case SERVO_BASE:
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, pulse);
        break;
    case SERVO_PLATE:
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, pulse);
        break;
    default:
        break;
    }
}

/**
 * @brief 初始化硬件并开启 PWM
 */
int BSP_Servo_Init(void)
{
    /* 1. 启动硬件 PWM 通道 */
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

    /* 2. 设置初始位置 (复位姿态) */
    Servo_SetAngle(SERVO_ARM, 10);   /* 爪子张开 */
    Servo_SetAngle(SERVO_BASE, 0);   /* 底座复位 */
    Servo_SetAngle(SERVO_PLATE, 17); /* 物料盘初始位 */

    LOG_I("Mechanical Arm Driver (ARM/BASE/PLATE) Init Ready.");
    return RT_EOK;
}

INIT_DEVICE_EXPORT(BSP_Servo_Init);
