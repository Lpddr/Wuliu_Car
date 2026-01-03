/**
 ******************************************************************************
 * @file    bsp_motor.h
 * @author  lingxing
 * @brief   电机底层驱动
 ******************************************************************************
 */

#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "main.h"

/**
 * @usage 使用说明:
 * 1. 初始化: 调用 BSP_Motor_Init()
 * 2. 控速:   调用 BSP_Motor_SetSpeed(&motor_1, 5000) (motor_1~5)
 * 3. 停止:   调用 BSP_Motor_Stop(&motor_1)
 * 4. 读位置: 调用 BSP_Motor_GetSteps(&motor_1)
 * 5. 复位位置: 调用 BSP_Motor_ResetSteps(&motor_1)
 * 6. 使能:   调用 BSP_Motor_Enable(&motor_1, 1)  (1:开启, 0:关闭)
 */

/* 电机硬件配置结构体 */
typedef struct
{
    TIM_HandleTypeDef *htim; /* PWM 定时器句柄 */
    uint32_t channel;        /* PWM 通道 */

    struct
    {
        GPIO_TypeDef *port;
        uint16_t pin;
    } dir; /* 方向控制引脚 */

    struct
    {
        GPIO_TypeDef *port;
        uint16_t pin;
    } en; /* 使能控制引脚 (如有) */

    uint8_t reverse; /* 是否反向：0-正常，1-反向 */
} Motor_Config_t;

/* 电机控制句柄结构体 */
typedef struct
{
    Motor_Config_t config; /* 硬件配置 */
    int32_t speed;         /* 当前速度 (-10000 到 10000) */
    int32_t dead_zone;     /* 死区补偿值 */
    int32_t total_steps;   /* 累计脉冲数 (用于控制距离/里程计) */
} Motor_t;

/* 声明外部可用电机示例 */
extern Motor_t motor_1;
extern Motor_t motor_2;
extern Motor_t motor_3;
extern Motor_t motor_4;
extern Motor_t motor_5;

/* 函数接口 */
void BSP_Motor_Init(void);
void BSP_Motor_SetSpeed(Motor_t *motor, int32_t speed);
void BSP_Motor_Stop(Motor_t *motor);
void BSP_Motor_Enable(Motor_t *motor, uint8_t enable);
int32_t BSP_Motor_GetSteps(Motor_t *motor);
void BSP_Motor_ResetSteps(Motor_t *motor);

#endif /* __BSP_MOTOR_H */
