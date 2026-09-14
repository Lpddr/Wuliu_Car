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
 * 1. 初始化: 调用 BSP_Motor_Init()   （已通过 INIT_DEVICE_EXPORT 自动执行）
 * 2. 控速:   调用 BSP_Motor_SetSpeed(&motor_1, 5000) (motor_1~5)
 * 3. 停止:   调用 BSP_Motor_Stop(&motor_1)
 * 4. 读位置: 调用 BSP_Motor_GetSteps(&motor_1)
 * 5. 复位位置: 调用 BSP_Motor_ResetSteps(&motor_1)
 * 6. 使能:   调用 BSP_Motor_Enable(&motor_1, 1)  (1:开启, 0:关闭)
 */

/* ---------------------------------------------------------------------------
 * 第 5 个电机（升降机构 / Z 轴）开关
 *
 * 该电机按设计挂在 TIM2_CH2 上，但当前 CubeMX 工程里根本没有配置 TIM2：
 *   - cubemx/Src/tim.c 只生成 htim1 / htim5 / htim9，没有 htim2；
 *   - 工程里唯一未定义的硬件符号就是 `U htim2`（arm-none-eabi-nm 可验证）。
 * 如果直接引用，链接会失败；即便绕过，HAL_TIM_OC_Start_IT(NULL,...) 也会
 * 立刻 HardFault。所以这里用一个显式开关把它隔离掉。
 *
 * 【启用步骤】在 CubeMX 中：
 *   1. Timers -> TIM2 -> Clock Source = Internal Clock；
 *   2. Channel2 -> Output Compare（Mode 选 Toggle on match，与 TIM1 一致）；
 *   3. NVIC Settings 勾选 "TIM2 global interrupt"；
 *   4. 检查 Channel2 对应的引脚（TIM2_CH2 只能是 PA1 / PB3，**不在 GPIOG 上**，
 *      现有注释里的 PG11/PG12 只是 DIR/EN 普通 GPIO，需要单独在 GPIO 里配好）；
 *   5. 生成代码后把下面这个宏改成 1。
 * ------------------------------------------------------------------------- */
#define MOTOR_LIFT_ENABLED 0

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
int  BSP_Motor_Init(void);
void BSP_Motor_SetSpeed(Motor_t *motor, int32_t speed);
void BSP_Motor_Stop(Motor_t *motor);
void BSP_Motor_Enable(Motor_t *motor, uint8_t enable);
int32_t BSP_Motor_GetSteps(Motor_t *motor);
void BSP_Motor_ResetSteps(Motor_t *motor);

#endif /* __BSP_MOTOR_H */
