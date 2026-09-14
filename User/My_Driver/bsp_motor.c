/**
 ******************************************************************************
 * @file    bsp_motor.c
 * @author  lingxing
 * @brief   电机底层驱动
 ******************************************************************************
 */

#include "bsp_motor.h"
#include <rtthread.h>

extern TIM_HandleTypeDef htim1;
#if MOTOR_LIFT_ENABLED
extern TIM_HandleTypeDef htim2;
#endif
#include "../../cubemx/Inc/main.h"

#define DBG_TAG "bsp.motor"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* 电机实例 */
Motor_t motor_1;
Motor_t motor_2;
Motor_t motor_3;
Motor_t motor_4;
Motor_t motor_5;

/**
 * @brief  初始化电机硬件
 * @note   返回 int 是为了匹配 RT-Thread 的 init_fn_t 原型，可以直接被
 *         INIT_xxx_EXPORT 导出（BSP_Servo_Init 也是这个约定）。
 */
int BSP_Motor_Init(void)
{
    /* 电机 1: TIM1_CH1, PD0(DIR), PD1(EN) */
    motor_1.config.htim = &htim1;
    motor_1.config.channel = TIM_CHANNEL_1;
    motor_1.config.dir.port = GPIOD;
    motor_1.config.dir.pin = GPIO_PIN_0;
    motor_1.config.en.port = GPIOD;
    motor_1.config.en.pin = GPIO_PIN_1;
    motor_1.config.reverse = 0;

    /* 电机 2: TIM1_CH2, PD3(DIR), PD4(EN) */
    motor_2.config.htim = &htim1;
    motor_2.config.channel = TIM_CHANNEL_2;
    motor_2.config.dir.port = GPIOD;
    motor_2.config.dir.pin = GPIO_PIN_3;
    motor_2.config.en.port = GPIOD;
    motor_2.config.en.pin = GPIO_PIN_4;
    motor_2.config.reverse = 0;

    /* 电机 3: TIM1_CH3, PD6(DIR), PD7(EN) */
    motor_3.config.htim = &htim1;
    motor_3.config.channel = TIM_CHANNEL_3;
    motor_3.config.dir.port = GPIOD;
    motor_3.config.dir.pin = GPIO_PIN_6;
    motor_3.config.en.port = GPIOD;
    motor_3.config.en.pin = GPIO_PIN_7;
    motor_3.config.reverse = 0;

    /* 电机 4: TIM1_CH4, PG9(DIR), PG10(EN) */
    motor_4.config.htim = &htim1;
    motor_4.config.channel = TIM_CHANNEL_4;
    motor_4.config.dir.port = GPIOG;
    motor_4.config.dir.pin = GPIO_PIN_9;
    motor_4.config.en.port = GPIOG;
    motor_4.config.en.pin = GPIO_PIN_10;
    motor_4.config.reverse = 0;

    /* 启动 TIM1 的四个通道 (使用中断模式以统计步数) */
    HAL_TIM_OC_Start_IT(motor_1.config.htim, motor_1.config.channel);
    HAL_TIM_OC_Start_IT(motor_2.config.htim, motor_2.config.channel);
    HAL_TIM_OC_Start_IT(motor_3.config.htim, motor_3.config.channel);
    HAL_TIM_OC_Start_IT(motor_4.config.htim, motor_4.config.channel);

#if MOTOR_LIFT_ENABLED
    /* 电机 5: TIM2_CH2, PG12(DIR), PG11(EN) */
    motor_5.config.htim = &htim2;
    motor_5.config.channel = TIM_CHANNEL_2;
    motor_5.config.dir.port = GPIOG;
    motor_5.config.dir.pin = GPIO_PIN_12;
    motor_5.config.en.port = GPIOG;
    motor_5.config.en.pin = GPIO_PIN_11;
    motor_5.config.reverse = 0;

    HAL_TIM_OC_Start_IT(motor_5.config.htim, motor_5.config.channel);
    LOG_I("motor_5 (lift / TIM2_CH2) enabled.");
#else
    /* TIM2 未配置：保持 htim 为 NULL，SetSpeed/Stop 会因为空指针保护而静默忽略 */
    motor_5.config.htim = RT_NULL;
    motor_5.speed = 0;
    motor_5.total_steps = 0;
    LOG_W("motor_5 (lift) DISABLED: TIM2 not configured in CubeMX.");
#endif

    return RT_EOK;
}

/* 导出为设备级自动初始化：系统启动时（main 线程 rt_components_init 阶段）自动调用。
 * 必须晚于 rt_hw_board_init() 里的 MX_TIM1_Init()，此处顺序天然满足。 */
INIT_DEVICE_EXPORT(BSP_Motor_Init);

/**
 * @brief  设置电机速度
 */
void BSP_Motor_SetSpeed(Motor_t *motor, int32_t speed)
{
    uint32_t arr_val = 0;
    uint8_t direction = 0;

    if (motor == RT_NULL)
    {
        return;
    }

    /* 未配置定时器的电机（例如 TIM2 缺失时的 motor_5）：只记录目标速度，
     * 不碰任何寄存器，避免 __HAL_TIM_SET_AUTORELOAD(NULL,...) 触发 HardFault */
    if (motor->config.htim == RT_NULL)
    {
        motor->speed = speed;
        return;
    }

    /* 限制范围 */
    if (speed > 10000)
        speed = 10000;
    if (speed < -10000)
        speed = -10000;

    motor->speed = speed;

    if (speed == 0)
    {
        BSP_Motor_Stop(motor);
        return;
    }

    /* 确定方向与周期 (频率 = 1 / ARR) */
    if (speed > 0)
    {
        direction = motor->config.reverse ? 1 : 0;
        arr_val = 10200 - speed; // 与原工程逻辑对标：速度越大，载装载值越小，频率越高
    }
    else
    {
        direction = motor->config.reverse ? 0 : 1;
        arr_val = 10200 + speed;
    }

    /* 写入方向 */
    HAL_GPIO_WritePin(motor->config.dir.port, motor->config.dir.pin,
                      direction ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* 修改定时器周期 (改变脉冲频率) */
    __HAL_TIM_SET_AUTORELOAD(motor->config.htim, arr_val);
}

/**
 * @brief  电机停止
 */
void BSP_Motor_Stop(Motor_t *motor)
{
    if (motor == RT_NULL)
    {
        return;
    }

    motor->speed = 0;

    /* 未配置定时器的电机只需清速度，不能碰寄存器 */
    if (motor->config.htim == RT_NULL)
    {
        return;
    }

    /* 停止脉冲 (或者将 ARR 设为极大值) */
    __HAL_TIM_SET_AUTORELOAD(motor->config.htim, 65535);
}

/**
 * @brief  电机使能控制
 */
void BSP_Motor_Enable(Motor_t *motor, uint8_t enable)
{
    if (motor != RT_NULL && motor->config.en.port != NULL)
    {
        HAL_GPIO_WritePin(motor->config.en.port, motor->config.en.pin,
                          enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}
/**
 * @brief  定时器输出比较中断回调 (每产生一个脉冲执行一次)
 * @note   在这里进行步数统计 (HAL 层本能反应)
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        /* 判定是哪个通道触发的脉冲 */
        Motor_t *p_motor = NULL;
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
            p_motor = &motor_1;
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
            p_motor = &motor_2;
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
            p_motor = &motor_3;
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
            p_motor = &motor_4;

        if (p_motor != NULL && p_motor->speed != 0)
        {
            /* 根据速度方向进行加减 (用于计算里程) */
            if (p_motor->speed > 0)
                p_motor->total_steps++;
            else
                p_motor->total_steps--;
        }
    }
#if MOTOR_LIFT_ENABLED
    else if (htim->Instance == TIM2)
    {
        /* 第 5 个电机挂在 TIM2_CH2 上 */
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            if (motor_5.speed != 0)
            {
                if (motor_5.speed > 0)
                    motor_5.total_steps++;
                else
                    motor_5.total_steps--;
            }
        }
    }
#endif
}

/**
 * @brief  获取电机累积步数 (原子操作，不依赖 RTOS)
 */
int32_t BSP_Motor_GetSteps(Motor_t *motor)
{
    int32_t steps;
    /* 保存当前中断状态并关闭全局中断 (临界区保护) */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    steps = motor->total_steps;

    /* 恢复中断状态 */
    __set_PRIMASK(primask);
    return steps;
}

/**
 * @brief  复位电机累积步数 (原子操作)
 */
void BSP_Motor_ResetSteps(Motor_t *motor)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    motor->total_steps = 0;

    __set_PRIMASK(primask);
}
