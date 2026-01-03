/**
 ******************************************************************************
 * @file    app_move_proc.c
 * @author  lingxing
 * @brief   底盘运动控制层
 ******************************************************************************
 */

#include <math.h>
#include "app_move_proc.h"
#include "app_param.h"
#include "app_imu_proc.h"
#include "../Components/imu_wit.h"
#include "../My_Driver/bsp_uart.h"
#include "../My_Driver/bsp_motor.h"
#include "../My_Driver/bsp_pid.h"
#include "app_task_proc.h"

#define DBG_TAG "app.move"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define ABS(x) ((x) < 0 ? -(x) : (x))

/* 1. 轨迹生成与纠偏控制变量 */
static Move_Mode_t current_mode = MOVE_STOP;
static float target_speed = 0.0f;  /* 目标速度 (mm/s) */
static float current_speed = 0.0f; /* 当前平滑速度 (mm/s) */

static float target_yaw = 0.0f; /* 目标航向角 (0-360) */

/* 2. 位移控制变量 (Displacement) */
static int32_t target_pulse_x = 0; /* 目标 X 轴总脉冲数 */

/* 3. PID 实例 */
static PID_t pid_yaw;                 /* 用于直线行驶的“航向锁” */
static PID_t pid_turn;                /* 新增：用于旋转到特定角度的“位置环” */
static float yaw_compensation = 0.0f; /* PID 计算出的旋转修正量 */

/* 采样周期 20ms */
#define MOVE_CONTROL_TICK 20

/* 线程参数 */
#define MOVE_THREAD_STACK_SIZE 1024
#define MOVE_THREAD_PRIORITY 8 /* 优先级高于大脑任务 */
#define MOVE_THREAD_TIMESLICE 5

static rt_thread_t move_thread = RT_NULL;

/* ========================================================================== */
/*                          1. 内部辅助工具 (Internal Helpers)                  */
/* ========================================================================== */

/**
 * @brief  [内部函数] 根据物理位移残余，计算当前允许的物理安全限速边界
 * @param  dist_mm: 剩余位移 (mm)
 * @return 允许的最大速度 (mm/s)
 * @note   公式：V^2 = 2*a*s -> V = sqrt(2*a*s)
 */
static float Get_Velocity_Boundary(float dist_mm)
{
    if (dist_mm <= 0.5f)
        return 1.0f; // 极小距离限速，防止开方抖动

    // v = sqrt(2 * a * d)
    return sqrtf(2.0f * MOVE_ACCEL_VAL * dist_mm);
}

/**
 * @brief  [内部函数] 加减速斜坡跟随逻辑
 */
static float Move_Step_Towards(float current, float target, float step)
{
    if (current < target)
    {
        current += step;
        if (current > target)
            current = target;
    }
    else if (current > target)
    {
        current -= step;
        if (current < target)
            current = target;
    }
    return current;
}

/* ========================================================================== */
/*                          2. 运动控制核心线程 (Core Thread)                   */
/* ========================================================================== */

/**
 * @brief  [内部函数] 运动控制主线程循环
 */
static void move_proc(void *parameter)
{
    while (1)
    {
        if (current_mode != MOVE_STOP)
        {
            /* --- 步骤 1: 物理状态解算 --- */
            float step = MOVE_ACCEL_VAL * (MOVE_CONTROL_TICK / 1000.0f); // 本周期最大速度增量
            float current_pulse = (float)ABS(BSP_Motor_GetSteps(&motor_1));

            // 计算剩余距离 (mm)。如果是 0 则代表巡航模式，给予极大值
            float remain_dist = 999999.0f;
            if (target_pulse_x > 0)
            {
                remain_dist = (target_pulse_x - (int32_t)current_pulse) / PULSE_PER_MM;
                if (remain_dist < 0)
                    remain_dist = 0;
            }

            /* --- 步骤 2: 速度规划与限制 --- */
            // A. 计算当前物理限速 (物理边界)
            float limit_speed = Get_Velocity_Boundary(remain_dist);

            // B. 计算实时目标：用户想跑 vs 物理允许
            float final_target_v = (target_speed < limit_speed) ? target_speed : limit_speed;

            // C. 速度斜坡跟随：让当前速度平滑向安全目标靠拢
            current_speed = Move_Step_Towards(current_speed, final_target_v, step);

            // D. 自动停车判定：防止无限接近 0 导致的抖动，设定物理停止阈值
            if (target_pulse_x > 0 && remain_dist <= 0.1f && current_speed <= 5.0f)
            {
                Move_Stop();
                rt_event_send(&mission_event, EV_MOVE_FINISHED);
                // LOG_D("Move dist done, signaling brain.");
            }

            /* --- 步骤 3: 运动模式映射 (Kinematics) --- */
            float m1, m2, m3, m4;
            float out_speed = current_speed * MOVE_SPEED_SCALE;

            switch (current_mode)
            {
            case MOVE_FORWARD:
                yaw_compensation = BSP_PID_CalcPositional(&pid_yaw, imu_app_data.yaw);
                m1 = m3 = out_speed - yaw_compensation;
                m2 = m4 = out_speed + yaw_compensation;
                break;

            case MOVE_BACKWARD:
                yaw_compensation = BSP_PID_CalcPositional(&pid_yaw, imu_app_data.yaw);
                m1 = m3 = -out_speed - yaw_compensation;
                m2 = m4 = -out_speed + yaw_compensation;
                break;

            case MOVE_SLIDE_LEFT: // 左平移：M1-, M2+, M3+, M4-
                m1 = -out_speed;
                m4 = -out_speed;
                m2 = out_speed;
                m3 = out_speed;
                break;

            case MOVE_SLIDE_RIGHT: // 右平移：M1+, M2-, M3-, M4+
                m1 = out_speed;
                m4 = out_speed;
                m2 = -out_speed;
                m3 = -out_speed;
                break;

            case MOVE_TURN_LEFT: // 原地左转：M1-, M2+, M3-, M4+
                m1 = -out_speed;
                m3 = -out_speed;
                m2 = out_speed;
                m4 = out_speed;
                break;

            case MOVE_TURN_RIGHT: // 原地右转：M1+, M2-, M3+, M4-
                m1 = out_speed;
                m3 = out_speed;
                m2 = -out_speed;
                m4 = -out_speed;
                break;

            case MOVE_TURN_ABS:
            {
                /* 绝对角度旋转：使用 pid_turn 闭环控制 */
                float error = target_yaw - imu_app_data.yaw;
                while (error > 180.0f)
                    error -= 360.0f;
                while (error < -180.0f)
                    error += 360.0f;

                if (ABS(error) < TURN_ERROR_THRESHOLD)
                {
                    Move_Stop();
                    rt_event_send(&mission_event, EV_MOVE_FINISHED);
                    LOG_D("Turn abs done, signaling brain.");
                    continue;
                }

                float vrot = BSP_PID_CalcPositional(&pid_turn, imu_app_data.yaw);
                m1 = m3 = -vrot;
                m2 = m4 = vrot;
                break;
            }

            default:
                Move_Stop();
                continue;
            }

            /* 最终下发底层驱动：将计算出的平滑速度输出给步进电机驱动层 */
            if (current_mode != MOVE_STOP)
            {
                BSP_Motor_SetSpeed(&motor_1, (int32_t)m1);
                BSP_Motor_SetSpeed(&motor_2, (int32_t)m2);
                BSP_Motor_SetSpeed(&motor_3, (int32_t)m3);
                BSP_Motor_SetSpeed(&motor_4, (int32_t)m4);
            }
        }
        else
        {
            current_speed = 0;
            BSP_Motor_Stop(&motor_1);
            BSP_Motor_Stop(&motor_2);
            BSP_Motor_Stop(&motor_3);
            BSP_Motor_Stop(&motor_4);
        }

        rt_thread_mdelay(MOVE_CONTROL_TICK);
    }
}

/* ========================================================================== */
/*                          3. 外部暴露接口库 (Public API)                      */
/* ========================================================================== */

/**
 * @brief  [API] 初始化运动控制系统的底层依赖与线程
 */
int App_Move_Init(void)
{
    /* 1. 初始化直线纠偏 PID  */
    BSP_PID_Init(&pid_yaw,
                 PID_KP_STRAIGHT,
                 PID_KI_STRAIGHT,
                 PID_KD_STRAIGHT,
                 0,       /* 初始目标角度 */
                 200.0f); /* 最大修正量幅度限幅 */

    /* [新增] 2. 初始化角度旋转 PID */
    BSP_PID_Init(&pid_turn,
                 PID_KP_TURN,
                 PID_KI_TURN,
                 PID_KD_TURN,
                 0,       /* 目标角度由 API 设置 */
                 300.0f); /* 旋转动力限幅 */

    /* 2. 创建线程 */
    move_thread = rt_thread_create("move_proc",
                                   move_proc,
                                   RT_NULL,
                                   MOVE_THREAD_STACK_SIZE,
                                   MOVE_THREAD_PRIORITY,
                                   MOVE_THREAD_TIMESLICE);

    if (move_thread != RT_NULL)
    {
        rt_thread_startup(move_thread);
        return RT_EOK;
    }

    return -RT_ERROR;
}

/* 导出自动初始化 */
INIT_APP_EXPORT(App_Move_Init);

/* --- API 实现桩位 --- */

/**
 * @brief [API] 方向控制
 */
void Move_Now(Move_Mode_t mode, float speed_mm_s, float distance_mm)
{
    current_mode = mode;
    target_speed = speed_mm_s;

    /* 设置位移目标 */
    target_pulse_x = (int32_t)(ABS(distance_mm) * PULSE_PER_MM);

    if (mode == MOVE_FORWARD || mode == MOVE_BACKWARD)
    {
        target_yaw = imu_app_data.yaw;
        BSP_PID_SetTarget(&pid_yaw, target_yaw);
    }

    /* 重置里程计 (通过专业 API) */
    BSP_Motor_ResetSteps(&motor_1);
    BSP_Motor_ResetSteps(&motor_2);
    BSP_Motor_ResetSteps(&motor_3);
    BSP_Motor_ResetSteps(&motor_4);
}

/**
 * @brief [API] 绝对角度旋转
 */
void Move_Turn_Abs(float abs_angle)
{
    target_yaw = abs_angle;
    target_speed = 0;
    target_pulse_x = 0; // 角度旋转不依赖里程计位移
    current_mode = MOVE_TURN_ABS;
    BSP_PID_Reset(&pid_turn);
}

/**
 * @brief [API] 紧急停止
 */
void Move_Stop(void)
{
    current_mode = MOVE_STOP;
    target_speed = 0;
    current_speed = 0;
    target_pulse_x = 0;
    BSP_Motor_Stop(&motor_1);
    BSP_Motor_Stop(&motor_2);
    BSP_Motor_Stop(&motor_3);
    BSP_Motor_Stop(&motor_4);
}
