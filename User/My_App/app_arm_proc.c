/**
 ******************************************************************************
 * @file    app_arm_proc.c
 * @author  lingxing
 * @brief   机械臂动作组合逻辑
 ******************************************************************************
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
 * 机械臂下降距离参数
 * =======================================================================
 * 这里的数值代表从最高点(HOME)向下运行的距离
 */
#define DIST_PLATE 8000  /* 到原料区(货架)下降距离 */
#define DIST_CAR 4000    /* 到车内转盘下降距离 */
#define DIST_STACK 10000 /* 到暂存区二层(码垛)下降距离 */
#define DIST_FLOOR 15000 /* 到地面(加工区/暂存一层)下降距离 */

/* 爪子角度参数已移至 app_param.h 统一管理 */

extern Motor_t motor_5; /* 升降步进电机 */

#define ARM_STACK_SIZE 2048
#define ARM_PRIORITY 13
#define ARM_TICK 10
#define ARM_MQ_DEPTH 4

typedef enum
{
    ARM_CMD_PICK_RAW = 0,
    ARM_CMD_PICK_FLOOR,
    ARM_CMD_PLACE_CAR,
    ARM_CMD_PICK_CAR,
    ARM_CMD_PLACE_FLOOR,
    ARM_CMD_PLACE_STACK,
    ARM_CMD_PREPARE,
    ARM_CMD_RETURN_CENTER,
    ARM_CMD_RESET
} Arm_Command_t;

typedef struct
{
    Arm_Command_t command;
    uint8_t tray_num;
} Arm_Task_Msg_t;

static rt_mq_t arm_mq = RT_NULL;
static rt_thread_t arm_thread = RT_NULL;

/* 供低优先级看门狗线程读取：空闲不超时，忙碌时限制最长动作时间。 */
static volatile rt_bool_t arm_busy = RT_FALSE;
static volatile rt_bool_t arm_fault = RT_FALSE;
static volatile rt_tick_t arm_action_start_tick = 0;

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
static void Arm_Pick_From_Raw_Execute(void)
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
}

/**
 * @brief  放置到车内 (CAR)
 */
static void Arm_Place_To_Car_Execute(uint8_t tray_num)
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
}

/**
 * @brief  从车内转盘抓取 (CAR)
 */
static void Arm_Pick_From_Car_Execute(uint8_t tray_num)
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
}

/**
 * @brief  放置到地面 (FLOOR)
 */
static void Arm_Place_To_Floor_Execute(void)
{
    LOG_I("Action: [Floor] Unloading...");
    Servo_SetAngle(SERVO_BASE, 0); // 回归正前方 (对标原厂 0 度)
    rt_thread_mdelay(400);

    Arm_Move_Dist(DIST_FLOOR, 1); // 下降
    rt_thread_mdelay(200);

    Servo_SetAngle(SERVO_ARM, CLAW_OPEN);
    rt_thread_mdelay(400);

    Arm_Move_Dist(DIST_FLOOR, 0); // 返回最高点
}

/**
 * @brief  从地面抓取 (FLOOR)
 */
static void Arm_Pick_From_Floor_Execute(void)
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
}

/**
 * @brief  放置到二层码垛 (STACK)
 */
static void Arm_Place_To_Stack_Execute(void)
{
    LOG_I("Action: [Stack] Stacking...");
    Servo_SetAngle(SERVO_BASE, 0); // 面向前方码放区
    rt_thread_mdelay(400);

    Arm_Move_Dist(DIST_STACK, 1); // 下降到二层高度
    rt_thread_mdelay(200);

    Servo_SetAngle(SERVO_ARM, CLAW_OPEN);
    rt_thread_mdelay(400);

    Arm_Move_Dist(DIST_STACK, 0); // 返回最高点
}

/**
 * @brief  一键复位至最高点安全位
 */
static void Arm_Reset_Pos_Execute(void)
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

/**
 * @brief  视觉定位前将机械臂调整到安全姿态
 */
static void Arm_Prepare_Execute(void)
{
    Servo_SetAngle(SERVO_BASE, 0);
    Servo_SetAngle(SERVO_ARM, CLAW_OPEN);
}

/**
 * @brief  完赛时将机械臂底座收回车体中心
 */
static void Arm_Return_Center_Execute(void)
{
    Servo_SetAngle(SERVO_BASE, 98);
}

/**
 * @brief  [Internal] 向机械臂线程投递动作命令
 */
static rt_err_t Arm_Send_Command(Arm_Command_t command, uint8_t tray_num)
{
    Arm_Task_Msg_t msg;

    if (arm_mq == RT_NULL || arm_fault == RT_TRUE)
    {
        LOG_E("arm command rejected: queue unavailable or arm fault.");
        return -RT_ERROR;
    }

    msg.command = command;
    msg.tray_num = tray_num;
    return rt_mq_send(arm_mq, &msg, sizeof(msg));
}

/**
 * @brief  机械臂独立线程入口
 * @note   空闲时阻塞等待消息；动作完成后通过事件集通知 brain。
 */
static void arm_proc(void *parameter)
{
    Arm_Task_Msg_t msg;

    (void)parameter;

    while (1)
    {
        if (rt_mq_recv(arm_mq, &msg, sizeof(msg),
                       RT_WAITING_FOREVER) != RT_EOK)
        {
            continue;
        }

        arm_busy = RT_TRUE;
        arm_action_start_tick = rt_tick_get();

        if (msg.command <= ARM_CMD_PLACE_STACK && motor_5.config.htim == RT_NULL)
        {
            LOG_E("lift motor is unavailable; arm command cannot run.");
            arm_fault = RT_TRUE;
        }
        else
        {
            switch (msg.command)
            {
            case ARM_CMD_PICK_RAW:
                Arm_Pick_From_Raw_Execute();
                break;
            case ARM_CMD_PICK_FLOOR:
                Arm_Pick_From_Floor_Execute();
                break;
            case ARM_CMD_PLACE_CAR:
                Arm_Place_To_Car_Execute(msg.tray_num);
                break;
            case ARM_CMD_PICK_CAR:
                Arm_Pick_From_Car_Execute(msg.tray_num);
                break;
            case ARM_CMD_PLACE_FLOOR:
                Arm_Place_To_Floor_Execute();
                break;
            case ARM_CMD_PLACE_STACK:
                Arm_Place_To_Stack_Execute();
                break;
            case ARM_CMD_RESET:
                Arm_Reset_Pos_Execute();
                break;
            case ARM_CMD_PREPARE:
                Arm_Prepare_Execute();
                break;
            case ARM_CMD_RETURN_CENTER:
                Arm_Return_Center_Execute();
                break;
            default:
                LOG_E("unknown arm command: %d", msg.command);
                arm_fault = RT_TRUE;
                break;
            }
        }

        arm_busy = RT_FALSE;

        if (arm_fault == RT_TRUE)
        {
            rt_event_send(&mission_event, EV_ARM_ERROR);
        }
        else
        {
            rt_event_send(&mission_event, EV_ARM_FINISHED);
        }
    }
}

rt_err_t Arm_Pick_From_Raw(void)
{
    return Arm_Send_Command(ARM_CMD_PICK_RAW, 0);
}

rt_err_t Arm_Pick_From_Floor(void)
{
    return Arm_Send_Command(ARM_CMD_PICK_FLOOR, 0);
}

rt_err_t Arm_Place_To_Car(uint8_t tray_num)
{
    return Arm_Send_Command(ARM_CMD_PLACE_CAR, tray_num);
}

rt_err_t Arm_Pick_From_Car(uint8_t tray_num)
{
    return Arm_Send_Command(ARM_CMD_PICK_CAR, tray_num);
}

rt_err_t Arm_Place_To_Floor(void)
{
    return Arm_Send_Command(ARM_CMD_PLACE_FLOOR, 0);
}

rt_err_t Arm_Place_To_Stack(void)
{
    return Arm_Send_Command(ARM_CMD_PLACE_STACK, 0);
}

rt_err_t Arm_Prepare(void)
{
    return Arm_Send_Command(ARM_CMD_PREPARE, 0);
}

rt_err_t Arm_Return_Center(void)
{
    return Arm_Send_Command(ARM_CMD_RETURN_CENTER, 0);
}

rt_err_t Arm_Reset_Pos(void)
{
    return Arm_Send_Command(ARM_CMD_RESET, 0);
}

rt_bool_t App_Arm_IsHealthy(void)
{
    if (arm_fault == RT_TRUE)
    {
        return RT_FALSE;
    }

    if (arm_busy == RT_TRUE &&
        (rt_tick_get() - arm_action_start_tick) >
            rt_tick_from_millisecond(ARM_ACTION_TIMEOUT_MS))
    {
        arm_fault = RT_TRUE;
        LOG_E("arm action timeout (>%u ms).", ARM_ACTION_TIMEOUT_MS);
        return RT_FALSE;
    }

    return RT_TRUE;
}

void App_Arm_EmergencyStop(void)
{
    arm_fault = RT_TRUE;
    BSP_Motor_Stop(&motor_5);
    BSP_Motor_Enable(&motor_5, 0);
}

/**
 * @brief  初始化机械臂消息队列和独立线程
 */
int App_Arm_Init(void)
{
    arm_mq = rt_mq_create("mq_arm", sizeof(Arm_Task_Msg_t),
                          ARM_MQ_DEPTH, RT_IPC_FLAG_FIFO);

    arm_thread = rt_thread_create("arm_proc",
                                  arm_proc,
                                  RT_NULL,
                                  ARM_STACK_SIZE,
                                  ARM_PRIORITY,
                                  ARM_TICK);

    if (arm_mq != RT_NULL && arm_thread != RT_NULL)
    {
        rt_thread_startup(arm_thread);
        return RT_EOK;
    }

    LOG_E("arm thread or message queue create failed.");
    arm_fault = RT_TRUE;
    return -RT_ERROR;
}

INIT_APP_EXPORT(App_Arm_Init);
