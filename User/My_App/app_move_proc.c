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

/* 4. 命令集保护
 * ---------------------------------------------------------------------------
 * current_mode / target_speed / target_pulse_x / target_yaw 是一组"必须一起生效"
 * 的命令参数，加上"里程计已复位"这个事实，共同构成一个命令上下文：
 *   写侧 = brain 线程 (优先级 10)，读侧 = move_proc 线程 (优先级 8)。
 * 二者不是生产者-消费者关系（没有队列、没有"取走即消费"），而是"状态发布-订阅"：
 * 消费者每个控制周期都要读同一份命令，并不"消耗"它。
 *
 * 因此这里用 1 把互斥锁做「原子发布 + 原子快照」，而不是教科书的 empty/full 信号量对：
 *   - move_proc 是 20ms 定周期控制器，必须每拍都跑（即使没有新命令也要维持速度斜坡、
 *     跑航向 PID、判到位），若阻塞在 full_sem 上会直接破坏控制周期；
 *   - 命令是"电平"(持续状态) 不是"边沿"(一次性事件)，消费方反复读同一份命令也不会
 *     去 release(empty)，写侧第 2 条命令就会永久阻塞在 empty 上 → 任务流程卡死。
 *
 * 关于 flag：RT-Thread 的 rt_mutex 无论 flag 是 PRIO 还是 FIFO，rt_mutex_take()
 * 里都会无条件做优先级继承（ipc.c 中 "change the owner thread priority of mutex"
 * 那段）；flag 只决定等待线程在挂起队列里的排队方式。这里取 PRIO 只是让"等待者
 * 按优先级唤醒"的语义更明确，配合继承一起避免 brain(10) 持锁时被中间优先级线程
 * 抢占、把 move_proc(8) 一起拖住。
 * ------------------------------------------------------------------------- */
static struct rt_mutex move_cmd_mutex;
static rt_bool_t        move_cmd_mutex_ready = RT_FALSE;

/* brain 线程与 main 线程同为优先级 10，可能在 App_Move_Init() 之前就调用 Move_Stop()，
 * 所以这里必须容忍"锁尚未创建"的情形（静态对象全 0，直接 rt_mutex_take 会异常）。 */
static void move_cmd_lock(void)
{
    if (move_cmd_mutex_ready)
    {
        rt_mutex_take(&move_cmd_mutex, RT_WAITING_FOREVER);
    }
}

static void move_cmd_unlock(void)
{
    if (move_cmd_mutex_ready)
    {
        rt_mutex_release(&move_cmd_mutex);
    }
}

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
        App_Watchdog_ReportMoveAlive();

        /* --- 步骤 -1: 取一份「命令 + 里程计」的自洽快照 ---
         * brain(优先级 10) 写入时本线程(优先级 8) 可以随时抢占，而 brain 是分多步写的：
         *     current_mode = mode;  →  target_speed = ...;  →  target_pulse_x = ...;
         *     →  target_yaw = ...;  →  BSP_Motor_ResetSteps() ×4
         * 若在两次写之间被抢占，本线程会读到"新模式 + 旧目标距离"的混合态。最坏的情况：
         * 新目标 (如 864mm) 小于上一段残留的累计步数 (如 876mm)，且此时 current_speed
         * 刚好为 0 → remain_dist 直接算成 0 → 误判"已到位"，向大脑发出一个假的
         * EV_MOVE_FINISHED：小车一步没走，任务流程却推进了。
         * 把命令组和里程计放在同一个临界区里取值，此后整轮只用本地副本，
         * 保证一个控制周期内看到的是同一份命令上下文。 */
        move_cmd_lock();
        Move_Mode_t cmd_mode       = current_mode;
        float       cmd_speed      = target_speed;
        int32_t     cmd_pulse_x    = target_pulse_x;
        float       cmd_target_yaw = target_yaw;
        int32_t     cmd_steps      = BSP_Motor_GetSteps(&motor_1); /* 只读一次：ABS 是宏，会重复展开 */
        move_cmd_unlock();

        float cmd_pulse_now = (float)(cmd_steps < 0 ? -cmd_steps : cmd_steps);

        if (cmd_mode != MOVE_STOP)
        {
            /* --- 步骤 0: 取一份 IMU 一致快照，并做新鲜度检查 ---
             * 为什么不是 full_sem：姿态是「状态」不是「队列」，消费者只需要知道
             * "这个值有多新"，时间戳就够了；配对信号量会把状态语义变成队列语义，
             * IMU 100~200Hz 产、本线程 50Hz 消，生产者很快被背压卡死并丢整帧。
             * 这一步同时替代了原先对 imu_app_data.yaw 的多处无锁盲读。 */
            App_IMU_Data_t imu;
            if (App_IMU_GetData(&imu, rt_tick_from_millisecond(IMU_DATA_MAX_AGE_MS)) != RT_EOK)
            {
                LOG_E("IMU data stale (>%d ms), emergency stop.", IMU_DATA_MAX_AGE_MS);
                Move_Stop();
                rt_event_send(&mission_event, EV_ALL_ERROR); /* 上报大脑，转紧急错误流程 */
                rt_thread_mdelay(MOVE_CONTROL_TICK);         /* continue 会跳过循环末尾的延时，这里补上防止空转 */
                continue;
            }

            /* --- 步骤 1: 物理状态解算 --- */
            float step = MOVE_ACCEL_VAL * (MOVE_CONTROL_TICK / 1000.0f); // 本周期最大速度增量

            // 计算剩余距离 (mm)。如果是 0 则代表巡航模式，给予极大值
            float remain_dist = 999999.0f;
            if (cmd_pulse_x > 0)
            {
                remain_dist = (cmd_pulse_x - (int32_t)cmd_pulse_now) / PULSE_PER_MM;
                if (remain_dist < 0)
                    remain_dist = 0;
            }

            /* --- 步骤 2: 速度规划与限制 --- */
            // A. 计算当前物理限速 (物理边界)
            float limit_speed = Get_Velocity_Boundary(remain_dist);

            // B. 计算实时目标：用户想跑 vs 物理允许
            float final_target_v = (cmd_speed < limit_speed) ? cmd_speed : limit_speed;

            // C. 速度斜坡跟随：让当前速度平滑向安全目标靠拢
            current_speed = Move_Step_Towards(current_speed, final_target_v, step);

            // D. 自动停车判定：防止无限接近 0 导致的抖动，设定物理停止阈值
            if (cmd_pulse_x > 0 && remain_dist <= 0.1f && current_speed <= 5.0f)
            {
                Move_Stop();
                rt_event_send(&mission_event, EV_MOVE_FINISHED);
                // LOG_D("Move dist done, signaling brain.");

                /* 必须 continue：本轮快照里的 cmd_mode 仍是旧模式（不是 MOVE_STOP），
                 * 若继续往下走会带着"已经停下来的命令"再解算一次并重新驱动电机。 */
                continue;
            }

            /* --- 步骤 3: 运动模式映射 (Kinematics) --- */
            float m1, m2, m3, m4;
            float out_speed = current_speed * MOVE_SPEED_SCALE;

            switch (cmd_mode)
            {
            case MOVE_FORWARD:
                yaw_compensation = BSP_PID_CalcPositional(&pid_yaw, imu.yaw);
                m1 = m3 = out_speed - yaw_compensation;
                m2 = m4 = out_speed + yaw_compensation;
                break;

            case MOVE_BACKWARD:
                yaw_compensation = BSP_PID_CalcPositional(&pid_yaw, imu.yaw);
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
                float error = cmd_target_yaw - imu.yaw;
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

                /* 同一个 imu 快照既用于到位判定、又用于 PID 计算，
                 * 因此不会出现"用新值判定到位、却用旧值算输出"的错位 */
                float vrot = BSP_PID_CalcPositional(&pid_turn, imu.yaw);
                m1 = m3 = -vrot;
                m2 = m4 = vrot;
                break;
            }

            default:
                Move_Stop();
                continue;
            }

            /* 最终下发底层驱动：将计算出的平滑速度输出给步进电机驱动层 */
            if (cmd_mode != MOVE_STOP)
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
    /* 0. 创建"命令集"互斥锁
     * 必须在 rt_thread_startup(move_thread) 之前完成：线程一旦就绪，其优先级(8)高于
     * main 线程(10)，随时可能被调度起来读命令。 */
    if (rt_mutex_init(&move_cmd_mutex, "move_cmd", RT_IPC_FLAG_PRIO) != RT_EOK)
    {
        LOG_E("move cmd mutex init failed.");
        return -RT_ERROR;
    }
    move_cmd_mutex_ready = RT_TRUE;

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
 * @note  MOVE_FORWARD / MOVE_BACKWARD 依赖 IMU 锁航向，因此会先校验 IMU 数据
 *        新鲜度；若数据已过期（IMU 掉线），本函数直接返回、不启动运动，
 *        并保持原有的 current_mode 不变（通常是 MOVE_STOP）。
 * @note   命令字段与里程计复位作为一次原子发布（move_cmd_mutex 保护），
 *        move_proc 线程只会看到"整组已更新"或"整组未更新"两种状态。
 */
void Move_Now(Move_Mode_t mode, float speed_mm_s, float distance_mm)
{
    float start_yaw = 0.0f;

    /* 先验证、再改状态：避免"已经切换了模式，却发现没法安全执行"的中间态 */
    if (mode == MOVE_FORWARD || mode == MOVE_BACKWARD)
    {
        App_IMU_Data_t imu;
        if (App_IMU_GetData(&imu, rt_tick_from_millisecond(IMU_DATA_MAX_AGE_MS)) != RT_EOK)
        {
            LOG_W("Move_Now rejected: IMU data stale, refuse to start straight move.");
            return;
        }
        start_yaw = imu.yaw;
    }

    /* 原子发布整组命令：要么 4 个字段 + 里程计复位全部生效，要么全部不生效。
     * 中间不能让 move_proc 插进来，否则它会读到"新模式 + 旧里程计"这种致命组合。 */
    move_cmd_lock();

    current_mode = mode;
    target_speed = speed_mm_s;

    /* 设置位移目标 */
    target_pulse_x = (int32_t)(ABS(distance_mm) * PULSE_PER_MM);

    if (mode == MOVE_FORWARD || mode == MOVE_BACKWARD)
    {
        target_yaw = start_yaw;
        BSP_PID_SetTarget(&pid_yaw, target_yaw);
    }

    /* 重置里程计 (通过专业 API)：必须与上面的目标距离同属一次发布，
     * 否则 move_proc 可能用"新目标"减"上一段残留的旧步数"，算出负的剩余距离。 */
    BSP_Motor_ResetSteps(&motor_1);
    BSP_Motor_ResetSteps(&motor_2);
    BSP_Motor_ResetSteps(&motor_3);
    BSP_Motor_ResetSteps(&motor_4);

    move_cmd_unlock();
}

/**
 * @brief [API] 绝对角度旋转
 */
void Move_Turn_Abs(float abs_angle)
{
    move_cmd_lock();

    target_yaw = abs_angle;
    target_speed = 0;
    target_pulse_x = 0; // 角度旋转不依赖里程计位移
    current_mode = MOVE_TURN_ABS;
    BSP_PID_Reset(&pid_turn);

    move_cmd_unlock();
}

/**
 * @brief [API] 紧急停止
 */
void Move_Stop(void)
{
    move_cmd_lock();

    current_mode = MOVE_STOP;
    target_speed = 0;
    current_speed = 0;
    target_pulse_x = 0;

    move_cmd_unlock();

    /* 电机停机放在锁外：BSP_Motor_Stop 只写 ARR 寄存器，不参与命令一致性，
     * 且 move_proc 自己也会直接调 BSP_Motor_Stop，放在锁内徒增持锁时间。 */
    BSP_Motor_Stop(&motor_1);
    BSP_Motor_Stop(&motor_2);
    BSP_Motor_Stop(&motor_3);
    BSP_Motor_Stop(&motor_4);
}
