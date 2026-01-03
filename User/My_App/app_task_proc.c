/**
 * @file    app_task_proc.c
 * @brief   中央任务指挥部实现 (基于 RTOS 事件驱动 + Pure MQ)
 */

#include "app_task_proc.h"
#include "app_arm_proc.h"
#include "app_move_proc.h"
#include "app_qr_proc.h"
#include "app_vision_proc.h"
#include "../My_Driver/bsp_key_led.h"
#include "../My_Driver/bsp_servo.h"

#define DBG_TAG "app.brain"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <stdlib.h>

/* 1. 声明 RTOS 资源 */
struct rt_event mission_event;

/* 任务清单：存储解析后的颜色序列 (1:红, 2:绿, 3:蓝) */
static uint8_t g_batch1[3] = {0};
static uint8_t g_batch2[3] = {0};

/* 2. 当前状态全局追踪 */
static Mission_State_t current_state = STATE_IDLE;

/**
 * @brief 大脑指揮中心线程入口
 */
static void brain_thread_entry(void *parameter)
{
    rt_uint32_t recved_ev; // 接收到的事件缓存

    LOG_I("Brain thread started, waiting for start signal...");

    while (1)
    {
        switch (current_state)
        {
        case STATE_IDLE:
            /* 等待按下 S1 按键发出启动信号 */
            if (rt_event_recv(&mission_event, EV_MISSION_START,
                              RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                              RT_WAITING_FOREVER, &recved_ev) == RT_EOK)
            {
                LOG_I(">>>>> Mission START signaled! <<<<<");
                current_state = STATE_SCAN_QR;
            }
            break;

        case STATE_SCAN_QR:
            /* 第一步：扫码位移准备 (左移 132mm) */
            Move_Now(MOVE_SLIDE_LEFT, 100.0f, 132.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            /* 前进 686mm */
            Move_Now(MOVE_FORWARD, 300.0f, 686.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            /* 第二步：核心：阻塞等待 MQ 包裹Pure MQ 模式 */
            QR_Task_Msg_t task_msg;
            if (rt_mq_recv(qr_result_mq, &task_msg, sizeof(task_msg), RT_WAITING_FOREVER) == RT_EOK)
            {
                /* 第三步：在大脑层行使“语义解释权” (将字符转为任务数组) */
                // "123+231"
                g_batch1[0] = task_msg.content[0] - '0';
                g_batch1[1] = task_msg.content[1] - '0';
                g_batch1[2] = task_msg.content[2] - '0';

                g_batch2[0] = task_msg.content[4] - '0';
                g_batch2[1] = task_msg.content[5] - '0';
                g_batch2[2] = task_msg.content[6] - '0';

                current_state = STATE_GO_PLATE_1;
            }
            break;

        case STATE_GO_PLATE_1:
            /* 第三步：前往原料区 (第1次) - 前进 699mm */
            Move_Now(MOVE_FORWARD, 300.0f, 699.0f);
            /* 等待底盘停稳信号 (来自 app_move_proc.c) */
            if (rt_event_recv(&mission_event, EV_MOVE_FINISHED,
                              RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                              RT_WAITING_FOREVER, &recved_ev) == RT_EOK)
            {
                LOG_I("Reached Plate Area 1.");
                current_state = STATE_PICK_PLATE_1;
            }
            break;

        case STATE_PICK_PLATE_1:
            LOG_I("[State] Picking Batch 1...");
            for (int i = 0; i < 3; i++)
            {
                /* 1. 战备整备：底座转正 0 度，爪子张开 */
                Servo_SetAngle(SERVO_BASE, 0);
                Servo_SetAngle(SERVO_ARM, CLAW_OPEN); //
                rt_thread_mdelay(1000);               // 给机械臂和视觉留出稳定时间

                /* 2. 身份校验：锁定本轮任务色 (g_batch1[i]) */
                uint8_t target_id = g_batch1[i];
                LOG_I("[Pick] Waiting for Color ID: %d", target_id);

                while (1)
                {
                    /* 只有当看到的 ID 匹配，且视野中确实有物料时才判定成功 */
                    if (vision_app_data.is_found && vision_app_data.target_id == target_id)
                    {
                        // LOG_I("Target Matched! Start Picking Item %d", i + 1);
                        break;
                    }
                    rt_thread_mdelay(20); // 降低 CPU 占用，给其他线程运行机会
                }

                /* 3. 执行物理动作：抓取并存入车内对应格位 */
                Arm_Pick_From_Raw();
                Arm_Place_To_Car(i + 1);
            }

            // LOG_I("First Batch Successfully Loaded.");
            current_state = STATE_GO_FLOOR_1;
            break;

        case STATE_GO_FLOOR_1: // 移动到粗加工区
            LOG_I("[State] Moving to Floor 1...");

            /* 1. 后退 323mm */
            Move_Now(MOVE_BACKWARD, 350.0f, 323.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            /* 2. 右转 90° (绝对角度 270°) */
            Move_Turn_Abs(270.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            /* 3. 后退 1672mm */
            Move_Now(MOVE_BACKWARD, 550.0f, 1672.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            /* 4. 再次右转 90° (绝对角度 180°) */
            Move_Turn_Abs(180.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            LOG_I("Reached Floor Area.");
            current_state = STATE_PICK_CAR_FLOOR_1;
            break;

        case STATE_PICK_CAR_FLOOR_1:
            LOG_I("[State] Unloading to Floor 1...");
            for (int i = 0; i < 3; i++)
            {
                /* 1. 阶段平移：侧移到对应的仓位大体位置 */
                if (i == 1)
                {
                    /* 前进 150mm */
                    Move_Now(MOVE_FORWARD, 100.0f, 150.0f);
                    rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                }
                else if (i == 2)
                {
                    /* 后退 300mm */
                    Move_Now(MOVE_BACKWARD, 200.0f, 300.0f);
                    rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                }

                /* 2. 战备：机械臂回正，开爪，确保视野清爽 */
                Servo_SetAngle(SERVO_BASE, 0);
                Servo_SetAngle(SERVO_ARM, CLAW_OPEN);
                rt_thread_mdelay(1500);

                /* 3. 视觉纠偏 (简洁版 XY 修正) */
                while (1)
                {
                    uint8_t ring_id = g_batch1[i] + 3;

                    // 仅当找到目标且 ID 匹配时才获取坐标
                    if (!vision_app_data.is_found || vision_app_data.target_id != ring_id)
                    {
                        rt_thread_mdelay(100);
                        continue;
                    }

                    int16_t x = vision_app_data.target_x;
                    int16_t y = vision_app_data.target_y;

                    // 优先对齐 X (中心 160)，后对齐 Y (目标 140)，容差 10 像素
                    if (abs(x - 160) > 10)
                    {
                        // X > 160 代表目标在视野右侧 (对应车体偏向货格前方)，需后退校准
                        Move_Now((x > 160) ? MOVE_BACKWARD : MOVE_FORWARD, 20.0f, 10.0f);
                        rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                    }
                    else if (abs(y - 140) > 10)
                    {
                        // Y > 140 代表目标在视野下方 (对应离货格太远)，需侧移靠近
                        Move_Now((y > 140) ? MOVE_SLIDE_RIGHT : MOVE_SLIDE_LEFT, 20.0f, 10.0f);
                        rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                    }
                    else
                        break; // 瞄准成功
                    rt_thread_mdelay(20);
                }

                /* 4. 执行取放动作 */
                Arm_Pick_From_Car(i + 1);
                Arm_Place_To_Floor();
            }
            LOG_I("Unloading to Floor 1 completed.");
            current_state = STATE_PICK_FLOOR_CAR_1;
            break;

        case STATE_PICK_FLOOR_CAR_1:
            // LOG_I("[State] Picking from Floor back to Car 1...");
            /* 1. 先前进回到中心位 (前进 150mm) */
            Move_Now(MOVE_FORWARD, 50.0f, 150.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            for (int i = 0; i < 3; i++)
            {
                /* 2. 阶段平移：运动到对应的仓位大体位置 (已适配前后布局) */
                if (i == 1) // 移向 Position 1 (前进 150mm)
                {
                    Move_Now(MOVE_FORWARD, 400.0f, 150.0f);
                    rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                }
                else if (i == 2) // 移向 Position 2 (后退 300mm)
                {
                    Move_Now(MOVE_BACKWARD, 500.0f, 300.0f);
                    rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                }

                /* 3. 战备：机械臂回正，开爪 */
                Servo_SetAngle(SERVO_BASE, 0);
                Servo_SetAngle(SERVO_ARM, CLAW_OPEN);
                rt_thread_mdelay(800);

                /* 4. 直接执行抓取并放回车内 */
                Arm_Pick_From_Floor();
                Arm_Place_To_Car(i + 1);
            }
            LOG_I("Reloading to Car 1 completed.");
            current_state = STATE_GO_FLOOR_END_1;
            break;

        case STATE_GO_FLOOR_END_1:
            LOG_I("[State] Moving to FloorEnd 1...");
            /* 1. 后退 876mm */
            Move_Now(MOVE_BACKWARD, 360.0f, 876.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            /* 2. 转向 90度 */
            Move_Turn_Abs(90.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            /* 3. 后退 864mm */
            Move_Now(MOVE_BACKWARD, 360.0f, 864.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            current_state = STATE_PICK_CAR_FLOOREND_1;
            break;

        case STATE_PICK_CAR_FLOOREND_1:
            LOG_I("[State] Unloading to FloorEnd 1...");
            for (int i = 0; i < 3; i++)
            {
                /* 1. 阶段平移：运动到对应的大体位置 (适配前后布局) */
                if (i == 1) // 移向前方
                {
                    Move_Now(MOVE_FORWARD, 200.0f, 150.0f);
                    rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                }
                else if (i == 2) // 移向后方
                {
                    Move_Now(MOVE_BACKWARD, 250.0f, 300.0f);
                    rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                }

                /* 2. 战备：机械臂回正，开爪 */
                Servo_SetAngle(SERVO_BASE, 0);
                Servo_SetAngle(SERVO_ARM, CLAW_OPEN);
                rt_thread_mdelay(1500);

                /* 3. 视觉纠偏 (适配 90° 旋转后的 XY 映射) */
                while (1)
                {
                    uint8_t ring_id = g_batch1[i] + 3;
                    if (!vision_app_data.is_found || vision_app_data.target_id != ring_id)
                    {
                        rt_thread_mdelay(100);
                        continue;
                    }

                    int16_t x = vision_app_data.target_x;
                    int16_t y = vision_app_data.target_y;

                    if (abs(x - 160) > 10)
                    {
                        Move_Now((x > 160) ? MOVE_BACKWARD : MOVE_FORWARD, 50.0f, 15.0f);
                        rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                    }
                    else if (abs(y - 140) > 10)
                    {
                        Move_Now((y > 140) ? MOVE_SLIDE_RIGHT : MOVE_SLIDE_LEFT, 50.0f, 15.0f);
                        rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                    }
                    else
                        break;
                    rt_thread_mdelay(20);
                }

                /* 4. 执行放置动作 */
                Arm_Pick_From_Car(i + 1);
                Arm_Place_To_Floor();
            }
            LOG_I("Unloading to FloorEnd 1 completed.");
            current_state = STATE_GO_PLATE_2;
            break;

        case STATE_GO_PLATE_2:
            LOG_I("[State] Returning to Plate 2...");
            /* 1. 后退 565mm */
            Move_Now(MOVE_BACKWARD, 466.0f, 565.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            /* 2. 转向到 0° (基准方向) */
            Move_Turn_Abs(0.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            /* 3. 再后退 369mm */
            Move_Now(MOVE_BACKWARD, 350.0f, 369.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            current_state = STATE_PICK_PLATE_2;
            break;

        case STATE_PICK_PLATE_2:
            LOG_I("[State] Picking Batch 2...");
            for (int i = 0; i < 3; i++)
            {
                /* 1. 战备整备：底座转正 0 度，爪子张开 */
                Servo_SetAngle(SERVO_BASE, 0);
                Servo_SetAngle(SERVO_ARM, CLAW_OPEN);
                rt_thread_mdelay(1000); // 给机械臂和视觉留出稳定时间

                /* 2. 身份校验：锁定本轮任务色 (g_batch2[i]) */
                uint8_t target_id = g_batch2[i];
                LOG_I("[Pick] Waiting for Color ID: %d", target_id);

                while (1)
                {
                    if (vision_app_data.is_found && vision_app_data.target_id == target_id)
                    {
                        break;
                    }
                    rt_thread_mdelay(20);
                }

                /* 3. 执行物理动作：抓取并存入车内对应格位 */
                Arm_Pick_From_Raw();
                Arm_Place_To_Car(i + 1);
            }

            LOG_I("Second Batch Successfully Loaded.");
            current_state = STATE_GO_FLOOR_2;
            break;

        case STATE_GO_FLOOR_2:
            LOG_I("[State] Moving to Floor 2...");

            /* 1. 后退 323mm */
            Move_Now(MOVE_BACKWARD, 350.0f, 323.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            /* 2. 右转 90° (绝对角度 270°) */
            Move_Turn_Abs(270.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            /* 3. 后退 1665mm */
            Move_Now(MOVE_BACKWARD, 600.0f, 1665.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            /* 4. 再次右转 90° (绝对角度 180°) */
            Move_Turn_Abs(180.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            LOG_I("Reached Floor Area for Batch 2.");
            current_state = STATE_PICK_CAR_FLOOR_2;
            break;

        case STATE_PICK_CAR_FLOOR_2:
            LOG_I("[State] Unloading to Floor 2...");
            for (int i = 0; i < 3; i++)
            {
                /* 1. 阶段平移：运动到对应的位置 (适配前后布局) */
                if (i == 1)
                {
                    /* 前进 150mm */
                    Move_Now(MOVE_FORWARD, 100.0f, 150.0f);
                    rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                }
                else if (i == 2)
                {
                    /* 后退 300mm */
                    Move_Now(MOVE_BACKWARD, 100.0f, 300.0f);
                    rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                }

                /* 2. 战备：机械臂回正，开爪 */
                Servo_SetAngle(SERVO_BASE, 0);
                Servo_SetAngle(SERVO_ARM, CLAW_OPEN);
                rt_thread_mdelay(1500);

                /* 3. 视觉纠偏 (适配 90° 旋转后的 XY 映射) */
                while (1)
                {
                    uint8_t ring_id = g_batch2[i] + 3;
                    if (!vision_app_data.is_found || vision_app_data.target_id != ring_id)
                    {
                        rt_thread_mdelay(100);
                        continue;
                    }

                    int16_t x = vision_app_data.target_x;
                    int16_t y = vision_app_data.target_y;

                    if (abs(x - 160) > 10)
                    {
                        Move_Now((x > 160) ? MOVE_BACKWARD : MOVE_FORWARD, 50.0f, 15.0f);
                        rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                    }
                    else if (abs(y - 140) > 10)
                    {
                        Move_Now((y > 140) ? MOVE_SLIDE_RIGHT : MOVE_SLIDE_LEFT, 50.0f, 15.0f);
                        rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                    }
                    else
                        break;
                    rt_thread_mdelay(20);
                }

                /* 4. 执行取放动作 */
                Arm_Pick_From_Car(i + 1);
                Arm_Place_To_Floor();
            }
            LOG_I("Unloading to Floor 2 completed.");
            current_state = STATE_PICK_FLOOR_CAR_2;
            break;

        case STATE_PICK_FLOOR_CAR_2:
            // LOG_I("[State] Picking from Floor back to Car 2...");
            /* 1. 先前进回到中心位 (前进 150mm) */
            Move_Now(MOVE_FORWARD, 50.0f, 150.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            for (int i = 0; i < 3; i++)
            {
                /* 2. 阶段平移：运动到对应的仓位大体位置 (已适配前后布局) */
                if (i == 1) // 移向前方
                {
                    Move_Now(MOVE_FORWARD, 200.0f, 150.0f);
                    rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                }
                else if (i == 2) // 移向后方
                {
                    Move_Now(MOVE_BACKWARD, 250.0f, 300.0f);
                    rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                }

                /* 3. 战备：机械臂回正，开爪 */
                Servo_SetAngle(SERVO_BASE, 0);
                Servo_SetAngle(SERVO_ARM, CLAW_OPEN);
                rt_thread_mdelay(800);

                /* 4. 直接执行抓取并放回车内 */
                Arm_Pick_From_Floor();
                Arm_Place_To_Car(i + 1);
            }
            LOG_I("Reloading to Car 2 completed.");
            current_state = STATE_GO_FLOOR_END_2;
            break;

        case STATE_GO_FLOOR_END_2:
            LOG_I("[State] Moving to FloorEnd 2...");
            /* 1. 后退 874mm */
            Move_Now(MOVE_BACKWARD, 450.0f, 874.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            /* 2. 转向 90度 */
            Move_Turn_Abs(90.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            /* 3. 后退 864mm */
            Move_Now(MOVE_BACKWARD, 450.0f, 864.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            current_state = STATE_PICK_CAR_FLOOREND_2;
            break;

        case STATE_PICK_CAR_FLOOREND_2:
            LOG_I("[State] Stacking to FloorEnd 2 (Second Layer)...");
            for (int i = 0; i < 3; i++)
            {
                /* 1. 阶段平移：运动到对应的大体位置 (之前已送达中心位) */
                if (i == 1) // 移向前方 (前进 150mm)
                {
                    Move_Now(MOVE_FORWARD, 150.0f, 150.0f);
                    rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                }
                else if (i == 2) // 移向后方 (后退 300mm)
                {
                    Move_Now(MOVE_BACKWARD, 250.0f, 300.0f);
                    rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                }

                /* 2. 战备：机械臂回正，开爪 */
                Servo_SetAngle(SERVO_BASE, 0);
                Servo_SetAngle(SERVO_ARM, CLAW_OPEN);
                rt_thread_mdelay(1500);

                /* 3. 视觉纠偏 (瞄准第一层已放好的货/色环) */
                while (1)
                {
                    // 瞄准点依然是当初放置第一批次时的色环 ID
                    uint8_t ring_id = g_batch1[i] + 3;

                    if (!vision_app_data.is_found || vision_app_data.target_id != ring_id)
                    {
                        rt_thread_mdelay(100);
                        continue;
                    }

                    int16_t x = vision_app_data.target_x;
                    int16_t y = vision_app_data.target_y;

                    if (abs(x - 160) > 10)
                    {
                        Move_Now((x > 160) ? MOVE_BACKWARD : MOVE_FORWARD, 50.0f, 15.0f);
                        rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                    }
                    else if (abs(y - 140) > 10)
                    {
                        Move_Now((y > 140) ? MOVE_SLIDE_RIGHT : MOVE_SLIDE_LEFT, 50.0f, 15.0f);
                        rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);
                    }
                    else
                        break;
                    rt_thread_mdelay(20);
                }

                /* 4. 执行放置动作 (码垛放置至二层) */
                Arm_Pick_From_Car(i + 1);
                Arm_Place_To_Stack();
            }
            LOG_I("Final Stacking completed.");
            current_state = STATE_GO_HOME;
            break;

        case STATE_GO_HOME:
            LOG_I("[State] Returning Home...");

            /* 1. 机械臂收回，归位到车体中心 */
            Servo_SetAngle(SERVO_BASE, 98);
            rt_thread_mdelay(500);

            /* 2. 第一阶段后退：离开暂存区 1090mm */
            Move_Now(MOVE_BACKWARD, 450.0f, 1090.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            /* 3. 转向到 0° (基准起始方向) */
            Move_Turn_Abs(0.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            /* 4. 第二阶段长距离后退：返回起始区 (后退 2162mm) */
            Move_Now(MOVE_BACKWARD, 700.0f, 2162.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            /* 5. 最终平移：侧移 120mm 对齐起始点 */
            Move_Now(MOVE_SLIDE_RIGHT, 300.0f, 120.0f);
            rt_event_recv(&mission_event, EV_MOVE_FINISHED, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved_ev);

            current_state = STATE_DONE;
            break;

        case STATE_DONE:
            LOG_I("Mission ALL COMPLETED! Returning to IDLE.");
            current_state = STATE_IDLE;
            break;

        default:
            break;
        }

        rt_thread_mdelay(10); // 轮询周期
    }
}

/**
 * @brief 按键监听线程入口 (触发启动信号)
 */
static void key_monitor_thread_entry(void *parameter)
{
    while (1)
    {
        BSP_Key_Scan();
        if (g_key_down == 1) // S1 被按下
        {
            LOG_I("Key S1 Pressed - Signaling Mission Start!");
            rt_event_send(&mission_event, EV_MISSION_START);
        }
        rt_thread_mdelay(200); // 降低频率
    }
}

/**
 * @brief 指挥部初始化
 */
int App_Task_Brain_Init(void)
{
    /* 1. 初始化事件组 */
    rt_event_init(&mission_event, "mission", RT_IPC_FLAG_FIFO);

    /* 2. 创建并启动大脑线程 */
    rt_thread_t tid = rt_thread_create("brain",
                                       brain_thread_entry, RT_NULL,
                                       2048, 10, 20); // 优先级设为 10
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }

    /* 3. 创建按键监听线程 */
    rt_thread_t ktid = rt_thread_create("key_mon",
                                        key_monitor_thread_entry, RT_NULL,
                                        512, 15, 10);
    if (ktid != RT_NULL)
    {
        rt_thread_startup(ktid);
    }

    return RT_EOK;
}

INIT_APP_EXPORT(App_Task_Brain_Init);
