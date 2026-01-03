/**
 ******************************************************************************
 * @file    app_task_proc.h
 * @author  lingxing
 * @brief   中央任务逻辑控制 
 ******************************************************************************
 */

#ifndef __APP_TASK_PROC_H
#define __APP_TASK_PROC_H

#include <rtthread.h>

/*
 * -----------------------------------------------------------------------
 * 1. RTOS 事件标志位定义
 * -----------------------------------------------------------------------
 */
#define EV_MISSION_START (1 << 0) /* 启动按键按下 */
#define EV_QR_FINISHED (1 << 1)   /* 二维码解析完成 (码单到手) */
#define EV_MOVE_FINISHED (1 << 2) /* 底盘移动并停稳 (坐标到达) */
#define EV_ARM_FINISHED (1 << 3)  /* 机械臂动作组执行完毕 */
#define EV_ALL_ERROR (1 << 7)     /* 系统紧急错误信号 */

/*
 * -----------------------------------------------------------------------
 * 2. 全车任务状态机枚举
 * -----------------------------------------------------------------------
 */
typedef enum
{
    STATE_IDLE = 0, /* 0. 待机 */
    STATE_SCAN_QR,  /* 1. 扫码 */

    /* --- 第一批次 (Round 1) --- */
    STATE_GO_PLATE_1,   /* 2. 前往原料区 */
    STATE_PICK_PLATE_1, /* 3. [动作] 原料区 -> 车上 */

    STATE_GO_FLOOR_1,       /* 4. 前往粗加工区 */
    STATE_PICK_CAR_FLOOR_1, /* 5. [动作] 车上 -> 粗加工区地面 (卸货) */

    STATE_PICK_FLOOR_CAR_1,    /* 6. [动作] 粗加工区地面 -> 车上 (重新上车!) */
    STATE_GO_FLOOR_END_1,      /* 7. 前往暂存区 (带着货走) */
    STATE_PICK_CAR_FLOOREND_1, /* 8. [动作] 车上 -> 暂存区地面 (最终卸货) */

    /* --- 第二批次 (Round 2) --- */
    STATE_GO_PLATE_2,   /* 9. 返回原料区 (空车) */
    STATE_PICK_PLATE_2, /* 10. [动作] 原料区 -> 车上 */

    STATE_GO_FLOOR_2,       /* 11. 前往粗加工区 */
    STATE_PICK_CAR_FLOOR_2, /* 12. [动作] 车上 -> 粗加工区地面 */

    STATE_PICK_FLOOR_CAR_2,    /* 13. [动作] 粗加工区地面 -> 车上 (重新上车!) */
    STATE_GO_FLOOR_END_2,      /* 14. 前往暂存区 */
    STATE_PICK_CAR_FLOOREND_2, /* 15. [动作] 车上 -> 暂存区码垛 (Stacking) */

    STATE_GO_HOME, /* 16. 完赛回家 */
    STATE_DONE     /* 17. 结束 */
} Mission_State_t;

/**
 * @brief  [API] 指挥部初始化 (创建线程与事件组)
 */
int App_Task_Brain_Init(void);

/*
 * 全局事件对象导出，供子模块 (动作、底盘、视觉) 发送信号使用
 */
extern struct rt_event mission_event;

#endif /* __APP_TASK_PROC_H */
