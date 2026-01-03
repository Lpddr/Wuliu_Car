/**
 * @file    app_qr_proc.h
 * @brief   二维码识别处理任务
 */

#ifndef __APP_QR_PROC_H
#define __APP_QR_PROC_H

#include <rtthread.h>

/**
 * @brief 二维码任务消息包 (用于纯 MQ 传输)
 */
typedef struct
{
    char content[16]; /* 任务字符串，例如 "123+231" */
} QR_Task_Msg_t;

extern rt_mq_t qr_result_mq; /* 核心队列：扫码结果直投位 */
extern rt_mq_t qr_mq;        /* 内部队列：ISR -> 线程 */

/**
 * @brief  [API] 初始化二维码识别任务
 * @return 0: 成功, -1: 失败
 */
int App_QR_Init(void);

#endif /* __APP_QR_PROC_H */
