/**
 ******************************************************************************
 * @file    bsp_key_led.h
 * @author  lingxing
 * @brief   按键与 LED 驱动
 ******************************************************************************
 */

#ifndef __BSP_KEY_LED_H
#define __BSP_KEY_LED_H

#include <stdint.h>
#include "board.h"
#include <stm32f4xx_hal.h>

/**
 * @usage 使用说明:
 * 1. 在计时器或循环中调用: BSP_Key_Scan();
 * 2. 判断按键: if(g_key_down == 1) { // 对应 S1 按下 }
 * 3. LED控制: BSP_LED_Toggle();
 */

/* 全局按键状态变量 (VET6 核心逻辑) */
extern uint8_t g_key_val;  /* 当前键值 */
extern uint8_t g_key_down; /* 触发按键 (仅在按下瞬间为真) */
extern uint8_t g_key_up;   /* 释放按键 */
extern uint8_t g_key_old;  /* 上次键值 */

/* --- LED 接口 --- */

void BSP_LED_On(void);
void BSP_LED_Off(void);
void BSP_LED_Toggle(void);

/* --- 按键接口 --- */
void BSP_Key_Scan(void);    /* 核心：四行代码消抖扫描 */
uint8_t BSP_Key_Read(void); /* 读取原始 IO 映射 */

#endif /* __BSP_KEY_LED_H */
