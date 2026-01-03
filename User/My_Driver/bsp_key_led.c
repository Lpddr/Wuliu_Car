/**
 ******************************************************************************
 * @file    bsp_key_led.c
 * @author  lingxing
 * @brief   按键与 LED 极简驱动
 ******************************************************************************
 */

#include "bsp_key_led.h"

/* 全局变量定义 */
uint8_t g_key_val = 0;
uint8_t g_key_down = 0;
uint8_t g_key_up = 0;
uint8_t g_key_old = 0;

void BSP_LED_On(void) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); }
void BSP_LED_Off(void) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); }
void BSP_LED_Toggle(void) { HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); }

/**
 * @brief  读取按键原始映射
 * @return 0:无按键, 1:S1(PA0), 2:KEY(PA15), 3:S3(PE5), 4:S4(PE6)
 */
uint8_t BSP_Key_Read(void)
{
    /* 低电平有效 */
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
        return 1;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET)
        return 2;
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
        return 3;
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6) == GPIO_PIN_RESET)
        return 4;

    return 0;
}

/**
 * @brief  核心：四行代码消抖扫描
 * @note   建议 10ms - 20ms 调用一次
 */
void BSP_Key_Scan(void)
{
    g_key_val = BSP_Key_Read();                       /* 1. 读取当前值 */
    g_key_down = g_key_val & (g_key_val ^ g_key_old); /* 2. 捕捉下降沿 (按下) */
    g_key_up = ~g_key_val & (g_key_val ^ g_key_old);  /* 3. 捕捉上升沿 (松开) */
    g_key_old = g_key_val;                            /* 4. 更新旧状态 */
}
