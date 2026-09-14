/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-10-12     RealThread   first version
 */

#include <rtthread.h>
#include <board.h>
#include <drv_common.h>

/* CubeMX 外设初始化函数声明（实现位于 cubemx/Src/ 下）。
 * 注意：drv_common.h 里把 Error_Handler 定义成了函数式宏
 *       #define Error_Handler() _Error_Handler(__FILE__, __LINE__)
 *       而 main.h 里是函数声明 void Error_Handler(void);，两者同名会直接报
 *       "macro passed 1 arguments, but takes just 0"，所以先取消宏再包含。 */
#undef Error_Handler
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

RT_WEAK void rt_hw_board_init()
{
    extern void hw_board_init(char *clock_src, int32_t clock_src_freq, int32_t clock_target_freq);

    /* Heap initialization */
#if defined(RT_USING_HEAP)
    rt_system_heap_init((void *)HEAP_BEGIN, (void *)HEAP_END);
#endif

    hw_board_init(BSP_CLOCK_SOURCE, BSP_CLOCK_SOURCE_FREQ_MHZ, BSP_CLOCK_SYSTEM_FREQ_MHZ);

    /* ==================== CubeMX 外设初始化 ====================
     * 背景：cubemx/Src/main.c 中的 main() 被标为 __WEAK，且该文件不在编译列表中
     *      （见 Debug/cubemx/Src/subdir.mk），所以 CubeMX 生成的那一整条
     *      MX_GPIO_Init / MX_DMA_Init / MX_USARTx_UART_Init / MX_TIMx_Init
     *      初始化链从来没有被执行过 —— 串口的 GPIO 复用与 DMA、TIM1/5/9 的时基
     *      全部处于未配置状态，上层驱动再怎么导出也拿不到可用的句柄。
     * 位置：hw_board_init() 内部已经完成 HAL_Init() + 系统时钟 + SysTick，
     *      所以这里只补"外设"部分；且必须在 rt_components_board_init() 之前，
     *      否则 INIT_BOARD_EXPORT 注册的驱动会先于外设初始化执行。
     * ========================================================= */
    MX_GPIO_Init();          /* 按键 / 舵机 / 电机方向与使能引脚 */
    MX_DMA_Init();           /* 使能 DMA1 / DMA2 控制器时钟 */
    MX_USART1_UART_Init();   /* 二维码摄像头 */
    MX_USART2_UART_Init();   /* IMU (HWT101) */
    MX_USART6_UART_Init();   /* 物料识别摄像头 */
    MX_TIM5_Init();          /* 舵机 PWM (机械臂 / 底盘转盘) */
    MX_TIM1_Init();          /* 底盘 4 路步进电机脉冲 (CH1~CH4) */
    MX_TIM9_Init();          /* 舵机 PWM (底座 / 物料盘) */

    /* Set the shell console output device */
#if defined(RT_USING_DEVICE) && defined(RT_USING_CONSOLE)
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

    /* Board underlying hardware initialization */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}
