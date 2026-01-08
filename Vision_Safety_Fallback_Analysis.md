# 深度诘问：摄像头卡死怎么办？

> **第二问**：
> “假如摄像头突然卡死，不再更新全局变量，但你的 PID 线程还在狂读这个‘旧值’，你的车会发生什么？你在软件层面做了什么兜底机制吗？”

这是一道非常经典的**工程容错性 (Fault Tolerance)** 面试题。它考察的不是你会不会写代码，而是你有没有**“系统安全意识”**。

## 1. 并没有悬念的后果：积分饱和与暴走

如果没有任何兜底机制（正如目前代码所示），当摄像头在 $t_0$ 时刻卡死，全局变量 `vision_app_data` 将永远保持 $t_0$ 时刻的值（比如 $X_{err} = 50$）。

### 灾难过程推演：
1.  **传感器冻结**：$X_{err}$ 恒定为 50。
2.  **PID 狂读**：运动控制线程每 20ms 读一次，发现误差还在。
3.  **积分累积 (I项)**：$I = I + 50$。随着时间推移，积分项迅速膨胀（Integral Windup）。
4.  **发狂**：电机输出迅速达到 PWM 100% 满转。
5.  **结果**：小车会像“瞎了眼的野牛”一样，朝着一个方向全速撞过去，直到撞墙或把电机烧毁。

---

## 2. 软件兜底：超时看门狗 (Software Watchdog)

在工业级代码中，**永远不要相信外部传感器**。
我们在 `app_vision_proc.c` 中其实已经埋下了一颗伏笔（虽然目前的消费者代码还没用上）：

```c
/* 生产者更新了时间戳 */
vision_app_data.last_update = rt_tick_get();
```

### 必加的兜底代码 (Code Implementation)
我们需要在消费者 (`app_task_proc.c` 或PID控制处) 增加**数据鲜度检查 (Freshness Check)**。

#### 修改后的读取逻辑：

```c
/* 定义一个超时阈值 (例如 200ms) */
#define VISION_TIMEOUT_MS  200

/* 安全读取函数 */
void Safe_Vision_Control(void)
{
    rt_tick_t current_tick = rt_tick_get();
    
    /* 1. 检查数据是否“变质” */
    if ((current_tick - vision_app_data.last_update) > rt_tick_from_millisecond(VISION_TIMEOUT_MS))
    {
        /* 🚨 紧急情况：数据过期了！摄像头可能卡死了 */
        LOG_E("[Safety] Vision Data Timeout! Stopping car...");
        
        Move_Stop();        // 立即停车
        vision_app_data.is_found = RT_FALSE; // 强制置为未找到
        return;
    }

    /* 2. 数据新鲜，正常 PID 控制 */
    int16_t x = vision_app_data.target_x;
    // ... 执行原有逻辑
}
```

## 3. 还有更高级的吗？

除了简单的超时，我们还可以做：
1.  **不变性检测**：如果连续 N 帧数据的 X, Y 坐标**完全一模一样**（连噪声都没有），说明图像可能卡住了（ISP 冻结）。
2.  **动态降级**：如果视觉挂了，自动切换到 **纯盲跑 (Dead Reckoning)** 模式，依靠编码器里程计走完剩下的路（虽然不准，但比暴走强）。

## 4. 总结

*   **现状**：当前代码**会发生暴走**。
*   **对策**：必须利用 `last_update` 时间戳，在读取前计算 `dt`。
*   **原则**：控制算法中，**No Data (没数据) 优于 Old Data (旧数据)**。没数据最多停在那，旧数据会要命。
