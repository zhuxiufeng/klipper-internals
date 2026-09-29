# 5.1 零 RTOS 的轻量级事件/定时器调度内核

> 在资源受限的嵌入式世界中，最快的代码就是“不存在的代码”。Klipper MCU 固件没有选择 FreeRTOS 或任何第三方实时操作系统，而是手写了一套数十行汇编级的极简微内核。

很多人初次翻阅 Klipper MCU 端源码（`src/` 目录）时都会感到惊讶：
通常复杂的数控固件动辄需要多线程、任务互斥锁（Mutex）、信号量（Semaphore）以及复杂的任务堆栈上下文切换（Context Switch）。
而在 Klipper 的微控制器固件中，甚至连操作系统堆栈都不需要切换，所有代码均运行在单一的系统栈上。

本节我们将深入 [`src/sched.c`](file:///home/zxf/workspace/code/agy/book/klipper/src/sched.c)，剖析这套极其精巧的微调度器（Micro-Scheduler）。

---

## 一、 双层调度模型：硬件中断（Timers）与主循环任务（Tasks）

Klipper 将 MCU 上的所有工作严格划分为两级：

```mermaid
flowchart TD
    subgraph Layer1["第 1 层：硬实时定时器中断队列 (Hardware Timers)"]
        direction LR
        T1["timer_list 头指针"] --> T2["步进脉冲事件 (stepper_event)"]
        T2 --> T3["下一个定时器..."]
        T3 --> T4["periodic_timer (100ms 保活)"]
        T4 --> T5["sentinel_timer (哨兵节点)"]
    end
    
    subgraph Layer2["第 2 层：软实时普通任务轮询 (Background Tasks)"]
        direction LR
        K1["命令包解析 (command.c)"] --> K2["ADC 温度采样 (adc.c)"]
        K2 --> K3["状态统计更新 (basecmd.c)"]
        K3 --> K1
    end

    Layer1 -.->|硬件定时器比较中断 (随时抢占)| Layer2
```

1. **硬实时定时器列表（Timers）**：
   * 负责步进电机 STEP/DIR 引脚电平翻转、高精度 PWM 等。
   * 完全在**硬件定时器比较中断（IRQ）**中执行，纳秒级响应，执行时间必须极短（通常几十个时钟周期）。
2. **软实时后台任务（Tasks）**：
   * 负责串口命令解析、ADC 读取、心跳与看门狗刷新。
   * 在 `main()` 函数的主 `while(1)` 循环中协作式轮询执行。如果发生定时器中断，CPU 自动挂起主循环，执行完中断后再返回继续。

---

## 二、 哨兵节点与环形环绕：无 NULL 检查的链表插入

在一般的单向链表中，插入一个按时间排序的节点通常需要处理多种边界情况：
* 链表是否为空？
* 是否需要插在最头部？
* 遍历到尾部是否会遇到 `NULL`？

在每秒数十万次的中断调度中，每一个 `if (p == NULL)` 都是对分支预测和 CPU 周期的巨大浪费。

Kevin O'Connor 在 [`src/sched.c`](file:///home/zxf/workspace/code/agy/book/klipper/src/sched.c#L34-L64) 中运用了极为高超的数据结构技巧：

```c
// src/sched.c
static struct timer periodic_timer, sentinel_timer;

static uint_fast8_t
periodic_event(struct timer *t)
{
    sched_wake_tasks();
    periodic_timer.waketime += timer_from_us(100000); // 每 100ms 触发
    sentinel_timer.waketime = periodic_timer.waketime + 0x80000000;
    return SF_RESCHEDULE;
}

static struct timer sentinel_timer = {
    .func = sentinel_event,
    .waketime = 0x80000000,
};
```

### 绝妙之处：
1. **永不为空的链表**：链表中永远常驻 `periodic_timer`（周期定时器）和 `sentinel_timer`（哨兵定时器）。
2. **32位无符号时钟回绕（Wraparound）处理**：
   哨兵的触发时间永远被设为 `periodic_timer.waketime + 0x80000000`（刚好相差 21.47 亿个 Tick，即整型模空间的一半）。
   根据有符号差分比较规则：
   ```c
   static inline int timer_is_before(uint32_t time1, uint32_t time2) {
       return (int32_t)(time1 - time2) < 0;
   }
   ```
   **任何新添加的合法定时器，其触发时间绝对小于哨兵定时器！**
3. **消除分支预测**：在遍历链表寻找插入位置时：
   ```c
   static void __always_inline
   insert_timer(struct timer *pos, struct timer *t, uint32_t waketime)
   {
       struct timer *prev;
       for (;;) {
           prev = pos;
           pos = pos->next;
           if (timer_is_before(waketime, pos->waketime))
               break;
       }
       t->next = pos;
       prev->next = t;
   }
   ```
   循环内部**完全不需要检查 `pos == NULL`**！因为一旦到达末尾，哨兵必定会满足 `break` 条件。这省去了宝贵的比较跳转指令。

---

## 三、 定时器删除的无锁替换：`deleted_timer`

当上位机需要临时取消一个定时器时，如果直接从链表中解开指针，正在执行的中断上下文可能会发生竞态破坏。

Klipper 设计了一个静态占位的 `deleted_timer`：
```c
// src/sched.c
static uint_fast8_t deleted_event(struct timer *t) {
    return SF_DONE;
}
```
当删除头节点的活跃定时器时，固件只需原子替换为 `deleted_timer`。当硬件中断触发时，执行空操作并自动从链表脱落，以零锁开销保证了极端高并发中断下的线程安全性。

---

## 小结

Klipper 的 MCU 调度内核向我们展示了极简系统设计的魅力：
* 放弃臃肿的 RTOS 抢占与上下文切换开销；
* 用双层模型解耦“硬实时脉冲”与“软实时通信”；
* 依靠精妙的哨兵节点与模空间运算，将链表插入的汇编指令压榨到极限。

有了这套高吞吐的微调度内核，步进电机驱动中断是如何在它之上运行的？
在下一节中，我们将剖析整个 MCU 固件中最核心的函数——[`src/stepper.c`](file:///home/zxf/workspace/code/agy/book/klipper/src/stepper.c) 中的 `stepper_event`。
