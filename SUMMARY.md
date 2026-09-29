# 目录 (Summary)

* [前言与导读](README.md)

## 第一篇：架构哲学与通信基石
* [1.1 传统固件瓶颈与分布式双脑设计](chapters/01_architecture/01_design_philosophy.md)
* [1.2 数据字典与极简二进制通信协议](chapters/01_architecture/02_protocol_and_encode.md)
* [1.3 驯服时钟漂移：微秒级时钟同步数学原理](chapters/01_architecture/03_clock_synchronization.md)

## 第二篇：运动规划与轨迹生成
* [2.1 从 G-code 到运动块（Move Queue）](chapters/02_motion_planner/04_gcode_pipeline.md)
* [2.2 前瞻规划（Lookahead）与结偏差（Junction Deviation）](chapters/02_motion_planner/05_trapezoid_lookahead.md)
* [2.3 挤出机动力学与压力提前（Pressure Advance）](chapters/02_motion_planner/06_pressure_advance.md)

## 第三篇：运动学解算与步进生成
* [3.1 运动学变换（Cartesian, CoreXY, Delta）](chapters/03_kinematics_and_itersolve/07_kinematics_solvers.md)
* [3.2 迭代步进求解器（itersolve）原理与 C 扩展加速](chapters/03_kinematics_and_itersolve/08_itersolve_internals.md)
* [3.3 步进序列压缩算法（Step Compression）](chapters/03_kinematics_and_itersolve/09_step_compression.md)

## 第四篇：共振抑制与控制理论
* [4.1 机械共振物理模型与瞬态响应](chapters/04_input_shaping/10_vibration_physics.md)
* [4.2 输入整形算法族（ZV, ZVD, MZV, EI, 2Hump-EI）](chapters/04_input_shaping/11_shaper_algorithms.md)
* [4.3 ADXL345 采样流水线与 FFT 频谱估算](chapters/04_input_shaping/12_resonance_testing.md)

## 第五篇：MCU 硬实时微内核
* [5.1 零 RTOS 的轻量级事件/定时器调度内核](chapters/05_mcu_firmware/13_micro_scheduler.md)
* [5.2 极致精准的步进定时器队列（stepper.c）](chapters/05_mcu_firmware/14_stepper_timer_irq.md)
* [5.3 多 MCU 与 CAN 总线异构节点协同](chapters/05_mcu_firmware/15_multi_mcu_sync.md)
* [5.4 源码诊断实战：“Timer too close” 终极排查指南](chapters/05_mcu_firmware/16_debug_timer_too_close.md)

## 配套实验代码 (Simulations)
* [时钟漂移与回归仿真](simulations/01_clock_sync_sim.py)
* [前瞻梯形规划器原型](simulations/02_lookahead_planner.py)
* [输入整形频率响应绘制](simulations/03_input_shaper_freq_resp.py)
