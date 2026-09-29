#!/usr/bin/env python3
"""
simulations/01_clock_sync_sim.py
独立仿真 Klipper 上位机与 MCU 的分布式时钟同步与漂移补偿算法

原理：
  1. MCU 硬件晶振存在制造公差与温漂（例如标称 72MHz，实际可能为 71.998MHz 或随温度微小变动）。
  2. 串口/USB 通信存在网络/协议栈传输延迟（RTT抖动）与系统调度抖动。
  3. Klipper 在 klippy/clocksync.py 中实现了带指数衰减（Exponential Decay）的
     在线加权线性回归（Linear Regression），配合最小时延追踪（Min-Half-RTT）和异常值剔除（Outlier Rejection）。
"""

import math
import random
import time

RTT_AGE = 0.000010 / (60.0 * 60.0)
DECAY = 1.0 / 30.0
TRANSMIT_EXTRA = 0.001


class ClockSyncSimulator:
    def __init__(self, nominal_freq=72000000.0):
        self.nominal_freq = nominal_freq
        self.mcu_freq = nominal_freq

        # 统计量：指数衰减加权均值与方差
        self.time_avg = 0.0
        self.time_variance = 0.0
        self.clock_avg = 0.0
        self.clock_covariance = 0.0
        self.prediction_variance = (0.001 * self.nominal_freq) ** 2
        self.last_prediction_time = 0.0

        # RTT 追踪
        self.min_half_rtt = 999999999.9
        self.min_rtt_time = 0.0

        # 当前最优时钟估计：(sample_time, clock, freq)
        self.clock_est = (0.0, 0.0, nominal_freq)

    def init_sync(self, initial_systime, initial_mcu_clock):
        self.time_avg = initial_systime
        self.clock_avg = initial_mcu_clock
        self.clock_est = (self.time_avg, self.clock_avg, self.nominal_freq)
        self.prediction_variance = (0.001 * self.nominal_freq) ** 2

    def update_best_rtt(self, sent_time, receive_time):
        half_rtt = 0.5 * (receive_time - sent_time)
        aged_rtt = (sent_time - self.min_rtt_time) * RTT_AGE
        if half_rtt < self.min_half_rtt + aged_rtt:
            self.min_half_rtt = half_rtt
            self.min_rtt_time = sent_time
            return True
        return False

    def update_regression(self, sent_time, clock):
        old_freq = self.clock_est[2]
        # 基于当前线性模型推算预测值
        exp_clock = (sent_time - self.time_avg) * old_freq + self.clock_avg
        clock_diff2 = (clock - exp_clock) ** 2

        # 异常值剔除逻辑 (Outlier Rejection)
        if (clock_diff2 > 25.0 * self.prediction_variance
                and clock_diff2 > (0.000500 * self.mcu_freq) ** 2):
            if clock > exp_clock and sent_time < self.last_prediction_time + 10.0:
                # 认为是偶然通信拥堵引入的单向过大延迟，直接丢弃
                return False
            # 持续偏差则重置方差
            self.prediction_variance = (0.001 * self.mcu_freq) ** 2
        else:
            self.last_prediction_time = sent_time
            self.prediction_variance = (
                (1.0 - DECAY) * (self.prediction_variance + clock_diff2 * DECAY)
            )

        # 指数衰减加权在线线性回归
        diff_sent_time = sent_time - self.time_avg
        self.time_avg += DECAY * diff_sent_time
        self.time_variance = (1.0 - DECAY) * (
            self.time_variance + (diff_sent_time ** 2) * DECAY
        )

        diff_clock = clock - self.clock_avg
        self.clock_avg += DECAY * diff_clock
        self.clock_covariance = (1.0 - DECAY) * (
            self.clock_covariance + diff_sent_time * diff_clock * DECAY
        )
        return True

    def handle_sample(self, sent_time, receive_time, mcu_clock):
        if not self.update_regression(sent_time, mcu_clock):
            return False, "Outlier Discarded"

        new_freq = self.clock_covariance / self.time_variance
        self.update_best_rtt(sent_time, receive_time)
        self.clock_est = (
            self.time_avg + self.min_half_rtt,
            self.clock_avg,
            new_freq,
        )
        return True, f"Estimated Freq: {new_freq:.2f} Hz"

    def estimate_clock(self, eventtime):
        sample_time, clock, freq = self.clock_est
        return int(clock + (eventtime - sample_time) * freq)


def run_simulation():
    print("=" * 65)
    print(" Klipper 时钟漂移与在线回归同步仿真 (ClockSync Simulation)")
    print("=" * 65)

    # 设定仿真条件：
    # MCU 标称频率 72MHz，真实晶振因硬件误差为 72.015MHz (偏快 15kHz)
    NOMINAL_FREQ = 72000000.0
    TRUE_MCU_FREQ = 72015000.0

    sync = ClockSyncSimulator(nominal_freq=NOMINAL_FREQ)

    curr_systime = 100.0
    curr_mcu_clock = 100.0 * TRUE_MCU_FREQ

    sync.init_sync(curr_systime, curr_mcu_clock)

    print(f"[*] 标称 MCU 频率: {NOMINAL_FREQ/1e6:.3f} MHz")
    print(f"[*] 真实 MCU 频率: {TRUE_MCU_FREQ/1e6:.3f} MHz (偏差 +15,000 Hz)")
    print(f"[*] 开始进行周期为 ~1s 的心跳包采样测试 (共 20 次采样)...")
    print("-" * 65)
    print(f"{'采样序号':<6} | {'发送时间':<8} | {'真实RTT(ms)':<10} | {'估算频率 (Hz)':<16} | {'估算误差 (Hz)'}")
    print("-" * 65)

    for i in range(1, 21):
        # 模拟上位机每 ~1 秒发送一次 get_clock
        time_step = 0.98 + random.uniform(-0.02, 0.02)
        curr_systime += time_step

        # 模拟真实网络往返延迟 (正常单程 0.5ms~1.5ms)
        one_way_latency = 0.0008 + random.uniform(-0.0003, 0.0005)
        # 偶发性系统抖动/毛刺 (第 12 次故意制造一次 50ms 异常高延迟)
        if i == 12:
            one_way_latency += 0.050

        # MCU 接收并记录到达时的硬件时钟
        mcu_arrival_time = curr_systime + one_way_latency
        measured_mcu_clock = mcu_arrival_time * TRUE_MCU_FREQ

        # 回程延迟
        return_latency = 0.0008 + random.uniform(-0.0003, 0.0005)
        receive_systime = mcu_arrival_time + return_latency

        # 送入 Klipper 算法处理
        sent_systime = curr_systime
        accepted, status = sync.handle_sample(
            sent_systime, receive_systime, measured_mcu_clock
        )

        rtt_ms = (receive_systime - sent_systime) * 1000.0
        est_freq = sync.clock_est[2]
        err_freq = est_freq - TRUE_MCU_FREQ

        if accepted:
            print(f"{i:<8} | {sent_systime:<8.2f} | {rtt_ms:<10.2f} | {est_freq:<16.2f} | {err_freq:+8.2f}")
        else:
            print(f"{i:<8} | {sent_systime:<8.2f} | {rtt_ms:<10.2f} | {'[已拦截过滤]':<16} | [异常毛刺]")

    print("-" * 65)
    # 验证最终预测精度
    test_future_systime = curr_systime + 5.0
    true_future_clock = int(test_future_systime * TRUE_MCU_FREQ)
    pred_future_clock = sync.estimate_clock(test_future_systime)
    clock_error = pred_future_clock - true_future_clock
    time_error_us = (clock_error / TRUE_MCU_FREQ) * 1e6

    print(f"\n[验证] 预测未来 5 秒后的 MCU 硬件时钟:")
    print(f"  -> 真实硬件 Tick: {true_future_clock}")
    print(f"  -> 算法预测 Tick: {pred_future_clock}")
    print(f"  -> 绝对 Tick 偏差: {clock_error} cycles")
    print(f"  -> 等效时间误差:   {time_error_us:.3f} 微秒 (µs)")
    print("=" * 65)


if __name__ == "__main__":
    run_simulation()
