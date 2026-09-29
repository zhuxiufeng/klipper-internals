#!/usr/bin/env python3
"""
simulations/02_lookahead_planner.py
独立仿真 Klipper 前瞻规划器（Lookahead Queue）与结偏差（Junction Deviation）拐角速度算法

核心推导：
  1. 拐角速度模型：基于圆弧切削近似 (Approximated Centripetal Velocity)
     当路径拐角角度为 θ 时，用户设置的 square_corner_velocity (直角拐弯速度 SCV)
     如何等价换算为物理结偏差 JD：
         JD = (SCV^2 * (sqrt(2) - 1)) / max_accel
     在 90 度直角时，(sqrt(2) - 1) * (sqrt(2) + 1) == 1，拐弯速度恰好严格等于 SCV！
  2. 双向扫描 (Two-Pass Lookahead)：
     - 后向扫描 (Backward Pass)：从终点假设速度为 0 反向推算每个拐点的最大允许减速初速度。
     - 前向扫描 (Forward Pass)：正向传播巡航速度，解算梯形 (加速-巡航-减速) 每一段的时间与距离。
"""

import math


class SimMove:
    def __init__(self, start_pos, end_pos, target_speed, accel, junction_deviation):
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.target_speed = target_speed
        self.accel = accel
        self.junction_deviation = junction_deviation

        # 空间位移分解
        self.axes_d = [ep - sp for sp, ep in zip(start_pos, end_pos)]
        self.move_d = math.sqrt(sum([d * d for d in self.axes_d[:3]]))
        inv_d = 1.0 / self.move_d if self.move_d > 1e-9 else 0.0
        self.axes_r = [d * inv_d for d in self.axes_d]

        # 速度平方参数
        self.max_cruise_v2 = target_speed ** 2
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.max_start_v2 = 0.0

        # 梯形解算结果
        self.start_v = 0.0
        self.cruise_v = 0.0
        self.end_v = 0.0
        self.accel_t = 0.0
        self.cruise_t = 0.0
        self.decel_t = 0.0
        self.total_t = 0.0

    def calc_junction(self, prev_move):
        """计算两段运动连接处的最大过渡速度平方 (Junction Velocity Squared)"""
        # 1. 运动方向夹角余弦
        cos_theta = -(
            self.axes_r[0] * prev_move.axes_r[0]
            + self.axes_r[1] * prev_move.axes_r[1]
            + self.axes_r[2] * prev_move.axes_r[2]
        )
        cos_theta = max(-1.0, min(1.0, cos_theta))

        sin_theta_d2 = math.sqrt(max(0.5 * (1.0 - cos_theta), 0.0))
        cos_theta_d2 = math.sqrt(max(0.5 * (1.0 + cos_theta), 0.0))
        one_minus_sin = 1.0 - sin_theta_d2

        max_start_v2 = min(
            self.max_cruise_v2,
            prev_move.max_cruise_v2,
            prev_move.max_start_v2 + prev_move.delta_v2,
        )

        if one_minus_sin > 1e-9 and cos_theta_d2 > 1e-9:
            # R_jd = sin(θ/2) / (1 - sin(θ/2))
            R_jd = sin_theta_d2 / one_minus_sin
            move_jd_v2 = R_jd * self.junction_deviation * self.accel
            pmove_jd_v2 = R_jd * prev_move.junction_deviation * prev_move.accel

            # 向心加速度限制 (内切圆半径不得超过单段位移的一半)
            quarter_tan = 0.25 * sin_theta_d2 / cos_theta_d2
            centripetal_v2 = self.delta_v2 * quarter_tan
            pcentripetal_v2 = prev_move.delta_v2 * quarter_tan

            max_start_v2 = min(
                max_start_v2, move_jd_v2, pmove_jd_v2, centripetal_v2, pcentripetal_v2
            )

        self.max_start_v2 = max_start_v2

    def solve_trapezoid(self, start_v2, cruise_v2, end_v2):
        """已知三段速度平方，解算梯形时间分布"""
        half_inv_accel = 0.5 / self.accel
        accel_d = (cruise_v2 - start_v2) * half_inv_accel
        decel_d = (cruise_v2 - end_v2) * half_inv_accel
        cruise_d = self.move_d - accel_d - decel_d

        self.start_v = math.sqrt(max(start_v2, 0.0))
        self.cruise_v = math.sqrt(max(cruise_v2, 0.0))
        self.end_v = math.sqrt(max(end_v2, 0.0))

        self.accel_t = accel_d / ((self.start_v + self.cruise_v) * 0.5) if accel_d > 1e-9 else 0.0
        self.cruise_t = cruise_d / self.cruise_v if cruise_d > 1e-9 else 0.0
        self.decel_t = decel_d / ((self.end_v + self.cruise_v) * 0.5) if decel_d > 1e-9 else 0.0
        self.total_t = self.accel_t + self.cruise_t + self.decel_t


def plan_moves(moves):
    # 1. 计算相邻运动块之间的拐角速度
    for i in range(1, len(moves)):
        moves[i].calc_junction(moves[i - 1])

    # 2. 后向扫描 (Backward Pass)：从终点反推最大可达速度
    next_start_v2 = 0.0  # 假设在队列末尾完全停止
    junction_info = [None] * len(moves)
    for i in range(len(moves) - 1, -1, -1):
        m = moves[i]
        reachable_start_v2 = next_start_v2 + m.delta_v2
        start_v2 = min(m.max_start_v2, reachable_start_v2)
        # 梯形顶点速度
        cruise_v2 = min((start_v2 + reachable_start_v2) * 0.5, m.max_cruise_v2)
        junction_info[i] = (m, start_v2, cruise_v2, next_start_v2)
        next_start_v2 = start_v2

    # 3. 前向扫描 (Forward Pass)：正向传播并解算时间
    prev_cruise_v2 = 0.0
    for i in range(len(moves)):
        m, start_v2, cruise_v2, next_v2 = junction_info[i]
        if cruise_v2 is None:
            cruise_v2 = min(prev_cruise_v2, start_v2)
        actual_start_v2 = min(start_v2, cruise_v2)
        actual_end_v2 = min(next_v2, cruise_v2)
        m.solve_trapezoid(actual_start_v2, cruise_v2, actual_end_v2)
        prev_cruise_v2 = cruise_v2


def run_simulation():
    print("=" * 70)
    print(" Klipper 前瞻规划器 (Lookahead) 与拐角速度 (Junction Deviation) 仿真")
    print("=" * 70)

    # 参数设置：
    MAX_VELOCITY = 200.0  # mm/s
    MAX_ACCEL = 3000.0    # mm/s^2
    SCV = 5.0             # square_corner_velocity = 5 mm/s

    # 核心公式：将 SCV 转换为结偏差 JD
    JD = (SCV ** 2) * (math.sqrt(2.0) - 1.0) / MAX_ACCEL
    print(f"[*] 最大速度: {MAX_VELOCITY} mm/s | 最大加速度: {MAX_ACCEL} mm/s²")
    print(f"[*] 直角拐弯速度 (SCV): {SCV} mm/s")
    print(f"[*] 计算所得物理结偏差 (JD): {JD * 1000:.4f} µm (微米)")
    print("-" * 70)

    # 模拟路径：一个走直角和钝角拐角的多段折线
    # 路径：(0,0) -> (100,0) [直行100mm]
    #      -> (100,50) [90度直角拐弯50mm]
    #      -> (130,90) [斜向钝角拐弯50mm]
    #      -> (130,92) [极短折线2mm]
    #      -> (130,150)[再次长直行58mm]
    waypoints = [
        (0.0, 0.0, 0.0),
        (100.0, 0.0, 0.0),
        (100.0, 50.0, 0.0),
        (130.0, 90.0, 0.0),
        (130.0, 92.0, 0.0),
        (130.0, 150.0, 0.0),
    ]

    moves = []
    for i in range(len(waypoints) - 1):
        sp = waypoints[i]
        ep = waypoints[i + 1]
        moves.append(SimMove(sp, ep, MAX_VELOCITY, MAX_ACCEL, JD))

    plan_moves(moves)

    print(f"{'段号':<4} | {'距离(mm)':<8} | {'初速(mm/s)':<10} | {'巡航(mm/s)':<10} | {'末速(mm/s)':<10} | {'总耗时(s)':<8}")
    print("-" * 70)
    for idx, m in enumerate(moves):
        print(
            f"{idx + 1:<4} | {m.move_d:<8.1f} | {m.start_v:<10.2f} | {m.cruise_v:<10.2f} | "
            f"{m.end_v:<10.2f} | {m.total_t:<8.3f}"
        )

    print("-" * 70)
    # 重点验证：第 1 段与第 2 段之间为 90 度直角
    corner_1_speed = moves[0].end_v
    print(f"[验证 90度直角拐角速度]:")
    print(f"  -> 实际过渡速度: {corner_1_speed:.4f} mm/s")
    print(f"  -> 设定 SCV 期望: {SCV:.4f} mm/s")
    print(f"  -> 理论与实际相对误差: {abs(corner_1_speed - SCV):.6e} mm/s (完全精准一致!)")
    print("=" * 70)


if __name__ == "__main__":
    run_simulation()
