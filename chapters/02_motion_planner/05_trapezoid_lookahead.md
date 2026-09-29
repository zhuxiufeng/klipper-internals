# 2.2 前瞻规划（Lookahead）与结偏差（Junction Deviation）

> 如果说步进脉冲是执行器的神经信号，那么前瞻规划（Lookahead）就是运动控制系统的“预判视力”。

在多轴数控系统中，刀轨通常由成千上万条极短的微小线段拟合而成。在相邻线段相交的拐角处，打印头必须减速，以避免惯性离心力摧毁步进电机或引起打印机剧烈抖动。

传统固件（如早期的 Marlin）采用的是一种粗糙的**瞬态突跳（Jerk）模型**——假设机械结构能在 0 时间内凭空突变一个速度矢量 $\Delta v$。这不仅不符合物理规律，还会导致机器在折角处发出刺耳的撞击声。

Klipper 彻底摒弃了 Jerk 模型，采用并拓展了基于物理向心加速度的**结偏差（Junction Deviation, JD）模型**与**双向扫描前瞻规划器（Two-Pass Lookahead）**。

---

## 一、 拐角动力学建模：内切圆切削与向心加速度

设前一段移动矢量为 $\vec{v}_1$，后一段移动矢量为 $\vec{v}_2$，两段路径之间的夹角为 $\theta$。

```text
       \           /
        \         /
    Move 1\       / Move 2
           \  .  /
            \ . /
             \./
              *  拐角顶点
             / \
            / R \  内切虚拟圆弧 (半径 R)
           /_____\ 
           <- δ ->  结偏差 (Junction Deviation, 切削公差)
```

我们将拐弯过程等效为：打印头以速度 $v$ 沿着一个虚拟的内切圆做向心运动。

根据向心加速度物理公式：
$$a_c = \frac{v^2}{R} \le a_{\max} \implies v^2 \le a_{\max} \cdot R$$

从几何三角关系可知，如果允许打印头偏离尖锐拐角顶点的最大物理容差为 $\delta$（即 **结偏差 Junction Deviation**），则该内切圆的半径 $R$ 满足：
$$\delta = R \cdot \left(\frac{1}{\sin(\theta/2)} - 1\right) = R \cdot \frac{1 - \sin(\theta/2)}{\sin(\theta/2)}$$

移项可得内切圆半径 $R_{jd}$：
$$R_{jd} = \delta \cdot \frac{\sin(\theta/2)}{1 - \sin(\theta/2)}$$

代入向心速度公式，得到拐角处允许的最大过渡速度平方：
$$v_{\text{junction}}^2 = a_{\max} \cdot \delta \cdot \frac{\sin(\theta/2)}{1 - \sin(\theta/2)}$$

---

## 二、 破解黑科技：`square_corner_velocity` 是如何推导出来的？

对于普通 3D 打印用户来说，配置一个几微米的抽象参数（如 `junction_deviation = 0.02`）极其不直观。

为此，Kevin O'Connor 在 Klipper 中提出了一个极为优雅的设计概念：**直角拐弯速度（`square_corner_velocity`, 简称 SCV）**。用户只需配置：“当打印头遇到标准的 90 度直角时，你希望它以多少速度滑过？”（默认通常为 5.0 mm/s）。

上位机系统内部是如何将用户配置的 $v_{\text{scv}}$ 自动换算成物理结偏差 $\delta$ 的呢？

在 [`klippy/toolhead.py`](file:///home/zxf/workspace/code/agy/book/klipper/klippy/toolhead.py#L534-L537) 中，我们可以找到这段核心源码：

```python
# klippy/toolhead.py
def _calc_junction_deviation(self):
    scv2 = self.square_corner_velocity**2
    self.junction_deviation = scv2 * (math.sqrt(2.) - 1.) / self.max_accel
```

### 数学严谨证明：
当路径转角为 90 度直角时，$\theta = 90^\circ$：
$$\theta/2 = 45^\circ \implies \sin(45^\circ) = \frac{\sqrt{2}}{2}$$

代入几何因式：
$$\frac{\sin(45^\circ)}{1 - \sin(45^\circ)} = \frac{\frac{\sqrt{2}}{2}}{1 - \frac{\sqrt{2}}{2}} = \frac{\sqrt{2}}{2 - \sqrt{2}} = \frac{1}{\sqrt{2} - 1} = \sqrt{2} + 1$$

现在我们将 Klipper 计算出的 $\delta = \frac{v_{\text{scv}}^2 (\sqrt{2} - 1)}{a}$ 代回拐角过渡速度公式：
$$v_{\text{junction}}^2 = a \cdot \delta \cdot (\sqrt{2} + 1)$$
$$v_{\text{junction}}^2 = a \cdot \left(\frac{v_{\text{scv}}^2 (\sqrt{2} - 1)}{a}\right) \cdot (\sqrt{2} + 1)$$
$$v_{\text{junction}}^2 = v_{\text{scv}}^2 \cdot \underbrace{(\sqrt{2} - 1)(\sqrt{2} + 1)}_{= 2 - 1 = 1} = v_{\text{scv}}^2$$
$$\therefore v_{\text{junction}} = v_{\text{scv}}$$

**Q.E.D. 证毕！**
这个巧妙的数学因式分解，使得用户可以在配置文件中使用非常直观的物理速度单位（mm/s），而在底层依然完整保留了基于微米级几何切削公差的动力学严谨性！

---

## 三、 双向扫描前瞻规划器（Two-Pass Lookahead）

当每一段路径的拐角最大允许过渡速度确定后，如果后面跟着一段非常短的线段，或者在第 10 段之后打印机需要彻底停下，前面的线段应该以多大的加速度运行？

Klipper 的 `LookAheadQueue`（[`klippy/toolhead.py`](file:///home/zxf/workspace/code/agy/book/klipper/klippy/toolhead.py#L135-L185)）采用经典高效的**双向动态规划扫描算法**：

```mermaid
flowchart LR
    subgraph Pass1["第 1 遍：后向扫描 (Backward Pass)"]
        direction RL
        E["终点 (速度强制为 0)"] --> D["线段 N-1: 根据可达制动距离限制最大初速"]
        D --> C["线段 ..."]
        C --> B["线段 1: 确保绝不超速超程"]
    end
    subgraph Pass2["第 2 遍：前向扫描 (Forward Pass)"]
        direction LR
        F["线段 1: 确定起步速度与巡航速度峰值"] --> G["线段 2: 正向传播巡航速度"]
        G --> H["线段 N: 输出最终梯形时间参数"]
    end
    Pass1 --> Pass2
```

### 1. 后向扫描（Backward Pass）
从待规划队列的最后一段向前遍历。假设在队列末尾打印头必须能够安全静止（$v_{\text{end}} = 0$）：
$$v_{\text{reachable\_start}}^2 = v_{\text{next\_start}}^2 + 2 \cdot a \cdot d$$
$$v_{\text{start}}^2 = \min(v_{\text{max\_junction}}^2, v_{\text{reachable\_start}}^2)$$
这一遍扫描确保了**任何时刻打印机都有足够的减速距离来停下，永远不会冲出轨道或失步**。

### 2. 前向扫描（Forward Pass）
从前向后遍历，综合上一段传递下来的实际初速度与巡航上限，解算出每一段运动块的顶峰速度 $v_{\text{cruise}}$，并划分出加速、匀速与减速的精确时间。

---

## 四、 动手实验：验证拐角速度与微线段规划

我们在随书配套脚本 [`simulations/02_lookahead_planner.py`](file:///home/zxf/workspace/code/agy/book/klipper/book/simulations/02_lookahead_planner.py) 中提取了该算法的纯 Python 原型。

你可以通过以下命令直接运行实验：

```bash
python3 book/simulations/02_lookahead_planner.py
```

### 仿真输出实测数据：
```text
[*] 最大速度: 200.0 mm/s | 最大加速度: 3000.0 mm/s²
[*] 直角拐弯速度 (SCV): 5.0 mm/s
[*] 计算所得物理结偏差 (JD): 3.4518 µm (微米)
----------------------------------------------------------------------
段号   | 距离(mm)   | 初速(mm/s)   | 巡航(mm/s)   | 末速(mm/s)   | 总耗时(s)  
----------------------------------------------------------------------
1    | 100.0    | 0.00       | 200.00     | 5.00       | 0.565   
2    | 50.0     | 5.00       | 200.00     | 13.84      | 0.311   
3    | 50.0     | 13.84      | 200.00     | 13.84      | 0.308   
4    | 2.0      | 13.84      | 110.41     | 110.41     | 0.032   
5    | 58.0     | 110.41     | 200.00     | 0.00       | 0.330   
----------------------------------------------------------------------
[验证 90度直角拐角速度]:
  -> 实际过渡速度: 5.0000 mm/s
  -> 设定 SCV 期望: 5.0000 mm/s
  -> 理论与实际相对误差: 1.776357e-15 mm/s (完全精准一致!)
```

### 重点分析：
1. **直角精确锁定**：在第 1 段末尾（90 度直角转弯），末速度被准确无误地平滑约束为 $5.0000\text{ mm/s}$，浮点误差在 $10^{-15}$ 级别。
2. **极短线段平滑过渡**：第 4 段只有短短的 $2.0\text{ mm}$，前瞻规划器识别出在这段距离内无法加速到 $200\text{ mm/s}$，因此自动将巡航极值降为 $110.41\text{ mm/s}$，平稳衔接后续运动，完全避免了由于急停急起造成的挤出机空喷或电机啸叫。

在攻克了空间轨迹规划之后，挤出轴（E 轴）还有一个更棘手的流体动力学问题——熔融塑料的粘滞与弹性蓄压。

在下一节中，我们将剖析 **压力提前（Pressure Advance）** 的动力学物理方程与 C 语言卷积平滑实现。
