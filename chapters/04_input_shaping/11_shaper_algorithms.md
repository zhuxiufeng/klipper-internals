# 4.2 输入整形算法族（ZV, ZVD, MZV, EI, 2Hump-EI）

> 输入整形绝不是简单的低通滤波或模糊算法，而是一组严格放置在复频域共振极点上的离散冲激响应（FIR Filter）。

在上一节我们建立了单频率共振反相抵消的物理直觉。但在真实世界中，3D 打印机的固有频率不是静态的——打印头在热床上不同位置、皮带松紧度以及载物重量变化都会使共振频率产生漂移。

为了在**整形时间延迟（Shaper Duration）**与**抗频漂鲁棒性（Robustness）**之间权衡，控制界发展出了完整的输入整形算法族。

核心源码见：
* Python 算法配置层：[`klippy/extras/shaper_defs.py`](file:///home/zxf/workspace/code/agy/book/klipper/klippy/extras/shaper_defs.py)
* C 语言轨迹卷积层：[`klippy/chelper/kin_shaper.c`](file:///home/zxf/workspace/code/agy/book/klipper/klippy/chelper/kin_shaper.c)

---

## 一、 输入整形器的通用数学约束方程

任何一个输入整形器都可以表示为 $n$ 个离散冲激的加权和：
$$\text{IS}(t) = \sum_{i=1}^n A_i \delta(t - T_i)$$

为了让整形器在工业物理机床上正常工作，它必须满足以下三大约束：

1. **单位增益约束（Unity Gain）**：
   $$\sum_{i=1}^n A_i = 1$$
   保证经过整形后的最终位置与切片原始期望位移严格一致，不产生几何缩放偏差。
2. **非负振幅约束（Positivity）**：
   $$A_i > 0, \quad \forall i$$
   禁止出现负振幅脉冲，避免电机在加速过程中发生反向制动撕扯。
3. **共振点零残余振动（Zero Vibration）**：
   在机架共振复频率 $s = -\zeta \omega_n \pm j \omega_d$ 处，整形器的拉普拉斯变换零点必须精确覆盖机架极点：
   $$V(\omega_n, \zeta) = e^{-\zeta \omega_n T_n} \sqrt{\left(\sum_{i=1}^n A_i e^{\zeta \omega_n T_i} \cos(\omega_d T_i)\right)^2 + \left(\sum_{i=1}^n A_i e^{\zeta \omega_n T_i} \sin(\omega_d T_i)\right)^2} = 0$$

---

## 二、 算法族特性与参数推导

在 [`klippy/extras/shaper_defs.py`](file:///home/zxf/workspace/code/agy/book/klipper/klippy/extras/shaper_defs.py) 中，Klipper 实现了以下五大主力整形器：

```text
整形器脉冲结构对比：
ZV:       |--------| (2 脉冲, 耗时 0.5 Td)
MZV:      |----|---| (3 脉冲, 耗时 0.75 Td)
ZVD:      |----|---| (3 脉冲, 耗时 1.0 Td)
EI:       |----|---| (3 脉冲, 宽陷波谷底)
2HUMP_EI: |--|--|--| (4 脉冲, 耗时 1.5 Td, 极宽陷波)
```

设周期 $T_d = \frac{1}{f_n \sqrt{1 - \zeta^2}}$，阻尼衰减系数 $K = e^{-\frac{\zeta \pi}{\sqrt{1 - \zeta^2}}}$：

### 1. ZV（Zero Vibration，零振动整形器）
* **脉冲数量**：2 个。
* **参数推导**：
  $$T = [0, 0.5 T_d], \quad A = \frac{1}{1 + K} [1, K]$$
* **特点**：延迟最短（仅需半个共振周期，约 10ms），转角圆角（Smoothing）最小。但陷波带极窄，一旦机架频率因载荷漂移 10%，残余振动立刻飙升到 15% 以上。

### 2. ZVD（Zero Vibration and Derivative，零振动导数整形器）
* **脉冲数量**：3 个。
* **核心数学改进**：不仅强制 $V(\omega_n) = 0$，还强制残余振动对频率的一阶导数也为零：
  $$\left. \frac{d V}{d \omega} \right|_{\omega = \omega_n} = 0$$
* **参数推导**：
  $$T = [0, 0.5 T_d, T_d], \quad A = \frac{1}{(1 + K)^2} [1, 2K, K^2]$$
* **特点**：陷波曲线底部极其平缓宽广，抗频率漂移能力强，但整形耗时翻倍（$1.0 T_d$），转角圆角略大。

### 3. MZV（Modified Zero Vibration，改良型零振动）
由 Klipper 核心贡献者 Dmitry Butyugin 针对 3D 打印场景专门优化设计：
* **脉冲数量**：3 个。
* **参数特性**：总耗时控制在 $0.75 T_d$（介于 ZV 和 ZVD 之间）。
* **评价**：3D 打印领域的“黄金平衡点”，在极短的平滑损失下提供了显著优于 ZV 的抗频漂鲁棒性。

### 4. EI 与 2HUMP_EI（Extra Insensitive，超不敏感整形器）
* **核心思想**：放弃“共振频率处残余振动必须严格为 0”的强执念，允许在中心频率附近保留微弱的容许振动（例如 $V \le 5\%$），换取一个跨度极宽的“马鞍形（Hump）”抑制阻带。
* **适用场景**：大型床动打印机（Y 轴随着打印高度增加、模型重量成倍攀升，共振频率大幅偏移）。

---

## 三、 C 语言层的高效轨迹卷积与质心对齐

确定了脉冲参数后，如何将输入整形施加到运动学中？

在 [`klippy/chelper/kin_shaper.c`](file:///home/zxf/workspace/code/agy/book/klipper/klippy/chelper/kin_shaper.c#L32-L45) 中，Klipper 没有采用耗时的离散卷积算法，而是将输入整形抽象为一个**动态时间偏移变换器**：

$$x_{\text{shaped}}(t) = \sum_{i=1}^n A_i \cdot x_{\text{raw}}(t + T_i - t_{\text{centroid}})$$

### 零相位延迟黑科技：`shift_pulses`
如果在时域直接向后延迟各脉冲，会导致整个打印轨迹向未来滞后，产生相位差。
Klipper 在初始化整形器时，首先计算脉冲序列的**时间质心（Centroid）**：

$$t_{\text{shift}} = \sum_{i=1}^n A_i \cdot T_i$$

然后将所有脉冲关于 $t_{\text{shift}}$ 进行中心平移：
```c
// klippy/chelper/kin_shaper.c
static void
shift_pulses(struct shaper_pulses *sp)
{
    double ts = 0.;
    for (int i = 0; i < sp->num_pulses; ++i)
        ts += sp->pulses[i].a * sp->pulses[i].t;
    // 平移脉冲时间轴，使其质心落于 0 点
    for (int i = 0; i < sp->num_pulses; ++i)
        sp->pulses[i].t -= ts;
}
```

**这一步带来的惊人数学特性：**
对于匀速直线运动 $x(t) = v \cdot t$：
$$x_{\text{shaped}}(t) = \sum_{i=1}^n A_i \cdot v \cdot (t + t_i - t_{\text{shift}}) = v \cdot t \underbrace{\sum A_i}_{=1} + v \cdot \underbrace{\left(\sum A_i t_i - t_{\text{shift}}\right)}_{=0} = v \cdot t$$
**输入整形器对于匀速直线运动具有恒等映射（Identity Transformation）特性，完全不会引入任何空间位移偏差或拖尾！**

---

## 四、 动手实验：运行频率响应仿真

我们在随书配套脚本 [`simulations/03_input_shaper_freq_resp.py`](file:///home/zxf/workspace/code/agy/book/klipper/book/simulations/03_input_shaper_freq_resp.py) 中提取了该算法族的频域模值计算。

你可以通过以下命令直接运行：

```bash
python3 book/simulations/03_input_shaper_freq_resp.py
```

### 实测频响扫描结果（机架共振中心频率 $45\text{ Hz}$）：
```text
实际振动频率(Hz)       | 未整形 (None)     | ZV         | ZVD        | MZV       
----------------------------------------------------------------------
30.0             | 100.0        % | 44.70    % | 19.98    % | 42.24    %
40.0             | 100.0        % | 14.98    % | 2.24     % | 19.99    %
45.0 (中心共振点) | 100.0        % | 0.00     % | 0.00     % | 11.40    %
50.0             | 100.0        % | 14.46    % | 2.09     % | 5.09     %
60.0             | 100.0        % | 40.23    % | 16.19    % | 0.11     %
```

从数据可以清晰看出：
* 在 $45\text{ Hz}$ 中心处，ZV 和 ZVD 均实现 $0.00\%$ 残余振动；
* 当频率漂移到 $50\text{ Hz}$ 时，ZV 残余振动暴增至 $14.46\%$，而 ZVD 依然牢牢压制在 $2.09\%$，完美印证了控制理论关于导数平整带的预测。

但这里还留有一个先决条件：**我们如何知道机架的固有共振频率究竟是 45Hz 还是 60Hz？**

在下一节中，我们将剖析 Klipper 是如何通过 ADXL345 加速度计采集数万个离散样本，并在 Python 中通过快速傅里叶变换（FFT）自动测定机架频谱特性的。
