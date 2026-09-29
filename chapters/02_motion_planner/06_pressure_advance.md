# 2.3 挤出机动力学与压力提前（Pressure Advance）

> 3D 打印不仅仅是几何刚体的运动学，它本质上是一个“固体机械推拉 + 粘弹性流体挤出”的机电-流体混合系统。

在高速 3D 打印中，最常遇到的打印缺陷之一是：**线条加速起步处塑料供料不足变细甚至断流，而在减速拐角处塑料堆积凸起（Corner Bulging）**。

为了解决这个问题，Klipper 实现了极为先进的**压力提前（Pressure Advance, PA）算法**与**三角窗卷积平滑算法（Smooth Time）**。

核心代码见：
* 上位机配置层：[`klippy/kinematics/extruder.py`](file:///home/zxf/workspace/code/agy/book/klipper/klippy/kinematics/extruder.py)
* C 语言解算层：[`klippy/chelper/kin_extruder.c`](file:///home/zxf/workspace/code/agy/book/klipper/klippy/chelper/kin_extruder.c)

---

## 一、 为什么单纯依靠几何联动是不够的？

在理想假设下，挤出机电机的推进速度应该与打印头在 XY 平面的合速度成严格的正比：
$$v_e(t) \propto v_{xy}(t)$$

然而现实中，耗材（PLA/PETG/ABS）从进料齿轮经过喉管，进入加热块熔化为粘稠的高分子熔体，再通过 0.4mm 的微细喷嘴挤出。根据泊肃叶流体定律（Poiseuille's law），流过喷嘴的熔体体积流率 $Q$ 与喷嘴熔融腔内的流体压强差 $\Delta P$ 成正比：
$$Q(t) \propto \Delta P(t)$$

同时，固态耗材本身以及特氟龙管具有明显的**弹性储能特性（像一根被压缩的弹簧）**。熔融腔内的压强与耗材被额外压入的位移量 $\Delta x$ 成正比：
$$\Delta P(t) \propto \Delta x(t)$$

联立以上两式可得：
$$\Delta x(t) \propto Q(t) \propto v_{xy}(t)$$

```text
       加速阶段 (喷嘴欠压)                     减速阶段 (喷嘴过压残余)
  速度 ^                                 速度 ^
       |     /------------                    | ------------\
       |    /                                 |              \
       +---+-------------> 时间               +---+-----------\---> 时间
  挤出 |                                 挤出 |              | 
  流量 |   / (供料滞后，发白发细)          流量 |              \ (熔体继续溢出，转角膨胀)
       +--+--------------> 时间               +---+-----------\---> 时间
```

* **加速时**：为了达到更高的挤出速度，必须先在熔融腔内建立更高的压强。但若电机只按名义位置进料，一部分进料量会被用来压缩弹簧（积蓄压力），导致实际从喷嘴流出的塑料滞后，拐角后起步变细。
* **减速时**：打印头虽然已经慢下来，但熔融腔内蓄积的高压仍未释放，残留的塑料会被继续强行压出喷嘴，导致拐角处堆料鼓包。

---

## 二、 基础压力提前模型

为了补偿这一滞后，必须在电机进料位移上**人为叠加一个与当前工具头速度成正比的提前补偿量**：

$$\text{Position}_{PA}(t) = \text{Position}_{\text{nominal}}(t) + k_{pa} \cdot v(t)$$

其中 $k_{pa}$ 即为用户在配置文件中设置的 `pressure_advance` 系数（单位为秒 $s$）。

我们对时间求导，看看挤出机电机的理论速度：
$$v_{PA}(t) = \frac{d}{dt} \text{Position}_{PA}(t) = v(t) + k_{pa} \cdot a(t)$$

### 致命缺陷：加速度突变带来的“齿轮打滑”
如果直接采用上述理想模型，会引入一个灾难性的工程问题：
在上一节我们看到，Klipper 的运动块是经典的三段式梯形规划，其加速度 $a(t)$ 是一个**阶跃函数（Step Function）**——从 0 瞬间跳变到 $+a$。

这会导致挤出机速度 $v_{PA}(t)$ 在加速起始瞬间**发生瞬时阶跃跳变**！
对速度阶跃求导，意味着挤出机电机在这一瞬间需要承受**无穷大的角加速度（$\infty$ Jerk）**。对于小扭矩步进电机而言，这必然会导致挤出齿轮发出“咔咔”的暴力丢步声，甚至咬断塑料耗材。

---

## 三、 Klipper 的破局之策：三角窗连续卷积滤波

为了彻底消除速度跳变、保证挤出机运动轨迹平滑，Kevin O'Connor 在 [`klippy/chelper/kin_extruder.c`](file:///home/zxf/workspace/code/agy/book/klipper/klippy/chelper/kin_extruder.c#L28-L33) 中设计了一套基于三角窗的加权积分平滑算法：

```c
// klippy/chelper/kin_extruder.c
// When pressure advance is enabled:
//     pa_position(t) = nominal_position(t) + pressure_advance * nominal_velocity(t)
// Which is then "smoothed" using a weighted average:
//     smooth_position(t) = (
//         definitive_integral(pa_position(x) * (smooth_time/2 - abs(t-x)) * dx,
//                             from=t-smooth_time/2, to=t+smooth_time/2)
//         / ((smooth_time/2)**2))
```

### 数学展开：
设平滑窗口时间为 $\tau = \text{pressure\_advance\_smooth\_time}$（默认为 0.040 秒即 40 毫秒）。
窗函数 $W(u)$ 为中心对称的三角形核：
$$W(u) = \frac{\tau}{2} - |u|, \quad u \in [-\frac{\tau}{2}, \frac{\tau}{2}]$$

平滑后的挤出机位置为：
$$P_{\text{smooth}}(t) = \frac{1}{(\tau/2)^2} \int_{t - \tau/2}^{t + \tau/2} P_{PA}(x) \left( \frac{\tau}{2} - |t - x| \right) dx$$

### 为什么选择三角窗？
1. **二次可微性**：阶跃函数经过一次积分变为连续折线，经过两次积分变为光滑样条曲线。对阶跃速度进行三角窗卷积，可以完美保证电机加速度有界且连续，彻底消除了瞬态冲击。
2. **定积分闭式解析解（Analytical Closed-Form Solution）**：
   在 [`klippy/chelper/kin_extruder.c`](file:///home/zxf/workspace/code/agy/book/klipper/klippy/chelper/kin_extruder.c#L36-L57) 中，被积函数是一个不超过 3 次的多项式：
   $$P_{PA}(x) = c_0 + c_1 x + c_2 x^2$$
   Klipper 在 C 语言扩展中直接写出了它的**不定积分代数解析式**（`extruder_integrate` 与 `extruder_integrate_time`），只需代入端点做四则运算，完全不需要进行低效的数值积分！

```c
// klippy/chelper/kin_extruder.c
// 零数值逼近误差，纳秒级解析求值
static double
extruder_integrate(double base, double start_v, double half_accel
                   , double start, double end)
{
    double half_v = .5 * start_v, sixth_a = (1. / 3.) * half_accel;
    double si = start * (base + start * (half_v + start * sixth_a));
    double ei = end * (base + end * (half_v + end * sixth_a));
    return ei - si;
}
```

---

## 小结

通过本章的三节内容，我们完整剖析了 Klipper 从宏观指令到高阶运动规划的全链路：
1. **G-code 解析流水线**：将离散文本映射为含方向单位矢量的 `Move` 对象；
2. **前瞻规划与结偏差**：以物理向心加速度为准绳，实现 $90^\circ$ 直角拐弯速度与物理公差的无缝等价映射；
3. **压力提前与流体动力学**：通过三角窗闭式积分，既补偿了熔体滞后，又彻底保护了挤出机电机不受冲击载荷破坏。

到目前为止，我们所有的规划结果都还停留在**连续的时间和位移函数** $s(t)$ 层面。

那么，连续的位移函数是如何转化为不同机架（CoreXY、笛卡尔、Delta）的各轴坐标，并最终生成步进电机的每一个离散时序脉冲的？

在下一篇中，我们将进入 **第三篇：运动学解算与步进生成**，揭秘 Klipper 最为底层的 C 语言加速引擎：`itersolve` 与步进序列压缩算法。
