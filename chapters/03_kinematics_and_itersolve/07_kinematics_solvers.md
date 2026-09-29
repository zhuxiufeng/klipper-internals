# 3.1 运动学变换（Cartesian, CoreXY, Delta）

> 无论是皮带交叉联动的 CoreXY，还是三角并联臂 Delta，在 Klipper 眼中，所有机架本质上都是“从三维笛卡尔工具头轨迹到各驱动轴电机的非线性投影映射”。

在第 2 篇中，前瞻规划器已经将用户的 G-code 切割成了一系列三维空间中的三段式梯形运动块 $s(t)$。
本节我们将深入探讨：连续的工具头空间位置 $(X(t), Y(t), Z(t))$，是如何通过各种机械拓扑结构的运动学逆解（Inverse Kinematics），映射到各个步进电机的轴位移上的。

核心涉及文件：
* 笛卡尔：[`klippy/chelper/kin_cartesian.c`](file:///home/zxf/workspace/code/agy/book/klipper/klippy/chelper/kin_cartesian.c)
* CoreXY：[`klippy/chelper/kin_corexy.c`](file:///home/zxf/workspace/code/agy/book/klipper/klippy/chelper/kin_corexy.c)
* Delta：[`klippy/chelper/kin_delta.c`](file:///home/zxf/workspace/code/agy/book/klipper/klippy/chelper/kin_delta.c)

---

## 一、 经典笛卡尔（Cartesian）：解耦正交投影

在传统的笛卡尔机架（如 i3 架构、床动打印机）中，X、Y、Z 三个电机的转动完全独立正交。

其运动学关系为最纯粹的线性比例映射：
$$x_{\text{stepper\_a}}(t) = X(t)$$
$$y_{\text{stepper\_b}}(t) = Y(t)$$
$$z_{\text{stepper\_c}}(t) = Z(t)$$

在 [`klippy/chelper/kin_cartesian.c`](file:///home/zxf/workspace/code/agy/book/klipper/klippy/chelper/kin_cartesian.c) 中，对应的 C 语言求解回调极其精简：
```c
// klippy/chelper/kin_cartesian.c
static double
cart_stepper_calc_position(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    // 获取当前时间点工具头在规划队列中的标量位移 s(t)
    double move_dist = move_get_distance(m, move_time);
    // 乘以对应的轴分量单位矢量 (axes_r)
    return sk->commanded_pos + move_dist * sk->axis_r;
}
```

---

## 二、 CoreXY 架构：皮带差动联动的几何解耦

在高速现代打印机（如 Voron 2.4、Bambu Lab、RatRig）中，CoreXY 架构是绝对的主流。
它的特点是将两颗沉重的步进电机 A 和 B 固定在机架后方，通过一根连续交叉循环穿插的正时皮带拖动轻量化的工具头。

```text
              后梁固定电机 A              后梁固定电机 B
                   [Motor A]                [Motor B]
                       |                        |
                       |       皮带交叉穿插      |
                       +-----------X------------+
                                   |
                             [ 工具头 Toolhead ]
                                  (X, Y)
```

### 1. 物理联动方程
根据皮带的几何拓扑：
* 电机 A 和电机 B **同时正转**：工具头沿 X 方向平移。
* 电机 A 和电机 B **一正一反**：工具头沿 Y 方向平移。

设 A 电机轴位移为 $S_A$，B 电机轴位移为 $S_B$，则逆运动学方程为：
$$S_A = X + Y$$
$$S_B = X - Y$$

反之，正运动学为：
$$X = \frac{S_A + S_B}{2}, \quad Y = \frac{S_A - S_B}{2}$$

### 2. Klipper 的 C 语言加速实现
在 [`klippy/chelper/kin_corexy.c`](file:///home/zxf/workspace/code/agy/book/klipper/klippy/chelper/kin_corexy.c) 中：
```c
// klippy/chelper/kin_corexy.c
static double
corexy_stepper_a_calc_position(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    double move_dist = move_get_distance(m, move_time);
    // SA = X + Y
    return sk->commanded_pos + move_dist * (m->axes_r.x + m->axes_r.y);
}

static double
corexy_stepper_b_calc_position(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    double move_dist = move_get_distance(m, move_time);
    // SB = X - Y
    return sk->commanded_pos + move_dist * (m->axes_r.x - m->axes_r.y);
}
```
可以看到，在工具头合位移已由梯形规划确定的前提下，CoreXY 求解各电机的理论位置仅需一次加减法和一次乘法，开销与纯笛卡尔机架几乎完全一样低廉！

---

## 三、 Delta 三角洲并联机械臂：非线性勾股曲面逆解

如果说笛卡尔与 CoreXY 的逆解是线性的，那么 Delta（并联外骨骼机器人）则是典型的**高阶非线性空间球相交解算**。

Delta 由三根垂直耸立的立柱（Tower A, Tower B, Tower C）构成，每个滑块连接一对定长连杆（碳纤维杆，长度为 $L$），共同悬吊着末端效应器（工具头）。

```text
       立柱 A             立柱 B             立柱 C
       |                 |                 |
     滑块 A            滑块 B            滑块 C
       \                 /                 /
     连杆 L            连杆 L            连杆 L
         \             /                 /
          \           /                 /
           +---------+-----------------+
                    [ 末端效应器 ]
                       (X, Y, Z)
```

### 1. 球面相交几何推导
设立柱 $i$ 在水平面内的固定底座坐标为 $(X_i, Y_i)$，连杆定长为 $L$。滑块在立柱上的高度为 $W_i$。
根据空间三维欧氏距离公式，末端工具头 $(X, Y, Z)$ 到立柱滑块 $(X_i, Y_i, W_i)$ 的距离恒等于杆长 $L$：

$$(X - X_i)^2 + (Y - Y_i)^2 + (Z - W_i)^2 = L^2$$

展开并对滑块高度 $W_i$ 求解：
$$(W_i - Z)^2 = L^2 - \left[ (X - X_i)^2 + (Y - Y_i)^2 \right]$$
$$W_i(t) = Z(t) + \sqrt{L^2 - (X(t) - X_i)^2 - (Y(t) - Y_i)^2}$$

### 2. 非线性带来的工程挑战
注意根号内的项：
$$D_{xy}^2 = (X(t) - X_i)^2 + (Y(t) - Y_i)^2$$
当工具头在空中沿一条笔直的水平线做匀速运动（$Z$ 恒定，$X, Y$ 线性变化）时，$W_i(t)$ 关于时间 $t$ 是一个包含平方根的**复杂超越函数**！
这意味着：**即使末端工具头在做匀速直线运动，各个立柱上的步进电机也必须做非线性的加减速变频运动。**

传统固件在此处会遇到严重的性能瓶颈——必须不断地将一条直线细分成无数个 1mm 的微小微元分别求解。

而 Klipper 的架构彻底颠覆了微元切分思路——它将所有这些连续运动学位置函数统一抽象为**黑盒连续方程 $P(t)$**，然后由底层的通用数值根求解器（`itersolve`）进行直接求交。

在下一节中，我们将深入探究 `itersolve` 是如何用纯 C 语言结合割线法与二分法，将连续运动方程解构成微秒级精度的脉冲时序的。
