# 运动规划模块

`planner` 将 `MotionRequest` 组织为可前瞻的 `PlannerBlock`，并向下游发布已冻结的
运动 block。它不计算 segment、周期、AMASS，也不操作 STEP/DIR/RMT。

## 当前实现

- 32 槽 block ring；每个 block 自己持有 `Lookahead`、`Frozen` 或 `Executing` 状态。
  已冻结前缀不会再参与前瞻，后续 block 可继续参与前瞻。
- 使用 junction deviation、反向 pass 和正向 pass 求解相邻 block 的 entry/exit 速度边界。
- 梯形 profile 是当前稳定路径；SD 雕刻实测中，block 供给和 segment ring 均保持连续。
- 当前完成事件、feed hold、resume、reset 和执行故障仍由 planner 回收；这些控制路径将在
  阶段七迁移到独立 coordinator，不参与阶段二的速度数学修改。

## 运行诊断

保留 `PLANNER-REJECT` 与 `EXECUTION-FAULT`。高频 `TRAPEZOID-TRACE` 已删除，避免诊断
串口输出改变运动时序。

## 已知限制

S 曲线不是当前验收基线。复杂图案的剩余声学差异未归因到 planner，暂不改变其算法。

## 端到端数学模型

一条 G-code 经 `command` 解析为 `MotionRequest`，其中包含绝对起点、终点、进给和激光
功率。`motion` 将 X/Y 坐标量化为步数，构造 `PlannerBlock` 后提交到 32 槽 block ring。
圆弧 G2/G3 先离散为一批线性请求，再作为批次提交，因此后续路径与普通 G1 使用同一套
junction 和前瞻约束。

### 路径与轴约束

对单个 block：

```text
dx = x1 - x0
dy = y1 - y0
L  = sqrt(dx^2 + dy^2)
u  = (dx/L, dy/L)
```

`u` 是路径单位向量。轴加速度需要投影到路径，故实际可用路径加速度为：

```text
a_path = min(Ax / |ux|, Ay / |uy|)       // 仅对参与运动的轴取最小值
```

其中 `Ax`、`Ay` 来自 `machine`。速度使用 mm/min，故用于匀加速公式时：

```text
a = a_path * 60^2                         // mm/min^2
```

### Junction 速度上限

相邻 block `P` 和 `C` 的方向分别为 `uP`、`uC`。planner 使用 Grbl junction deviation
模型，当前偏差 `jd = 0.01 mm`：

```text
c = -(uP dot uC)
s = sqrt((1 - c) / 2)                     // sin(theta/2)
a_j = min(aP, aC) * 60^2

v_junction^2 = a_j * jd * s / (1 - s)
```

特殊情形：同向直线的 `c` 接近 `-1`，可连续通过；反向的 `c` 接近 `+1`，降到最小
junction 速度；普通转角得到有限上限。每个 block 的几何入口上限还受两侧 nominal speed
约束：

```text
max_entry_i^2 = min(v_nominal_i^2,
                     v_nominal_(i-1)^2,
                     v_junction_(i-1,i)^2)
```

### 多 Block 前瞻

仅计算尚未发布为 `StepSegment` 的可变窗口；已发布前缀冻结，不能被后续 G-code 改写。

反向 pass 从队尾向前传播可减速性。尾部默认要能停到零：

```text
v_entry_i^2 <= v_entry_(i+1)^2 + 2 * a_i * L_i
```

正向 pass 从窗口入口向后传播可加速性：

```text
v_entry_(i+1)^2 <= v_entry_i^2 + 2 * a_i * L_i
```

最终 block 边界严格相接：

```text
v_exit_i = v_entry_(i+1)
```

这三个限制共同决定最终入口速度：它不能超过转角允许值，也不能超过前段可加速到的速度，
更不能超过本段能在其长度内减速到下一段的速度。

### 梯形或三角形 Profile

对已经确定 `v0`、`v1`、`vn` 和 `a` 的 block：

```text
d_acc = (vn^2 - v0^2) / (2a)
d_dec = (vn^2 - v1^2) / (2a)
```

若 `d_acc + d_dec <= L`，profile 为加速、匀速、减速的梯形；否则没有匀速段，峰值自动
降低为：

```text
v_peak = sqrt((2aL + v0^2 + v1^2) / 2)
```

### Segment、DDA 与脉冲

`stepper_prep` 按约 1 ms 目标时长切分 profile：

```text
segment_count = min(ceil(total_time_us / 1000), major_steps)
```

每段按起止距离求速度、时间和 Timer 周期。20 MHz Timer Group 消费 segment；AMASS 需要时
将一次基础步周期细分为 `2^level` 个 DDA tick。DDA 使用整个 block 的步数而不是 segment
局部步数，因此跨 segment 不会重新排列对角线脉冲：

```text
denominator = block_major_steps << AMASS_MAX
increment_axis = block_axis_steps << (AMASS_MAX - level)
error_axis(initial) = denominator / 2

error_axis += increment_axis
if error_axis > denominator:
    emit STEP(axis)
    error_axis -= denominator
```

Timer ISR 为需要运动的轴触发对应 RMT channel。RMT 以 4 MHz 输出 0.25 us idle 后的 3 us
有效 STEP 脉冲。普通 G-code 的实际位置在每个已完成 segment 按已发脉冲数更新；归零等系统
运动仍逐脉冲更新，以满足限位停止语义。
