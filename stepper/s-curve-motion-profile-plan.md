# S 曲线运动 Profile 接入计划

## 背景与结论

当前直线高速雕刻使用恒加速度的梯形/三角形 profile。实机对照表明，在相同 `F6000`、相同路径和相同 STEP 后端下：

- `500 mm/s^2` 会出现可见的横向振动/线条弯曲；
- `250 mm/s^2` 的结果明显改善；
- 问题的主导变量是梯形 profile 在加速开始和结束时的加速度突变，而不是匀速段本身或 segment/RMT 断供。

目标不是以降低最大加速度换取稳定性，而是在保留较高加速能力的前提下，以限制 jerk 的 S 曲线减少对机械模态的激发。

## 当前边界

`planner` 继续拥有连续路径的速度边界：

- junction 上限；
- 前瞻的正向/反向可达性扫描；
- 每个 block 的入口、出口和 nominal 速度。

`stepper_prep` 当前拥有梯形/三角形的具体实现：

- 根据路径长度、入口/出口速度、nominal 速度和恒定加速度计算峰值速度；
- 根据距离计算速度、累计时间和 segment 的 interval；
- 生成不可变的 `StepSegment`。

S 曲线应替换后者，不应移入 planner、Timer ISR、RMT 或激光模块。Timer ISR 仍只消费已生成的 segment。

## 设计目标

1. 保留梯形/三角形 profile 作为对照与可靠回退路径。
2. 将 profile 的数学实现从 `stepper_prep.cpp` 的流程代码中隔离，避免两套算法交织。
3. 统一用距离查询速度和累计时间，保证 segment 仍按约 750 us 生成。
4. 保持 junction、冻结前缀、短段合并、M4、Feed Hold、Resume、Abort 和实际步数累积的既有语义。
5. 参数可观测、可切换，先实机验证收益，再改变默认模式。

## 不做什么

- 不以预处理抖动或随机间隔伪造 S 曲线。
- 不在 ISR 计算 jerk、浮点 profile 或随机数。
- 不直接修改 `steps_per_mm`、RMT pulse 宽度或 AMASS 来处理本问题。
- 不在第一阶段把 jerk-aware 可达性扫描塞进 planner。
- 不改变 block 的几何路径、junction 判定或冻结规则。

## 目标接口

在 `stepper` 模块中引入 profile 类型和独立的准备结果。接口应以数学数据为输入，不依赖 GPIO、FreeRTOS、planner ring 或 segment ring。

```text
输入：
  length
  entry speed
  exit speed
  nominal speed
  maximum acceleration
  maximum jerk

查询：
  speed_at_distance(distance)
  time_at_distance(distance)
  phase_at_distance(distance)
  total_time
```

建议有显式模式，而不是编译宏：

```cpp
enum class MotionProfileMode {
    Trapezoid,
    SCurve,
};
```

编译宏会让 Hold/Resume、诊断和 profile 初始化在同一函数中分叉。运行时模式或集中配置可保留同一条调用链，也能让同一份固件完成 A/B 对照。

## 阶段一：抽离当前梯形 Profile

1. 新建 stepper 内部 profile 数据结构与纯计算接口。
2. 将当前梯形/三角形的峰值速度、距离速度和距离时间计算迁入该实现。
3. `stepper_prep_start_block()` 只负责把 block 边界转换为 profile 输入；segment 生成只通过统一查询接口取速度和时长。
4. Feed Hold 的减速 profile 与 Resume 的剩余 block profile 同样走该接口，但本阶段仍使用梯形实现。
5. 在 `$POS/STATUS` 增加 profile 模式；必要时显示当前 segment 的理论加速度，不扩大 protocol task 栈。

验收标准：

- 默认梯形模式下，现有直线、短段、圆弧分解、完整 SD 文件的最终步数和轨迹不变；
- `underflows=0`、`retrigger=0`，无 watchdog、stack overflow 或 position invalidation；
- `!`、`~`、`0x18` 的已有语义不回归；
- 同一基准 G-code 的时间与现有实现仅允许量化级微小差异。

## 阶段二：S 曲线数学 Profile

1. 实现对称、限 jerk 的七段 S 曲线；短距离时自动退化为无巡航或无恒加速度段的连续 profile。
2. 保证任意距离查询得到非负、连续的速度，累计时间严格单调。
3. 用最大加速度和最大 jerk 参数计算每一段边界；profile 无法达到 nominal speed 时计算可达峰值速度。
4. 保持输入/输出速度与 planner 给定边界一致，不擅自跨 block 修改 junction 速度。
5. 将 S 曲线 profile 的距离、时间和速度函数用主机侧或纯函数测试覆盖边界情况。

验收标准：

- 静止到静止、非零 entry/exit、梯形可巡航、短距离峰值受限四类 profile 均连续；
- `speed_at_distance(0)` 等于 entry speed，终点等于 exit speed；
- segment 步数总和和 block 量化终点保持正确；
- 不进入 ISR 浮点计算，不增加 segment ring 的写入竞争。

## 阶段三：参数与实机 A/B 验证

1. 添加集中配置的 profile 模式、最大加速度和最大 jerk 参数；默认仍为梯形，直到验收通过。
2. 使用同一条纯 Y、`F6000`、恒定激光直线，对比：
   - 梯形 `500 mm/s^2`；
   - S 曲线 `500 mm/s^2`，不同 jerk 上限；
   - 梯形 `250 mm/s^2` 基线。
3. 记录加速距离、到达恒速位置、总时间、线条直线度与机械声音。
4. 选择满足直线度且总时间显著优于梯形 `250 mm/s^2` 的 jerk 参数。

验收标准：

- S 曲线 `500 mm/s^2` 的直线结果至少不差于梯形 `250 mm/s^2`；
- 运行时间明显优于梯形 `250 mm/s^2`；
- 不通过降低 G-code F、修改激光功率或改变路径来取得改善；
- 状态日志持续显示无 underflow、retrigger 和执行故障。

## 阶段四：Hold、Resume 与完整作业回归

1. 为 S 曲线实现 Hold 时从当前 segment 边界速度到零的限 jerk减速 profile。
2. Resume 按保存的 prep 上下文重新生成剩余路径的 S 曲线，而不跳步、不重放已执行路径。
3. 在包含高密度短线段、圆弧分解和 M4 的完整 SD 作业上回归。
4. 阶段通过后再评估 planner 是否需要使用 jerk-aware 可达性约束；此项单独设计，不与 profile 接入混合。

验收标准：

- `!` 平滑减速至零，`~` 连续恢复且无漏刻、重刻或位置失效；
- `0x18` 仍立即取消并清空旧 profile 上下文；
- 完整作业结束后实际位置、planned 位置和队列状态一致；
- M3/M4 语义与已有激光安全行为保持正确。

## 提交边界

每个阶段在对应的实机验收通过后单独提交。阶段一只做结构整理，不改变默认运动曲线；阶段二只引入可选数学实现；阶段三才开始决定默认参数；阶段四处理 Hold/Resume 与完整作业回归。
