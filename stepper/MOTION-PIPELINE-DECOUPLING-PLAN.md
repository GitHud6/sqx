# 运动链路解耦重构方案

## 目的

当前运动路径已经具备梯形 profile、junction 前瞻、segment ring、Timer 和 RMT
输出能力，但职责跨层混合。尤其是 profile 扩展、feed hold、segment refill 和
执行完成回收同时分布在 `planner`、`stepper_prep` 与
`timer_stepper_executor` 中。继续在这些共享路径上为 S 曲线增加分支，会继续影响
已验证的梯形运动。

本方案的目标不是立刻重新引入 S 曲线，而是先建立可验证的单向数据流，使梯形成为
干净、可回归的基线；随后才能在独立 profile backend 中实现 S 曲线。

```text
G-code
  -> Planner: geometry, lookahead, junction boundary
  -> Profile backend: immutable block execution plan
  -> Segment producer: immutable execution plan -> StepSegment ring
  -> Executor: StepSegment -> DDA -> Timer -> RMT STEP
  -> Completion event: executed result -> planner lifecycle
```

## 当前耦合事实

### Planner 同时承担调度和运动执行生命周期

`planner/planner.cpp` 除了维护 block ring、junction 与前后扫，还直接：

- 调用 `stepper_prep_start_block()` 和 `stepper_prep_fill()`；
- 根据 segment ring 空闲量决定何时生成 profile segment；
- 根据 executor busy 状态决定何时启动 timer；
- 消费 completion、释放 prep context、丢弃 block、处理 hold/resume/reset；
- 使用 `s_segment_prepared_blocks` 决定 block 是否冻结。

这使 planner 同时成为几何规划器、profile 调度器和执行生命周期协调器。block 的
前瞻可变性与 segment 是否已经发布被两个模块共同维护。

### Prep 同时承担 profile、队列和恢复状态机

`stepper/stepper_prep.cpp` 同时保存：

- `PlannerBlock` 副本与原始 block；
- 梯形 profile 和 segment 生成进度；
- context FIFO、segment ring 和 completion ring；
- 已执行步数和当前速度；
- feed hold 减速、跨 block hold 与 resume 后残余 block 重建。

因此 prep 不是单纯的 profile 采样器。它既生产数据，也拥有部分执行状态，还会在
hold/resume 时重写 block 的剩余几何。

### StepSegment 同时携带多层语义

当前 `StepSegment` 同时包含：

- timer/DDA 参数：steps、interval、AMASS、方向；
- block 级 DDA 相位输入：`block_major_steps`、`block_axis_steps`；
- 规划生命周期：`execution_id`、`completes_block`、hold 标志；
- 激光语义：功率、spindle mode、motion mode、phase。

它不是纯执行描述，因此任何 profile 或控制语义变化都会传入 Timer ISR。

### Executor 承担过多非硬件职责

`stepper/timer_stepper_executor.cpp` 的 ISR 同时负责：

- DDA 和 RMT 触发；
- segment handoff、ring underflow 和 refill 通知；
- 位置提交；
- completion 发布；
- feed hold 边界处理；
- 激光 PWM、方向、使能、timer stop/start。

Timer executor 应该只拥有当前不可变执行段和硬件外设状态，而不应理解 profile
重建、block 生命周期或 feed hold 残余几何。

### 多个状态源共同代表同一个 block

一个 block 的生命周期同时存在于 planner ring、冻结计数、prep context、segment
ring、executor motion state、completion ring、active block 和位置 checkpoint 中。
这些状态在 reset、hold、drain 和错误路径中必须全部同步，造成残留、重复清理和
`position-unknown` 风险。

### 梯形切片实现存在双轨

`trapezoid_segment_generator.cpp` 已实现按时间切片与余数补偿的纯算法，并有自测；
但实际 `stepper_prep.cpp` 仍采用按总步数等分、按距离反解时间的旧切片方式。
测试对象与实际运动对象不同，不能作为脉冲时序正确性的证据。

## 目标边界

### Planner

Planner 唯一负责：

- 将 `MotionRequest` 变成 `PlannerBlock`；
- 计算路径几何、轴约束、junction 上限；
- 通过前后扫确定每个 block 的 `entry_speed` 和 `exit_speed`；
- 将已交给 profile backend 的 block 标记为 immutable；
- 仅根据 block completion/cancellation 回收 block。

Planner 不应读取 segment ring 水位、不应启动 timer、不应调用 profile 的逐段填充，
也不应重建 hold 残余几何。

### Trapezoid Profile Backend

梯形 backend 的输入是不可变的 block 边界：

```text
length, axis steps, direction,
entry speed, exit speed, nominal speed, acceleration,
laser policy, execution id
```

输出是不可变的 `TrapezoidExecutionPlan`。它只计算：

- 梯形或三角形峰值速度；
- 总时间；
- 按时间切片得到的 step interval、每段主轴步数和速度边界；
- 必要的 DDA continuity descriptor。

它不访问 FreeRTOS、GPIO、RMT、planner queue、全局 hold 标志或实际位置。

### Segment Producer

segment producer 是唯一的 `StepSegment` ring 写入者。它从已经构建好的 execution
plan 顺序写入 segment，且不重新解释 profile 数学。其职责仅限：

- 管理 ring 的 producer cursor；
- 按低水位补充已有 plan 的 segment；
- 在 plan 全部发布后报告 `plan-published`。

### Executor

executor 是唯一的 segment ring 读取者和 STEP/DIR/RMT 所有者。它只负责：

- 消费下一个 `StepSegment`；
- 保持同一 block 的 DDA 相位；
- 设置方向、更新 timer period、触发 RMT；
- 报告 `segment executed`、`block executed` 或 `underflow`。

executor 不访问 planner block、不调用 profile/prep、不构造 residual block、不决定
hold 减速曲线。激光 duty 可保留为 segment 的执行字段，但 laser 的生命周期许可
必须有单一 owner。

### 控制与恢复

feed hold、resume、reset 必须通过独立控制协议进入链路：

```text
control request -> executor reports exact executed boundary
                -> planner/profile coordinator chooses next immutable plan
                -> producer publishes that plan
```

不允许 ISR 改写 profile 几何，也不允许 prep 在未明确执行边界的情况下猜测剩余 block。

## 分阶段实施

### 阶段一：固定梯形数学和真实生成器

1. 以 `motion_profile` 与 `trapezoid_segment_generator` 为纯计算组件。
2. 用 `TrapezoidExecutionPlan` 承载一个 block 的完整切片序列或可恢复 generator state。
3. 让真实 segment 生产路径只使用这一套 generator。
4. 删除或隔离 `stepper_prep.cpp` 中第二套按距离等分的切片算法。

验收标准：

- self-test 覆盖的 generator 与实际运动调用同一实现；
- 对长直线、短线段、三角形 profile，计划步数、实际发布步数、累计 timer 时间一致；
- 不改变 planner junction 数学、RMT 配置和 DDA 规则；
- 梯形 G-code 完整运行，无 underflow、watchdog、segment gap 或位置失效。

### 阶段二：分离 plan 生命周期与 transport

1. 以独立 `ExecutionPlanQueue` 替代 prep context 同时承担 profile 和 ring 的方式。
2. planner 只提交已冻结 block 的 execution plan，不再检查 segment ring 水位。
3. segment producer 独占 ring 写入；executor 独占 ring 读取。
4. completion 和 refill 使用明确区分的事件类型，避免复用同一个无类型 task notification。

验收标准：

- block 的唯一生命周期状态可由 execution id 查询，不依赖多个计数交叉推断；
- planner 不再直接调用 `stepper_prep_fill()` 或 `timer_stepper_executor_start_from_segment_ring()`；
- SD 流、UART 流与单条 G-code 都不会因为 producer 调度产生 stop/restart；
- `$POS` 的 planned、executing 与实际 position 在完成后统一归零或转入明确 idle 状态。

### 阶段三：收敛 executor 为硬件执行层

1. 将 executor 输入收敛为完整、不可变的 execution segment。
2. executor 仅发布 typed events：segment completed、block completed、underflow、control boundary。
3. 将激光生命周期许可移到单一控制 owner；executor 仅应用 segment duty。
4. 将 normal motion 的位置提交与 execution event 对齐，保留 system motion 的逐脉冲安全语义。

验收标准：

- executor 不包含 profile 重建、planner block 操作或 hold residual 计算；
- ISR 不调用非必要的控制逻辑；
- 同一 block 内 DDA 相位跨 segment 连续，方向只在实际变化时写入；
- RMT 仍是唯一 STEP 输出路径。

### 阶段四：在独立 backend 引入 S 曲线

仅在阶段一至三的梯形路径通过实机验收后实施。

1. 新建 `SCurveProfileBackend` 和独立的跨 block acceleration/jerk boundary context。
2. 复用 planner 的几何和 junction 速度上限，但不复用梯形的 profile context 或 hold residual 实现。
3. S 曲线输出同一种 immutable execution plan，由同一个 producer 和 executor 消费。
4. profile mode 在作业开始前固定；运行中不切换 backend。

验收标准：

- 梯形与 S 曲线可以独立构建和验证；
- 切换 S 曲线不会改变梯形编译产物中的 profile 状态或运行分支；
- 短 block、圆弧离散路径、SD 长作业均不出现 position invalidation、underflow 或 watchdog；
- S 曲线仅在跨 block 的速度、加速度、jerk 边界都满足时连续通过，否则正确退化为较少阶段的受限 profile。

## 实施原则

- 先恢复单一梯形真实路径，再谈性能和 S 曲线。
- 每个阶段只改变一个所有权边界，先完成实机验收再提交。
- 不以增加 ring 容量、增加 task 栈或 ISR 微优化替代生命周期正确性。
- 不允许 test-only generator 与实际 generator 并存。
- 不以 profile mode 条件分支侵入 planner、segment ring 或 executor；差异必须留在 profile backend。
