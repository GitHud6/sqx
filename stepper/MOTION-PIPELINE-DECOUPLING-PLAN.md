# 运动链路职责收敛基线

状态：2026-09-02 重写。本文是当前唯一的运动模块重构计划，取代本文件之前的历史阶段描述。

## 目标

将一组 G-code 转换为步进脉冲的过程恢复为单向、可验证的数据流。每个模块只拥有自己的状态，
不跨层重算速度、步数或硬件时序。

```text
G-code
  -> command/motion
  -> planner block
  -> profile plan
  -> segment producer
  -> segment ring
  -> timer executor
  -> RMT STEP/DIR
```

核心原则：`ray2-main` 只作为职责划分和已验证梯形离散算法的参考，不直接复制其 Arduino 外设代码，
也不以“和 ray2 不同”作为当前故障结论。当前项目的异常必须由本项目各模块边界上的数据证明。

## 一、G-code 与 Motion

### 唯一职责

- 解析 G0/G1/G2/G3、坐标、进给速度和激光请求。
- 将圆弧离散为几何运动请求。
- 计算本次运动的几何长度、轴步数、方向和请求进给速度。
- 将结果提交给 planner。

### 不得做的事情

- 不计算 junction 入口/出口速度。
- 不生成 profile、segment 或 Timer 周期。
- 不访问 segment ring、Timer 或 RMT。

### 需要删除或隔离的功能

- 删除 motion 层中任何面向执行器的周期预计算。
- 保留圆弧离散，但将其输出限定为连续的几何 block 请求。

## 二、Planner

### 唯一职责

- 管理 `PlannerBlock` 队列。
- 根据相邻 block 的方向、加速度、额定速度和 junction deviation，计算 block 的
  `entry_speed`、`exit_speed` 和 `nominal_speed`。
- 完成前向扫描和反向扫描。
- 在 block 参数冻结后，将不可变的规划结果交给 profile backend。
- 接收 block completed、cancelled、faulted 事件并维护作业生命周期。

### 不得做的事情

- 不调用 `stepper_prep_fill()`。
- 不直接操作 segment ring。
- 不直接启动或停止 Timer。
- 不计算整数步数余数、AMASS 或 RMT 参数。
- 不根据 segment ring 水位重新决定 block 的速度。

### 需要删除或迁移的功能

- 删除 planner 中周期性 `TRAPEZOID-TRACE` 及其统计状态。
- 将 segment refill 和 Timer 启动调度移出 planner，交给 producer/transport 协议。
- 将 feed hold 的剩余几何重建移出 planner 的普通规划路径，集中到独立 control coordinator。
- `s_segment_prepared_blocks` 不能继续作为 planner 速度或 block 冻结的隐式状态源。

## 三、Profile Backend

### 唯一职责

输入一个已经冻结的 block 边界：

```text
length
axis steps
entry speed
exit speed
nominal speed
acceleration
```

输出一个只读的 profile plan，描述速度、时间和位移关系。梯形 profile 必须支持：

- 加速、匀速、减速；
- 梯形无法完整展开时退化为三角形或单段加减速；
- 入口和出口速度不为零的 block；
- 很短的 block。

### 不得做的事情

- 不访问 planner queue、FreeRTOS、GPIO、Timer 或 RMT。
- 不决定 segment ring 容量和 refill 时机。
- 不处理 feed hold、reset、homing 或执行器故障。
- 不在 profile 内部偷偷切换另一种 profile backend。

### 当前基线

`grbl_trapezoid_prep` 已通过 `$PROFILE/SELFTEST` 的三组离线测试，但阶段二实机适配尚未完成验收。
它只能作为参考后端，直到运行时输出与实际执行路径完全对齐。

## 四、Stepper Prep / Segment Producer

### 唯一职责

将 profile plan 按固定时间片转换为可执行的 `StepSegment`：

- 按 1 ms 时间片推进 profile；
- 使用累计 `steps_remaining` 计算整数主轴步数；
- 使用 `dt_remainder` 传递分数步时间误差；
- 计算未缩放 Timer tick、AMASS level 和 segment 速度信息；
- 按 block 的轴步数比例生成本段 X/Y 步数；
- 将完整的 segment 写入 ring。

### 不得做的事情

- 不重新计算 planner junction。
- 不根据消费速度改变 profile 速度。
- 不在同一普通 block 内混用两套梯形离散算法。
- 不让 Timer executor 再次计算 AMASS 或重新分配普通运动轴步数。
- 不在生成函数或 ISR 内输出串口日志。

### 需要删除或迁移的功能

- 删除旧的“按总 segment 数平均分配步数”的普通梯形路径。
- 删除普通路径中 `TrapezoidExecutionPlan` 与 `GrblTrapezoidPrep` 的双重推进。
- 将 feed hold、resume 的残余 block 处理迁移到独立控制流程，避免污染普通 profile 状态。
- 清晰区分 profile plan 状态和 segment ring 状态；不得用同一个 cursor 表示两者。

## 五、StepSegment 与 Ring

### 唯一职责

`StepSegment` 是 producer 到 executor 的不可变传输记录，只包含执行所需的数据：

```text
execution id
axis steps
direction
major steps
step interval ticks
AMASS level
laser output for this segment
block completion flag
```

Ring 只负责并发安全的写入、读取和容量判断。

### 不得做的事情

- Ring 不计算速度、不计算几何、不重建 block。
- Segment 不携带可被 executor 再解释的 profile 几何状态。
- 不用 `block_axis_steps` 让 executor 重新推导 prep 结果。
- 不把 feed hold、planner 状态和硬件错误状态混入普通 segment 数据。

### 清理方向

- 审查并删除 `block_major_steps`、`block_axis_steps` 等仅为 executor 重建 DDA 而保留的字段。
- 保留字段前必须证明它是硬件执行所必需，而不是上一层遗漏的计算。
- 将激光生命周期交给单一 owner；segment 只携带已决定的输出值。

## 六、Timer Executor 与 RMT Driver

### Timer Executor 唯一职责

- 从 ring 取出当前 segment。
- 按 segment 的周期和 AMASS level 运行硬件定时器。
- 执行已经确定的 X/Y 步数分配。
- 在固定的时序路径中触发 RMT STEP。
- 设置必要的 DIR/ENABLE。
- 发布 segment completed、underflow 和硬件 fault 事件。

### RMT Driver 唯一职责

- 初始化 RMT channel。
- 接收一次 STEP 触发请求。
- 产生固定脉宽和电平的 STEP 脉冲。
- 返回硬件错误。

### 不得做的事情

- Timer executor 不访问 planner block 或 profile backend。
- 不重新缩放 AMASS，不重新计算 segment 周期。
- 不重建 feed hold residual profile。
- RMT 不参与运动规划，不保存 block 状态，不决定速度。
- 普通运动 ISR 不打印日志、不分配内存、不进入复杂临界区。

## 七、控制与生命周期

reset、feed hold、resume、homing 和执行故障属于控制层，不属于 profile 或 RMT。

统一流程：

```text
control request
  -> executor 在明确边界停止并发布事件
  -> control coordinator 清理或创建新的 immutable plan
  -> producer 发布新 segment
  -> executor 继续消费
```

必须保证：

- 一个 execution id 只有一个 owner；
- reset 后旧 segment、旧 completion 和旧 profile 不能跨代残留；
- 普通运动完成、暂停和故障的清理路径互不复用隐式状态；
- 位置状态只由实际执行结果更新，不能由 planner 的 planned position 冒充。

## 八、日志与诊断边界

保留：

- `SD-POSITION` 生命周期；
- `SD-GCODE-REJECT`、`PLANNER-REJECT`、`EXECUTION-FAULT`、`TIMER-REJECT`；
- `$POS/STATUS`；
- `$PROFILE/SELFTEST`。

删除：

- 周期性 `TRAPEZOID-TRACE`；
- `$PROTOCOL/TRACE`；
- `$PULSE/TRACE` 及其 ISR 环形采样；
- block/segment 高频串口日志；
- 只为解释声音而加入的实时打印。

诊断优先使用离线序列比较或低开销环形计数，禁止用实时串口输出改变被测运动。

## 九、八阶段实施顺序

实施阶段与前面八个模块一一对应。每个阶段只修改对应模块的 owner 和接口，不跨阶段顺手修补
下游行为。一个阶段完成并通过验收后，先提交该阶段，再开始下一阶段。八阶段完成前不重新引入
S 曲线优化，也不以调大 ring、改变 Timer 频率或增加日志代替边界修正。

### 阶段一：G-code 与 Motion 边界清理

目标：让输入层只产生几何运动请求，彻底移除其对执行时序的越权。

改动范围：G0/G1/G2/G3 解析、圆弧离散、坐标和进给转换，以及提交给 planner 的 block 请求接口。

删除或迁移：

- 删除 motion 层对 step interval、segment 数量、AMASS 和 RMT 的预计算。
- 将圆弧处理限定为几何离散；不得在此处提前决定 profile 阶段。
- 将执行器相关的状态写入迁移到 planner 或 producer 的明确接口。

前置条件：保留现有可运行的梯形基线，先记录输入 block 的长度、轴步数、方向和 feed。

验收标准：相同 G-code 在不改变 planner 代码的情况下生成相同的几何 block；直线、对角线和
圆弧离散输入均可提交；该层不再引用 profile、segment、Timer 或 RMT 类型。

### 阶段二：Planner 职责收敛

目标：使 planner 成为唯一的多 block 速度边界 owner。

改动范围：block 队列、前向扫、反向扫、junction 约束、entry/exit/nominal 速度冻结和完成事件。

删除或迁移：

- 删除 planner 对 segment refill、Timer 启停和 AMASS 的直接控制。
- 删除用 ring 水位反向修改 block 速度的逻辑。
- 将 hold、reset、fault 的残余 block 重建迁移到控制生命周期模块。
- planner 只向下游发布冻结后的不可变 block 规划结果。

前置条件：阶段一已经能稳定提供几何 block，且 block 的轴步数和方向不再由下游重算。

验收标准：多 block 的前后扫结果可重复；junction 入口和出口速度满足方向变化及向心约束；
planner 不访问 segment ring、Timer 或 RMT；长作业过程中不会因下游消费速度改变已冻结边界。

### 阶段三：Profile Backend 单一化

目标：让单个 block 的速度-时间-位移曲线只由 profile backend 计算一次。

改动范围：梯形 profile 的加速、匀速、减速，三角形退化，非零入口/出口速度和短 block 处理。

删除或迁移：

- 删除旧梯形算法与参考梯形算法并行推进同一 block 的路径。
- 删除 executor 或 producer 对 profile 阶段的二次推导。
- profile 不得访问 RTOS、planner queue、ring、Timer 或 RMT。
- 保留 `$PROFILE/SELFTEST` 作为纯计算测试，不接触实际运动链路。

前置条件：阶段二能够提供冻结的 length、entry、exit、nominal 和 acceleration。

验收标准：每个 block 明确得到一种完整 profile 计划；长 block、三角形短 block、非零边界速度
均满足位移和时间守恒；自测通过，且 profile 输出不包含硬件控制副作用。

### 阶段四：Stepper Prep / Segment Producer 收敛

目标：把 profile 计划稳定、单向地离散为 segment，消除重复积分和重复量化。

改动范围：固定时间片积分、整数步数累计、分数余数传递、轴步数分配、周期计算和 AMASS 选择。

删除或迁移：

- 删除按 segment 总数平均分配步数的旧路径。
- 删除 `TrapezoidExecutionPlan` 与参考 prep 对同一 profile 的双重推进。
- producer 不重新计算 junction 或修改 profile 速度。
- 明确区分 profile cursor、block cursor 和 ring cursor。

前置条件：阶段三的 profile plan 已有稳定的时间和位移结果。

验收标准：segment 累计轴步数等于 block 目标步数；时间误差和余数有界；AMASS 只缩放一次；
producer 可在 ring 满和 ring 空边界下继续工作，不产生零周期或非法 segment。

### 阶段五：StepSegment 与 Ring 数据瘦身

目标：将 ring 变成纯粹的不可变 segment 传输通道。

改动范围：`StepSegment` 字段、生产者/消费者索引、满/空判断、完成标记和执行代次。

删除或迁移：

- 删除 executor 为重建 DDA、profile 或 block 几何而保留的冗余字段。
- 删除 ring 内的速度计算、几何重建和生命周期判断。
- 将 execution id、方向、步数、周期、AMASS、激光输出和完成标记限定为必要传输数据。
- 将 hold、fault 和 planner 状态移出 segment 数据结构。

前置条件：阶段四已经能生成完整且自洽的 segment。

验收标准：消费者只读取 segment，不需要访问 block 或 profile；满/空切换无丢段、重复消费和
跨代残留；ring 水位不会改变 segment 内容或运动速度。

### 阶段六：Timer Executor 与 RMT Driver 收敛

目标：确保硬件层只消费已完成的 segment，使用固定、短、可预测的脉冲路径。

改动范围：硬件 Timer 回调、segment handoff、周期装载、DIR/ENABLE 时序、RMT STEP 触发和错误上报。

删除或迁移：

- 删除 executor 对 profile、planner、block 几何和 AMASS 的重新解释。
- 删除普通 ISR 中的串口输出、内存分配和复杂临界区。
- RMT 只负责固定脉宽的 STEP/DIR 脉冲，不保存运动规划状态。
- 将 underflow、invalid request 和 hardware fault 作为事件发布给上层，不在 ISR 内重规划。

前置条件：阶段五已保证 segment 字段足够且不可变；Timer 的 tick 单位和 AMASS 约定已固定。

验收标准：单轴、双轴、最高目标脉冲率和连续 segment 均能移动；无零 tick、重复 AMASS、
`TIMER-REJECT` 或脉冲断供；executor 的耗时不随 block 数量或 G-code 几何复杂度增长。

### 阶段七：控制与生命周期隔离

目标：将 reset、feed hold、resume、homing 和 fault 从普通运动数据流中隔离出来。

改动范围：停止边界、execution id、队列清理、位置状态、恢复计划和作业完成/故障状态迁移。

删除或迁移：

- 删除 profile、producer 或 executor 内部对控制命令的隐式重置和残余重建。
- 为停止、清空、重建和重新发布定义唯一的 control coordinator。
- 保证旧 segment、completion 和 profile 状态不能跨 execution id 进入新作业。
- 位置状态只接受实际执行结果，不用 planned position 冒充 machine position。

前置条件：阶段六的普通运动链已经能独立完成连续雕刻。

验收标准：普通雕刻不执行额外控制分支；reset、hold、resume、homing 和 fault 各自有可追踪的
状态迁移；新作业不会继承旧 ring、旧 profile 或旧完成事件；异常时位置状态能明确标记失效。

### 阶段八：日志与诊断最终收敛

目标：保留能定位模块边界错误的低开销信息，移除会改变运动时序的诊断代码。

改动范围：启动、SD 生命周期、位置状态、拒绝原因、执行故障和离线 profile 自测日志。

删除或迁移：

- 删除周期性 planner、block、segment、Timer 和 RMT 高频串口日志。
- 删除 ISR 采样、实时追踪命令及只服务于观察声音的统计输出。
- 统计信息优先使用离线 dump 或低开销计数器，并由明确 owner 发布。
- 保留 `SD-POSITION`、`SD-GCODE-REJECT`、`PLANNER-REJECT`、`EXECUTION-FAULT`、
  `TIMER-REJECT`、`$POS/STATUS` 和 `$PROFILE/SELFTEST`。

前置条件：前七阶段已经固定每个状态和事件的 owner。

验收标准：正常雕刻日志不会改变运动结果；故障日志能够区分输入拒绝、规划拒绝、执行故障、
定时器拒绝和位置失效；关闭诊断日志后，脉冲序列和作业完成结果保持一致。

## 十、八阶段完成后的 S 曲线工作

S 曲线不是上述八阶段中的兼容分支，而是八阶段完成并提交后的独立 profile backend 项目。
它可以复用阶段二输出的几何和速度边界，以及阶段四到六定义的传输接口，但必须独立拥有：

- 七段式或退化 profile 的状态机；
- 时间、位移、余数和边界速度计算；
- 独立的 profile 自测与验收数据；
- 与梯形 backend 分离的实验开关和故障统计。

验收要求是 S 曲线失败时梯形编译产物、运行路径和硬件执行器均不改变。只有 S 曲线单独证明
边界连续、位移守恒、无 segment gap 且不丢位置后，才允许进入默认 profile 选择。

## 当前结论

当前最优先的问题不是继续调整速度、加速度、ring 容量或 RMT 参数，而是按八个模块逐层收回
跨层职责。每完成一个阶段都必须以对应验收标准确认 owner 和数据契约没有越权，才进入下一阶段。
只有八阶段完成后，才重新评估 S 曲线；在此之前不得把 S 曲线的兼容需求混入梯形基线。
