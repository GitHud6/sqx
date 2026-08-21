# motion

阶段三状态：运动请求桩接口已实现。当前会校验并记录合法请求，返回 `Accepted`；非法请求返回 `Rejected`。Planner 满载产生的 `Busy` 结果将在后续 Planner 阶段实现。

阶段四约束：`command` 只有在 `motion_submit()` 返回 `Accepted` 后才提交位置、模态、进给和激光状态；解析失败、`Busy` 或 `Rejected` 都不会改写正式解析状态。

本模块是 G-code 解析层与后续 Planner 之间的运动请求边界。

当前只实现 `motion_submit()` 桩接口：记录最后一次 `MotionRequest` 和请求数量，并返回 `Accepted`。它不检查 Planner 空间、不执行加减速、不操作 GPIO，也不产生 STEP 脉冲。

## 当前接口

`MotionRequest` 包含：

- 运动类型：Rapid 或 Linear
- 起点和绝对目标，单位为毫米
- 当前有效进给，单位为 mm/min
- 当前行生效的激光功率 `S`
- 预留的主轴模式

当前只有 `command` 层调用 `motion_submit()`，因此请求记录不需要并发保护。以后增加 APP、SD 或其他命令来源时，应把提交者统一到 motion owner，再决定队列和锁的策略。

## 后续实施步骤与验收标准

### 步骤一：替换为 Planner 提交

实现 Planner 空间检查、软限位检查和 block 入队。

验收标准：Planner 接受的请求才返回 `Accepted`；满载返回 `Busy`；越界或非法参数返回 `Rejected`。

### 步骤二：接入速度与加速度规划

使用 RayCube 的每轴 `steps_per_mm`、最大速度和加速度生成可执行 block。

验收标准：请求中的目标、F、加速度约束被完整传入 Planner，且连续短行不会丢失顺序或 S 值。

### 步骤三：接入步进执行器

将 Planner block 转换为 STEP/DIR 时序，交给 `stepper` 执行。

验收标准：stepper 只执行 Planner 已确认的 block；急停、错误和队列耗尽时电机回到安全状态。
