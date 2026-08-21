# motion

阶段三状态：命令解析闭环已接入并完成基础实机验证。`protocol_task` 将完整行交给 `command_dispatch()`，由 `execute_line()` 构造请求并调用 `motion_submit()`；只有运动执行成功才返回 `ok` 并提交解析状态。Planner 满载产生的 `Busy` 结果将在后续 Planner 阶段实现。

阶段四约束：`command` 只有在 `motion_submit()` 返回 `Accepted` 后才提交位置、模态、进给和激光状态；解析失败、`Busy` 或 `Rejected` 都不会改写正式解析状态。

阶段一状态：已实现 `MotionExecutionPlan`。它按 RayCube 的 `steps_per_mm` 将每轴毫米位移四舍五入为整数步数，并记录正负方向。当前校验 RayCube X/Y 行程、每轴最大进给和 Z 轴为零；仍不调用 `stepper`。

阶段二状态：单轴真实执行已完成历史验证；执行权现已迁移到 `planner_task`。

阶段四状态：已实现进给对应的步频计算。步频按路径长度与主轴步数推导，并限制在 10 Hz 到 10000 Hz；当前不包含加速度曲线。

阶段五状态：已实现最小 XY 直线插补。执行器以步数较大的轴为主轴，使用整数误差累加分配另一轴 STEP 脉冲；X/Y 使用相同的主步间隔，且只使能参与本次运动的轴。

阶段六状态：已接入 `planner` block 队列。`motion_submit()` 的 `Accepted` 表示请求已经安全入队，`Busy` 表示队列已满，`Rejected` 表示参数非法或 Planner 未启动；物理 STEP 执行改由 `planner_task` 独占完成。

本模块是 G-code 解析层与后续 Planner 之间的运动请求边界。

当前 `motion_submit()` 已校验请求、建立执行计划并尝试提交 Planner。它不直接操作 stepper，不执行加速度规划，也不驱动激光 PWM。

## 当前接口

`MotionRequest` 包含：

- 运动类型：Rapid 或 Linear
- 起点和绝对目标，单位为毫米
- 当前有效进给，单位为 mm/min
- 当前行生效的激光功率 `S`
- 预留的主轴模式

当前只有 `command` 层调用 `motion_submit()`。`planner` 的 FreeRTOS 队列承担生产者与执行器之间的并发边界；以后增加 APP、SD 或其他命令来源时，应继续统一经过本提交接口。

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
