# 运动规划模块

`planner` 负责把 `motion` 提交的运动请求组织为有序 `PlannerBlock`，进行连续 block 的
junction 前瞻和速度约束计算，并将已确定的 block 交给 `stepper_prep` 生成 segment。

它是生产者与实际运动执行器之间的缓冲和规划边界，不直接操作 STEP、DIR、RMT、GPTimer
或激光 GPIO。

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `planner.hpp` | 定义 PlannerBlock、提交结果、诊断快照、执行事件和公共接口。 |
| `planner.cpp` | 管理 block ring、前瞻重算、planner task、segment 供料、完成回收和控制状态。 |

## 运行链路

```text
command / motion
        |
        v
planner_submit()
        |
        v
Planner block ring
        |
        +--> junction + reverse pass + forward pass
        |
        v
stepper_prep
        |
        v
StepSegment ring
        |
        v
GPTimer / RMT
```

`planner_submit()` 成功只表示 block 已进入规划队列，不代表电机已经开始或完成运动。
实际位置由 STEP 执行路径累计，planner 只维护 block 的计划位置和执行生命周期。

## PlannerBlock

每个运动 block 绑定以下内容：

- 原始 `MotionRequest` 和离散化后的 `MotionExecutionPlan`；
- 机床坐标起点、目标点和 XY 路径长度；
- programmed feed、轴向单位向量和轴约束加速度；
- `entry_speed_sqr`、`exit_speed_sqr`、最大入口速度和最大 junction 速度；
- `execution_id` 与 reset generation，用于生命周期和异常一致性检查。

`SpindleDisable` 是无位移控制 block，用于保证 M5 等关闭事件与运动队列保持顺序；它不生成
STEP segment。

## Block ring 与所有权

当前 ring 容量为 32。`head` 写入新 block，`tail` 指向待执行 block，`count` 表示队列中
仍未回收的 block 数。访问 ring 的共享状态受临界区保护。

```text
tail                                      head
  |                                         |
  v                                         v
[ 已发布/冻结前缀 ][ 未发布可变前瞻窗口 ][ 空闲槽位 ]
```

### 冻结规则

- block 尚未转换并发布为 `StepSegment` 时，仍可参与前瞻重算。
- block 的 segment 已发布到 segment ring 后，该 block 及其 profile 进入冻结前缀。
- 冻结前缀是从 `tail` 开始的连续前缀，由 `s_segment_prepared_blocks` 表示。
- 新 block 到来时，只重算冻结前缀之后的可变窗口，以及它与冻结前缀之间的速度衔接。
- 已执行或已发布的 segment 不得被后续 G-code 或前瞻计算修改。

冻结不是按方向或 junction 类型分组，也不是等待 ring 填满；它是下游不可撤销消费后的
所有权边界。

## Junction 与前瞻

相邻可规划运动通过单位方向向量计算几何关系：

- 共线同向：junction 几何限制很高，速度可连续传递；
- 小角度转弯：允许较高但有限的交界速度；
- 90 度转弯：按 junction deviation 和加速度限制降速；
- 反向或非连续 block：交界速度回到安全低速或零速。

随后对未冻结窗口执行：

1. 反向 pass：从末端停止约束向前传播减速可达性。
2. 正向 pass：从起点或冻结前缀的出口速度向后传播加速可达性。
3. 将后一 block 的入口速度与前一 block 的出口速度连接起来。

planner 的前瞻只决定 block profile；`stepper_prep` 再把 profile 转换为时间片 segment，
GPTimer ISR 只消费 segment，不执行浮点规划或读取 planner 队列。

## Planner task

`planner_start()` 创建常驻 `planner` 任务。任务主要负责：

- 消费 segment 完成通知并回收队首 block；
- 填充 segment ring；
- 启动 segment 执行；
- 处理 SpindleDisable block；
- 在空闲时关闭参与运动的轴；
- 处理 Hold、Resume、Reset 和执行失败。

正常路径通过任务通知唤醒，不在 planner task 中等待每个物理 STEP。segment ring 负责隔离
规划生产和硬件消费速度。

## 提交结果

| 结果 | 含义 |
| --- | --- |
| `Accepted` | block 已进入 ring。 |
| `Full` | ring 已满，未覆盖已有 block。 |
| `Rejected` | block 类型、步间隔或 planner 状态不合法。 |

上游必须根据结果决定是否提交 ParserState 和回复 `ok`；planner 不替上游伪造成功。
队列满时可通过 `planner_wait_for_free_block()` 等待容量，但不能在临界区内等待。

## 控制与异常

- `planner_request_feed_hold()` 将规划状态切换为 Hold，并撤销激光运动输出许可；
  已有执行链路按安全边界停止。
- `planner_resume()` 恢复运行许可，但不会绕过 segment 和 block 的生命周期顺序。
- `planner_reset()` 增加 reset generation，停止执行器，清空 block 与 segment 状态，并使
  开环执行位置失效；后续必须重新建立可信坐标。
- block 取消、执行失败和队列排空均通过执行事件通知适配层。

## 执行事件

planner 可以注册一个 `ExecutionCallback`。事件包括 block 开始、segment batch 生命周期、
Hold、Resume、完成、取消、失败和队列排空。

回调运行在调用 planner API 的任务上下文中，必须快速返回，不得再次调用 planner API，
也不得执行文件系统、阻塞等待或长时间计算。激光模块通过该接口处理安全许可和关闭，
但不拥有 planner 队列。

## 诊断

`planner_diagnostics()` 提供只读快照：

- 当前队列数量和冻结 block 数；
- 前三个 block 的状态：Executing、Frozen 或 Lookahead；
- 每个 block 的 execution id、入口速度、出口速度、junction 上限和 programmed feed。

`$POS/STATUS` 使用这些数据观察前瞻是否持续生效。正常连续运动应看到冻结前缀之后仍有
可变的 Lookahead block，而不是所有 block 都被提前固定为零入口/出口速度。

## 模块边界

上游依赖：`motion` 提供已经校验和离散化的请求。

下游依赖：`stepper_prep`、`timer_stepper_executor` 和 `stepper_driver` 执行已规划结果。

联动模块：`laser` 监听执行事件；`command`、`protocol` 和 `sd` 只通过上游提交路径
间接使用 planner。

本模块不得直接解析 G-code、访问文件系统、操作 GPIO/RMT，也不得让 ISR 反向调用 planner。

## 当前范围

- 已支持 RayCube Cartesian XY 的 G0/G1 block 规划。
- 已支持连续 block 的 junction、反向/正向前瞻和冻结前缀边界。
- 已支持 segment ring 供料、Hold、Resume、Reset、执行失败与队列排空生命周期。
- 圆弧 G2/G3、Z 轴及更完整 Grbl 系统命令由其他后续模块负责。

## 验收重点

- 连续同向短线的相邻 block 具有非零连续速度，不在每个 block 边界人为停到零。
- 90 度转角按 junction 约束降速，反向运动安全减速。
- segment 已开始消费后，新 block 仍能进入未冻结前瞻窗口。
- 运动完成后实际步数与最终计划坐标一致，且无 segment underflow、RMT retrigger、
  watchdog 或位置失效。
- 队列满、Hold、Resume、Reset、执行失败和 M5 的返回与安全状态明确可观测。
