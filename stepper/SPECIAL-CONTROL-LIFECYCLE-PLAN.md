# 特殊命令与运动生命周期重构计划

状态：阶段一已实现，待编译和实机验收。

本文只处理特殊控制命令与运动生命周期，不再修改已经验收的普通运动链：

```text
G-code -> planner -> profile -> stepper_prep -> segment ring -> Timer/RMT
```

当前普通运动基线为 `d2e83db`。阶段五、六已经完成 segment ring 独立化和
Timer completion 事件化。后续工作的目标，是让 `!`、`~`、`0x18`、硬限位和
`$H` 通过独立的控制生命周期完成，而不是把控制逻辑塞进 profile 或普通
segment 生成路径。

## 一、问题定义

特殊命令不是普通 G-code：

- `!` 请求受控暂停，不能立即丢弃当前运动数据。
- `~` 请求从已停止的暂停状态恢复，不能从 planned position 恢复。
- `0x18` 是终止命令，必须立即停止并使位置失效。
- 硬限位是安全故障，必须立即停止脉冲，不能等待普通 block 完成。
- `$H` 是独立的系统运动流程，完成回零后才能建立已知坐标。

这些命令共同影响 Timer、segment ring、prep context、planner block、实际
位置和协议状态。如果没有唯一的生命周期 owner，容易出现以下问题：

- 暂停后 ring 中仍有旧 segment，恢复时重复或跳过路径。
- 复位请求晚于 `$H` 执行，重新把已知位置改成 invalidated。
- 使用 planned position 代替实际已经输出的步数。
- homing 被普通雕刻的 reset/fault 路径中断。
- ISR 直接调用 planner 或 profile，导致时序和锁耦合。

## 二、RAY2-main 参考基线

RAY2-main 使用 Grbl 已验证的实时执行模型，而不是新增一个包揽所有运动资源
的控制器：实时输入和限位只发布轻量标志，`protocol_execute_realtime()` 统一
消费标志并驱动状态迁移，Stepper 只在安全边界响应控制状态。

```text
Serial/limit ISR
    -> realtime flag / alarm（必要时立即关闭输出）
    -> protocol_execute_realtime()
    -> Stepper control state
    -> st_prep_buffer() 生成或重算 segment
    -> Timer/RMT 消费 segment
```

RAY2-main 的关键做法：

- `!`、`~`、安全门和复位通过实时标志进入统一执行器，不作为普通 G-code 排队。
- 暂停通过 `st_update_plan_block_parameters()` 通知 Stepper 按当前执行速度
  重算减速，不在 Timer ISR 中重建 planner/profile。
- Stepper 只消费 segment、维护 DDA，并在 segment 耗尽时报告 cycle stop。
- 硬限位是快速停机加 alarm 标志；报警状态、队列清理和协议响应在实时执行器中完成。
- `$H` 使用 `executeSysMotion` 独立执行，自己管理 seek、pull-off、失败和原点提交。

本项目的 `execution generation` 只用于隔离旧 segment/completion 事件，不能扩展
成第二套 planner 状态机。

## 三、最终职责边界

### 1. 实时输入层

只负责在最短路径识别实时字节，并设置实时标志或投递固定大小事件：

- 不解析 G-code。
- 不修改 planner block。
- 不访问 profile、segment ring 或 RMT。
- `0x18` 可以触发快速关闭输出，但队列和状态清理由 realtime executor 完成。

### 2. Protocol realtime executor

以现有协议实时执行入口为控制生命周期 owner，负责消费实时标志并驱动
`Running / Hold / Alarm / Idle / Homing` 等状态迁移：

- 接收 hold、resume、reset、hard-limit 和 homing 的状态请求。
- 暂停时通知 Stepper 重算减速，等待 Stepper 报告停止完成。
- 复位时按固定顺序停止运动、清理队列和 prep 状态、使位置失效。
- 只通过明确的 planner/stepper API 操作运动模块，不直接访问内部 ring 游标。
- 管理 execution generation 的递增和旧事件丢弃，但不负责生成 profile 或 segment。

不得新增一个与协议实时执行器平行的总控状态机。若当前代码需要适配层，适配层
只能把现有控制请求转换为实时标志和公开 API，不能成为第二个资源 owner。

### 3. Timer/RMT Executor

只负责：

- 消费不可变 `StepSegment`。
- 运行 DDA、Timer 和 RMT STEP/DIR 输出。
- 在安全边界发布实际执行快照。
- 响应已经定义好的 stop/freeze 请求。

禁止：

- 重建 profile 或 planner block。
- 在 ISR 中执行 reset、resume 或 homing 规划。
- 直接修改 planner 或 prep 生命周期状态。

### 4. Planner/Profile/Prep

- planner 只负责普通 block 的多 block 速度规划。
- profile 只负责单个 block 的速度、时间和位移曲线。
- prep 只负责将确定的 profile 转换为 segment。
- 暂停后的残余 block 必须由 realtime executor 依据实际快照通过公开 API 重新提交，不能由
  executor 私自修补。

### 5. Homing

`$H` 使用独立的 system-motion 流程：

- 只能在普通 planner 空闲时启动。
- 不复用普通雕刻 block 的 completion 语义。
- 限位触发、pull-off、原点提交和失败处理由 homing owner 管理。
- 只有 `stepper_set_machine_position_mm()` 成功后，位置才变为 known。

## 三、冻结数据模型

暂停恢复必须保存“实际执行状态”，而不是只保存一个速度值。冻结快照至少包括：

```text
execution_generation
execution_id
active segment id
active segment elapsed ticks
active segment emitted X/Y steps
active segment DDA error
active segment end speed
source block geometry
source block remaining X/Y steps
planner queue boundary
machine position at the last emitted STEP
```

快照由 executor 在 ISR 中写入预分配的固定结构，realtime executor 在任务上下文读取。
ISR 不分配内存、不打印日志、不计算剩余几何。

快照必须满足：

```text
snapshot.machine_steps
    = reset_before_job + all_steps_actually_emitted
```

不能使用：

```text
planned position
segment end position
G-code target position
```

## 四、暂停与恢复状态机

### `!` 暂停

```text
Running
  -> HoldRequested
  -> 当前 segment 安全边界
  -> Decelerating
  -> Held
```

具体规则：

1. 实时输入层投递 `HoldRequested`。
2. 当前 segment 完成或到达定义的安全停止边界后，executor 发布快照。
3. realtime executor 通过公开 Stepper API 进入 hold，保留快照和 planner block。
4. Stepper 根据当前执行速度和剩余距离生成减速 profile。
5. 减速 segment 执行到零速后，状态变为 `Held`。
6. machine position 继续保持 known，因为这是受控停止，不是位置故障。

暂停期间不得：

- 将当前位置改成 planned position。
- 直接把 block 标记为 completed。
- 用旧 segment ring 内容继续运行。
- 在 ISR 中调用 planner 的前后扫或 profile 重建。

### `~` 恢复

```text
Held
  -> RestoreRequested
  -> 依据冻结快照重建残余 block
  -> Running
```

恢复步骤：

1. 仅允许从 `Held` 状态接受 `~`。
2. 使用冻结快照计算当前 block 的剩余轴步数。
3. 使用实际当前位置作为残余 block 的起点。
4. 对残余 block 和后续 block 重新执行 planner 的 junction 约束。
5. 重新生成 profile、prep 和 segment。
6. 新 segment 使用新的 execution generation。
7. 首个 segment 发布成功后才清除 `Held` 状态并启动 executor。

恢复失败时必须保持 `Held`，不能偷偷从头运行或丢弃剩余路径。

## 五、复位与故障

### `0x18` 复位

`0x18` 是终止，不是暂停：

1. 实时路径立即请求 executor 停止 Timer/RMT。
2. realtime executor 增加 execution generation。
3. 清空 segment ring、completion ring 和 prep context。
4. 清空 planner block queue 和 active block。
5. 清除 hold snapshot。
6. 将位置标记为 invalidated，原因是 reset。
7. 丢弃所有旧 generation 事件。

复位调用返回前，旧运动资源必须不可再启动。后续 `$H` 不能被旧 reset 事件
覆盖。

### 硬限位

硬限位必须分为两个场景：

- 普通运动中：立即停 Timer/RMT，进入 `Faulted`，位置 invalidated。
- homing 中：交给 homing 状态机处理 seek/pull-off，不走普通雕刻 fault 路径。

硬限位 ISR 只做硬件停机和固定事件投递。位置失效、队列清理和错误响应由
realtime executor 在任务上下文完成。

## 六、`$H` 生命周期

```text
Idle
  -> Homing
  -> Seek
  -> PullOff
  -> CommitOrigin
  -> Known/Idle
```

验收要求：

- `$H` 启动前限位输入状态明确且极性正确。
- seek 触发两个轴的限位后停止。
- pull-off 后两个限位释放。
- 原点写入成功后 `$POS/STATUS` 显示 `state=known`。
- `$H` 失败时才显示 `state=invalidated`，不能把正常 homing 结束误报为 reset。
- `$H` 不受旧的 `0x18`、completion 或 planner block 事件影响。

## 七、execution generation 规则

每次普通作业、恢复、复位和 homing 都必须有可识别的 generation：

- 新作业生成新的 generation。
- segment 携带 generation 或等价的 execution id。
- completion 携带同一代次。
- realtime executor 只接受当前 generation 的事件。
- reset 后旧事件即使晚到，也只能被丢弃，不能改变位置或 planner 状态。

generation 不是速度规划状态，也不是 ring cursor；它只用于生命周期隔离。

## 八、八阶段实施顺序

每一阶段完成后先编译并进行实机验收，再提交后进入下一阶段。

### 阶段一：接入 RAY2 风格实时执行边界

复用现有实时标志和协议执行入口，补齐固定大小 control event、execution generation
和状态映射。只接入事件通道与公开 Stepper 控制 API，不新增平行 Coordinator，
不改变当前普通运动行为。

当前完成项：

- 增加协议拥有的 `RealtimeControlState` 状态映射。
- 实时控制消费统一经过 `realtime_task` 和 `protocol_handle_realtime_command()`。
- `!`、`~`、`0x18` 分别发布请求态、执行态和复位态，不进入普通 G-code 队列。
- 保留复位的快速终止路径，planner/segment 清理仍通过既有公开接口完成。

验收：普通雕刻完整完成；事件可投递、可消费；旧 generation 事件会被拒绝。

### 阶段二：实现 Stepper 冻结快照

在 executor 中增加预分配快照和安全停止边界，只记录实际 tick、DDA、轴步数和
execution id，不重建 profile。

验收：直线、对角线和圆弧文件完整雕刻；快照中的实际轴步数与完成事件一致；无
`TIMER-REJECT`、underflow 或路径偏移。

### 阶段三：实现 Stepper 受控暂停

由 realtime executor 消费快照，通过 Stepper 的受控减速路径停止；不得在控制层
直接截断内部 ring 或在 ISR 中生成停止 profile。

验收：运行中发送 `!` 后稳定停止；位置保持 known；停止前后实际步数连续；不重复
或跳过 segment。

### 阶段四：实现恢复

依据冻结快照重建当前 block 残余几何和后续 planner 队列，使用新的 generation
重新生成 segment。

验收：`!` 后 `~` 能恢复到正确路径；连续暂停/恢复不产生偏移；文件最终只完成一次。

### 阶段五：实现终止复位

将 `0x18` 接入 realtime executor 的终止事务，确保停止、代次递增、ring/prep/planner
清空和位置失效具有明确顺序。

验收：任意运动阶段复位后立即停止；新 G-code 不继承旧 segment；`$POS/STATUS` 显示
invalidated；随后 `$H` 可以重新建立 known 位置。

### 阶段六：实现硬限位故障

将普通运动限位与 homing 限位分流，普通运动进入 fault，homing 继续自己的 seek/pull-off
状态机。

验收：普通运动触发限位后立即停机并位置失效；`$H` 完成后位置为 known；限位抖动不会
重复创建 reset 或 fault 事务。

### 阶段七：实现 `$H` 独立生命周期

清理 homing 与普通 planner 的启动、停止、pull-off、原点提交和响应边界。

验收：空闲、已知位置、复位后和硬限位后均可正确 `$H`；异常中止不会留下旧 ring 或
旧 completion。

### 阶段八：控制日志与回归收敛

只保留控制状态迁移、generation、冻结快照、停止原因、恢复结果、homing 结果和位置
状态日志；删除会改变 Timer 时序的高频 ISR 日志。

验收：关闭诊断日志不改变运动结果；每个特殊命令都能通过日志还原完整状态迁移；普通
雕刻、暂停恢复、复位、硬限位和 `$H` 回归通过。

## 当前执行原则

- 在阶段七完成前，不重新引入 S 曲线。
- 不通过增大 ring、修改 Timer 频率或增加日志掩盖生命周期错误。
- 不让特殊命令进入普通 profile 计算路径。
- 不把 `position-unknown` 当作暂停的正常结果。
- 阶段一到八均以已验证的普通梯形运动基线为回归标准。
# 当前状态：暂缓实现

当前雕刻基线暂不提供 `0x18`、`!`、`~` 及其暂停、恢复、复位运动控制。
普通雕刻链路优先保持单一方向：G-code -> planner -> profile -> segment ->
timer/RMT。本文档仅保留未来重新设计特殊控制时的参考，不应被当前代码实现引用。
