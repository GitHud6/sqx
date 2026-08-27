# RAY2-main 对齐的受控暂停与恢复计划

## 目标

实现符合 Grbl/RAY2-main 语义的实时 `!` / `~`：暂停沿原始运动路径平滑减速至零，恢复从停稳点继续尚未完成的路径。整个过程不回退坐标、不重放历史 G-code、不复制或重排 planner block ring。

`0x18` 仍是独立的不可恢复 Abort：立即停止、关闭激光、清空执行状态并使位置失效。

## 设计结论

当前应采用 RAY2-main 的执行模型：

```text
! -> HoldRequested
  -> stepper prep 基于当前真实速度生成减速 segment
  -> 停稳，保留当前部分执行 block 与 prep/Bresenham 上下文
  -> planner 从当前队首重新前瞻

~ -> ResumeRequested
  -> 重新填充 segment ring
  -> 从停稳点继续当前 block 的剩余步数
```

暂停期间，唯一允许改变的是当前 block 的速度 profile；路径几何、planner ring 顺序和已经执行的脉冲计数均不可回退。激光在减速与静止期间关闭，在恢复的有效雕刻 segment 中才重新启用。

## 职责边界

| 模块 | 职责 | 禁止事项 |
| --- | --- | --- |
| `protocol` | 捕获 `!` / `~`，更新实时状态，立即发布状态包 | 等待减速、清空队列、重放 G-code |
| `planner` | 保留当前队首 block，停稳后重算 entry/exit/junction | 根据检查点回退 ring 或修改实际位置 |
| `stepper_prep` | 保存当前 block 的进度，生成正常或强制减速 segment | 在 ISR 中做规划 |
| `timer_stepper_executor` | 消费不可变 segment、输出脉冲、上报完成 | 决定暂停语义、清空 segment ring 或选择恢复位置 |
| `laser` | 跟随 segment 的有效功率状态 | 根据预期坐标提前开光 |

## 必须保留的执行状态

`stepper_prep` 在一个 block 的生命期内维护以下状态，直到该 block 的全部步数实际输出后才释放：

- 当前 planner block 身份与几何参数；
- 每轴剩余步数、major step 进度与 Bresenham 误差；
- 当前实际速度、当前速度曲线阶段、已完成/剩余距离；
- AMASS 相关缩放和 segment 时间余量；
- 当前 block 的激光模式与功率上下文。

真实执行位置只由 executor 确认输出的脉冲累计。`planned` 与 lookahead 结果只用于诊断和供料，不得用于恢复定位。

## 状态机

```text
Running
  ! -> HoldRequested -> HoldDecelerating -> HoldComplete
  ~ -> ResumeRequested -> Running

Running / HoldRequested / HoldDecelerating / HoldComplete
  0x18 -> Abort -> Alarm
```

- `HoldRequested`：实时命令已接收；协议立即报告 `<Hold>`，但当前已启动 segment 必须完整执行。
- `HoldDecelerating`：prep 不再生成正常速度 profile，而是连续生成沿原路径减速的 segment。
- `HoldComplete`：速度为零，timer 停止；当前部分执行 block 和后续 planner block 均仍在队列中。
- `ResumeRequested`：仅在 `HoldComplete` 有效；触发重算与重新供料，不创建恢复运动或回退运动。

## 阶段一：建立可恢复的 prep 上下文

阶段状态：代码已实现，等待实机验收。

### 实现步骤

1. 将当前 `FeedHoldSnapshot` 限定为诊断信息，不参与恢复决策。
2. 在 `stepper_prep` 定义持久的执行上下文，覆盖当前 block、剩余步数、Bresenham 状态、当前速度和 profile 进度。
3. 调整 block 生命周期：只有 prep 明确报告该 block 的全部步数已被 executor 完成后，planner 才能弹出队首 block。
4. 将 executor 的 Hold 行为改为仅完成已启动 segment 并通知 prep；不得清空 ring、不得宣布 block 完成、不得使位置失效。
5. 为 `$POS/STATUS` 增加低频只读诊断：hold 状态、活动 block、剩余步数、当前速度、segment ring 深度。

### 验收标准

- 连续运动中 `!` 不触发 watchdog、位置失效、重复完成回调或队列重置。
- 停稳后当前 block 仍是 planner 队首，活动 prep 上下文仍有效且剩余步数大于等于零。
- 已完成 block 不再出现，部分执行 block 不被提前弹出。
- 不增加逐脉冲日志；实机验证通过后提交。

## 阶段二：沿路径强制减速

### 实现步骤

1. `!` 仅置位原子 `HoldRequested` 并唤醒 planner/prep；协议和 UART 不等待该过程。
2. prep 在完成当前不可中断 segment 后，基于当前真实速度与当前 block 加速度计算强制减速 profile，目标速度为零。
3. 若当前 block 剩余距离不足以停稳，则加载紧随其后的原始 block，并以继承的实际退出速度继续减速。
4. 每个减速 segment 仍使用原 block 的 Bresenham 进度和方向，不得构造新的直线目标或 system motion。
5. 激光在 HoldRequested 时关闭；减速段的激光功率固定为零。
6. prep 生成到零速后停止供料，executor 在最后一个减速 segment 完成后停止 timer，发布 `HoldComplete`。

### 验收标准

- 高速直线和雕刻轨迹内发送 `!` 后，龙门架连续减速，无硬停止晃动。
- 减速路径不偏离原 G-code，且停止距离由速度/加速度决定，不依赖文件剩余长度。
- segment ring 无 underflow，实际位置保持 known，激光在减速全过程关闭。
- 实机验证通过后提交。

## 阶段三：停稳后重算并继续

### 实现步骤

1. 在 `HoldComplete` 调用等价于 `plan_cycle_reinitialize()` 的接口：以仍在队首的部分执行 block 为起点，重置可变 entry/exit speed 与冻结边界，重新进行 junction 前瞻。
2. 重算必须使用 prep 的剩余距离和停稳时的零入口速度；不得把已执行部分重新计入 block 几何。
3. `~` 仅在 `HoldComplete` 被接受。清除 Hold 标志，恢复正常 prep profile，并唤醒 segment 填充与 timer。
4. 继续执行当前 block 的剩余 Bresenham 步数，然后正常消费后续 block；恢复首段从零速按正常加速度起步。
5. 恢复成功后清除仅用于诊断的暂停快照。重复 `~` 不得创建第二次启动。

### 验收标准

- `!` 后任意时刻 `$POS/STATUS` 能立即返回，`~` 后无需 `$H`，也不会发生回退运动。
- 恢复从停稳位置平滑起步，路径无跳段、重复段或幅面外运动。
- 连续多次 `!` / `~` 后，planner、segment、实际位置及 LightBurn 状态一致。
- 完整雕刻任务能暂停、恢复、结束；实机验证通过后提交。

## 阶段四：Abort 隔离与回归

### 实现步骤

1. 保持 `0x18` 独立：立即停止 timer/RMT，关闭激光，清空 planner、prep、segment 与供料残留，使位置 invalidated。
2. Abort 取消任何 Hold/Resume 请求，禁止后续 `~` 恢复。
3. 覆盖 UART、SD 卡及后续 socket/FFAT 供料源，确保它们观察到相同的终止结果。

### 验收标准

- 任意 Hold 阶段发送 `0x18` 均立即终止，不减速恢复、不回退、不输出残留脉冲。
- Abort 后必须 `$H` 或显式设定位置，才能开始新运动。
- 回归验证 `?`、`!`、`~`、`0x18` 与正常 SD/LightBurn 雕刻互不阻塞；实机验证通过后提交。

## 参考实现

- `D:\RAY2-main\components\Grbl_Esp32_Source\Grbl_Esp32\src\Protocol.cpp`：实时 Hold/Cycle Start 与 `cycle_stop` 状态收敛。
- `D:\RAY2-main\components\Grbl_Esp32_Source\Grbl_Esp32\src\Stepper.cpp`：`st_prep_buffer()` 的强制减速、跨 block `decelOverride` 与 Bresenham 上下文保留。
- `D:\RAY2-main\components\Grbl_Esp32_Source\Grbl_Esp32\src\Planner.cpp`：`plan_cycle_reinitialize()` 从部分执行的队首 block 重新前瞻。

## 非目标

- 不以检查点回退、回放 G-code 或复制 block ring 实现恢复。
- 不在 GPTimer ISR 中进行浮点规划、ring 遍历、动态内存分配或复杂锁操作。
- 不用增加队列容量、延迟实时命令或维持低速激光输出掩盖暂停问题。
