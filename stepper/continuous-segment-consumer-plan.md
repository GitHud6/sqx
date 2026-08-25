# 连续 Segment 生产与消费链路计划

## 目标分析

RMT 与 GPTimer 的硬件所有权已经收敛到 `driver/`：GPTimer 负责每个 step event 的时间，
RMT 只负责一个满足脉冲宽度要求的 STEP 脉冲。旧的“一个 segment 对应一笔 RMT
transaction”的后端已经删除，不能重新引入任务调度或 RMT API 造成的段间空洞。

当前仍是阻塞 block 执行模型：`planner_task` 调用
`timer_stepper_executor_execute()` 后等待整个 block 完成，`stepper_prep` 的 segment ring
没有成为实际执行链路的一部分。它无法让多个灰度像素 block 在电机运动期间持续预填充，
也无法将 block 完成时刻绑定到 ISR 的实际最后一步。

目标链路：

```text
输入供料层
  -> execute_line() / motion
  -> planner block ring（前瞻与冻结边界）
  -> stepper_prep（唯一 segment 生产者）
  -> 固定容量 SPSC segment ring
  -> GPTimer ISR（唯一消费者）
  -> RMT 单脉冲 -> STEP
```

其中输入来源可以是 UART、SD、socket 或 FFAT，但它们只向 planner 提交 block；不直接感知
segment 水位、RMT 或 timer 背压。

## 目标职责与并发规则

| 模块 | 唯一职责 | 不允许做的事 |
| --- | --- | --- |
| planner | block 入队、前瞻、冻结已 segment 化的 block、回收已实际完成 block | 不直接发 STEP，不等待某个 step event |
| stepper_prep | 将已冻结的 block 增量转换为 `StepSegment` | 不访问 GPTimer、RMT、激光硬件或 FreeRTOS 队列 |
| segment ring | 在 task 生产者与 ISR 消费者之间保存不可变 segment | 不覆盖未消费数据，不以 mutex 或 ISR 队列实现 |
| stepper executor | segment 装载、Bresenham、DIR/ENABLE、实际步数和完成记录 | 不调用 planner、日志、文件、LEDC、阻塞或等待任务 |
| driver | GPTimer 周期与 RMT 单脉冲硬件能力 | 不理解 block、segment、位置或激光业务 |
| laser | 根据已开始执行的 segment 管理功率和安全关断 | 不决定 STEP 时序 |

### SPSC ring 规则

- 生产者只有 planner task；消费者只有 GPTimer ISR。
- ring 使用静态数组，容量初始保持 `64` 项，目标时间预算约 `48 ms`（`750 us` segment）。
- task 只写 head 指向的空槽，完成整个 `StepSegment` 写入后以 release 语义发布 head。
- ISR 只读 tail 指向的已发布槽，完成装载后以 release 语义推进 tail。
- ISR 绝不进入临界区、不等待 FreeRTOS queue、不记录逐 segment 日志。
- 初始水位：高水位 `48`、低水位 `16`。低水位只通知 task 补料，不停止当前脉冲序列。
- segment ring 为空时属于真实下溢：停止 timer、禁止继续 STEP、使位置失效并触发安全激光关断。

### Block 冻结与完成规则

1. planner 可以只对尚未 segment 化的 block 重新做前瞻计算。
2. 第一个 segment 被发布后，该 block 的 entry、exit、方向和功率描述不可再被 look-ahead 修改。
3. 一个 block 的 `completes_block` segment 被 ISR 实际消费完毕后，ISR 向固定容量
   completion ring 写入 `execution_id`，再通过 task notification 唤醒 planner。
4. planner 收到 completion 后才递增 completed 计数、发出 `BlockCompleted`、从 block ring 回收
   该 block，并通知容量等待者。
5. planner 预期位置可以领先实际位置；只有 Reset、下溢、硬件错误或执行取消才令实际位置失效。

### Direction、Enable 与激光安全

- `StepSegment` 必须补充 X/Y 方向信息；方向变化不能依赖 task 在任意时刻抢占插入。
- 当前 segment 最后一步与下一 segment 第一脉冲之间，若方向改变，executor 通过 ISR-safe
  DIR 写入原语建立方向，并使用硬件规定的 direction-setup 时间作为下一次 step event 的首周期。
- 连续作业期间轴保持 enable；只有 block ring、segment ring、当前 segment 和 completion ring
  都为空，且经过空闲延时，才关闭轴。
- ISR 不调用 LEDC。发生 Hold、Reset、下溢或执行错误时，必须调用 ISR-safe 的激光硬件关断
  原语；常规 PWM duty 更新在 segment 开始边界通过可证明非阻塞的交接完成。

## 实施阶段

### 阶段一：建立可观测的 SPSC segment 基础（代码已实现，待验收）

实现步骤：

1. 将 `stepper_prep` 的 segment ring 改为显式 SPSC publish/consume API，移除 ISR 侧 mutex
   依赖。
2. 增加稳定的 `SegmentExecutionDiagnostics`：queued、high-water、underflow、active、
   active execution id、completed pending；不包含 RMT transaction 计数。
3. 扩展 `StepSegment`，固化方向、执行 id、功率、完成标志和 segment 的实际周期数据。
4. 添加 completion ring 与 task notification，但暂不让 ISR 连续启动下一个 segment。

验收标准：

- task 可以发布、读取并回收 64 项 segment，顺序、内容和边界不发生覆盖或重复。
- ISR 模拟消费者不使用 mutex、FreeRTOS queue、日志或动态内存。
- `$POS/STATUS` 能显示 segment 水位与 underflow 次数，且不恢复 `rmt=(...)` 字段。

结束改动标准：

- 仅建立数据结构与诊断，不改变已验证的短线实际运动路径。
- 完成编译和对应静态/实机诊断验收后提交，再进入阶段二。

当前实现状态：

- `stepper_prep` 已将 segment ring 改为 task producer / ISR consumer 的序号型 SPSC ring，
  publish 与 consume 使用 acquire/release 语义，不再在 ISR 路径使用 `portMUX`。
- `StepSegment` 已固化方向和平均 step 周期；已增加固定 completion ring 与 ISR task notification
  接口，但当前短线 executor 尚未调用它们。
- `$POS/STATUS` 已增加稳定的 `segment=(...)` 诊断。当前 active、id、completed_pending 均应为
  零值，因为阶段二以前不接入实际消费链路。

### 阶段二：将 Timer executor 改为非阻塞单 segment 执行器（代码已实现，待验收）

实现步骤：

1. 用 `start_segment()` / `stop()` 替代等待二值信号量的 block 级 `execute()`。
2. task 在 segment ring 非空且 executor idle 时装载首段并启动 timer；调用立即返回。
3. ISR 在一个 segment 最后步完成时累计实际位置、记录 completion 条件，但暂不自动装载下一段。
4. 保留当前 `2 mm` 以内短线验证入口作为兼容适配层，确保执行步数、方向和安全停止不变。

验收标准：

- 单段单轴、XY、正反向短线仍正确，planner task 不会因等待执行而阻塞。
- 一个 segment 完成后位置累计与该 segment 的 X/Y 步数一致。
- Reset 在任何时刻都停止 timer、使 STEP 空闲、关闭激光并让位置失效。

结束改动标准：

- 已实机验证非阻塞单 segment 生命周期。
- 未启用连续自动切段；验收后提交，再进入阶段三。

当前实现状态：

- Timer executor 已提供非阻塞 `start_segment()`；最后一步由 ISR 写入 completion ring 并通知
  planner，不再使用等待整个 block 的二值信号量。
- planner 已将当前 Motion block 临时适配为一个完整 segment，收到 completion 后才发出
  `BlockCompleted`、递增完成数并回收 block。
- ISR 尚不自动读取下一 segment；相邻 block 仍会经过 task 调度边界，这个连续性问题留待阶段三。

### 阶段三：ISR 无缝切换与低水位补料

实现步骤：

1. ISR 完成当前 segment 后直接从 SPSC ring 读取下一项并装载 Bresenham、周期和方向状态。
2. 当 ring 水位到达低水位时，用 task notification 请求 planner 补料；通知合并不影响 ISR 时序。
3. planner task 在收到新 block、低水位或完成通知时，尽可能填充到高水位，再回到等待。
4. 将 `stepper_prep` 改为可跨多次 fill 续写当前 profile，并在空间允许时继续生成后续已冻结 block。
5. 下溢不产生虚假 STEP：ISR 停 timer、记录一次低频故障、发布失败 completion，并执行安全关断。

验收标准：

- 连续 `G1 X0.05 S...` 输入时，逻辑分析或等效时序测量显示 segment 边界无任务调度级空洞。
- 正常运行时 ring 水位不跌至零；故意停止供料时可观测到一次 underflow 与明确安全停止。
- planner 完成计数只在最后 segment 被 ISR 消费后增加。
- 连续短线期间电机不会在相邻 pixel block 间重复 enable/disable。

结束改动标准：

- 已完成多段单轴、XY 与方向变化实机验证。
- 无 watchdog、重复脉冲、漏脉冲或未解释的位置失效；验收后提交，再进入阶段四。

### 阶段四：激光 segment 同步与控制状态

实现步骤：

1. 将 M3/M4 的功率在 segment 发布时固化，在该 segment 实际开始时应用。
2. 为常规 PWM 更新建立非阻塞交接；ISR 只触发已准备好的动作或安全 GPIO gate，不调用 LEDC。
3. Hold 在当前安全边界停止后续 segment；Resume 重新建立预填充后再启动 timer。
4. Reset、下溢、执行错误、作业结束先安全关断激光，再清空/取消相应的 segment 与 block。

验收标准：

- 灰度 `S` 值不提前泄漏到下一像素，也不会在运动停止后残留出光。
- M3、M4、M5、Hold、Resume、Reset 与下溢的激光状态均可实机验证。
- PWM 处理不引入 segment 边界卡顿。

结束改动标准：

- 已用短灰度扫描线验证运动连续性和功率边界。
- 通过后提交，再进入阶段五。

### 阶段五：真实灰度文件与供料压力验收

实现步骤：

1. 使用 SD 中真实 `.gc` 或 `.gc.gz` 灰度文件运行完整任务；G-code 不支持项仍按增量对齐流程处理。
2. 分别测量 planner block 水位、segment ring 水位、underflow、实际位置和任务完成状态。
3. 以 UART 灰度流重复压力验证，证明输入来源变化不会影响 step event 时序。
4. 仅在真实任务稳定后调节 ring 容量或低/高水位；不以扩大缓冲掩盖生产速度不足。

验收标准：

- 完整灰度任务无周期性“嘟嘟”声、task watchdog、重启、segment 下溢或位置无故失效。
- 灰度图案不再有由段间停顿产生的扭曲，任务结束后 planner/segment/executor 均为空闲且激光关闭。
- SD 与 UART 供料均通过同一 planner 接口，且运动侧不暴露输入来源特例。

结束改动标准：

- 保存一组完整任务的关键状态与故障证据。
- 每个阶段已实机验收并按项目规则提交后，本计划完成。

## 不纳入本计划

- 新 G-code 语句、M9 或厂商私有命令的语义实现。
- gzip 压缩算法、SD 文件系统、UART 协议 `ok` 策略或网络输入实现。
- 限位、回零、坐标系持久化、机械传动、电流、皮带张力和共振调校。

这些问题会影响最终雕刻质量，但不能改变本计划的核心约束：STEP 时间轴由 ISR 连续消费固定
segment，任何 task 调度、文件读取、G-code 解析或 RMT transaction 都不得进入该热路径。
