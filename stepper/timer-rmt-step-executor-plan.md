# 定时器驱动 RMT 单脉冲执行器计划

## 目标分析

当前 SD 灰度文件已验证可以持续经过以下链路：

```text
SD gzip reader -> execute_line() -> planner block ring
-> stepper segment ring -> RMT stream -> STEP/DIR
```

实机结果表明，SD 供料、G-code 解析、planner 入队以及 RMT 事务完成回调均能持续工作：

- 在遇到 `M9` 前，任务已处理到第 1912 行。
- RMT 已提交 15437 个 segment，完成 15378 个 segment。
- `timeouts=0`，不存在 RMT transaction 永久悬挂。
- `M9` 当前未实现，触发 SD 作业安全中止和 Planner reset；这属于独立的 G-code 对齐事项。

但灰度雕刻仍有明显的周期性“嘟嘟”声，轨迹与图案扭曲。根因是当前执行模型将约 750 us 的 stepper segment 当作一笔独立 RMT transaction：

```text
segment 完成中断
-> 唤醒 FreeRTOS task
-> 回收 in-flight segment
-> rmt_sync_reset()
-> rmt_transmit(X)
-> rmt_transmit(Y)
-> 下一 segment
```

其中出现过 `1 step / 902 us` 的 segment。任务调度和 RMT 驱动 API 的固定开销已经与 segment 时长同量级，段与段之间必然产生软件空洞。电机在空洞中减速或停顿，因而出现异响和几何失真。

`ray2-main` 的 RMT 模型不是把 segment 当作 RMT 传输。它保留约 60 项的 segment ring，由高频 stepper timer ISR 连续消费；RMT 只负责每一次 STEP 脉冲的高低电平宽度。段切换仅更新步数、周期和激光参数，不能等待一次任务调度或 RMT transaction 完成。

本计划以此为目标替换当前 RMT stream 执行模型：

```text
planner block ring
-> stepper segment ring
-> 高优先级 step timer ISR
-> RMT 单脉冲触发 X/Y STEP
```

## 设计边界

- `command`、`motion`、`sd` 不得感知定时器或 RMT 细节。
- `planner` 继续拥有 block 队列、前瞻、Hold、Reset 和 block 完成语义。
- `stepper_prep` 继续产生 segment，但 segment 是定时器消费单位，不是 RMT transaction。
- RMT 仅生成满足 STEP 高电平宽度要求的单脉冲；不可用它承载 750 us segment 的完整波形。
- STEP 定时由 ISR 连续推进；ISR 不得等待队列、互斥锁、日志、LEDC 或文件系统。
- 激光功率更新必须与将要执行的 segment 对齐，但具体 PWM 写入不得破坏 ISR 的实时性。
- 现有 `$POS/STATUS`、RMT timeout 诊断和位置失效语义保留，供迁移对比使用。
- 本计划不处理 `M9` 的语义兼容；该命令应继续在增量 G-code 对齐文档中单独实现和验收。

## 阶段一：冻结当前事务化执行路径并建立基线

阶段状态：代码已完成，待构建与上电验收。

实现内容：

1. 保留当前 RMT stream 的诊断字段，明确记录 submitted、started、completed、failed、timeouts 与 active segment。
2. 为定时器执行器定义独立接口和统计结构，不修改 `command`、`motion`、SD 文件读取及 planner block 的公开接口。
3. 确认 `stepper_prep` 产生的 segment 包含连续执行所需的步数、方向、持续时间、激光功率和 block 完成标志。
4. 明确 RMT stream 只作为迁移期间的基线与回退实现，不再对其进行性能调优。

验收标准：

- 当前短线运动、位置状态和 RMT diagnostics 的输出格式可用于迁移前后比较。
- 新接口加入后不改变现有 RMT stream 的执行行为。
- 代码中不存在同时由两个执行器驱动同一组 STEP 引脚的路径。

结束改动标准：

- 已记录一组灰度任务基线，包括异响、轨迹现象和 `$POS/STATUS` 统计。
- 已确认 timer executor 尚未接管实际 STEP 输出。
- 实机验收通过后提交本阶段改动，再开始阶段二。

当前实现记录：已保留现有 RMT stream diagnostics，并新增
`timer_stepper_executor` 模块。阶段一的 `timer_stepper_executor_init()` 仅建立
执行器状态，不创建硬件 timer、不申请 STEP GPIO、不产生 STEP 脉冲；启动路径调用
它以验证模块初始化。`TimerStepperDiagnostics` 预留 initialized、STEP 引脚所有权、
timer 运行、ISR 次数和 segment 下溢统计，阶段二和阶段三才允许写入后四项。

## 阶段二：实现单脉冲 RMT 与 step timer 基础时序

阶段状态：代码、构建与实机短线验收已通过。

实现内容：

1. 为 X/Y 保留 RMT 单脉冲配置：每次触发只输出一个固定宽度的 STEP 高电平及回落。
2. 创建高优先级 step timer ISR，以当前 segment 的步间周期重装下一次中断。
3. 在 ISR 中执行 Bresenham 计数，决定本次 X、Y 是否各输出一个 STEP 脉冲。
4. DIR 必须在 segment 或 block 方向切换时提前稳定；ENABLE 在首脉冲前打开，空闲后才关闭。
5. 暂时只接入固定周期的单轴和双轴短线，不接入完整 segment ring。

验收标准：

- `G91` 下单轴和 XY 短线的终点步数正确，未参与轴不出脉冲。
- 逻辑分析仪或等效测量确认 STEP 高电平不低于机器要求，连续脉冲间隔由 timer 控制。
- 在连续数百个脉冲中，不存在由 FreeRTOS task 调度造成的毫秒级间隙。
- RMT 初始化、timer 启停、Reset 和错误路径均能使 STEP 保持低电平并关闭激光。

结束改动标准：

- 已实机验证固定周期单轴、45 度和非等比 XY 直线。
- 没有出现看门狗、重复脉冲、漏脉冲或方向错误。
- 实机验收通过后提交本阶段改动，再开始阶段三。

当前实现记录：阶段二已切换为 timer executor 独占 STEP 输出。启动时通过 ESP32-S3
RMT LL 寄存器和 GPIO matrix 将 RMT0/RMT1 分别连接到 X/Y STEP 引脚，并在 APB
FIFO 预装固定 3 us 高电平的单脉冲。GPTimer 以 1 MHz 运行；ISR 只执行整数
Bresenham、触发对应 RMT 读指针重置和启动、累计实际步数，并在最后一步通知等待的
Planner task。Planner 的 motion block 已只调用短线 timer executor，旧 RMT stream
分支被禁用。阶段二限制主轴最多 256 步，超过限制会安全拒绝；Hold 尚未接入，留待
阶段三的 segment ring 生命周期。

验收记录：已使用 `$POS/SET=0,0`、`G91`、单轴正反 `G1 X1` / `G1 Y1` 与双轴正反
`G1 X1 Y1` 短线命令完成实机验证。初版错误复用了 `planner_task` 的 task notification
等待 timer 完成，导致后续 block 的入队通知被误当作前一 block 的完成事件；现已改为
专用二值信号量。验证后连续命令正常执行，未再出现 `busy=1`、`execution-failure` 或
`position-unknown`。

## 阶段三：由 timer ISR 连续消费 stepper segment ring

实现内容：

1. 让 `stepper_prep` 持续填充既有 segment ring，直到达到高水位或 Planner 没有兼容 block。
2. timer ISR 在当前 segment 的最后一个 step event 后无缝加载下一个 segment 的速度、Bresenham 上下文和激光描述。
3. segment ring 为空时停止 timer，不产生虚假 STEP；低水位只由 task 侧补料，ISR 不等待补料。
4. block 仅在其最后一个 segment 已被 ISR 实际消费后才向 planner 回报完成。
5. 保留实际执行步数累计，禁止上层以 planner 预期位置覆盖已执行位置。

验收标准：

- 连续 `G1 X0.05 S...` 灰度式短线输入时，STEP 时间轴连续，不在 segment 边界出现可闻停顿。
- segment ring 在正常运动中保持非零水位；出现下溢时有低频诊断，不静默扭曲轨迹。
- planner 的完成数量只在对应 block 的最后一段实际完成后递增。
- `$POS/STATUS` 中 machine position 与累计 STEP 数一致。

结束改动标准：

- 已使用多段 XY 与短像素连续指令完成实机验证。
- 确认 segment 边界不再依赖 `rmt_transmit()` 完成回调推进。
- 实机验收通过后提交本阶段改动，再开始阶段四。

## 阶段四：激光 segment 同步与控制状态

实现内容：

1. 将 M3/M4/M5 与每个 segment 的功率描述明确绑定。
2. 在 segment 切换点更新激光状态；若 PWM API 不适合 ISR，采用经验证的硬件或无阻塞交接方式。
3. Hold、Reset、segment 下溢、RMT 错误和作业结束时，优先关闭激光并停止后续 STEP。
4. 保持 M4 动态功率与实际 segment 速度的既有计算语义。

验收标准：

- 连续灰度 `S` 值与对应的 STEP segment 一一对应，不提前点亮下一像素。
- 空闲、Hold、Reset 和异常路径不会残留激光输出。
- 激光同步处理不会重新引入 segment 边界的运动间隙。

结束改动标准：

- 已用短灰度扫描线实机观察功率变化与运动连续性。
- 已验证 M3、M4、M5 和异常停机的安全输出。
- 实机验收通过后提交本阶段改动，再开始阶段五。

## 阶段五：完整 SD 灰度文件验收与旧执行器退役

实现内容：

1. 使用 `longer_studio11.gc.gz` 或等价真实灰度文件进行完整运行测试。
2. 通过增量 G-code 对齐处理文件遇到的必要命令；当前已知下一项为 `M9`。
3. 对比迁移前后的 RMT、segment、planner 和位置诊断，确认不再存在软件段间隙。
4. 在新执行器通过完整验证后，移除或隔离旧的“每 segment 一个 RMT transaction”执行路径，防止两个模型长期并存。

验收标准：

- 灰度文件运行期间无周期性嘟嘟声、RMT timeout、任务看门狗或异常重启。
- 雕刻轨迹不再呈现由段间停顿造成的扭曲。
- 正常结束时 planner、segment、timer 和 RMT 均为空闲，激光关闭。
- 位置未无故进入 `invalidated`；真实 Reset 或错误时仍准确标记失效原因。

结束改动标准：

- 已完成完整灰度文件实机验收并保存关键诊断结果。
- 旧事务化 RMT 路径不再承担正常灰度雕刻。
- 每个已验收阶段均已单独提交后，本计划才视为完成。

## 不纳入本计划

- `M9`、其他未知 G-code 或厂商私有命令的语义实现。
- SD、gzip、UART、网络和 FFAT 的统一供料接口。
- 限位、回零、坐标系持久化和报警状态机。
- 机械传动、驱动电流、皮带张力、共振等硬件问题。

这些工作可能影响最终雕刻质量，但不能掩盖当前已经实证的执行器时序缺陷。
