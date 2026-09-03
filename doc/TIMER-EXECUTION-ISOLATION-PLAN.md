# Timer 执行时序隔离与临界区收缩计划

## 1. 文档目的

本计划只处理普通梯形 profile 雕刻过程中的 Timer 执行时序、临界区阻塞和
RMT 脉冲消费链路问题。目标是降低步进脉冲的时间抖动，消除由 Timer ISR
迟到、长临界区和边界切换造成的异响与运动不平滑。

本计划不引入或恢复以下功能：

- S 曲线 profile；
- `0x18`、`!`、`~`、暂停/恢复和硬限位控制；
- 新的 junction 算法或新的曲率模型；
- 为了“看起来更平滑”而修改速度、加速度或 AMASS 数值的经验性补丁。

这些功能属于其他边界，不能与 Timer 执行链路混合修改。

## 2. 当前基线与日志结论

当前作业可以完整完成，位置状态最终保持 `known`，RMT 计数对账一致，且
`underflow=0`。因此目前没有证据表明主要问题是脉冲数量丢失或 RMT 计数错误。

最新完整作业统计中，关键数据如下：

```text
Timer interval 最小值：1250 ticks = 62.5 us
Timer jitter：samples=186592，max=1738.80 us
boundary jitter max：1655.05 us
ISR body max：75.00 us
ISR body over 10 us：22756
underflow：0
RMT trigger/reset/start/timer：计数一致
```

AMASS 统计为：

```text
changes=366
3->0=0
0->3=0
```

这说明已经观察到的 AMASS 变化不等于“相邻圆弧 block 无条件在 3 和 0
之间跳变”。同时，最严重的 planner 速度塌陷样本为真实的近折返：

```text
id=697->698
turn=177.63 deg
entry=1220.9
exit=20.3
next entry=20.3
junction=20.3
```

该样本不能作为普通平滑圆弧被 planner 错误压速的证据。

当前最有依据的异常点是：

1. Timer ISR 的最长执行时间已经超过最短步进周期，存在 ISR 本体过重的风险。
2. Timer 入场迟到达到毫秒级，显著大于 ISR 本体耗时，说明 Timer ISR 进入前
   可能被同核任务或长临界区阻塞。
3. `planner_recalculate_locked()` 在 planner 临界区内执行多轮队列遍历、
   浮点计算和 `sqrt`，可能与 Timer ISR 共享 CPU 核，导致 Level-3 Timer ISR
   长时间无法进入。
4. XY 同时脉冲比例不足以解释全部抖动，不能把问题简单归因于双轴 RMT
   触发。

因此本阶段的优先级是先隔离执行时序，再判断是否还存在真正的 profile 或
AMASS 问题。

## 3. 目标职责边界

数据流必须保持单向：

```text
G-code/parser
    -> planner
    -> profile
    -> stepper_prep
    -> segment ring
    -> timer executor
    -> RMT pulse output
```

各模块职责如下：

### G-code/parser

只解析几何和运动请求，生成 block 输入数据。不参与 Timer、RMT、ISR 或
临界区管理。

### planner

只负责 block 队列、junction 约束和 `entry/exit/nominal` 速度。前瞻重算不得
直接操作 Timer 或 RMT，也不得在 Timer ISR 中执行。

### profile

只根据单个 block 的边界速度、额定速度、加速度和位移，决定梯形、三角形或
相应的退化形式，并生成连续的速度/时间模型。不负责硬件定时器调度。

### stepper_prep

只消费已经确定的 profile，按固定时间片积分并量化为整数步数、步间隔和
AMASS 元数据。不得修改 planner 的速度边界，不得等待 Timer，不得进入
长时间硬件临界区。

### segment ring

只传递已经完成的 segment。生产者和消费者之间使用短、可预测的同步操作，
不得在 ring 锁内执行 profile、浮点重算或日志格式化。

### Timer/RMT

只消费 segment、维护当前步进状态并产生硬件脉冲。Timer ISR 不得执行 planner
前瞻、profile 求解、动态内存分配、格式化日志或阻塞等待。

## 4. 分阶段实施计划

### 阶段一：确认 Timer 与长临界区的核归属

#### 实现内容

- 记录 Timer 初始化所在 CPU 核、Timer ISR 实际运行核以及 planner、SD、
  protocol、TX 任务的实际运行核。
- 为长临界区增加进入/退出计数和最长持续时间统计，重点覆盖 planner 重算、
  segment 生产和其他可能与 Timer 同核的路径。
- 保留当前 jitter、ISR body、boundary 和 underflow 统计，统一使用 ISR 内
  低开销计数方式。
- 日志只做统计汇总，不在高频 ISR 中逐事件打印。

#### 禁止事项

- 不修改速度、加速度、junction、AMASS 判定；
- 不改变 Timer 频率和 RMT 波形；
- 不把日志字符串拼接放入 ISR 或长临界区。

#### 验收标准

- 能明确得到 Timer ISR 与 planner 临界区的核归属；
- 能得到长临界区最长持续时间及发生次数；
- 普通直线和 `longer_studio11.gc.gz` 均可完整执行；
- `underflow=0`、RMT 计数无丢失、位置保持 `known`；
- 日志不会导致任务栈溢出或改变雕刻流程。

### 阶段二：固定实时执行与上层任务的资源隔离

#### 实现内容

- 将 Timer/RMT 实时执行路径固定到明确的实时 CPU 核；
- 将 SD、protocol、planner 和 protocol TX 等非实时任务固定到另一核，或
  至少避免它们与 Timer ISR 共享关键实时资源；
- 明确任务优先级、CPU affinity 和中断分配，形成可复现的启动配置；
- 不改变上层 planner 的数学行为，仅改变执行资源归属。

#### 验收标准

- Timer ISR 不再与 planner 长临界区运行在同一实时核；
- `jitter max` 和 `boundary jitter max` 相比阶段一显著下降，且不出现新的
  毫秒级迟到；
- ISR body max 不因任务迁移而恶化；
- 完整雕刻无卡死、无 underflow、无位置失效；
- 实际声音至少不劣于阶段一基线。

### 阶段三：把 planner 前瞻重算移出长临界区

#### 实现内容

- 将 planner 队列快照、前瞻计算和结果提交拆成两个步骤：
  1. 在短锁内取得稳定输入快照；
  2. 在锁外完成遍历、浮点计算和 `sqrt`；
  3. 再以短锁提交结果，并检查 generation/version 是否仍然有效。
- 若快照失效，丢弃本轮计算并重新取快照，不阻塞 Timer。
- ring、planner 和 profile 的提交顺序必须保持一致，不能提交半套边界数据。

#### 验收标准

- planner 长临界区不再包含完整前瞻重算；
- 长临界区最长时间降到可控且与最短 Timer 周期相比有充分余量；
- `entry/exit/junction` 数值与改动前基线一致或仅有可解释的时序差异；
- 平滑圆弧不出现新的非预期速度塌陷；
- 完整雕刻保持路径、步数和结束状态一致。

### 阶段四：收缩 Timer ISR 热路径

#### 实现内容

- 将边界判断、segment 切换和下一次 alarm 更新压缩为固定上界的操作；
- 把非必要统计移出 ISR，必要统计只保留整数累加器；
- 将 RMT 触发所需的最小状态预先放入无锁或短锁可读结构；
- 禁止 ISR 中调用可能触发锁、堆分配、日志输出或复杂数学运算的函数；
- 对 segment 边界切换采用预先准备好的数据，避免在边界时刻临时生成。

#### 验收标准

- ISR body max 小于最短步进周期，并保留明确安全余量；
- `body over10us` 比基线显著下降；
- boundary jitter 不因边界切换而出现新的尖峰；
- RMT 触发、复位、启动和 Timer 对账一致；
- 完整雕刻不出现 underflow、停顿或路径偏移。

### 阶段五：验证 segment 供给与消费的稳定性

#### 实现内容

- 统计 segment ring 的低水位、高水位、生产速率、消费速率和边界等待时间；
- 确认 stepper_prep 不因下游 Timer 消费速度而停止连续供给；
- 对 segment 边界记录原始 interval、有效 interval、AMASS 和剩余步数，
  只在异常或低频汇总时输出；
- 检查生产者与消费者的同步是否存在不必要的互斥等待。

#### 验收标准

- ring 不出现 underflow；
- 生产和消费速率在最坏圆弧文件中仍保持稳定；
- segment 边界没有因供给短缺产生额外停顿；
- AMASS 变化能够由原始 interval 和整数步数解释，而不是随机跳变；
- 运动声音不再随 ring 水位周期性变化。

### 阶段六：区分真实运动约束与执行时序问题

#### 实现内容

- 在执行时序稳定后，重新检查短 G1 圆弧拆分的方向变化、junction 速度和
  向心约束；
- 仅当 Timer jitter、ISR body 和 ring 供给达到验收标准后，才判断是否需要
  修改 planner 的曲率或连续 block 策略；
- 对比同一文件的 block 长度、转角、entry/exit/junction、raw/effective
  interval 和 AMASS，建立可重复的异常样本。

#### 验收标准

- 能明确区分“planner 有意降速”和“Timer 实际迟到”；
- 对平滑圆弧和真实近折返分别有统计结果；
- 不通过单纯提高 junction 或降低加速度掩盖执行层问题；
- 若需要 planner 改动，能够给出对应数学依据和最小复现文件。

### 阶段七：清理诊断与建立稳定基线

#### 实现内容

- 删除逐事件、重复和与目标无关的日志；
- 只保留低频汇总、故障计数和必要的最终状态信息；
- 将最终 Timer、segment、planner 的职责和指标写入项目 README 或对应
  模块文档；
- 记录本阶段完成时的提交点，后续任何 profile 或特殊命令改动都从该基线
  单独分支进行。

#### 验收标准

- 普通雕刻日志足够短，不影响实时执行；
- 关键故障仍可追踪；
- 完整雕刻、复位后的首次雕刻和重复雕刻结果一致；
- 基线提交后，后续改动可以明确归因到单一模块。

## 5. 统一验收指标

每一阶段都必须使用同一组测试，避免不同文件或不同参数造成误判：

1. 短直线 G-code：确认基本步进、方向和 Timer 热路径。
2. 多段 XY 圆弧 G-code：确认 segment 边界、双轴步进和 AMASS。
3. `longer_studio11.gc.gz`：确认真实雕刻完整性和声音表现。
4. 复位后立即执行雕刻：确认位置状态为 `known`，无 `position-unknown`。
5. 作业结束后读取 `$POS/STATUS`：确认状态输出完整，不因缓冲区截断而
   静默失败。

至少记录以下指标：

```text
timer jitter max / over2us / over5us / over10us
boundary jitter max
ISR body max / over10us
最长临界区持续时间 / 次数
segment ring low-water / underflow
RMT trigger/reset/start/timer 对账
planner completion mismatch
最终 position state
```

## 6. 当前执行顺序

当前只进入阶段一：先确认核归属和长临界区证据，再决定是否实施阶段二。
没有阶段一的证据，不修改 Timer 频率、AMASS、junction 或加速度参数。

每个阶段完成并通过对应验收后，先提交该阶段改动，再开始下一阶段，避免
多个层次同时变化导致无法判断异常来源。

