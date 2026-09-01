# Timer ISR 热路径与脉冲连续性改造记录

## 1. 目的

当前 `ray2-idf` 使用同一条链路完成运动输出：

```text
G-code -> planner -> stepper_prep -> StepSegment ring -> Timer ISR -> RMT STEP
```

同一个雕刻文件在 `ray2-main` 上运行时电机声音清脆，而在当前 IDF 版本上会出现“呼呼呼”或轻微摩擦感。当前 planner、圆弧解析和 DDA 已经具备基本正确性，因此本记录只关注执行层：Timer ISR、segment handoff、临界区和 RMT 触发之间的时序成本。

本记录不是立即实施方案，也不改变当前梯形 profile 基线。所有改造都必须先保持直线、圆弧、限位、暂停和位置报告行为不回退。

## 2. 已确认的事实

### 2.1 当前测试结果

同圆心四段圆弧测试已经能够：

- 正确完成四个 90 度圆弧；
- `radius=10.000`、`sweep=90.0`、`segments=8`；
- 返回起点，最终 machine position 正确；
- `underflow=0`；
- `junction break=0`；
- DDA continuation 数量接近 segment handoff 数量。

因此目前没有证据表明主要问题是圆弧几何错误、planner 供给耗尽、junction 速度断裂或 segment 边界重置。

### 2.2 当前执行频率

当前普通运动的 segment 目标约为 1 ms。测试中观察到：

```text
segment handoff 约 1000 次/秒
timer event 约 11000 次/秒
平均 timer interval 约 87~92 us
```

这意味着 CPU 不会逐个 STEP 生成波形，但大约每 1 ms 要介入一次 segment 切换，并且每个 timer event 都要经过 Timer callback 和 DDA 执行路径。

### 2.3 与 `ray2-main` 已对齐的硬件参数

两边的基础参数基本一致：

- Timer Group 0 / Timer 0；
- Timer 频率 20 MHz；
- AMASS 基准约 8000 step/s，最大 level 3；
- RMT 时钟 4 MHz；
- STEP 高电平约 3 us。

所以当前差距不能首先归因于 RMT 时钟、STEP 脉宽或 AMASS 阈值不同。

## 3. 当前 ISR 的实际职责

当前 `stepper/timer_stepper_executor.cpp` 的 `timer_alarm_callback()` 每次被调用时会：

```text
进入 s_motion_lock 临界区
检查 s_busy
遍历 X/Y DDA 累加器
必要时 reset RMT pointer 并 start RMT
递增 completed_ticks
```

只有在当前 segment 完成时，才会继续执行更重的边界路径：

```text
复制完成的 MotionState
提交已发出的普通运动步数
通知 stepper_prep
发布 block 完成事件
消费下一个 segment
更新方向 GPIO
构造下一个 MotionState
计算并记录速度连续性统计
继承或初始化 block DDA phase
写入 active segment 诊断状态
更新激光功率
更新 Timer alarm interval
检查低水位并可能唤醒 refill task
```

当前问题不在于这些操作每一个都必然错误，而在于它们被放在同一个实时执行上下文中，导致 segment 边界的执行时长和分支数量随 ring 状态、任务通知、激光状态和控制请求变化。

## 4. 临界区问题

当前 Timer ISR 使用：

```cpp
portENTER_CRITICAL_ISR(&s_motion_lock);
...
portEXIT_CRITICAL_ISR(&s_motion_lock);
```

这把锁保护 `s_motion`，但锁覆盖范围还包括 RMT 触发、位置提交、segment 消费、统计、激光处理和任务通知。由此产生三个风险：

1. 临界区持续时间不是固定值。普通 timer event 和 segment 边界 event 的路径不同。
2. 其他中断或另一执行上下文需要访问同一资源时，等待时间不可预测。
3. Timer callback 本身还经过 IDF 的 callback wrapper，临界区与 wrapper 的总成本可能放大周期抖动。

需要强调：临界区不是因为“使用了就一定错误”。限位、停止、启动和位置状态存在并发关系，直接删除锁可能造成数据竞争、错误位置或无法停止。正确方向是先重新划分所有权，再缩小锁的保护范围。

## 5. 与 `ray2-main` 的关键架构差异

`ray2-main` 使用传统 Grbl 的单体 stepper 状态机：

```text
Timer ISR
  -> 清中断
  -> busy 防重入
  -> stepper_pulse_func()
  -> 重新使能 alarm
```

其正常运动状态长期驻留在同一个 `st` 结构中。DDA、segment 进度、ramp 状态和 Timer 周期更新围绕该状态机运行，实时路径短且经过长期验证。

当前 IDF 实现采用模块化结构：

```text
Timer driver -> executor -> stepper driver
                         -> segment ring
                         -> prep notification
                         -> laser driver
                         -> RMT driver
```

模块化本身没有问题，但当前 executor 在 ISR 中承担了过多跨模块协作职责。应当保留模块边界，同时把实时状态收敛到 executor 内部，把协作事件延迟到任务上下文处理。

## 6. 目标架构

目标是建立“ISR 单写者”的正常运动模型：

```text
任务上下文                         Timer ISR
------------                       ---------
准备 StepSegment                   读取当前 MotionState
发布到 ring                        执行 DDA
写入启动快照                       触发 RMT
发送原子控制请求                   在边界切换预取 segment
处理完成通知                       写入最小完成记录
```

### 6.1 所有权规则

- 运动开始前，任务上下文初始化 `MotionState`。
- 运动运行期间，Timer ISR 独占当前 `MotionState`、DDA 累加器和 segment 进度。
- 任务上下文不直接修改运行中的 `MotionState`。
- feed hold、abort、limit 等控制只写入原子请求位或 mailbox，由 ISR 在安全点消费。
- ISR 不执行非必要的业务处理；完成、低水位和诊断通过最小事件通知任务上下文。
- 停止之后，任务上下文才回收和清理运行态。

### 6.2 热路径分层

每一个 timer event 的路径应尽量固定：

```text
清中断
读取本地运行态
更新 X/Y DDA
触发必要的 RMT
递增 tick
```

只有 segment 完成时才进入边界路径；只有控制请求或 block 结束时才进入控制路径。统计、日志和任务通知不能出现在每个 STEP 的路径上。

## 7. 分阶段修正方向

### 阶段一：清点所有权并隔离诊断

实施状态：代码已开始落地，尚待实机验收。已将 handoff ISR 中仅用于 trace 的速度差、最大跳变和 64 位原子/CAS 统计移除；保留供给、相位连续、Timer 计划量和故障统计，并改为 32 位窗口计数。运动锁、DDA、RMT、位置提交和控制语义在本阶段不变。

实施内容：

- 列出 `s_motion`、segment ring、执行位置、busy、feed hold 和停止状态的所有读写者。
- 将正常 timer event 必需字段与边界字段分开。
- 确认诊断统计不改变 RMT 触发路径；统计只使用单次递增或任务侧快照。
- 删除或移出热路径中的非必要日志、复杂统计和重复原子操作。
- 保留现有锁，先不改变并发语义。

验收标准：

- 梯形直线运动行为不变；
- 圆弧能完整执行并回到起点；
- 无新增 `underflow`、`position-unknown`、watchdog 或 `TIMER-REJECT`；
- 诊断统计值与改造前数量级一致；
- 不再有每个 timer event 的复杂诊断工作。

### 阶段二：缩小正常运动临界区

实施内容：

- 将 `s_motion_lock` 的保护对象限制为启动、停止和极少数跨上下文状态。
- 正常 planner 运动运行期间由 Timer ISR 独占 `s_motion`。
- 将 feed hold、abort、limit 改成原子请求，ISR 在 segment 边界处理。
- 禁止任务上下文在运行期间复制或改写整个 `MotionState`。
- 普通运动位置继续按既定的 segment 完成策略提交；系统运动和限位仍保留逐脉冲位置更新。

验收标准：

- 运行期间正常 timer event 不再进入完整 `s_motion_lock` 临界区；
- `$POS/STATUS`、feed hold、resume、reset 和 `$H` 行为正确；
- 限位触发后不会继续发出有效运动脉冲；
- 普通运动位置误差为零，异常时仍正确 invalidation；
- 长文件不出现卡死或状态无法清理。

### 阶段三：边界路径轻量化

实施内容：

- ISR 预取下一个 segment 时只做必要的方向、DDA 和 Timer 参数切换。
- 将完成通知、低水位 refill、诊断快照和累计统计合并为轻量事件。
- 低水位通知采用边沿触发，避免每个 handoff 重复唤醒任务。
- 预先把下一段所需的 `MotionState` 转换放到任务上下文，ISR 只交换已准备好的快照。
- 激光功率更新保留在必须的 segment 边界，但避免额外锁和复杂调用。

验收标准：

- segment handoff 不再执行不必要的 profile/结构转换；
- ring 长文件保持稳定供给；
- underflow、gap、restart 计数不增加；
- 圆弧和大量短 G1 的声音较当前基线改善；
- 梯形 profile 的最大速度和完成时间不发生意外变化。

### 阶段四：与成熟 Grbl 热路径做定向收敛

只有前三阶段稳定后才进行：

- 对照 `ray2-main` 的 Timer 清中断、alarm 重装和 DDA 更新顺序；
- 验证 IDF Timer Group API 是否可以保持相同的“当前 alarm 到下一 alarm”语义；
- 比较 segment 时间积分、余数修正和 AMASS 切换；
- 必要时把 executor 的正常运动路径收敛为单一状态机；
- 不迁移与当前 IDF 外设 API 不兼容的旧接口。

验收标准：

- 同一测试文件下两版 STEP 周期统计接近；
- 无以降低速度换取静音的隐藏退化；
- 直线、圆弧、对角线、短段和 block 边界均通过；
- 驱动层错误、位置状态和安全控制不回退。

## 8. 不应首先修改的部分

在没有证明执行层问题以前，不应优先修改：

- junction deviation 公式；
- 圆弧拟合误差；
- planner 前瞻方向；
- 梯形加速度数学模型；
- RMT 脉宽；
- 机器轴的物理速度参数。

这些参数会改变运动结果，容易掩盖 Timer ISR 的真实问题，而且当前日志已经证明 planner 能够完成测试轨迹。

## 9. 诊断与证据边界

没有逻辑分析仪时，软件日志只能证明：

- planner 是否持续供给；
- segment ring 是否 underflow；
- DDA 是否连续；
- Timer 是否拒绝请求或触发 watchdog；
- 最终位置是否正确。

软件日志不能直接证明每个 STEP 脉冲之间的微秒级间隔是否均匀。因此“ISR/临界区导致声学调制”目前是高可信架构假设，不是已经完成波形级证实的结论。

在现有条件下，最可靠的替代方法是：

1. 对同一条圆弧固定速度重复测试；
2. 对比 segment handoff、timer event、underflow、gap 和 restart；
3. 分别测试直线、单圆、连续短圆弧和真实雕刻文件；
4. 记录声音变化与 handoff 频率、AMASS 等级、速度区间的关系；
5. 每次只改变一个执行层变量。

## 10. 当前结论

当前首要问题不是“planner 公式需要继续复杂化”，而是模块化后的 Timer ISR 同时承担了脉冲时序和跨模块协作。修正重点是：

```text
减少每次 timer event 的固定成本
缩短并发保护范围
让 ISR 成为运行态的唯一写者
把通知、统计和 refill 协作移出热路径
保持 DDA 与 segment phase 连续
```

这条路线既能解释当前 IDF 与 `ray2-main` 的声学差异，也能在不重写 planner 的情况下逐步验证。任何阶段完成后，必须先提交并确认基线稳定，再进入下一阶段。
