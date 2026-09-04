# 步进执行器热路径重构

## 1. 目标与边界

本计划只解决 `StepSegment -> Timer ISR -> RMT STEP` 的执行时序。

当前梯形 profile 的时间片生成器已经验证总步数和总时间闭环，长 SD 作业也能完整结束，且没有 `underflow`、`SEGMENT-GAP`、坐标失效或 watchdog。与 `ray2-main` 相比，雕刻时仍有持续的“呼呼”异响。因此本轮不改 planner、junction、圆弧离散、梯形数学、RMT 时钟、STEP 高电平宽度或机器速度参数。

范围：

```text
StepSegment ring -> timer_stepper_executor -> RMT single pulse
```

不在本轮范围：S 曲线、feed hold/resume 的状态机扩展、homing 行为变更、激光功率算法、RMT API 替换。

## 2. 已确认的事实

### 2.1 机器和 RMT 参数

`ray2-idf` 与 `ray2-main` 的 RayCube 机器定义一致：

- X: STEP=15, DIR=7, EN=16
- Y: STEP=8, DIR=17, EN=18
- 80 steps/mm, 12000 mm/min, 500 mm/s^2
- RMT: APB 80 MHz / 20 = 4 MHz
- STEP active pulse: 3 us

参考实现的零方向延迟前导空闲为一个 RMT tick，即 0.25 us。将 IDF 实验改为 1 us 后，实机异响没有变化，故前导空闲时间不是当前声学问题的主因。

`ray2-idf` 当前两个轴设置了 `direction_inverted=true`，而 RayCube 参考默认方向掩码为 0。这是坐标和回零方向的正确性问题，不能未经实机验证直接变更；它不是持续异响的合理解释。

### 2.2 参考执行顺序

`ray2-main` 的步进 ISR 有明确的热路径原则：先发本周期已经准备好的 STEP 脉冲，再处理 segment 获取、DDA 状态推进、段结束和其他工作。方向 GPIO 仅在方向位变化时写入。

```text
Timer ISR
  -> compare current direction mask
  -> emit prepared STEP mask through RMT
  -> consume / prepare next DDA state and segment
```

这样 pulse 的开始时刻不会被当前周期的 segment 管理、DDA 初始化或其他状态处理推迟。

### 2.3 当前 IDF 的差异

当前 `timer_stepper_executor` 在同一个 ISR 临界区中：

1. 计算本周期 DDA；
2. 逐轴决定是否发 STEP；
3. 对每个命中轴执行 RMT reset/start；
4. 在段结束时处理 segment 切换、方向写入、功率和补给通知。

此外，当前段切换无条件写入两个轴的 DIR GPIO。即使电平未改变，这些 MMIO 写入和完整临界区路径也会增加脉冲开始时间的变化来源。

RMT 的每次脉冲仍应保留 `reset pointer + start`，因为这与参考 `RESET_RMT_TX` 的语义等价。问题是调用时机和 ISR 前半段负担，不是重复实现 RMT 寄存器序列。

## 3. 目标模型

执行器改为一周期前瞻模型：

```text
segment / DDA state N
  -> prepare step_mask(N + 1)
  -> next timer ISR immediately emits step_mask(N + 1)
  -> advances state and prepares step_mask(N + 2)
```

每次 timer callback 分为两个连续阶段：

1. **Pulse front**：只读取已准备的 `step_mask`，按固定轴顺序触发 RMT。
2. **State back**：DDA 计算、位置记账、segment 边界切换、方向变更、激光和 refill 通知，并准备下一次 `step_mask`。

方向状态采用已应用的 direction mask。只有新 segment 的 mask 与已应用值不同，才在下一次该方向的 STEP 之前写 DIR；若设备要求方向建立时间，仍由既有方向延迟规则覆盖，不能用无条件 GPIO 写入代替。

## 4. 不变量

- 每个已发出的 RMT STEP 恰好对应一次执行位置记账。
- 不允许在 segment 边界重复、漏发或跨方向发 STEP。
- 同一 block 的 DDA 相位继续跨 segment 继承。
- `StepSegment` ABI、ring 容量和 planner/prep 生产语义保持不变。
- RMT 只在 timer ISR 的 pulse front 触发；task、planner 和 prep 不直接触发 STEP。
- reset、取消、underflow、限位停止后，位置状态仍遵循“仅已发脉冲可信”。
- 不能为降低 ISR 负担而移除必要同步；临界区只能按共享状态实际所有权缩小。

## 5. 分阶段实施

### 阶段一：建立脉冲优先的可观测基线

实施状态：已完成并实机验收。单轴/对角线/圆弧及
`longer_studio11.gc.gz` 均保持执行正确性；长文件无 `underflow`、`SEGMENT-GAP`、
watchdog、`position-unknown` 或 `TIMER-REJECT`。方向实际写入已降为仅方向变化时发生，
但持续异响没有改善，因此本阶段只作为执行正确性基线提交，不作为声学修复。

实施内容：

- 为 executor 增加内部 `prepared_step_mask`、准备状态和已应用方向状态，不改变外部接口。
- 保持现有 DDA 数学，先仅把“确定本次应该 STEP 的轴”提前到上一个周期或 segment 装载时准备。
- timer callback 开头只执行 prepared mask 的 RMT 触发，随后执行原有状态处理。
- 加入 task 侧每秒汇总统计：pulse callback 数、每轴 RMT trigger 数、方向实际变更数、segment handoff 数；禁止在 ISR 逐条打印。
- 暂不改变 RMT symbol、timer 时钟、profile 或 segment 生成器。

验收标准：

- 单轴、对角线、圆弧测试到达目标坐标，方向正确；
- `longer_studio11.gc.gz` 完整结束，`underflow=0`，无 `SEGMENT-GAP`、watchdog、`position-unknown` 或 `TIMER-REJECT`；
- trigger 总数与每轴实际执行步数一致；
- 日志证明未发生方向变化的 segment 不产生 DIR 写入；
- 阶段验收后，单独提交再进入阶段二。

### 阶段二：收缩共享状态临界区

实施内容：

- 审计 `s_motion_lock` 覆盖的读写方：timer ISR、限位/停止 ISR、segment refill task 与状态报告。
- 将 pulse front 放在不需要覆盖 RMT MMIO 的最小同步范围外；共享 `MotionState` 的读写仍由短临界区保护。
- 将位置记账与已发脉冲绑定，普通 planner motion 保持既有段末批量提交语义，system motion 保持逐脉冲提交语义。
- 保持 underflow、hold、reset 的既有故障语义，不在本阶段重写状态机。

验收标准：

- 阶段一所有功能验收继续通过；
- 在同一测试文件中无新增 stop/restart、位置失效或 timer 拒绝；
- 每秒统计显示 timer callback 持续、RMT trigger 与步数一致；
- 圆弧和短段的电机声相对阶段一有可重复改善，否则不把该阶段作为声学修复提交。

### 阶段三：与 ray2-main 做执行时序对照

实施内容：

- 仅在阶段一、二稳定后，逐项对照 RMT trigger 顺序、direction delay、timer period 更新时机和段首次脉冲行为。
- 为启动、段交接和方向变更增加低频汇总数据，不在 ISR 输出逐脉冲日志。
- 如无示波器，使用固定单轴恒速、对角线恒速、圆弧短段三组 G-code 进行 A/B 对比；每次只变更一个执行时序因素。

验收标准：

- 同一 G-code 下的动作时间、最终位置和作业生命周期不退化；
- 阶段二无声学改善时，必须有新的计数或时序证据才允许修改 RMT/timer 配置；
- 目标是声音接近 `ray2-main`，不能通过降低 G-code F、加速度或放宽圆弧误差伪造改善。

## 6. 回退与提交策略

- 本计划与 `TRAPEZOID-SEGMENT-REFACTOR.md` 独立；后者只负责 profile 到 segment 的时间正确性。
- 阶段一不修改 `StepSegment` ABI，因此可以按文件级提交独立回退。
- 当前工作区的时间片生成器与 RMT idle 实验不是本计划的验收依据，不与本计划的阶段提交混合。
- 若 pulse-front 改造导致任何坐标、限位、hold 或 underflow 回归，先回退该阶段，不以额外状态补丁掩盖问题。

## 7. 当前结论

现有证据已经排除“机器选型错误”“RMT 时钟不一致”“3 us pulse 宽度不一致”和“1 us vs 0.25 us 前导空闲”作为首要原因。下一步应将执行器改为脉冲优先、计算后置，并用每秒汇总统计验证实际 RMT trigger、方向写入和执行步数的一致性。
