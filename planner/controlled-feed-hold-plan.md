# 受控 Feed Hold 与检查点恢复计划

## 目标

将 LightBurn/Grbl 实时 `!` 从“segment 边界硬停止”升级为受控减速暂停，并采用检查点回退恢复，确保暂停后不缺失雕刻路径。

- `!`：立即停止后续供料和激光输出；运动沿原 G-code 路径受限减速到零。
- `~`：激光关闭回退到最近完整 block 边界，从该检查点重放原始 block 及后续路径。
- `0x18`：独立的 Abort/Reset。它允许立即停止、关闭激光、取消作业并使位置失效，绝不参与恢复。

以 `D:\ray2-old\components\Grbl_Esp32_Source\Grbl_Esp32\src\Protocol.cpp` 和
`Planner.cpp` 的 Hold 后重规划行为作为语义参照；当前项目用明确检查点替代其共享全局状态模型。

## 选择的恢复模型

暂停不是无代价操作。若在减速期间关闭激光，停稳位置到暂停请求位置之间的路径已经发生运动、却没有雕刻；直接从停稳点继续会留下缺线。

因此采用以下模型：

```text
完整 block 边界 C ---- 已执行/正在减速 ---- 停稳点 P
                         激光关闭

恢复：P -- 激光关闭回退 --> C -- 按原轨迹重放 --> 后续 block
```

`C` 是最近一个完整 block 的起点，不是四舍五入后的毫米坐标。它由整数步数位置、对应 parser/modal 状态和 planner ring 边界组成。

该模型的成本是：

- 从 `P` 回退到 `C` 的空走时间；
- `C` 到 `P` 的少量路径会再次雕刻；
- 必须在暂停期间保留原始 block 数据，直到恢复或 Abort。

它换取三项明确保证：不跳过路径、不使用旧速度 profile 接续、不让减速时关闭的激光造成不可恢复的缺线。

## 状态机

```text
Running
  ! -> HoldDecelerating -> HoldStopped
  ~ -> RecoveringToCheckpoint -> Replaying -> Running

Running / HoldDecelerating / HoldStopped / RecoveringToCheckpoint / Replaying
  0x18 -> Abort
```

- `Running`：正常前瞻、冻结 block 和 segment 供料。
- `HoldDecelerating`：禁止新 block 进入执行窗口；激光关闭；受控减速仍严格沿已有 G-code 路径前进。
- `HoldStopped`：Timer 停止，位置可信，保存检查点和 ring 快照。
- `RecoveringToCheckpoint`：激光关闭，从停稳位置回退到检查点 `C`。
- `Replaying`：从 `C` 起以入口速度零重新运行，直到重新接回尚未执行的原始队列。
- `Abort`：清理执行、供料和协议残留，使位置失效。

## 阶段一：建立检查点与暂停快照

阶段状态：代码实现完成，等待实机验证。当前 `~` 仍沿用已验证的直接恢复，会在恢复后清除诊断快照；检查点回退与重放在阶段三实现。

### 实现步骤

1. 在每个 block 首次进入 segment 执行前建立检查点，记录：execution id、block 起点的整数轴步数、machine position、目标位置、parser/modal 状态和 planner ring 边界。
2. Hold 请求到达时锁存最近检查点、当前实际脉冲位置和当前 block 身份；禁止 planner 再填充原 profile。
3. 定义单一 `FeedHoldSnapshot`，只保存恢复所需的最近检查点与当前运行上下文；不得复制两份完整 block ring。
4. 检查点必须对应完整 block 边界。已经只部分执行的 block、旧 segment ring 和预期 planned position 都不能作为恢复起点。
5. 增加低频诊断：Hold 状态、检查点 execution id、检查点步数、停稳步数和快照有效性。不得增加逐脉冲日志。

### 验收标准

- 连续运动中发送 `!` 后，能观察到 `HoldDecelerating` 与有效检查点。
- 停稳后位置仍为 known，且停稳整数步数来自实际执行计数。
- 快照只保存一个检查点和必要元数据，不复制完整 planner block ring。
- 检查点始终早于或等于停稳前最后完整执行边界；实机验证通过后提交。

## 阶段二：沿原路径受控减速

### 接管边界

`!` 的实时捕获不等于立即中断正在执行的脉冲。暂停请求到达后，必须让
当前 active segment 完整结束；该 segment 的结束速度、已执行整数步数和运动
方向是受控减速的唯一入口状态。不得使用整个 block 的 `exit_speed` 替代该值，
因为一个 block 内可能包含加速、匀速和减速 segment。

当前 active segment 完成后，Timer executor 必须清除 segment ring 中全部尚未开始
的普通 segment，禁止它们继续被消费。随后由 planner 从这个精确 segment 边界，
根据未完成的当前 block 和后续原始 block 重新构建减速前缀；stepper prep 再将该
前缀拆分为新的不可变 segment。

不能仅生成一条沿当前方向的“自定义减速 block”。若制动距离超过当前 block 的剩余
距离，后续 block 可能发生转向；减速轨迹必须沿原始 block 序列和 junction 约束前进，
直到速度为零。

### 实现步骤

1. Hold 开始时立即关闭激光，并阻止新的普通 G-code 和新的 planner segment 供料进入执行窗口；当前 active segment 仍完整执行。
2. active segment 完成的边界由 executor 原子锁存：实际整数步数、segment end speed、方向与各轴步进比例；然后清除所有尚未开始的普通 segment。
3. planner 以锁存边界为入口，使用当前 block 的加速度、后续 block 和 junction 约束重新计算减速前缀；减速 segment 只能由任务上下文生成，GPTimer ISR 仅消费不可变 segment。
4. 减速过程必须沿原始 block 序列的几何路径执行，不能构造任意直线或改写目标坐标。当前 block 剩余距离不足以停稳时，按原 block 序列继续减速；完整经过的 block 正常记账，最终停在某个 block 内或一个 block 边界。
5. 达到零速后停止 Timer，保留检查点和原始 block 数据，发布 `HoldStopped`；不能将 Hold 停稳视为 completion failure 或 Abort。

### 验收标准

- 中高速直线与雕刻轨迹中发送 `!` 后，电机连续平滑降速，龙门架无硬停止晃动。
- 暂停延迟上界为当前 active segment 的剩余时间和制动距离；旧 segment ring 中未开始的条目不得继续执行。
- 减速期间激光保持关闭，停稳距离不随文件剩余长度增加。
- 减速轨迹不偏离 G-code 路径；无 segment underflow、watchdog、位置失效或重复 completion。
- 该阶段不恢复雕刻，仅验证受控减速和正确快照；实机通过后提交。

## 阶段三：回退、重放与重新接续

### 实现步骤

1. `~` 仅在 `HoldStopped` 状态有效；重复 `~` 不得重复提交恢复动作。
2. 清除所有基于旧连续速度 profile 的未消费 segment，但保留快照覆盖范围内的原始 block 数据。
3. 激光关闭，从停稳整数步数位置回退到检查点 `C`；回退必须受软限位、方向和加速度约束，且不得进入幅面外。
4. 到达 `C` 后，恢复检查点对应的 parser/modal 状态，以入口速度零重建并重放该 block 及减速期间已经经过的 block。
5. 当重放进度追上暂停前的原始未执行边界后，使用原始 absolute target block 重新执行 junction 前瞻，再回到 `Running`。
6. 激光只在重放的有效雕刻 segment 中恢复，不能在回退、Rapid、M5/M9 或状态不一致时出光。

### 验收标准

- `!` 后 `~` 先完成激光关闭回退，再从检查点平滑恢复。
- 暂停期间关闭激光经过的路径被重放覆盖，雕刻结果无缺线；允许检查点到停稳点的局部重复。
- 已完成检查点之前的 block 不重复，检查点之后未完成的 block 不跳过。
- 多次暂停/恢复后 planner、segment、实际位置、激光和 LightBurn 状态一致；实机通过后提交。

## 阶段四：实现独立 Abort/Reset

### 实现步骤

1. `0x18` 走独立 Abort 路径：立即停止 Timer/RMT、关闭激光、清理 planner/segment/协议残留和作业供料。
2. Abort 后释放 FeedHoldSnapshot，位置标记为 invalidated，禁止 `~` 恢复。
3. UART、SD 与后续 socket/FFAT 供料都必须接收同一个 Abort 结果。

### 验收标准

- 任意 Hold 状态下发送 `0x18` 都立即停止，不回退、不重放。
- Abort 后无残留运动、激光、旧 `ok` 或旧 G-code 继续执行。
- 必须重新 `$H` 或显式校准位置后才允许新作业；实机通过后提交。

## 不在本计划内

- 不复制整份 block ring 作为暂停副本。
- 不把 `0x18` 降级为可恢复暂停。
- 不在 GPTimer ISR 中增加浮点规划、planner ring 遍历、动态内存或复杂锁。
- 不通过增加队列长度、延迟实时任务或保持激光低速出光掩盖暂停问题。
