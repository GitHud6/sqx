# 梯形跨 Block 连续供给实施计划

## 已确认的问题

当前稳定基线为 `690cf25`，仅使用梯形/三角形速度曲线。完整雕刻任务可以安全完成，最终坐标正确，且没有常规 segment 下溢、RMT retrigger 或位置失效：

```text
underflow=0
retrigger=0
```

但新增的 `TRAPEZOID-FLOW` 统计表明，几乎每个运动 Block 都发生一次正常但不连续的边界停机和重启：

```text
boundary=(stop=+43..97,restart=+43..97,...)
total=1562/1562
```

这不是 block 内部的 segment 断供，因此原有 `underflow=0` 不能说明脉冲连续。实际链路当前为：一个 Block 的 profile 生成完毕并物理执行完毕，Timer 停止；随后 Planner 才启动下一个 Block 的 profile，重新启动 Timer。大量短线段或圆弧离散段会把该软件空洞转化为可闻卡顿。

`ray2-main` 的梯形基线不是这种逐 Block 停表模型：Stepper prep 在 segment ring 仍有空间时持续检出后续 Planner block，Timer 只在真实作业结束、Hold、Reset 或故障时停止。

## 目标与边界

目标运行链保持不变：

```text
motion_submit -> planner -> trapezoid profile -> StepSegment ring -> Timer -> RMT
```

需要改变的是 profile/context 与 segment ring 的生命周期：

```text
当前：单一 profile -> 生成/执行完成 -> Timer 停止 -> 下一个 Block
目标：多个梯形 context -> 共同填充 StepSegment ring -> Timer 连续消费
```

本计划只恢复和完善梯形连续供给基线，不做以下事项：

- 不重新引入 S 曲线、七段式 profile 或其历史 context/group 实现。
- 不改变 G-code、SD、激光、junction 或几何路径语义。
- 不以扩大 Planner 或 segment ring 容量替代跨 Block 生命周期修复。
- 不在 Timer ISR 中计算浮点 profile、访问 Planner ring 或执行复杂回调。
- 不在本计划中调节步频、Timer 时钟和加速度参数；这些在连续供给验收后独立对齐 `ray2-main`。

## 不变的安全契约

1. 已发布的 `StepSegment` 是不可变数据；后续 Planner 重算不能改写它。
2. Timer/RMT 仅按 segment 的 `execution_id` 消费，不推断 Planner 当前队首。
3. Planner block 仅能在其对应 context 的最终 segment 已被 ISR 确认完成后释放一次。
4. Hold、Reset、Abort、硬件错误优先于普通预填充；它们必须使未来 context 和未执行 segment 失效。
5. 真实作业排空时可以停表；普通连续 motion Block 的边界不可以停表。

## 阶段一：梯形 Context 所有权基础

### 实现内容

1. 在 `stepper` 内引入固定容量的小型梯形 context FIFO，初始容量为 4。每个 context 独占：
   - 不可变的 `MotionBlock` 副本；
   - 已求解的梯形/三角形 `StepperProfile`；
   - segment 生成游标与完成标记；
   - 稳定的 `execution_id`。
2. 明确区分：
   - generation context：当前向 segment ring 写入数据的 context；
   - execution context：Timer ISR 当前正在消费的 `execution_id`；
   - completion context：已生成完毕、等待 ISR 最终完成事件的 context。
3. 保留单 context 行为作为默认开关，先不启用跨 Block 预填充；此阶段只建立所有权、清理和诊断边界。
4. `$POS/STATUS` 与 `TRAPEZOID-FLOW` 增加 context 使用量、当前生成 ID、当前执行 ID 和待完成数量，避免将 Planner 队首误报为实际执行 block。

### 验收标准

- 单轴、XY、短段、完整 SD 文件的轨迹和最终步数与当前基线一致。
- `!`、`~`、`0x18`、作业结束后，FIFO 使用量回到零，无过期 execution id。
- `underflow=0`、`retrigger=0`，无 watchdog、position invalidation 或重复完成回调。
- 此阶段允许正常 Block 边界 stop/restart 继续存在，因为尚未开启预填充。

### 提交门槛

完成实机验收后单独提交。未提交前不得进入阶段二。

## 阶段二：跨 Block 梯形预填充

### 实现内容

1. 当前 generation context 已将其全部 segment 写入 ring 时，不等待它物理执行完毕；只要 ring 有可用槽位，立即冻结并启动下一个已完成 look-ahead 的运动 Block context。
2. 移除 Planner 中“当前 profile active 就返回”的单 profile 限制，仅对正常梯形运动启用 context FIFO 预填充。
3. 每个 segment 保留其来源 `execution_id`、方向、激光状态与 block-completion 标记；相邻 context 的 segment 在同一 ring 中按提交顺序排列。
4. Timer 在完成一个标记为 `completes_block` 的 segment 后，若 ring 已有后继 segment，直接装载后继而不停表；仅在 ring 为空时记录 boundary stop。
5. 保留 Planner 低水位通知，但把它作为补充唤醒机制，不以“上一个 Block 已完全执行”为下一 Block 的前提。

### 验收标准

- 使用 `longer_studio11.gc.gz`，稳态雕刻期间 `TRAPEZOID-FLOW` 必须出现：

```text
boundary=(stop=+0,restart=+0,...) underflow=+0
```

- 总 `block_boundary_stop_count` 仅允许在作业真实排空、Hold、Reset 或明确故障时增加；不得接近 motion block 数。
- `underflow=0`、`retrigger=0`，最终步数、坐标和 completed block 数正确。
- 实机圆弧/字母路径不再出现与 G-code 短段边界等频的固定卡顿声。

### 提交门槛

完成整文件实机验收后单独提交。未提交前不得进入阶段三。

## 阶段三：完成事件与 Planner 释放

### 实现内容

1. 将 ISR 完成事件与 context FIFO 头部按 `execution_id` 严格匹配。
2. 仅在对应 context 最终 segment 被实际完成后：弹出 context、提交实际步数、释放 Planner block、递增 completed 计数。
3. 修正状态查询和激光/附件回调，使其基于实际执行 context，不读取随后已预取的 Planner block。
4. 对 completion event 乱序、重复、缺失以及 FIFO 满等情况进入明确的安全故障路径，禁止静默释放错误 Block。

### 验收标准

- 高密度短线段、圆弧离散段、直线和混合 XY 作业均保持输入顺序与最终坐标。
- SD 作业流控正常，无 `position-unknown`、重复 `ok`、错误 block completed 计数或中途卡住。
- 结束时 Planner、context FIFO、segment ring 和 Timer 都处于一致的空闲状态。
- 连续运行不触发 Planner、protocol_tx 或 sd_job watchdog。

### 提交门槛

完成 SD 与串口流两类实机验收后单独提交。未提交前不得进入阶段四。

## 阶段四：Hold、Resume、Reset 与故障清理

### 实现内容

1. Feed Hold 只保留实际 execution context；丢弃尚未执行的 future contexts 与其 segment，并从真实执行步数建立减速/恢复状态。
2. Reset、Abort、SD fault 或 Timer/RMT error 时，在统一临界区内停止 Timer、清空 segment ring、使 context FIFO 失效，并使 Planner 冻结/队列状态一致。
3. 真实作业尾端显式进入 draining，再允许 Timer 停止；不得把正常尾端当作 block 内 underflow。
4. 保留并扩展 boundary/underflow 统计，以区分正常作业排空、Hold、Reset 与异常断供。

### 验收标准

- 重复 `!`、`~`、`0x18`、SD 中止和作业结束均不双重完成、不跳步、不恢复过期 segment。
- Hold/Resume 后最终位置与未中断的参考路径一致，Reset 后位置状态遵循既有安全语义。
- `TRAPEZOID-FLOW` 中普通运行的 boundary stop 为零；结束、Hold、Reset 的停表原因可明确区分。
- 无 watchdog、position invalidation 或悬挂的 context/segment。

### 提交门槛

完成安全路径实机验收后单独提交。

## 后续独立工作：参数对齐

只有阶段一至四通过后，才单独评估与 `ray2-main` 的参数差异：

- Planner block ring：当前 32，对照值 64；
- 最大步频：当前 10 kHz，对照机器在 `12000 mm/min * 80 step/mm` 时需要 16 kHz；
- step timer 时钟：当前 1 MHz，对照值 20 MHz；
- segment 目标时长：当前约 750 us，对照值 1 ms。

每次只调整一个参数组，并重复完整雕刻、`TRAPEZOID-FLOW`、最终坐标和声音验证。参数调优不能掩盖阶段二的 Block 边界停表问题。

## 参考依据

- 当前诊断：`planner/planner.cpp` 的 `TRAPEZOID-FLOW` 与 `stepper` 的 block-boundary 计数。
- 当前单 profile 限制：`planner/planner.cpp` 中 `stepper_prep_profile().active` 的早退逻辑。
- 对照实现：`D:\RAY2-main\components\Grbl_Esp32_Source\Grbl_Esp32\src\Stepper.cpp` 与 `Planner.cpp`。
- 设备参数对照：`D:\RAY2-main\components\Grbl_Esp32_Source\Grbl_Esp32\machine\RayCube\GRBLSettings.h`。
