# RayCube AMASS 脉冲平滑对齐计划

## 目标分析

当前 RayCube 的机床参数已经与 `ray2-main` 对齐：80 steps/mm、12000 mm/min、
500 mm/s²、0.01 mm junction deviation、3 µs STEP 高电平和 4 MHz RMT 时钟均一致。
但灰度雕刻仍出现连续的“吱吱”声，说明问题不再是供料中断或明显的 block 停顿。

`ray2-main` 的 `Stepper.cpp` 使用 AMASS（Adaptive Multi-Axis Step Smoothing）。它在较低
步频下提高 Bresenham 的时间分辨率，使非主轴的 STEP 脉冲均匀分布，避免多轴短线、斜线和
频繁比例变化时产生脉冲别名、机械啸叫或抖动。当前 IDF 的 timer ISR 每 tick 直接输出普通
Bresenham 脉冲，尚未具有该能力。

本任务的目标是把 AMASS 的效果迁入 `stepper`：保持 G-code 语义、planner、总步数、坐标统计
和 RMT 单脉冲职责不变，只优化多轴脉冲的时间分布。

## 对齐边界

```text
planner block / StepSegment
    -> stepper 计算 AMASS 级别与细分执行计数
    -> GPTimer ISR 按细分周期执行 Bresenham
    -> driver RMT 输出单个 STEP 脉冲
```

- `planner`：继续只负责速度、加速度和 junction，不感知 AMASS。
- `stepper_prep`：继续生成 segment；可增加仅供执行器使用的 AMASS 元数据。
- `timer_stepper_executor`：唯一负责 AMASS 细分后的 Bresenham 分配。
- `driver/rmt_step_pulse_driver`：仍只生成一枚固定宽度的 STEP 脉冲，不参与算法。
- `stepper_driver`：仅在实际发出该轴 STEP 时累计坐标，AMASS 不得改变总步数。

不纳入本任务：修改机床加速度、改动灰度 S 值、调整 UART 供料、改变 RMT 脉冲宽度、重写
planner 或引入圆弧插补。

## `ray2-main` 对照结论

| 项目 | ray2-main | 当前 IDF |
| --- | --- | --- |
| segment 更新周期 | 1 ms（1000 Hz） | 0.75 ms |
| segment buffer | 60 | 64 |
| planner buffer | 64 | 32 |
| 多轴低频平滑 | AMASS | 无 |
| segment 小数残余补偿 | 有 | 无 |

优先实现 AMASS；segment 时长与残余补偿应在 AMASS 实机结果后再决定是否继续处理。

## 实现步骤与验收标准

### 阶段一：建立 AMASS 元数据和纯计算选择函数

实现内容：

1. 在 `StepSegment` 增加 AMASS 级别或执行 tick 倍频字段。
2. 在 `stepper` 中实现纯计算函数：根据 segment 的主轴步频选择 level 0/1/2/3。
3. 选择函数不访问 GPIO、RMT、FreeRTOS 队列或全局运动状态。
4. 初始阈值参考 Grbl AMASS 的低步频策略，并以可读常量定义，禁止魔法数散落在 ISR。

验收标准：

- 给定相同 `StepSegment`，选择结果稳定、可重复。
- 高步频 segment 选择 level 0；较低步频按阈值提高 level。
- 不修改现有实际脉冲行为，阶段一仅增加元数据与可诊断输出。

结束改动标准：

- AMASS 选择逻辑脱离 ISR，可单独审查。
- 未引入 planner、driver 或 protocol 的反向依赖。

### 阶段二：在 timer 执行器实现 AMASS 细分 Bresenham

实现内容：

1. 将 segment 的主轴执行事件按 `2^level` 扩展为更高频的 timer tick。
2. 主轴只在正确的细分 tick 输出实际 STEP；非主轴可在中间 tick 按 Bresenham 误差输出。
3. 维持每个 segment 的 X/Y 总 STEP 数、方向和结束条件完全不变。
4. 定时器周期同步缩短为原周期的 `1 / 2^level`，不得低于 RMT 脉冲安全间隔。

验收标准：

- 直线 `G1 X20 Y10 F600`、`G1 X20 Y1 F600`、`G1 X1 Y20 F600` 的终点步数与目标严格一致。
- 单轴直线不产生额外脉冲，AMASS 可保持 level 0 或等价行为。
- `$POS/STATUS` 的实际坐标与 planned 坐标在作业完成后相等。
- 无 RMT retrigger、segment underflow、watchdog 或 position invalidated。

结束改动标准：

- AMASS 只影响 ISR 内的脉冲时序，不改变 planner block、ParserState 或 RMT 驱动边界。

当前实现状态：`timer_stepper_executor` 已改为按每轴误差累积的统一 DDA 执行。对于
`amass_level=N` 的 segment，执行 tick 数扩展为原主轴步数的 `2^N` 倍，GPTimer 周期缩短为
原周期的 `1 / 2^N`，而 X/Y 的累计 STEP 数保持 segment 原值。诊断中的
`interval=原周期->实际timer周期` 用于实机确认；尚待构建和实际步数、轨迹验证。

### 阶段三：连续短线和灰度文件实机对比

实现内容：

1. 使用同一份灰度 `.gc` 文件，与 AMASS 前的固件进行声音和成品对比。
2. 记录各 AMASS level 的进入次数、当前 level 和有效 timer interval，便于定位共振区。
3. 若仍存在局部高频声，区分它是段间周期量化、机械共振还是脉冲别名。

验收标准：

- 原有“吱吱”声显著降低，且无恢复到“嘟嘟”式供料卡顿。
- 雕刻尺寸、轮廓和灰度效果不劣于当前版本。
- segment 队列不低于低水位，不出现 RMT 失败或位置失效。

结束改动标准：

- 已以同一文件完成 AMASS 前后实机对比，并保留关键诊断结论。

### 阶段四：评估 1 ms segment 与残余时间补偿

实现内容：

1. 仅在阶段三后评估是否将 IDF segment 目标时长从 750 µs 调整到与 `ray2-main` 一致的 1 ms。
2. 若仍有可重复的周期性啸叫，设计跨 segment 的整数步/时间残余补偿。
3. 不以主观声音为唯一标准；必须同时检查轨迹、坐标与队列余量。

验收标准：

- 改动前后总步数不变。
- 不降低灰度雕刻的连续供料能力。
- 有明确证据表明该阶段改善了 AMASS 之后仍存在的问题，否则不合入。

结束改动标准：

- 保留 750 µs 或切换至 1 ms 有实机依据；残余补偿只有在必要时才实现。

## 关联文件

- `stepper/stepper_prep.hpp`
- `stepper/stepper_prep.cpp`
- `stepper/timer_stepper_executor.hpp`
- `stepper/timer_stepper_executor.cpp`
- `driver/rmt_step_pulse_driver.*`
- `planner/planner.cpp`
- `D:\ray2-main\components\Grbl_Esp32_Source\Grbl_Esp32\src\Stepper.cpp`
- `D:\ray2-main\components\longerlib_IDF\src\longerLib\project_extensions\grbl\machine\RayCube\GRBLSettings.h`
