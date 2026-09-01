# 梯形 Segment 时间积分重构

## 1. 目标与范围

当前 `ray2-idf` 的梯形运动能完整执行长雕刻文件，且没有 `underflow`、坐标丢失或 watchdog；但同一文件相对 `ray2-main` 仍有持续的“呼呼呼”异响。经实测，减少 Timer ISR 的锁和诊断工作后，作业稳定性没有退化，但声音没有变化。

因此本重构不再把 ISR 临界区当作首要声学问题，也不修改 planner junction、圆弧拟合、RMT 脉宽或机器物理参数。目标是重写 `stepper_prep` 对梯形 profile 的 segment 生成方式，使其与成熟 Grbl 的时间积分模型一致。

范围：

```text
PlannerBlock -> trapezoid profile -> StepSegment ring -> Timer ISR -> RMT
```

本轮只处理普通梯形 planner motion。S 曲线、暂停恢复、homing system motion 和外设驱动不随本重构扩展。

## 2. 已证实的基线

### 2.1 正常的部分

- `ray2-idf` 与 `ray2-main` 都使用 20 MHz Timer、4 MHz RMT、3 us STEP 高电平。
- 两边均使用 1000 Hz 加速度更新，即目标 segment 时间为 1 ms。
- `ray2-idf` 的长雕刻文件已完整执行，作业中 `underflow=0`，位置始终 `known`。
- 同一 block 内的 Bresenham/DDA 相位跨 segment 连续继承。
- planner 的前瞻、junction 出入口速度和圆弧解析没有出现可观测的断供或速度跳变。

结论：1 ms segment cadence 本身不是差异。`ray2-main` 也是 1 ms。

### 2.2 当前 IDF 的关键缺口

当前 `stepper_prep.cpp::build_segment()` 使用“总步数等分”模型：

```text
N = ceil(profile_total_time / 1ms)
first_step = floor(total_steps * i / N)
final_step = floor(total_steps * (i + 1) / N)

distance = length * step / total_steps
duration = t(final_distance) - t(start_distance)
interval = duration / integer_step_count
```

这保证总步数和最终位置正确，但每个 segment 的边界由整数步等分决定，而不是由 1 ms 的真实时间边界决定。每一段又以一个恒定 interval 输出所有步骤，整数化误差没有跨段补偿。

在大量短 G1 和圆弧离散段中，这会把本应连续的速度曲线变成一串不规则的恒速台阶，形成可听见的周期调制。它不会导致 underflow 或坐标错误，所以与当前实测现象一致。

## 3. 参考模型：ray2-main / 经典 Grbl

`ray2-main` 的配置为：

```text
CONFIG_ACCELERATION_TICKS_PER_SECOND = 1000
SEGMENT_BUFFER_SIZE = 60
DT_SEGMENT = 1 / (1000 * 60) min = 1 ms
```

其 prep 逻辑以固定最大时间片 `dt_max=DT_SEGMENT` 推进梯形加速、匀速、减速状态：

1. 对当前 ramp state 做速度积分，计算这段 `dt` 内的位移。
2. 若在 1 ms 内跨过 accel/cruise/decel 边界，则在同一个 segment 内继续积分剩余时间。
3. 把这段累计位移转换为理想步数变化。
4. 向下取整为实际整数 step 数。
5. 将未执行小数步对应的时间保存为 `dt_remainder`。
6. 下一段把 `dt_remainder` 加入时间，重新计算 step interval。

核心闭环为：

```text
dt_corrected = dt_segment + dt_remainder_previous
ideal_steps = travelled_distance * steps_per_mm
actual_steps = floor(previous_steps_remaining - ideal_steps)
inv_rate = dt_corrected / actual_steps
dt_remainder_next = fractional_steps * inv_rate
```

含义：本段无法发出的分数 STEP 不被丢掉，也不被任意挪到按步数等分的边界；它以时间误差的形式带到下一段。因此长期平均速度、加速度曲线和实际脉冲节奏都能收敛。

## 4. 新的 IDF 设计

### 4.1 保持的模块边界

```text
planner
  负责 block、junction、entry/exit/nominal speed

motion_profile
  负责梯形/三角 profile 的速度、距离、时间数学查询

stepper_prep
  以固定时间片积分 profile，量化为 StepSegment

timer_stepper_executor
  只消费 StepSegment，执行 DDA 并触发 RMT
```

executor 不重新计算速度曲线；planner 也不处理整数 STEP 余数。`dt_remainder` 是 prep 私有状态，只在同一 profile 内保存。

### 4.2 新的 prep context 状态

替换当前以 `total_segments` / `next_segment_index` 为核心的生成状态。普通梯形 context 至少需要：

```text
elapsed_time_us              已积分的 profile 时间
distance_mm                  已积分的 profile 距离
ideal_major_steps            已积分的连续主轴步数
emitted_major_steps          已发布的整数主轴步数
dt_remainder_us              上一段遗留的分数 STEP 时间
current_ramp_phase           Accelerate / Cruise / Decelerate
generation_complete          是否已生成 profile 末尾 segment
```

可选地保存 `next_profile_boundary_us` 或通过 `motion_profile` 查询相位边界。所有累计值必须在 block 内单调递增，最终严格落在 `total_major_steps` 和 `length_mm`。

### 4.3 固定时间片生成算法

对每个新 segment：

```text
target_time = min(elapsed_time + 1000us, profile_total_time)
target_distance = distance_at_time(target_time)

ideal_delta_steps = (target_distance - distance_mm) * total_major_steps / length_mm
corrected_time = (target_time - elapsed_time) + dt_remainder_us
integer_steps = floor(ideal_delta_steps + fractional_step_carry)
```

实现时必须使用“累计理想步数减已发布步数”的形式，避免逐段浮点累加漂移：

```text
ideal_total_steps = target_distance * total_major_steps / length_mm
integer_steps = floor(ideal_total_steps) - emitted_major_steps
```

若 `integer_steps == 0` 且 profile 尚未结束：

- 延长本段目标时间一个 `1 ms` 时间片；
- 继续积分，直到至少包含一个 STEP 或到达 profile 终点；
- 不允许发布 `major_steps=0` 的 segment。

之后：

```text
interval_ticks = round(corrected_time * timer_ticks_per_us / integer_steps)
dt_remainder_us = corrected_time - integer_steps * interval_ticks / timer_ticks_per_us
```

最终 segment 必须强制补齐：

```text
integer_steps = total_major_steps - emitted_major_steps
```

但这只能在 profile 末尾发生，不能作为常规时间量化的替代。

### 4.4 速度、激光和 AMASS

每个 segment 的状态字段应从时间边界取得，而不是从等分距离取得：

```text
start_speed = speed_at_time(elapsed_time)
end_speed   = speed_at_time(target_time)
phase       = phase_at_time(elapsed_time)
```

- 动态激光功率应基于该时间片的实际平均速度：
  `segment_distance / corrected_time`。
- AMASS 继续以量化后的 `interval_ticks` 和 source block 的轴几何选择。
- X/Y axis step 数继续由 block 总轴步数对累计主轴步数的比例映射生成，DDA phase 仍由 executor 跨 segment 继承。

## 5. 精度与不变量

每一个普通梯形 profile 必须满足：

```text
sum(segment.major_steps) == block.step_event_count
sum(segment.axis_steps[axis]) == block.plan.steps[axis]
sum(segment.duration_us) 约等于 profile.total_time_us
所有 segment.major_steps > 0
所有 interval_ticks >= kMinimumStepIntervalTicks
elapsed_time 单调增加且不越过 profile.total_time_us
distance 单调增加且不越过 length_mm
```

时间误差用 `dt_remainder_us` 有界传递；最后 segment 清零该余数。不得用降低 F、加大圆弧 chord 或跳过短 segment 掩盖误差。

## 6. 分阶段实施

### 阶段一：可验证的纯时间片生成器

实施状态：进行中。已新增未接入运行路径的纯时间片生成器和 profile 按时间查询接口；待组件编译及主机侧不变量测试确认后，才允许替换现有 `stepper_prep`。

实施内容：

- 新建纯函数/私有 helper，输入梯形 `MotionProfile`、轴步数和 `1 ms`，输出连续的时间片描述。
- 不接触 segment ring、Timer ISR、RMT、暂停或 homing。
- 实现累计距离、累计理想步数、最少一 STEP 延展和末尾补齐。
- 为直线、对角线、短 block、三角 profile 和含 accel/cruise/decel 的 block 写主机侧或组件级测试。
- 提供仅在机器空闲时可调用的 `$PROFILE/SELFTEST`。该命令只执行纯生成器，不读写 planner、segment ring、Timer 或 RMT；它输出每个 case 的预期/实际步数、时间片数、最终/预期时间和最大余数，随后返回 `ok` 或错误。

验收标准：

- 各测试满足本文件第 5 节的不变量；
- 相同 profile 每次生成结果完全一致；
- 1 ms 时间片在稳定匀速区的 interval 恒定；
- 加减速区的 interval 平滑单调变化，无由整数步等分产生的周期性往返；
- 不更改现有固件运行路径。
- 烧录后在空闲状态发送 `$PROFILE/SELFTEST`：三组 case 必须均显示 `ok=1`，汇总必须为 `result=ok` 且后续响应为 `ok`。
- 每个 case 的 `pulse` 是按 `step_interval_ticks * major_steps / timer_ticks_per_us` 累计的实际定时器脉冲时间；`error` 必须为非负且不超过 `slice_count / timer_ticks_per_us + 0.001 us`，证明剩余误差仅来自每个时间片的 tick 向上取整。

### 阶段二：替换普通梯形 `stepper_prep` 生成

实施内容：

- 以时间片生成器替换 `build_segment(context, segment_index)` 的总步数等分逻辑。
- 将 `total_segments/next_segment_index` 迁移为 elapsed-time/累计步数状态。
- 保持 `StepSegment` ABI、segment ring 和 executor 不变。
- 保留旧生成器作为一次可快速回退的内部实现，直到实机验收完成。

验收标准：

- 单轴、对角线、同圆心圆弧命令能精确到达目标位置；
- `$POS/STATUS` 显示的 active segment 合法，`major_steps` 不为零；
- 长文件无 `underflow`、`SEGMENT-GAP`、watchdog、`position-unknown` 或 `TIMER-REJECT`；
- 作业耗时与 profile 预期一致，不通过暗中降速获得稳定。

### 阶段三：对齐 `ray2-main` 的余数与速度细节

实施内容：

- 在实机基础稳定后，逐项对照 `ray2-main` 的 `dt_remainder`、最小位移延展、Timer tick 向上取整和 AMASS 切换顺序。
- 补齐动态激光功率使用的平均速度定义。
- 如有必要，增加轻量 trace：只记录 task 侧汇总的时间片数量、余数范围、末尾补齐次数；不得在 Timer ISR 增加高成本统计。

验收标准：

- 同一圆弧和 `headline` 类短段文件的声音明显接近 `ray2-main`；
- 不出现以降低速度、降低加速度或修改圆弧误差换取静音的退化；
- 连续 SD 雕刻完整完成，位置与作业生命周期正确；
- 梯形路径的声音改善可重复出现。

### 阶段四：清理旧模型并稳定基线

实施内容：

- 删除旧的总步数等分生成路径和不再需要的 `total_segments/next_segment_index` 状态。
- 更新 `stepper/README.md`，将“等分 step segment”的描述替换为“固定时间积分 + 整数步余数补偿”。
- 更新 `planner/README.md` 中涉及 prep/segment 的描述，使数学模型与代码一致。
- 只在阶段二、三实机验收并提交后进行。

验收标准：

- 无失效代码或双实现歧义；
- 文档、状态日志和实现一致；
- 梯形成为稳定雕刻基线；
- S 曲线不会共享或污染新的梯形 prep 状态。

## 7. 当前未提交的 ISR 实验

工作区当前有一组未提交的 Timer ISR 单写者/abort gate 实验。它在长文件上保持稳定，但实机声音没有改善，不能作为本重构的声学修复依据。

在阶段一开始实现时间片生成器前，应单独决定该实验：

- 若只保留架构改进，需要补足 reset、homing、feed hold 的验收后单独提交；
- 若以当前稳定基线推进 profile 重构，应先回退该实验，避免两类变量同时影响声音结果。

本文件默认推荐后者：先回到提交 `e293ec8` 的稳定执行基线，再只改变梯形 prep 的时间积分算法。

## 8. 当前结论

当前最有价值的方向不是继续削减 Timer ISR 的函数调用，而是修正梯形 profile 到整数 STEP 的离散化方式：

```text
固定 1 ms 时间积分
跨 ramp 边界填满当前时间片
累计理想步数而非等分总步数
保留并补偿分数 STEP 的时间余数
在 profile 末尾严格闭环总步数
```

这能解释“作业完整、坐标正确、无断供，但圆弧和短段仍有持续异响”的现象，并直接对齐 `ray2-main` 的成熟实现。
