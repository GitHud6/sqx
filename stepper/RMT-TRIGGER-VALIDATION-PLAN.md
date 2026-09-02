# RMT 脉冲重触发验证方案

## 目标

验证雕刻时的尖啸、呼呼声或卡顿是否由每个步进脉冲都执行一次 RMT
`reset_pointer + start` 引起，而不是继续凭声音猜测 planner 或加速度参数。

当前基线只关注普通雕刻。本文不恢复 `0x18`、`!`、`~`，也不引入暂停、恢复或安全控制逻辑。

## 当前待验证路径

```text
planner
  -> trapezoid profile
  -> stepper_prep 生成 StepSegment
  -> segment ring
  -> Timer ISR
  -> DDA 判定步进
  -> RMT reset_pointer + start
  -> STEP 脉冲
```

当前 RMT 每次触发都执行：

```cpp
rmt_ll_tx_reset_pointer(&RMT, axis_index);
rmt_ll_tx_start(&RMT, axis_index);
```

该路径可能使每个脉冲的启动相位、硬件访问耗时和 Timer ISR 占用时间产生变化，
从而在圆弧的大量短 G1 场景中形成可听见的周期性调制。

## 第一阶段：计数对账

当前实现状态：已加入 RMT 的 `trigger/reset/start/invalid` 计数，并通过
`$POS/STATUS` 输出；Timer 执行器原有的 `rmt_triggers` 作为 DDA/执行器侧对照计数。
下一步只需烧录后执行测试文件并收集状态输出，不需要逐脉冲串口日志。

### 需要统计的计数

RMT 驱动中增加按轴统计，计数器只在 ISR 中递增，不在 ISR 中打印：

```text
trigger_count[axis]       executor 请求 RMT 脉冲次数
reset_count[axis]         rmt_ll_tx_reset_pointer 调用次数
start_count[axis]         rmt_ll_tx_start 调用次数
invalid_trigger_count     未初始化或无效轴触发次数
```

Timer/执行器中增加：

```text
timer_event_count         Timer ISR 回调次数
prepared_pulse_count      DDA 判定出的轴脉冲次数
segment_handoff_count     segment 边界切换次数
```

每秒由普通任务读取并打印一次，禁止在 Timer ISR 或 RMT 触发函数中直接打印。

### 计数关系

当前单步重触发实现下，正常情况下应满足：

```text
trigger_count[axis] == reset_count[axis]
trigger_count[axis] == start_count[axis]
invalid_trigger_count == 0
```

同时，DDA 产生的轴脉冲数应与对应轴的 RMT trigger 数一致：

```text
prepared_pulse_count[axis] == trigger_count[axis]
```

如果 DDA 计数正确而 RMT 启动计数异常，问题位于 RMT 驱动；如果两者都正确但声音异常，
应继续检查 Timer 周期和 segment 边界，而不是先修改 planner。

推荐日志格式：

```text
RMT-TRACE timer=12000 prepared=(6000,6000) \
rmt_trigger=(6000,6000) reset=(6000,6000) start=(6000,6000) \
invalid=(0,0) handoff=950
```

## 第二阶段：RMT A/B 实验

只建立两个固件变体，planner、profile、Timer 频率、AMASS 和 G-code 必须完全相同。

### A：当前实现

```cpp
rmt_ll_tx_reset_pointer(&RMT, axis_index);
rmt_ll_tx_start(&RMT, axis_index);
```

### B：取消每次触发前的 reset

```cpp
rmt_ll_tx_start(&RMT, axis_index);
```

实验前确认 RMT 通道在一次脉冲完成后能够自动回到可启动状态。若出现重复脉冲、通道不动、
启动失败或状态字异常，应立即记录并恢复 A 版本；这说明 reset 可能是必要操作，但不代表
当前 reset 的时机和调用方式正确。

### 结果判定

| 现象 | 结论 |
|---|---|
| B 版本尖啸明显减弱或消失 | RMT 重置/重启动路径是主要嫌疑 |
| A、B 声音基本一致且计数正常 | 优先检查 Timer 周期量化和 segment 边界 |
| B 版本出现重复脉冲或无法运动 | RMT 需要重新定位指针，继续优化 reset/start 的时序 |
| trigger 正常但 start 或 invalid 计数异常 | RMT 硬件访问或通道状态存在问题 |
| DDA 计数与 trigger 计数不一致 | Timer ISR、DDA 或计数位置存在问题 |

## 第三阶段：segment 边界关联

在普通任务中打印 segment 边界，不在 ISR 中打印：

```text
SEGMENT-TRACE id=17 prev_interval=210 next_interval=224 handoff=1
```

重点观察尖啸发生时是否同时出现：

```text
segment_handoff 增加
step_interval 突然变化
RMT reset/start 高频执行
```

如果声音只在 `prev_interval` 和 `next_interval` 差值较大时出现，问题更接近 profile
量化或 Timer 周期更新；如果周期连续但声音随 RMT reset/start 频率变化，问题更接近 RMT
重触发开销。

## 日志设计约束

- ISR 只做计数和必要的硬件操作，不做格式化和串口输出。
- 每秒输出一组统计，避免日志反过来制造卡顿。
- 不记录每个脉冲的详细日志。
- 统计字段使用 32 位计数器即可，读取时使用无符号差值计算窗口增量。
- 每次实验只改变一个变量。
- 直线、单圆弧、`headline` 雕刻文件分别测试，便于区分基础脉冲问题和短 G1 场景问题。

## 验收标准

完成本验证后，必须能够回答以下问题：

1. DDA 判定的步数是否全部变成了 RMT trigger。
2. 每个 trigger 是否都伴随 reset 和 start。
3. Timer ISR 事件和 segment handoff 是否在尖啸位置异常增加。
4. 取消 reset 后声音和运动是否发生可重复变化。
5. 如果 RMT 不是根因，是否有数据证明应转向 Timer 周期或 profile 量化检查。

只有完成上述对账和 A/B 结果后，才进入代码修正阶段。
