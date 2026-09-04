# 步进执行模块

`stepper` 负责将 planner 生成的冻结运动 block 转换为 `StepSegment`，并由定时器
ISR 按 DDA/AMASS 规则驱动 STEP。该目录只保留当前固件运行所需的运动代码；历史诊断、
实验方案和阶段计划不属于运行时模块。

## 当前执行链路

```text
PlannerBlock -> stepper_prep -> StepSegment ring -> Timer Group ISR -> RMT STEP
```

- segment ring 容量为 64，目标段时长 1 ms，低水位为 16。
- `segment_producer` 是唯一的生成入口：它提交冻结 block、请求 `stepper_prep` 生成单个完整
  segment、检查 ring 容量并发布；低水位事件到达时继续生成，生成阶段不启动 Timer。
- `segment_ring` 拥有 segment/completion 队列、ring 水位、transport event 与执行快照；
  prep 不保存或修改这些传输状态。
- profile 的唯一生成 cursor 是 `TrapezoidSegmentGenerator` 的已发步数、已过时间和
  `dt_remainder_us`；不存在独立的 segment 序号或 ring cursor 来重新解释 profile 进度。
- Timer 时基为 20 MHz；AMASS 在低速时增加 DDA 时间分辨率。
- 同一 block 的 segment 继承 Bresenham/DDA 相位，避免分段时重新开始轴步进分布。
- 普通 G-code 在段结束时按已发出的步数批量提交位置；归零等系统运动逐脉冲提交，
  保证限位停止时位置可信。

## 文件职责

- `stepper_driver.*`：电机使能、方向控制、位置记录和位置可信状态。
- `motion_profile.*`：梯形 profile 的速度、距离和时间计算。
- `trapezoid_execution_plan.*`：将冻结 block 组织为可生成的梯形执行计划。
- `trapezoid_segment_generator.*`：按时间片生成 `StepSegment` 的步数和周期数据。
- `stepper_prep.*`：管理 profile 上下文并完成 segment 生成前的准备。
- `segment_producer.*`：协调 planner、prep 和 segment ring 的生产流程。
- `segment_ring.*`：提供任务与 ISR 之间的 segment 队列、完成通知和必要执行状态。
- `timer_stepper_executor.*`：消费 segment、执行 DDA/AMASS 并触发 Timer/RMT STEP。

上述文件共同组成正常运动链路，删除或合并前需要同步检查 planner、homing 和 Timer/RMT
调用方。

## 位置与故障

位置是开环估计值，只能由已触发的 STEP 更新。reset、取消、underflow 或执行异常后必须
标为无效，并通过 `$H` 或明确设定位置重新建立基准。

## 当前状态

梯形 G-code 和 SD 作业已实机完成且无 underflow。当前基线不包含 S 曲线，也不包含暂停、
恢复等尚未实现的扩展行为。工程诊断快照、原子统计和 `$PROFILE/SELFTEST` 已清理，
不会作为运行时运动路径的一部分维护。
