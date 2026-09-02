# 步进执行模块

`stepper` 使用单一的 `TrapezoidExecutionPlan` 将冻结的梯形 profile 转换为
`StepSegment`，并由定时器 ISR 按 DDA 发出 STEP。

## 当前执行链路

```text
PlannerBlock -> stepper_prep -> StepSegment ring -> Timer Group ISR -> RMT STEP
```

- segment ring 容量为 64，目标段时长 1 ms，低水位为 16。
- `segment_producer` 是唯一的生成入口：它提交冻结 block、请求 `stepper_prep` 生成单个完整
  segment、检查 ring 容量并发布；低水位事件到达时继续生成，生成阶段不启动 Timer。
- profile 的唯一生成 cursor 是 `TrapezoidSegmentGenerator` 的已发步数、已过时间和
  `dt_remainder_us`；不存在独立的 segment 序号或 ring cursor 来重新解释 profile 进度。
- Timer 时基为 20 MHz；AMASS 在低速时增加 DDA 时间分辨率。
- 同一 block 的 segment 继承 Bresenham/DDA 相位，避免分段时重新开始轴步进分布。
- `GrblTrapezoidPrep` 仅用于 `$PROFILE/SELFTEST` 的离线参考，不参与正常任务、segment 或
  硬件执行状态。
- 普通 G-code 在段结束时按已发出的步数批量提交位置；归零等系统运动逐脉冲提交，
  保证限位停止时位置可信。

## 位置与故障

位置是开环估计值，只能由已触发的 STEP 更新。reset、取消、underflow 或执行异常后必须
标为无效，并通过 `$H` 或明确设定位置重新建立基准。

## 当前状态

梯形 SD 作业已实机完成且无 underflow。S 曲线不属于当前稳定运行路径。
