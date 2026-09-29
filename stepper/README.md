# 步进执行模块

`stepper` 将 planner 已冻结的 block 转换为定长 `StepSegment`，并由定时器 ISR 消费 segment、执行 DDA/AMASS，再请求 `driver` 产生实际脉冲。

## 执行链路

```text
PlannerBlock -> execution plan / profile -> stepper_prep
             -> StepSegment ring -> Timer ISR -> RMT pulse driver
```

## 职责

- 为冻结 block 构建梯形执行计划，并按时间片生成 segment。
- 维护 segment 与完成事件的有界环形队列，以及生产者与 ISR 之间的通知。
- 保持同一 block 跨 segment 的 DDA 相位连续，避免轴步进分布在边界重新开始。
- 在 ISR 中消费已准备数据、触发脉冲、记录完成信息并请求补料。
- 在任务上下文处理补料、完成回收、暂停边界与执行故障。

## 并发与实时边界

- `segment_producer` 是 prep 上下文的唯一任务所有者；ISR 只读取已发布 segment。
- 环形队列是任务与 ISR 的传输边界，发布前 segment 必须完整初始化。
- ISR 只能进行常数时间的 DDA、队列和通知操作，不能分配内存、加锁、写文件、打印日志或调用 service。
- 实际位置只能依据已发出的步进更新；复位、取消、underflow 或执行故障后必须使位置状态失效。

## 修改步骤

1. 先确认改动属于 profile、segment 生成、队列传输还是 ISR 消费，避免跨层复制状态。
2. 改动 segment 字段时同步检查生产者、ISR、完成事件和 planner 回调。
3. 所有新的 ISR 数据必须放在可安全访问的内存中，并验证进入临界路径后的执行时间。
4. 调整暂停、复位或故障时，联测 `service` 的生命周期与位置可信状态。

## 验收标准

- 长路径与连续 block 不出现 segment underflow、重复步或遗漏步。
- 跨 segment 的多轴运动保持 DDA 相位连续。
- 暂停、恢复、复位和故障后队列与执行上下文可预测地保留或清除。
- ISR 路径不包含阻塞、动态分配、文件访问或高频日志。
