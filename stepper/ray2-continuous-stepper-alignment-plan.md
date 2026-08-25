# RAY2 连续步进执行模型对齐计划

## 目标

当前工程已经能够通过 SD 卡完整执行一个 G-code 任务，但灰度雕刻仍出现明显的周期性卡顿和“嘟嘟”声。本计划用于将当前按 Planner block 阻塞执行的模型，逐步对齐 `ray2-main` 的连续 Stepper segment 执行模型。

目标执行链路：

```text
SD/UART 输入
    -> execute_line()
    -> Planner block ring
    -> Stepper segment ring
    -> RMT/定时器持续消费
    -> STEP/DIR 连续输出
```

本计划不重新设计 G-code 业务语义，重点解决运动执行时序、缓冲深度、电机使能生命周期和激光 segment 同步问题。

## 当前问题基线

当前实现存在以下结构性差异：

1. `planner_task()` 执行一个 block 时调用 `rmt_tx_wait_all_done()`，完成后才处理下一个 block。
2. 每个 block 完成后调用 `disable_axes()`，导致灰度像素之间反复切换电机使能。
3. `stepper_prep` 的 segment 只在当前 block 内生成和消费，尚未形成跨 block 的持续 segment ring。
4. 灰度文件相邻 block 的 `S` 值通常不同，因此按相同功率合并 block 不能解决主要卡顿。
5. 激光功率当前按 block 事件更新，而 `ray2-main` 是在 segment 即将执行时更新。
6. 实际位置与 Planner 预期位置出现差异时，当前会将位置状态标记为 `invalidated`，需要保留可诊断性。

## 实现阶段与验收标准

### 阶段一：电机使能生命周期对齐

实现内容：

1. 移除连续运动路径中每个 block 完成后的立即 `disable_axes()`。
2. 增加 Planner/Stepper 的运动活跃状态判断。
3. 只有 Planner 队列、Stepper segment 队列和 RMT transaction 全部为空时，才允许关闭电机。
4. 保留 Feed Hold、Reset、RMT 错误和异常退出时的立即关闭路径。
5. 保留单次手动运动完成后的延迟空闲关闭能力。

验收标准：

- 连续发送多个 `G1` 时，电机使能不会在相邻 block 之间反复切换。
- 单个短线运动完成后，电机仍能按空闲策略关闭。
- Feed Hold、Reset、RMT 错误发生时，电机能够安全关闭。
- 普通 XY 直线运动的最终位置和方向不改变。
- 灰度雕刻的“嘟嘟”声明显减少，且没有新增丢步现象。

结束改动标准：

- 已完成上述实机验证并确认连续运动期间电机保持使能。
- 已确认异常路径仍能关闭电机。
- 阶段验证通过后提交本阶段改动，再进入阶段二。

### 阶段二：建立持续 Stepper segment ring

实现内容：

1. 将 `stepper_prep` 从“当前 block 生成后立即等待执行”改为持续填充固定容量 segment ring。
2. segment ring 必须能够同时容纳当前 block 和后续 Planner block 的已固化 segment。
3. 已进入 segment ring 的数据不得再受 Planner 后续 look-ahead 重算影响。
4. Planner block 只有在其所有 segment 已固化并完成消费后，才允许从 block ring 中释放。
5. 设计 segment ring 的空闲、半满、低水位和满状态。

验收标准：

- 连续 `G1 X0.05` 输入时，Stepper segment ring 能预先保存多个 block 的 segment。
- Stepper 消费 segment 时，Planner 不需要等待当前 block 的 RMT 完成。
- segment ring 不发生越界、覆盖、重复消费或永久空转。
- Planner 队列暂时为空时，已进入 segment ring 的运动仍能完整执行。
- Reset、Hold 和 RMT 错误能够清空或安全冻结 segment ring。

结束改动标准：

- 已通过短线、连续 XY 和长时间连续运动验证。
- 已能观测 segment ring 水位，并证明消费速度不低于生成需求。
- 已确认最终机器位置与累计执行步数一致。
- 阶段验证通过后提交本阶段改动，再进入阶段三。

### 阶段三：RMT 连续供给与非阻塞消费

实现内容：

1. RMT 不再以“一个 block 一次等待”为主要执行单位。
2. 将已固化的 segment 持续提交到 RMT transaction queue。
3. 使用 RMT 完成回调、通知或独立消费任务补充后续 transaction。
4. 发送端和消费端使用固定 payload 生命周期，禁止 RMT 仍在读取时覆盖 payload。
5. 对 RMT transaction queue 设置低水位保护，避免硬件断供。
6. 保留无法由 RMT 表达时的安全回退路径，但回退路径不得影响正常灰度流。

验收标准：

- 连续灰度运动中，RMT transaction queue 不周期性归零。
- 逻辑分析仪观察到 STEP 脉冲间隔连续，不存在 block 边界产生的软件空洞。
- 长时间运行不触发 Task Watchdog，不发生 RMT 队列下溢或 payload 覆盖。
- X/Y 插补、正反方向切换和单轴运动仍正确。
- Reset、Feed Hold、RMT 错误能够停止后续脉冲并关闭激光。

结束改动标准：

- 已完成实际 RMT 波形或等效统计验证。
- 灰度雕刻卡顿不再由 RMT transaction 供给不足造成。
- 阶段验证通过后提交本阶段改动，再进入阶段四。

### 阶段四：激光功率与 segment 执行同步

实现内容：

1. 将每个 segment 的激光功率和 `execution_id` 固化绑定。
2. 在 segment 即将执行时更新 PWM，避免提前使用下一像素的 `S` 值。
3. 明确 M3、M4、M5、Feed Hold、Reset 和异常路径的激光状态。
4. 对 M4 根据实际 segment 速度进行功率补偿；M3 保持固定功率语义。
5. 避免在 RMT ISR 中直接执行不可保证实时性的 LEDC 操作，必要时使用安全的事件或硬件同步方案。

验收标准：

- 每个灰度像素的 `S` 值与对应 segment 一一对应。
- 不出现提前点亮下一像素、延迟关闭上一像素或边界残光。
- M3、M4、M5、Hold、Reset 和 RMT 错误的激光行为符合预期。
- 激光 PWM 更新不会重新引入运动卡顿。

结束改动标准：

- 已使用 LightBurn 灰度文件完成实机验证。
- 已分别确认运动连续性、PWM 时序和雕刻能量分布。
- 阶段验证通过后提交本阶段改动，再进入阶段五。

### 阶段五：位置一致性与长期任务验收

实现内容：

1. 继续保留 `$pos/status` 按需查询，不恢复逐 block 高频日志。
2. 区分“Planner 预期位置超前”和“执行失败导致的位置失效”。
3. 在 segment、Planner block 和实际步数之间建立可追踪的累计关系。
4. 记录低频、可控的下溢、RMT 错误、执行失败和位置失效原因。
5. 使用完整灰度文件进行长时间压力测试。

验收标准：

- 正常完成任务后，机器位置、规划位置和累计脉冲一致。
- 正常运行过程中不会无故进入 `invalidated`。
- 真实 RMT 错误、Reset 或中断发生时，位置状态能够明确标记为不可信。
- 长时间灰度任务不发生周期性卡顿、看门狗触发、队列下溢或异常重启。
- 任务结束后激光关闭，电机按空闲策略处理。

结束改动标准：

- 已完成目标 LightBurn 灰度文件的完整任务验证。
- 已保留必要的故障证据，但不产生逐 block 高频日志。
- 本计划所有阶段均完成并分别提交后，才判定连续步进执行模型对齐完成。

## 明确不纳入本计划的内容

- G-code 新语句的业务解析和增量式兼容工作。
- SD 卡文件系统、目录显示和 gzip 格式设计。
- Wi-Fi、WebUI、蓝牙等输入通道。
- 机械结构、皮带、导轨和驱动器电流等物理问题。

如果软件波形已经连续但实机仍抖动，应另行检查机械共振、驱动器细分、电流和加速度参数，不能继续归因于 SD 或 Planner。

## 当前结论

当前阶段三的 block 合并不足以解决灰度雕刻卡顿。下一步应从阶段一开始，先修正电机使能生命周期，再建立真正跨 block 的 segment ring 和 RMT 非阻塞消费模型。未经阶段一至三完成，不应把卡顿问题归因于激光 PWM 或 gzip 解压。
