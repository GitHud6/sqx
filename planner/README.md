# planner

## 目标分析

本模块把 `motion` 已验证的单条运动计划转为有界的异步 block 队列。它的目的不是实现完整前瞻算法，而是先分离“命令已被接受”和“物理运动已完成”两个时间点，防止 `protocol_task` 在每条 G-code 的 STEP 循环中阻塞。

职责边界：

```text
motion: 参数校验、毫米到步数、构造 MotionBlock
planner: 有界队列、执行顺序、Feed Hold/Reset 状态、stepper 调用
stepper: GPIO DIR/ENABLE/STEP 时序
```

每个 `MotionBlock` 同时保存 `MotionRequest` 与 `MotionExecutionPlan`，因此该 block 的 `S` 功率和主轴模式会与对应位移保持绑定。当前不驱动真实 PWM。

## 实现步骤与验收标准

### 步骤一：建立有界 block 队列

实现：创建容量为 32 的 FreeRTOS 队列；`motion_submit()` 只尝试一次非阻塞提交。

验收标准：可入队返回 `Accepted`；队列满返回 `Busy`；参数不合法或未启动返回 `Rejected`；`protocol_task` 不等待物理运动完成。

### 步骤二：由 planner 任务独占执行器

实现：`planner_task` 依序取出 block，独占调用 `stepper`，执行完成后关闭所有参与轴。

验收标准：连续 block 的执行顺序与入队顺序一致；单个 block 异常不让电机保持使能；`motion` 不再包含 GPIO 或 STEP 调用。

### 步骤三：预留实时控制入口

实现：提供 `planner_request_feed_hold()`、`planner_resume()`、`planner_reset()`；Hold 在脉冲间隔处暂停，Reset 递增 block 代次、清空未执行队列，并让当前 block 在下一检查点停止。

验收标准：Hold 后不继续产生 STEP；Resume 后可继续当前 block；Reset 后当前 block 停止、待执行队列被清空且已使能轴被关闭。

## 当前限制

- 没有前瞻、加速度曲线或 junction 速度。
- 运行中的硬件错误尚未反向通知 `command` 的坐标状态；正式系统需要 Alarm 与位置可信度策略。
- Feed Hold/Reset 尚未接入 UART 实时字符。
- `esp_rom_delay_us()` 仍会占用 planner 任务；它不再阻塞协议接收，但不是最终定时方案。
