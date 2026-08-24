# Laser PWM 集成计划

## 目标分析

当前已打通的执行链路为：

```text
UART0
  -> protocol
  -> command
  -> motion
  -> planner block
  -> RMT STEP/DIR
```

`command` 已将 LightBurn 的模态 `M3`、`M4`、`M5` 和每行 `S` 值写入
`MotionRequest`。`planner` 又会为每个成功入队的 block 分配唯一的
`execution_id`，并在 block 生命周期中发出执行事件。现在缺少的末端是将该
事件转换为 RayCube 激光电源所需的硬件 PWM 输出。

本模块的第一目标是建立静态灰度功率闭环：一条已开始执行的 `G1 ... S...`
只能使用它所属 block 的 `S` 值更新 PWM；停止、取消和故障必须使激光输出回到
安全关闭状态。它不改变 G-code 解析、队列顺序、RMT STEP 时序或坐标状态。

目标边界：

```text
Planner ExecutionEvent
  -> laser event adapter
  -> S 0..1000 映射为 PWM duty
  -> laser hardware driver (LEDC + laser PWM GPIO)
```

本计划暂时不实现 M4 的动态功率补偿。当前执行器没有加速度曲线和实时速度反馈，
不能正确计算实际速度相对于编程速度的比例；第一版中 `M3` 和 `M4` 都按当前 block
的静态 `S` 值输出。该限制必须在对外行为和后续测试中明确保留。

## 架构约束

- 新模块目录为 `laser/`，只拥有激光 PWM 初始化、占空比映射和安全关闭。
- `command` 继续只产生 `MotionRequest`，不得包含 LEDC、GPIO 或 PWM 代码。
- `motion` 继续只做请求校验和执行计划构造，不得驱动激光。
- `planner` 只发布 `ExecutionEvent`；不得知道 LEDC channel、频率或 GPIO。
- `stepper` 只拥有 STEP/DIR/ENABLE 与 RMT，不得直接控制激光功率。
- 激光输出必须由 `planner_set_execution_callback()` 的事件驱动，不能读取一个会被
  下一条 G-code 覆盖的全局 `S` 值。
- 回调可能发生在 planner task 或实时控制调用方；laser 回调必须短小、不可阻塞，
  不得调用任何 Planner API，也不得等待 PWM 硬件完成传输。
- 系统启动、PWM 初始化失败、`M5`、Feed Hold 请求、Reset、block 取消、block 执行
  失败和队列排空都必须请求激光关闭。硬件默认关闭是优先于灰度正确性的安全条件。
- 仅在 `BlockStarted` 且 `spindle_mode != Off`、`spindle_power > 0` 时允许输出非零
  duty。Rapid 运动的激光策略必须在实现前由机器行为确认；默认安全策略是 Rapid
  block 不出光。
- PWM 定时外设、channel、频率、分辨率与激光 PWM GPIO 必须从 RayCube 原工程和
  实际原理图核对后写入 `machine` 配置；不得根据猜测硬编码。

## 实现步骤与验收标准

### 阶段一：确认 RayCube 激光 PWM 硬件映射和 ESP-IDF 外设能力

阶段状态：代码已完成，待构建和实机空闲关闭状态验收。

实现内容：

1. 从 `D:\ray2-main` 的 RayCube 机器定义和激光/主轴实现中确认 PWM GPIO、有效电平、
   PWM 频率、分辨率、最大 `S` 值与使能/互锁条件。
2. 阅读本机 ESP-IDF 5.5.1 LEDC 驱动头文件，确认 timer、channel、duty 更新和停止 API。
3. 在 `machine` 中建立激光硬件配置常量；新建 `laser` 接口但初始化后强制输出关闭。

验收标准：

- 构建通过，LEDC 初始化不影响 UART0、RMT STEP 或现有 GPIO。
- 上电、软件重启、初始化失败和 deinit 后，激光 PWM 引脚处于确认的关闭电平。
- 本阶段不注册 Planner 回调，不执行任何 G-code 出光。
- 逻辑分析仪确认空闲时 duty 为零或硬件定义的关闭状态。

当前实现说明：已从 `D:\ray2-main` 的 RayCube 定义确认 `LASER_OUTPUT_PIN`
为 GPIO4、`LASER_ENABLE_PIN` 为 GPIO5、默认 PWM 频率为 2 kHz、满功率为 `S1000`。
上游默认配置的 PWM 与 Enable 均不反相，因此 GPIO4 的 duty 0 和 GPIO5 低电平是关闭状态。
`machine::LaserHardware` 保存该映射，`laser_init()` 使用 ESP-IDF 5.5.1 LEDC 低速
timer/channel 0、10 bit 分辨率初始化 PWM，先将 GPIO5 拉低，最后调用 `ledc_stop()`
保持输出低电平。`app_main()` 在 UART、Planner 和 Protocol 启动前执行该初始化。
本阶段未注册 Planner callback，未提供功率设置接口，正常 G-code 不会出光。

### 阶段二：实现独立 PWM duty 映射和安全关闭接口

实现内容：

1. 定义 `laser_init()`、`laser_set_power()`、`laser_disable()` 和状态查询接口。
2. 将合法的 `S 0..1000` 线性映射到 LEDC 的完整 duty 范围；在接口处再次拒绝 NaN、
   负值和超过最大值的功率，防止错误输入直接写寄存器。
3. `laser_disable()` 作为幂等操作，负责把 duty 设为关闭状态；所有错误路径调用它。
4. 不在本阶段接入 Planner callback。

验收标准：

- `S0`、`S500`、`S1000` 得到单调递增且可计算的 PWM duty。
- 连续重复设置相同 duty 不引起异常；非法输入不改变当前安全输出状态。
- 调用 `laser_disable()` 后测得 PWM 关闭，并且重复调用结果一致。
- 本阶段不改变 X/Y 步数、方向、运动耗时和 Planner 队列顺序。

### 阶段三：将 Planner 生命周期事件接入激光输出

实现内容：

1. 在启动顺序中完成激光初始化，并注册唯一的 Planner 执行回调。
2. 收到 `BlockStarted` 时，只在 Linear block、激光模式非 Off 且 `S > 0` 的条件下
   设置对应 duty；保存正在出光 block 的 `execution_id` 仅用于诊断和事件一致性检查。
3. 收到 `FeedHoldRequested`、`BlockCompleted`、`BlockCancelled`、`BlockFailed` 或
   `QueueDrained` 时调用 `laser_disable()`。
4. 收到 `Resumed` 时保持关闭，直到下一条 `BlockStarted` 再按新 block 的功率开启。
5. 明确 `M5` 的无运动语义：它必须通过独立的激光状态入口立即关闭，而不能等待
   下一条运动 block；该入口不得伪造运动 block。

验收标准：

- 连续 `G1 X0.05 S111`、`G1 X0.05 S103.9` 等灰度命令中，每次实际开始的 block
  使用其自身 `S` 值，不会读取到相邻命令的功率。
- 连续 block 的 `execution_id` 单调递增，日志或测试回调能核对每个开始和停止事件。
- `M5`、Hold、Reset、RMT 执行错误和队列耗尽均使 PWM 关闭。
- 激光回调不会阻塞 planner task；连续串口输入仍按既有语义返回 `ok` 或
  `error:motion-busy`。
- 仅接入激光时，关闭激光的同一测试轨迹与接入前具有相同步数和终点。

### 阶段四：灰度行实机测量与边界行为验证

实现内容：

1. 使用低风险功率、遮光/测试负载或逻辑分析仪，发送与 LightBurn 相同格式的
   `M4 S...` 加连续 `G1 X0.05 S... F6000` 行。
2. 同时采集 PWM、X STEP 和必要时 Y STEP，记录每个 `execution_id` 的 PWM 开关时刻。
3. 检查队列填充、Hold、Reset 和长 block RMT 分块期间的输出状态。
4. 将实机参数、观察结果和已知限制写回本文件。

验收标准：

- 每个 `0.05 mm` 像素 block 的 PWM duty 与该行 `S` 值一致；不会发生前后像素交换。
- PWM 只在对应运动 block 的执行窗口内为非零，停止类事件后无残留输出。
- 在 `6000 mm/min、508 DPI` 测试下，串口继续接收、Planner 不触发 watchdog。
- 能量或速度出现周期性条纹时，记录其与当前 64 symbol RMT 分块重提交边界的关系，
  不将该问题误判为 `S` 解析或 PWM 映射错误。

## 完成标准

以下条件全部满足时，本模块的静态 PWM 集成完成：

- RayCube 激光 PWM 硬件映射已经由原工程和实物确认。
- 每个执行 block 的 `S` 值通过唯一 `execution_id` 驱动对应 duty。
- `M5`、Hold、Reset、取消、执行失败、队列排空和初始化故障都有确定的关闭路径。
- 激光模块没有反向耦合进 command、motion、planner 或 stepper 的硬件实现。
- 实机灰度测试记录已写回本文档，且本阶段改动已提交。

## 暂不实现

- `M4` 动态功率补偿、加减速能量补偿和速度比例缩放。
- 激光焦点、温控、门控互锁和外部安全链路的完整产品实现。
- RMT 的 DMA/encoder refill 与无缝连续 symbol 流。
- 加速度、junction 速度、look-ahead 和图像像素合并。

其中 RMT 无缝流和加速度规划会直接影响 `6000 mm/min、508 DPI` 灰度质量，应在本模块
得到基本 PWM 波形验证后作为独立性能阶段处理。
