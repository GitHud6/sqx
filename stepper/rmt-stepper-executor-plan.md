# RMT Stepper Executor 迁移计划

## 目标分析

当前已验证的运动链路是：

```text
UART0 -> protocol -> command -> motion -> planner block -> 软件 STEP/DIR
```

其中 `command`、`motion` 和 `planner` 已经证明了 G-code 状态、毫米转步数、进给换算、XY 步数比例和 block 队列顺序。当前的问题只在执行末端：`planner_task` 用 `esp_rom_delay_us()` 软件忙等生成 STEP 周期，长距离低速运动会持续占用 CPU 并触发任务看门狗。

本模块的目标是以 ESP-IDF 5.5.1 RMT TX 外设替换软件忙等，让硬件持续生成 STEP 脉冲；DIR 和 ENABLE 仍由 `stepper` 层通过 GPIO 控制。该迁移不改变上层 G-code、MotionRequest、Planner block 的语义，也不在本阶段加入加速度前瞻或真实激光 PWM。

目标边界：

```text
planner block
  -> RMT stepper executor
  -> 设置 DIR
  -> 满足 DIR setup 时间
  -> RMT 输出 STEP 脉冲序列
  -> 等待 / 续填 / 停止
  -> 禁用参与轴
```

RMT 层必须保持每个 block 的 X/Y 步数比例、方向、周期与 `S` 值关联。`S` 暂时只随 block 保存，不直接输出 PWM。

## 架构约束

- `command` 不得包含 RMT、GPIO 或电机时序代码。
- `motion` 只校验并构造 `MotionExecutionPlan`，不得调用 RMT。
- `planner` 继续拥有 block 队列、顺序、Busy 和 Hold/Reset 语义。
- `stepper` 成为唯一允许包含 `driver/rmt_tx.h`、GPIO DIR/ENABLE 与 RMT 句柄的模块。
- RMT 执行器不得在每个 STEP 间隔进行 CPU 忙等。
- 方向必须在当前 block 的首个 STEP 前稳定；ENABLE 必须在第一脉冲前有效，结束、错误、Hold/Reset 后必须恢复禁用。
- X/Y 不能因为分别启动两个 TX 通道而失去共同时间基准。使用同步管理器或经验证的等效启动方案。
- RMT 内存有限。长 block 不能假设整条路径的所有 symbol 能一次装入硬件；必须定义分块和 refill 策略。

## 实现步骤与验收标准

### 阶段一：确认 ESP-IDF 5.5.1 RMT 能力与硬件映射

阶段状态：代码已完成，ESP-IDF 5.5.1 全量构建已通过，待实机初始化验收。

实现内容：

1. 读取本机 ESP-IDF 5.5.1 的 `driver/rmt_tx.h`，确认 TX channel、copy encoder、DMA、同步管理器和停止 API 的真实名称与限制。
2. 为 RayCube X STEP GPIO15、Y STEP GPIO8 定义 RMT channel 和时钟分辨率配置；DIR/ENABLE 引脚保持现有 machine 配置。
3. 新建只初始化、不自动发脉冲的 RMT stepper 接口，启动时确保两个轴禁用。

验收标准：

- 工程可构建，RMT channel 能初始化和释放，不影响 UART0。
- 上电、重启和初始化失败时 X/Y DISABLE 都处于禁用电平。
- 本阶段不允许因初始化而移动电机。

当前实现记录：已依据本机 ESP-IDF 5.5.1 `rmt_tx.h` 确认并使用 `rmt_new_tx_channel()`、`rmt_enable()`、`rmt_new_sync_manager()`、`rmt_disable()` 和资源释放 API。新增独立 RMT 初始化接口，X/Y 使用机型定义的 STEP GPIO、1 MHz 分辨率、每通道 64 个 symbol、内部 TX 队列深度 1；当前未接入 `app_main` 或 Planner，因此不会改变已验证的软件 STEP 路径。已于 2026-08-21 完成全量构建，生成 `build/ray2_idf.bin`。

### 阶段二：RMT 单轴 block 执行与 G-code 验证

阶段状态：代码已完成，ESP-IDF 5.5.1 构建和短单轴实机验证已通过。

实现内容：

1. 用 RMT symbol 表达一个 STEP 高电平和低电平，保持现有至少 3 us 的高电平要求。
2. 将 Planner 的单轴 `MotionBlock` 转换为 RMT 脉冲序列，使用 block 的 `step_interval_us` 生成高低电平时间。
3. 设置方向和使能后再开始 TX，TX 完成后关闭使能；不再使用 `esp_rom_delay_us()` 维持步间周期。
4. 将本阶段直接接入 G-code 单轴命令，不创建独立的点动或老化测试入口。

验收标准：

- `G91` 后发送 `G1 X0.05 F600` 和 `G1 X0.1 F600`，分别输出 4 和 8 步。
- 逻辑分析仪或实机观察确认 STEP 高电平不少于 3 us。
- 正负方向正确，未参与轴无 STEP 脉冲。
- `G1 X... F...` 的步数和时间与现有进给计算一致。
- 连续低速长距离运动不触发 `task_wdt`。
- UART 在运动期间持续返回已入队命令的 `ok`，队列顺序正确。
- RMT 执行错误不会让电机保持使能。

当前实现说明：短单轴 block 使用 copy encoder 直接发送 `rmt_symbol_word_t` 数组；每个 symbol 为 3 us 高电平加剩余低电平。该阶段单次限制为 64 步，且周期不大于 32767 us（1 MHz RMT 单段计数上限）。超过范围的单轴 block 与所有 XY block 暂时继续使用软件执行路径，因此不会改变已验证的长距离运动行为。单轴 RMT 传输前会解除双轴同步组，避免只提交一个 TX 通道时等待另一通道；阶段三会按双通道提交语义重新建立同步组。

验收记录：已完成构建并刷机，`G91` 后的短单轴增量命令能够正常返回 `ok` 并运动；此前出现的 `error:motion-rejected` 是绝对坐标重复目标导致的零步请求，不是 RMT 传输错误。

### 阶段三：双轴同步与直线符号编码

阶段状态：代码已完成，ESP-IDF 5.5.1 构建和短线实机轨迹验证已通过。

实现内容：

1. 复用现有 Bresenham 规则，将每一个主步时刻转换为 X/Y 各自的 RMT 高低电平序列。
2. 为两个 RMT TX 通道建立共同启动机制，保证块首时刻对齐。
3. 对次轴未发脉冲的主步周期，在次轴 RMT 序列中编码等长低电平间隔。

验收标准：

- `G1 X0.5 Y0.5 F600` 的 X/Y 都为 40 步，路径接近 45 度。
- `G1 X0.5 Y0.25 F600` 的 X/Y 为 40:20 步，路径比例接近 2:1。
- 双轴首脉冲相对偏差满足 RMT 同步机制的预期，并通过逻辑分析仪或轨迹测量验证。
- 不因两个通道长度不同而截断任一轴的末尾步数。

当前实现说明：以较大步数为共同时间轴，为 X/Y 分别建立等长的 RMT symbol 数组。主轴每个时间片均为 STEP 高低脉冲；次轴在 Bresenham 判定需要步进的时间片输出同样脉冲，否则输出等长的低电平 symbol。提交两路 TX 前调用 `rmt_sync_reset()`，使硬件同步组在两路传输均就绪时共同启动。当前每轴最多 64 个主步时间片，故本阶段验收使用 0.5 mm 以内短线；长线分块留待阶段四。

验收记录：已完成构建和实机验证，短线双轴命令能正确运动。`G1 X10 Y10 F600` 与 `G1 X40 Y20 F600` 超出 64 主步限制后会回退到软件路径并触发既有 WDT，这确认阶段四必须接管长 block，不能将该现象视为 RMT 短线失败。

### 阶段四：长 block 分块、队列衔接和异常停止

实现内容：

1. 定义每次可编码的最大 symbol 数，长 block 按连续时间轴分块发送或通过 DMA/encoder refill。
2. 保证分块边界不多发、少发或改变 X/Y 步数比例。
3. 将 `planner_request_feed_hold()`、`planner_resume()`、`planner_reset()` 接到 RMT 执行器：Hold 在安全分块边界暂停，Reset 停止活动 TX、清空待执行 block 并禁用电机。

验收标准：

- 超过单次 RMT 内存容量的运动能完成，终点误差不超过一个步进单位。
- 连续 block 不丢失、不重排，队列满仍只返回 `Busy`。
- Hold 后不再输出新的 STEP；Resume 后继续明确的剩余 block 或分块。
- Reset 后 RMT 停止、待执行队列清空、X/Y 均禁用。

### 阶段五：为激光 PWM 同步建立执行回调

实现内容：

1. 在 RMT block 开始、结束、暂停和取消处定义执行状态回调。
2. 回调携带当前 block 的 `spindle_mode` 与 `spindle_power`，但本阶段不直接写 PWM。
3. 定义运动停止时 PWM 应关闭的默认安全语义。

验收标准：

- 每个执行回调对应唯一 MotionBlock，不会把相邻灰度行的 S 值错配。
- Hold、Reset、RMT 错误和队列耗尽时，激光同步层能获得明确状态。
- 接入 PWM 前不改变当前机械运动的步数、方向和队列顺序。

## 完成标准

只有以下条件全部通过，才可认为 RMT Stepper Executor 完成：

- 单轴和 XY 直线都由 RMT 生成 STEP，软件循环不再按步忙等。
- 连续低速和高频运动不再触发任务看门狗。
- 长 block 分块后步数比例、终点和方向仍正确。
- Queue 满、Hold、Resume、Reset 和执行错误都有确定的电机禁用路径。
- 每个执行 block 的 S 值能被后续 PWM 层唯一关联。
- 实机验证记录已经写回本文件，并在进入激光 PWM 阶段前提交。

## 暂不实现

- 加速度曲线、junction 速度和前瞻优化。
- G2/G3 弧线。
- 完整限位、回零和 Alarm 状态机。
- 实际激光 PWM、动态功率和 M4 能量补偿。

这些工作依赖稳定的 RMT 脉冲执行和可中断的 block 生命周期，应在本计划完成后拆分为独立模块。
