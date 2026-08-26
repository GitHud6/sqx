# LightBurn 实时控制低延迟改造计划

## 目标

解决 LightBurn 连续发送雕刻 G-code 时，点击暂停、恢复或停止后响应明显延迟的问题。

本计划以 `D:\ray2-old` 的 `Serial.cpp`、`Protocol.cpp`、`System.cpp` 和 `Report.cpp` 为行为参照，适配当前 ESP-IDF 分层架构。目标不是改变普通 G-code 的执行顺序，而是保证 `?`、`!`、`~` 和 `0x18` 不必等待普通 G-code 队列消费后才能生效。

## 已确认的问题

当前接收路径为：

```text
UART 硬件
  -> uart0_rx_task
  -> uart0_rx_queue（128 字节块）
  -> protocol_task（优先级 6）
  -> realtime 识别 / LineAssembler / command_dispatch
```

已实机确认：`?` 可以立即得到实时状态包，说明实时字符已从普通接收队列中成功分流。当前仍有以下四层问题：

1. 实时字符在 `protocol_task` 中才被识别。连续 G-code 会先占满 UART 驱动缓冲区、块队列和协议任务的处理时间，因此 `!` 可能排在普通数据之后。
2. `?` 当前需要经过协议任务才能输出状态，不能保证 LightBurn 轮询时的低延迟。
3. `0x18` 当前只调用 `realtime_command_receive()` 计数，没有完成 planner 清空、激光关闭、作业取消、普通命令清理和位置可信度处理，因此 Stop 还不是真正的停止。
4. 普通 G-code 在 planner block 队列满时会返回 `error:motion-busy`。LightBurn 以每行的 `ok` 作为流控确认；设备应当等待 planner 释放容量后再接收和确认该行，而不是用队列瞬时满拒绝合法作业。否则主机停止后续供料，雕刻会断流。
5. `protocol_task` 与 realtime task 都可能直接写 UART TX。即使 UART 驱动接受并发调用，状态包与 `ok/error` 也可能在字节级交错，导致 LightBurn 无法可靠匹配普通命令的确认响应。

## 设计原则

- 实时字符必须在 UART 接收侧或独立的高优先级 ingress 中尽早识别。
- 普通 G-code 必须保持原始字节顺序，不能因实时字符旁路而丢字节、重排或污染行内容。
- `?` 只读取快照并输出，不得阻塞 planner、segment、RMT 或 SD 任务。
- `!` 只请求进入 Hold，由 planner/执行链路完成减速和停稳。
- `~` 只请求恢复，不绕过 planner 的 Hold 状态机。
- `0x18` 是安全终止，不等同于普通命令，也不能仅返回 `ok`。
- 实时入口不得直接调用会阻塞的普通命令执行接口，不得在 UART 接收任务中执行文件读取、复杂日志或长时间持锁操作。
- 所有供料源最终必须服从同一个系统运行权限；本阶段先覆盖 UART/LightBurn，SD 任务必须能被停止流程取消。

## 目标架构

```text
UART RX
  -> realtime ingress（高优先级、短路径）
       -> ?    读取状态快照并快速发送
       -> !    planner_request_feed_hold()
       -> ~    planner_resume()
       -> 0x18 设置 reset/abort 请求
       -> 普通字节进入 normal_rx_queue

normal_rx_queue
  -> protocol_task
       -> LineAssembler
       -> 普通行容量门：planner 满时等待可用 block，不回复 error
       -> system-command dispatcher
       -> command_dispatch / execute_line
       -> planner

protocol_task / realtime task
  -> 单一 protocol TX 通道
       -> UART TX
```

实时 ingress 只负责识别、记录和投递不可阻塞的控制请求。需要跨模块清理的 Reset/Abort 由一个明确的控制处理点执行，避免 UART 任务直接操作多个模块造成锁顺序和生命周期问题。普通 G-code 的背压仅在普通命令路径等待，不得阻塞 UART 接收、实时 ingress 或 realtime task。

## 实施阶段

### 阶段一：建立实时 ingress 与普通字节分流

阶段状态：代码已完成。实时 ingress 已实机确认；普通 G-code 背压和单一 TX 发送任务待编译及 LightBurn 连续作业验收，阶段尚未验收。

#### 实现步骤

1. 阅读并固定 `driver/uart0_driver.*`、`protocol/protocol_task.*`、`protocol/realtime_command.*` 的所有权边界。
2. 在 UART 接收侧识别 `?`、`!`、`~`、`0x18`，实时字符不进入普通 G-code 队列。
3. 为普通字节保留顺序可靠的队列或块队列；队列满时必须有明确的溢出诊断，不能静默丢数据。
4. 保留 `protocol_task` 的 LineAssembler 和普通命令分发逻辑，确保 `$` 命令、注释、无换行 G-code 和实时字符混合时行为不变。
5. 为实时入口增加最小诊断：接收计数、处理计数、普通队列高水位和丢弃计数。诊断不能在每个字节上打印日志。
6. 将普通 G-code 的 planner 容量处理改为受控等待：遇到 `SubmitResult::Full` 或 `MotionBusy` 时，保留当前完整行，等待 `planner_wait_for_free_block()` 返回后重试该行；仅在真正非法或安全拒绝时回复 `error`。不得提前提交 ParserState，也不得重排后续行。
7. 将 protocol 普通 `ok/error` 与 realtime 状态包收敛到单一 UART TX 任务，保证每条响应完整连续，不能与另一条响应字节交错。普通响应使用 FIFO，保持原始确认顺序；实时状态仅保留一个最新待发快照，合并重复 `?`，防止高频轮询占满 TX。
8. 为普通行背压增加诊断：等待次数、等待中的行数（应至多一行）和最大等待时间。默认不输出逐行日志。

#### 验收标准

- 在连续 G-code 流中插入 `?`，状态报告不等待前面的整块 G-code 消费。
- 在连续 G-code 流中插入 `!`，实时任务能立即记录请求；实际运动停稳边界留给阶段二验收。
- `~` 能被实时入口接收；实时字符不产生 `error:expected-command-letter`。
- 连续 LightBurn 作业期间 planner 队列填满时，合法普通 G-code 不返回 `error:motion-busy`；相应 `ok` 可以延后，但必须在释放容量后按原始行顺序返回。
- 实时 `?` 包与普通 `ok/error` 不交错、不截断，LightBurn 不因确认流异常停止继续供料。
- 连续发送 `?` 时，未发送的旧状态包被最新快照替换；普通 `ok/error` 仍按提交顺序优先送出，不被状态轮询长期饿死。
- 普通命令内容和顺序与改造前一致；背压等待期间不丢行、不重排、不污染 ParserState。
- 队列满、接收异常和实时计数异常都有明确诊断，不能通过静默丢字节掩盖问题。

#### 阶段结束改动标准

- 实时字符和普通字节已经物理分流。
- `protocol_task` 不再承担实时字符的首个发现职责。
- 普通 G-code 的 planner 背压已转换为等待后确认，而不是队列满错误。
- UART TX 输出已串行化，状态报告不会破坏 `ok/error` 的协议边界。
- 实机验证通过后提交本阶段，再开始阶段二。

#### 本轮待验证重点

1. LightBurn 连续发送作业时，不再在约二十条普通 G-code 后停止供料。
2. 连续 `?` 轮询期间，普通 `ok` 仍持续到达；状态行可能被合并为较少的最新快照，这是预期行为。
3. 雕刻结束后 `$POS/STATUS` 的 `planner=(queued=0,...)`、`segment=(queued=0,...)` 与 LightBurn 的完成状态一致。
4. 无 `error:motion-busy`、UART 响应乱码、半行响应或 task watchdog。
5. 使用 `$PROTOCOL/TRACE` 确认真实作业中没有未对齐命令的 `error`；已确认 `M8` 为 LightBurn 文件中的标准冷却命令，RayCube 无冷却硬件时按 `ray2-old` 兼容接受且不改变激光输出。

### 阶段二：实现低延迟 Hold/Resume

阶段状态：代码实现完成，等待实机验证。

#### 实现步骤

1. 将 `!` 映射为 planner 的 Hold 请求，确认请求只设置状态/通知，不在接收任务中等待当前 block 完成。
2. 将 `~` 映射为 planner 的 Resume 请求，保持当前 segment、block 和 junction 状态的一致性。
3. 检查 planner、segment 和 stepper 对 Hold 的边界：已发出的脉冲必须完成安全收尾，未消费的 segment 不得继续无限生产。
4. 检查激光输出：进入 Hold 时关闭或按现有 Ray2 语义处理激光，恢复时不能产生意外出光。
5. 增加时间戳诊断：记录实时字符接收时间、请求生效时间和实际停稳时间；默认关闭逐事件日志。

#### 验收标准

- 雕刻连续运行时发送原始 `!`，设备快速进入 `<Hold|...>`，延迟不随待执行 G-code 文件长度增加。
- Hold 时电机不再继续消耗未授权的后续运动，激光状态符合 `ray2-old` 语义。
- 发送原始 `~` 后从当前安全位置继续，不能重复执行、跳过或重排 G-code。
- Hold/Resume 过程中无 planner 卡死、segment underflow、task watchdog 或位置状态错误。

#### 阶段结束改动标准

- 实机连续雕刻暂停和恢复通过。
- 诊断确认延迟来自实时路径而不是普通命令队列。
- 提交本阶段后再开始阶段三。

### 阶段三：实现真正的 Stop/Reset

#### 实现步骤

1. 将 `0x18` 转换为全局、原子且可重复处理的 Abort 请求，不在接收任务中直接执行长清理。
2. 在控制处理点按固定顺序执行：阻止新的普通命令进入 planner、停止供料、请求 stepper 安全停机、清理 planner/segment 队列、关闭激光、取消 SD job。
3. 清理协议层尚未执行的普通字节和半行，避免 Stop 后残留 G-code 被继续执行。
4. 按现有位置可信度模型将位置标记为 `invalidated/unknown`，不得把未完成的 planned position 冒充为 machine position。
5. 统一状态报告、`$POS/STATUS`、`?` 和 SD job 状态，确保 Stop 后不会继续报告 Running 或遗留激光开启状态。
6. 明确 Stop 后允许的命令：通常只允许复位、回零、解锁或状态查询；具体行为以 `ray2-old` 与当前项目安全策略为准。

#### 验收标准

- 连续雕刻中发送原始 `0x18`，运动和激光都停止，响应时间不随文件剩余长度增加。
- Stop 后不再执行已经排队但尚未消费的 G-code。
- `$POS/STATUS` 明确显示位置可信度已失效；重新 `$H` 或显式位置校准后才恢复为 known。
- SD 任务状态进入明确的取消/故障状态，文件句柄和作业任务均能退出。
- Stop、Hold、Resume 连续交错发送不会死锁、重启、触发 watchdog 或造成重复出光。

#### 阶段结束改动标准

- LightBurn 的暂停、恢复、停止均为真实业务行为，而不是只增加计数。
- 完成实机验证并提交本阶段后，才进入后续多供料源实时控制统一。

### 阶段四：多供料源与状态一致性收口

#### 实现步骤

1. 将 UART、SD、后续 socket/FFAT 的作业控制接入统一的系统运行状态和 Abort/Hold 权限。
2. 供料源只负责提交普通 G-code，不直接操作 planner 内部队列；实时控制由统一控制入口处理。
3. 为每个供料源定义 Active、Draining、Paused、Cancelled、Faulted 和 Completed 状态转换。
4. 明确普通命令、实时命令、SD 作业和状态报告之间的并发规则，避免多个供料源同时获得运行权限。

#### 验收标准

- 任意时刻只有一个供料源能提交当前雕刻任务。
- 实时 Hold/Resume/Stop 对 UART 和 SD 作业使用同一套控制语义。
- 状态报告不会出现供料源已停止但 planner 仍 Running，或 planner 已停止但激光仍开启的矛盾状态。

## 不在本计划内

- 不在 UART 接收热路径中加入 planner 前瞻、junction 或 segment 规划逻辑。
- 不通过提高队列容量掩盖实时路径延迟。
- 不把实时字符改造成需要换行的普通命令。
- 不在本阶段扩展尚未被 LightBurn 或 `ray2-old` 实际证明需要的 G-code。
- 不把 `0x18` 当作可见文本测试；必须使用串口工具发送原始控制字节。

## 建议验证命令

普通运动：

```gcode
$H
G90
G1 X20 Y10 F600
```

暂停/恢复：在连续雕刻或连续直线流运行期间，不带换行发送：

```text
!
```

恢复：

```text
~
```

停止：使用串口工具发送单个原始字节 `0x18`，不要发送字符串 `0x18`，也不要附加 `\r` 或 `\n`。

## 完成定义

1. 实时字符不会被普通 G-code 队列阻塞。
2. Hold/Resume/Stop 都能在实机上观察到明确、及时且安全的效果。
3. Stop 后 planner、segment、stepper、laser、SD job、协议缓冲区和位置可信度状态一致。
4. 每个阶段实机验收通过后提交对应改动；未通过前不进入下一阶段。
