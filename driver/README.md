# Driver Layer

`driver` 是本工程访问 ESP32-S3 片上外设的统一边界。

本层负责外设初始化、引脚配置、驱动缓冲、底层任务和最小读写接口；不负责 G-code、协议语义、Planner 或产品业务。

## 设计边界

上层模块通过 driver 暴露的接口使用硬件能力：

    protocol / command-parser / motion-planner
      -> driver
      -> ESP-IDF peripheral driver
      -> ESP32-S3 hardware

driver 不应该理解一条字节流是不是 G-code，也不应该决定电机要如何运动。它只保证外设以已定义的配置可靠工作，并把数据或硬件能力交给上层。

## 外设目录

| 外设 | 状态 | 当前实现 | 作用 |
| --- | --- | --- | --- |
| UART0 | 已实现 | `uart0_driver.hpp/.cpp` | 串口接收、接收队列、原始字节发送 |
| GPIO | 预留 | - | 限位、控制输入、方向和使能输出 |
| LEDC | 预留 | - | 激光 PWM |
| GPTimer | 已实现 | `step_timer_driver.hpp/.cpp` | Step event 定时基准；回调运动语义仍由 stepper 提供 |
| RMT | 迁移中 | `rmt_step_pulse_driver.hpp/.cpp` | 当前产生单个 STEP 脉冲；GPTimer 所有权仍待迁移 |
| SPI / SDMMC | 预留 | - | 后续存储或扩展外设 |

## UART0

UART0 是当前唯一启用的片上外设驱动。

### 配置

- 芯片目标：ESP32-S3
- TX：GPIO43
- RX：GPIO44
- 波特率：115200
- 格式：8N1，无硬件流控
- IDF 驱动 RX 缓冲：2048 字节
- IDF 驱动 TX 缓冲：2048 字节
- 上层接收队列：16 个 128 字节块

引脚配置与祖父项目 ESP32-S3 的 UART0 默认控制台映射保持一致，便于后续使用同一硬件线路进行对照。

### 运行模型

    app_main()
      -> uart0_start()
         -> UART0 参数和引脚初始化
         -> 安装 ESP-IDF UART 驱动
         -> 创建 uart0_rx_task
         -> 创建 uart0_rx_queue

    uart0_rx_task
      -> uart_read_bytes()
      -> Uart0RxChunk
      -> uart0_rx_queue

`uart0_rx_task` 是常驻接收任务，优先级为 8。它只负责把 UART RX 字节流按接收顺序放入队列；不组装行、不识别 G-code、不直接调用协议或 Planner。

### 对外接口

`uart0_driver.hpp` 当前提供：

- `uart0_start()`：初始化 UART0、接收队列和接收任务。
- `uart0_rx_queue()`：返回协议层消费的队列句柄。
- `uart0_write()`：向 UART0 TX 写入原始字节。

目前 `protocol_task` 是队列的唯一消费者，收到数据后原样写回 UART0 TX，用于验证整条链路。

### 当前协议交接模型

UART0 驱动不识别 G-code，也不在接收任务中查找 `\r` 或 `\n`。它只批量读取原始字节块并投递到接收队列：

    UART0 RX
      -> uart0_rx_task
      -> uart0_rx_queue
      -> protocol_task

`protocol_task` 消费队列中的字节块后，逐字节进行行帧化：

    原始字节流
      -> 查找 \r / \n
      -> 组装完整命令行
      -> 去除行结束符
      -> 原始命令文本 payload

这里的逐字节是内存中扫描一个批量 chunk，不是每个字节都调用一次 UART 驱动。一个 chunk 可以包含多条完整命令，也可以只包含某条命令的一部分；未完成的部分必须保留到下一批数据继续组装。

第一阶段不额外创建 command-parser 任务。`protocol_task` 得到完整行后，直接同步调用命令分发和解析路径：

    protocol_task
      -> line_assembler
      -> command_dispatch()
      -> execute_line()
      -> ok / error 响应

这样先用最少的任务跑通 UART、行帧化和 `execute_line()`。只有测量到 `execute_line()` 或 Planner 等待导致 UART 原始队列持续积压时，才拆分为独立的 `command_parser_task` 和 `command_queue`：

    protocol_task
      -> 完整命令行
      -> command_queue
      -> command_parser_task
      -> execute_line()

### 行帧化规则

- `\r`、`\n` 和 `\r\n` 都表示一条命令结束；`\r\n` 只能提交一次。
- 空行不进入命令语义解析。
- 实时字符不进入普通行缓存，后续由 protocol 层即时分流。
- 行缓存满后，丢弃当前行直到下一个行结束符，再报告 overflow；不能把截断命令交给 `execute_line()`。
- 行帧化后保留的是未解析的原始文本，例如 `G1 X0.05S100`；`G`、`X`、`S` 的语义属于 `execute_line()` 及后续 G-code 解析层。

### 当前限制

- 队列满时接收任务会阻塞；这适合当前回显原型，但后续需要统计队列等待时间和溢出情况。
- 尚未使用 UART 驱动事件队列；当前采用 `uart_read_bytes()` 的阻塞超时读取。
- 尚未区分日志输出和协议 TX 输出；实机测试时应避免日志和回显使用同一物理通道造成混杂。
- 暂未实现停止、重启或销毁接口；当前任务和驱动的生命周期与设备运行期一致。

## Stepper 时序外设：当前所有权登记

Stepper 的运动语义与 ESP32-S3 外设所有权正在拆分。当前运行路径和迁移期间的
冻结规则如下；本节描述的是现状，不代表最终文件归属。

| 资源 | 当前活动所有者 | 当前配置 | 迁移期间规则 |
| --- | --- | --- | --- |
| X STEP | `driver/rmt_step_pulse_driver.cpp` | GPIO15，RMT channel 0 | 该 driver 运行时独占此 GPIO；旧 RMT stream 不得初始化或重新绑定它 |
| Y STEP | `driver/rmt_step_pulse_driver.cpp` | GPIO8，RMT channel 1 | 该 driver 运行时独占此 GPIO；旧 RMT stream 不得初始化或重新绑定它 |
| RMT | `driver/rmt_step_pulse_driver.cpp` | RMT LL 寄存器，单脉冲触发 | `stepper` 只能通过 `rmt_step_pulse_driver_trigger()` 请求发脉冲 |
| GPTimer | `driver/step_timer_driver.cpp` | 1 MHz，优先级 3 | 该 driver 独占 timer lifecycle；stepper 只能注册固定 ISR 回调并请求启停 |
| 旧 RMT stream | `stepper/rmt_stepper_driver.cpp` | ESP-IDF RMT high-level TX、transaction queue、stream task | 仅保留为迁移参考；当前不得在启动时初始化、创建任务或绑定 X/Y STEP GPIO |

当前启动路径调用 `timer_stepper_executor_init()`，它再初始化
`rmt_step_pulse_driver`。因此在迁移完成前，禁止同时启用 `rmt_stepper_init()`、
`rmt_stepper_stream_start()` 或任何会使旧 RMT stream 获取 X/Y STEP GPIO 的路径。这样保证
同一 STEP 引脚始终只有一个运行时输出所有者。

### 稳定诊断边界

上层诊断应只依赖与具体脉冲后端无关的状态：

- Planner：排队 block 数、已完成 block 数、规划位置。
- Stepper：执行中/空闲/故障状态、实际累计步数、实际机床坐标、位置是否可信。
- Driver：初始化状态和硬件故障结果，仅供 stepper 的停止或失效处理使用。

`rmt_transmit()` 次数、RMT transaction pending/inflight 数、RMT stream task 状态都属于
旧实现细节，不应继续作为 protocol 的长期状态接口。

## 后续新增外设的规则

新增一个片上外设时，在 `driver` 下增加对应的 `<peripheral>_driver.hpp/.cpp`，并在本 README 中登记：

1. 引脚、时钟、频率和缓冲等硬件配置。
2. 初始化、运行和停止的生命周期。
3. 任务、ISR、队列或临界区的归属。
4. 对上层暴露的最小接口。
5. 不属于该驱动的业务责任。
6. 实机验证方法和已知限制。

这样每个外设都能独立理解和验证，同时不会把硬件细节扩散到协议和运动规划模块。
