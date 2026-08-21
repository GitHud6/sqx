# Protocol Module

`protocol` 负责把 UART0 的无边界原始字节流还原为完整命令行，并把完整行交给命令分发入口。

它不解释 G-code 的 `G`、`X`、`Y`、`F` 或 `S` 含义，不计算运动，也不直接访问 UART 硬件寄存器。

## 当前基线

当前实现仅用于验证 UART 链路：

    uart0_rx_task
      -> uart0_rx_queue
      -> protocol_task
      -> protocol_process_byte()
      -> UART0 TX 原样回显

第一阶段代码已完成，待实机串口验收。后续实施将把“单字节回显”替换为“组装完整行并同步分发”。在当前阶段，`protocol_task` 直接调用命令分发函数，不额外创建 command-parser 任务。

## 目标链路

    UART0 RX
      -> uart0_rx_task
      -> uart0_rx_queue
      -> protocol_task
      -> line_assembler
      -> command_dispatch()
      -> execute_line()
      -> ok / error 响应

`uart0_rx_task` 批量读取原始字节块；`protocol_task` 在内存中逐字节扫描字节块。逐字节扫描不是逐字节读取 UART，也不会增加 UART 驱动调用次数。

## 第一步：替换 chunk 原样回显

### 要做什么

修改 `protocol_task`：从 `uart0_rx_queue` 取出 `Uart0RxChunk` 后，不再把整个 chunk 直接写回 UART0 TX，而是遍历：

    chunk.data[0] 到 chunk.data[chunk.length - 1]

此时可以临时把逐字节处理后的数据仍写回 TX，以确认 chunk 内容、长度和顺序没有变化。

### 验收标准

- 发送任意 ASCII 文本，TX 回显与 RX 完全一致。
- 发送长度大于 128 字节的文本，跨 chunk 的回显字节顺序正确。
- 连续发送多个短文本，未出现丢字节、重复字节或乱序。
- `protocol_task` 仍通过 `xQueueReceive(..., portMAX_DELAY)` 阻塞等待，空闲时不轮询占用 CPU。

### 结束改动的标准

当逐字节扫描和原始 chunk 回显在实机上等价于旧实现时，停止修改本步骤。不要在本步骤添加 G-code 语义、行缓存或 `execute_line()`。

### 当前状态

- 代码：已完成。
- 实机验收：待执行。
- 下一步：确认逐字节回显与旧 chunk 回显等价后，实现行帧化。

## 第二步：实现行帧化

### 当前状态

- 代码：已完成。
- 构建：已验证。
- 实机验收：待执行。

### 要做什么

在 `protocol` 内增加私有行组装器，例如 `line_assembler.hpp/.cpp`。它拥有固定长度行缓存和以下状态：

    char line_buffer[LINE_BUFFER_SIZE]
    size_t line_length
    bool ignore_next_lf
    bool discard_until_eol

`protocol_task` 将每个字节交给行组装器。组装器输出三类事件：

    None      当前行尚未结束
    Ready     已得到完整命令行
    Overflow  当前行超过固定缓存上限

行结束规则：

- `\r`：提交当前行，并忽略紧随其后的一个 `\n`。
- `\n`：若是被忽略的 CRLF 第二字节则跳过，否则提交当前行。
- 普通字节：追加到当前行缓存。
- 超长行：不再追加，持续丢弃到下一处 `\r` 或 `\n`。

一个 UART chunk 可以产生零条、一条或多条完整命令；一条命令也可以跨多个 chunk。行缓存不能因为一个 chunk 结束就清空。

### 验收标准

使用日志或临时的完整行回显验证：

- `G91\n` 产生一条 `G91`。
- `G91\r` 产生一条 `G91`。
- `G91\r\n` 只产生一条 `G91`，不会额外产生空行。
- `G1 X0.` 与下一 chunk 的 `05S100\n` 组合后产生一条 `G1 X0.05S100`。
- 同一个 chunk 中的 `G90\nG91\n` 产生两条且顺序正确。
- 空行不进入命令语义处理。
- 超过 `LINE_BUFFER_SIZE - 1` 的行被完整丢弃到行尾；下一条正常行仍能正确产生。

### 结束改动的标准

当 CR、LF、CRLF、跨 chunk、多行 chunk、空行和超长行都能重复通过时，停止修改行组装器。行组装器不解析 `G1`、`X`、`S`，也不直接调用 Planner。

## 第三步：实现同步命令分发

### 当前状态

- 代码：已完成。
- 构建：已验证。
- 实机验收：待执行。

### 要做什么

当行组装器返回 `Ready` 时，`protocol_task` 直接调用：

    command_dispatch(line, length)

第一版 `command_dispatch()` 只接收已经去除行结束符的原始文本 payload，不解析 G-code。当前以明确格式返回完整行：

    RX: G1 X0.05S100

这一阶段不建立 `command_queue`，也不建立独立 `command_parser_task`。目的是先验证“完整行只分发一次”的行为，保持任务模型最小。

### 验收标准

- 每个完整命令行只触发一次 `command_dispatch()`。
- `command_dispatch()` 接收的文本不包含 `\r`、`\n` 或结尾垃圾字节。
- 连续灰度形式输入，例如 `G1 X0.05S111\nG1 X0.05S104\n`，按原始顺序逐行分发。
- 实时字符暂时可以记录为待实现，但绝不能混入普通命令 payload。
- 分发返回后，下一条命令的行缓存从空状态重新开始。

### 结束改动的标准

当同一串口输入可以稳定得到一一对应、顺序不变的完整 payload 时，停止修改分发层。不要在本步骤提前迁入 `gc_execute_line()` 或运动规划依赖。

## 第四步：接入 execute_line() 适配层

### 当前状态

- 代码：已完成最小适配层。
- 构建：已验证。
- 实机验收：待执行。

### 要做什么

将 `command_dispatch()` 的临时回显替换为最小解析适配层：

    Error result = execute_line(line, CLIENT_SERIAL, guest_auth)
    report_status_message(result, CLIENT_SERIAL)

祖父项目的 `execute_line()` 会分流：

    $ 或 [ 开头
      -> system_execute_line()

    其他普通行
      -> gc_execute_line()

当前最小适配层保留祖父项目 `execute_line()` 的入口分流和状态锁语义，但它不是完整 `GCode.cpp` 的逐行复制。当前已实现：

- `G90`、`G91`：绝对/增量坐标模式。
- `G20`、`G21`：英寸/毫米单位；内部位置和进给统一保存为毫米。
- `G0`、`G1`：运动模态。
- `X`、`Y`、`Z`、`F`、`S`：位置、进给和功率数据。
- `$`、`[`：返回 `error:system-command-unsupported`。
- Alarm/Jog：返回 `error:system-gc-lock`。
- 合法行：返回 `ok`；非法行：返回一次 `error:<reason>`。

当前解析结果写入最小 `ParserState`，尚未进入 MotionControl 或 Planner。它是后续运动规划模块的输入基线。

完整迁移时仍需迁入或桩化祖父项目的依赖：

- `Error` 和基础错误报告。
- `sys.state`。
- `State::Alarm` 与 `State::Jog` 的 G-code 锁定判断。
- `system_execute_line()` 的最小桩或受限实现。
- `gc_execute_line()` 的最小 G-code 入口。
- `report_status_message()` 的 `ok/error` 输出。

首批 G-code 可只验证：

    G90, G91, G20, G21, G0, G1, F, S

WiFi、WebUI、SD、OTA、蓝牙、显示和完整系统命令不属于本步骤；它们可以由受控的未实现响应或桩函数处理。

### 验收标准

- `G90`、`G91` 的模态状态切换正确。
- `G1 X... F...` 能进入最小运动控制入口，或至少能得到可观测的解析结果。
- `G1 X0.05S100` 的 `X` 和 `S` 都被保留并传入解析层。
- 报警或 Jog 状态下普通 G-code 返回对应锁定错误。
- 每条完整命令只返回一次 `ok` 或一次 `error`。
- 非法命令返回错误后，下一条合法命令仍能正常解析。
- 在连续短行输入下，UART 原始队列没有持续增长或溢出。

### 结束改动的标准

当 UART 输入到 `execute_line()`、结果返回和 `ok/error` 输出形成完整闭环，并能稳定处理基础运动命令时，停止修改同步模型。

只有在实测出现以下证据时，才进入下一轮架构改动：

- `execute_line()` 或 Planner 等待造成 `uart0_rx_queue` 持续积压。
- UART 驱动 RX 缓冲发生溢出。
- 队列峰值长期接近容量。
- 需要独立调度多个命令来源。

届时再拆分：

    protocol_task
      -> command_queue
      -> command_parser_task
      -> execute_line()

## 全流程完成定义

当前同步模型的完成不等于完成完整 Grbl 移植。它只表示以下链路已经被实机验证：

    UART0 字节流
      -> 原始 chunk 队列
      -> CR/LF 行帧化
      -> 原始命令 payload
      -> 同步 command_dispatch()
      -> execute_line()
      -> ok / error

在此之前，不进入 Stepper GPIO、激光 PWM 或完整 Planner 实时执行。这样任何问题都还能明确归因到 UART、协议行帧化或命令语义中的某一层。
