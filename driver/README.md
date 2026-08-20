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
| GPTimer | 预留 | - | Stepper 定时基准 |
| RMT | 预留 | - | 需要时用于精确定时输出 |
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

### 当前限制

- 队列满时接收任务会阻塞；这适合当前回显原型，但后续需要统计队列等待时间和溢出情况。
- 尚未使用 UART 驱动事件队列；当前采用 `uart_read_bytes()` 的阻塞超时读取。
- 尚未区分日志输出和协议 TX 输出；实机测试时应避免日志和回显使用同一物理通道造成混杂。
- 暂未实现停止、重启或销毁接口；当前任务和驱动的生命周期与设备运行期一致。

## 后续新增外设的规则

新增一个片上外设时，在 `driver` 下增加对应的 `<peripheral>_driver.hpp/.cpp`，并在本 README 中登记：

1. 引脚、时钟、频率和缓冲等硬件配置。
2. 初始化、运行和停止的生命周期。
3. 任务、ISR、队列或临界区的归属。
4. 对上层暴露的最小接口。
5. 不属于该驱动的业务责任。
6. 实机验证方法和已知限制。

这样每个外设都能独立理解和验证，同时不会把硬件细节扩散到协议和运动规划模块。
