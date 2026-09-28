# LaserCtl ESP32-S3 激光雕刻固件

`ray2-idf` 是面向 ESP32-S3 双轴激光雕刻机的 ESP-IDF 5.5.1 固件项目。它实现从
G-code 解析到物理 STEP 脉冲的完整执行链，并提供 SD 卡作业、归零、激光安全控制和
本地 Wi-Fi 网页门户。

项目以执行所有权划分边界。UART、HTTP、WebSocket 和 SD 作业都只是输入传输层，
它们不能直接操作 planner、步进定时器、RMT 或激光输出。所有运动和系统请求必须经过
共享的 `protocol` 与 `service` 边界。

## 功能范围

当前稳定功能：

- 支持两轴 G0/G1 直线和 G2/G3 圆弧；圆弧在进入运动规划前会离散为短直线。
- 支持前瞻规划、梯形速度曲线、AMASS/DDA 量化，以及 Timer Group 加 RMT 的 STEP
  脉冲输出。
- 支持归零、可信位置恢复和受软限位约束的运动请求。
- 支持 SD 卡文件列举、上传、下载、删除、目录创建和未压缩 G-code 作业执行。
- 支持 SD 作业进度、暂停、继续和硬复位，且 UART 与本地网页具有相同控制语义。
- 支持 NVS 持久化 AP、STA、AP+STA 配置，本地 HTTP 门户和自研 socket WebSocket
  控制会话。
- 支持 LIS2DH12 异常加速度检测，作为非实时安全输入。

明确不包含：

- S 曲线、闭环位置控制或振动补偿。
- OTA、云服务、MQTT Broker 或专用手机 App。
- gzip 压缩 SD 作业执行。`.gz` 文件可以保存到 SD 卡，但启动作业时会被拒绝。
- 高吞吐文件服务器能力。本地 Wi-Fi 门户面向机器控制和偶发文件管理。

## 总体架构

### 启动顺序

`main/app_main.cpp` 拥有启动顺序。硬件和运动服务在可选网络传输层之前初始化，因此
SD 卡或 Wi-Fi 不可用时，UART 和核心运动链仍可启动。

```text
NVS
 -> laser / SD / UART / Timer-RMT / limits / accelerometer
 -> homing / planner / laser-planner adapter / service
 -> Wi-Fi / network lifecycle（NVS 可用时）
 -> protocol / network command adapter
```

### 数据与控制路径

```text
UART ---------------------+
HTTP :8848 ---------------+--> network command adapter --> protocol
WebSocket :8849 ----------+                                |
SD job -------------------+                                +--> command --> motion --> planner
                                                                   |                    |
实时字节 ? ! ~ 0x18 ------------------------------------------------+                    v
                                                                service          profile / prep
                                                                   |                    |
                                                        laser / homing / SD   segment ring
                                                                                     |
                                                                                     v
                                                                      Timer Group ISR / RMT / STEP
```

普通命令先组装为完整行，再经过 `protocol -> command -> motion -> planner`。实时字节
在行组装前分流，保留系统控制语义，并经由 `service` 所拥有的控制路径处理，不作为普通
G-code 对待。

### 实时执行边界

```text
G-code parser
 -> MotionRequest
 -> planner block 与 junction 限制
 -> 梯形执行计划
 -> 固定时间片 stepper preparation
 -> 不可变 StepSegment ring
 -> Timer Group ISR
 -> RMT STEP pulse
```

Timer Group ISR 分配在 CPU0。所有阻塞工作、解析、规划、网络和文件系统访问固定在
CPU1。ISR 只消费已准备好的 segment，不执行 socket I/O、SD I/O、动态内存申请或复杂的
状态转换。

## 执行与安全模型

`service` 拥有任务级生命周期转换，并明确区分正常输入结束和破坏性中断：

| 事件 | 结果 |
| --- | --- |
| SD 文件到达 EOF 或被移除 | 停止后续供料，已准备的运动安全排空。 |
| `!` | 在 segment 边界请求暂停，并关闭激光输出。 |
| `~` | 在 service 状态允许时恢复已保留的执行上下文。 |
| `0x18` | 停止输出、清空未执行工作、终止作业并使位置失效。 |
| 硬限位或确认的加速度故障 | 走安全中止路径，并使位置失效。 |
| `$H` | 执行系统归零运动，重新建立可信机器位置。 |

SD 作业活动期间，无论命令来自 UART、HTTP 还是 WebSocket，普通 G-code 和系统配置
命令都会被统一门控。实时控制仍保持独立路径。

## 本地网络

Wi-Fi 设置保存在 `laserctl_wifi` NVS 命名空间。首次初始化默认 STA 模式；上电或执行
`$WIFI/BEGIN` 时，应用已保存的 `AP`、`STA` 或 `AP_STA` 模式。网络生命周期在 Wi-Fi
事件回调之外启动和停止 HTTP、WebSocket 服务。

| 服务 | 端口 | 作用 |
| --- | ---: | --- |
| HTTP | 8848 | 本地网页门户、命令请求和存储 API |
| WebSocket | 8849 | 单个活跃门户会话的命令、响应和实时状态 |

WebSocket 服务是显式的非阻塞 socket/select 实现，不依赖 ESP-IDF HTTPD WebSocket
升级支持。新接入的连接会替换先前活跃门户会话；有界发送队列确保慢速浏览器不会阻塞
protocol 或运动路径。

HTTP 上传先写入同级临时文件，完整正文写入并刷盘后才通过重命名提交。SD 文件访问与
活跃 SD 作业互斥。上传接收和 SD 写入使用有界 DMA 缓冲，以避免短暂 SD 写入抖动立刻
停止 socket 读取。

## 模块地图

| 模块 | 职责 | 主要边界 |
| --- | --- | --- |
| `command` | G-code 解析、模态状态和圆弧离散 | 产生运动请求，不直接驱动硬件。 |
| `motion` | 请求校验和运动几何构造 | 向 planner 提交有效运动。 |
| `planner` | block 队列、前瞻和 junction/速度限制 | 产生可执行 block 意图。 |
| `stepper` | 梯形计划、预处理、segment 和执行 | 以不可变 segment 驱动 Timer/RMT。 |
| `driver` | UART、Timer、RMT、SDIO、限位和 I2C 硬件适配 | 包含面向板级外设的访问。 |
| `protocol` | 行组装、实时字节分流、命令分派和报告 | 所有客户端共享的输入边界。 |
| `service` | 生命周期、安全、暂停/继续、复位和输入门控 | 拥有任务级状态转换。 |
| `sd` | 文件系统访问、上传事务和作业供料 | 拥有 SD 文件访问协调。 |
| `homing` | `$H` 系统运动和可信位置恢复 | 使用 service 与运动安全路径。 |
| `laser` | 激光输出许可和 planner 联动功率 | 在暂停、复位和故障时关闭输出。 |
| `accelerometer` | 非实时 LIS2DH12 异常运动检测 | 请求安全中止，不是反馈控制器。 |
| `machine` | 机器几何、引脚和工艺限制 | 读取构建期 JSON 生成的机器参数。 |
| `wifi` | NVS 设置、AP/STA 射频状态和 Wi-Fi 事件 | 仅提供网络传输可用性。 |
| `network` | HTTP/WebSocket 生命周期和网络命令适配 | 将网络请求桥接到 protocol。 |
| `httpd` | 门户资源、HTTP 命令和存储路由 | 不直接调用 motion 或 timer 代码。 |
| `websocket` | 单门户 socket 会话和有界通知 | 经网络适配层提交命令。 |

每个目录都包含模块 README，描述自身的状态所有权和验收条件。修改跨模块契约前，应先
阅读对应模块 README。

## 机器配置

板级 GPIO、轴几何和激光工艺限制定义在 `config/machine_profile.json`。构建前，
`cmake/generate_machine_profile.py` 会校验双轴结构、重复 GPIO、引脚范围、布尔极性和
有限的正运动参数，再生成仅供编译使用的 `machine_profile_generated.hpp`。运行期不解析
JSON，也不允许网页修改硬件拓扑。

因此 `machine` 模块仍向实时路径提供编译期常量；修改配置后必须重新构建、烧录并验证
方向、归零、限位与激光输出。JSON 是单一机器 profile 的可审查源文件，而不是远程配置
接口。

## 构建与烧录

使用 ESP-IDF 5.5.1 和 ESP32-S3 目标：

```powershell
idf.py build
idf.py -p COMx flash monitor
```

生成的 `sdkconfig` 是本地构建状态；仓库默认配置位于 `sdkconfig.defaults`。不要提交
机器本地 IDE 路径、串口日志、抓包文件或临时传输测试脚本。

## 上电验收

1. 构建并烧录目标 ESP32-S3。
2. 确认 UART 启动输出、激光安全空闲状态和 SD 卡挂载状态。
3. 发送 `$I` 与 `$RAM`，记录构建身份和内部/DMA 堆可用量。
4. 执行 `$H`，在普通运动或 SD 作业前确认位置可信。
5. 经 UART 验证短 G0/G1 作业，再测试 `!`、`~` 与 `0x18`。
6. 验证 SD 列表、上传、下载和一个未压缩 SD 作业。
7. 应用 Wi-Fi 设置并执行 `$WIFI/BEGIN`，在选定网络接口上验证 HTTP 门户和一个
   WebSocket 客户端。
8. 在禁用激光输出或采取适当安全措施后，验证硬限位和加速度故障行为。

## 项目边界

本仓库展示固件和相应的工程决策，不包含量产 App、云服务、专有作业文件或硬件制造
资料。后续加入 BLE、Telnet 等传输层时，应作为客户端适配层通过 `protocol` 提交请求，
而不是新增平行的运动控制路径。
