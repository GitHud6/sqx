# ray2-idf

`ray2-idf` 是面向 ESP32-S3 激光雕刻机的 ESP-IDF 5.5.1 固件实现。项目将 G-code
解析、前瞻速度规划、步进脉冲执行、作业生命周期与网络门户拆分为独立模块；网络或
存储功能不会进入 Timer/RMT 的实时热路径。

当前工程以稳定的梯形速度曲线为基线，提供二维 G-code 运动、SD 卡雕刻、暂停/继续、
归零、网络文件管理和本地网页控制。S 曲线与闭环振动补偿不属于当前稳定功能。

## 功能范围

- 支持 G0/G1 直线和 G2/G3 圆弧；圆弧在 `command` 层离散为短直线后复用同一条运动链。
- 支持 32 个 `PlannerBlock` 前瞻窗口、梯形 profile、AMASS/DDA 量化和 64 个
  `StepSegment` 环形队列。
- 支持 SD 卡未压缩 G-code 文件的列举、上传、删除、目录操作与作业执行。
- 支持 `$H` 归零、`?` 状态查询、`!` 暂停、`~` 继续和 `0x18` 硬复位。
- 支持 UART、HTTP 和 WebSocket 命令入口；所有普通命令最终进入同一个 `protocol`
  和 `service` 门控。
- 支持 Wi-Fi 配置持久化、SoftAP 与 STA 连接、本地网页门户、HTTP 文件接口和实时状态
  推送。
- 接入 LIS2DH12 加速度计作为非实时异常加速度保护，明确不用于振动补偿或脉冲闭环。

## 总体架构

```text
UART ────────────┐
HTTP :8848 ──────┼─> network command adapter ─> protocol ─> command / motion
WebSocket :8849 ─┤                                      │              │
SD job ──────────┘                                      │              v
                                                        │          planner
实时字节 ? ! ~ 0x18 ────────────────────────────────────┤              │
                                                        v              v
                                                    service      profile / prep
                                             作业状态与安全编排       │
                                                        │              v
                                      laser / homing / SD       segment ring
                                                                       │
                                                                       v
                                                           Timer ISR / RMT / STEP
```

`protocol` 是所有输入源共享的协议边界。`service` 是任务级协调层：它拥有作业
生命周期、暂停/继续、硬复位、输入结束和安全中止的状态转换，但不解析 G-code、计算
速度或生成脉冲。冻结后的 segment 只由 Timer/RMT 消费，ISR 不调用文件、网络或复杂
状态机代码。

## 运动执行链

```text
G-code/parser
  -> MotionRequest
  -> planner
  -> profile
  -> stepper_prep
  -> StepSegment ring
  -> Timer Group ISR
  -> RMT STEP pulse
```

各层职责保持单一：

| 层 | 职责 |
| --- | --- |
| `command` | 解析 G-code、维护模态状态、离散圆弧 |
| `motion` | 校验坐标和工艺请求，构造运动几何并提交 planner |
| `planner` | 管理 block、junction 及 entry/exit/nominal 速度边界 |
| `stepper` | 生成梯形 profile，按时间片积分、量化 segment，并消费 segment 输出脉冲 |
| `driver` | 提供 Timer、RMT、UART、SD、限位和 I2C 等硬件适配 |

位置仅在 STEP 脉冲实际产生后更新。硬复位、已确认的安全故障或执行异常会清空未执行
运动并使位置失效；之后必须通过 `$H` 重建可信坐标。

## 作业与安全控制

`service` 将不同介质的“输入结束”和“安全中止”区分处理：SD 文件读完或拔卡只停止
继续供料，已冻结的 block/segment 会自然排空；硬限位、确认的异常加速度和 `0x18` 则
立即切断激光、停止执行、清理未执行作业并使位置失效。

| 命令 | 语义 |
| --- | --- |
| `?` | 返回实时状态，不进入普通行解析 |
| `!` | 在 segment 边界暂停，关闭激光并保留未执行上下文 |
| `~` | 从已保留的上下文继续执行 |
| `0x18` | 硬复位：终止作业、清空待执行数据并使位置失效 |
| `$H` | 独立归零运动；成功后建立可信坐标 |

作业活动期间，普通 G-code 和系统配置命令经过统一输入门控；外部来源不能绕过当前
作业直接改变执行队列。实时命令仍保持独立快速路径。

## 网络与网页

Wi-Fi 配置保存在 NVS 中。设备始终建立 SoftAP，STA 是否连接外部路由器由已保存的
连接策略决定。网络生命周期模块负责在 Wi-Fi 可用时创建传输服务，并在 STA 断开或
重新获得 IP 后有序停止和重建服务，避免在 Wi-Fi 事件回调中直接操作 socket。

| 服务 | 端口 | 用途 |
| --- | --- | --- |
| HTTP | `8848` | 网页门户、命令、文件与存储接口 |
| WebSocket | `8849` | 网页会话、命令输入与实时状态/响应推送 |

网页是设备自身的控制门户，不依赖外部 App。HTTP 与 WebSocket 都只是客户端适配层，
不会直接访问 planner、stepper、Timer/RMT 或激光控制。

## 模块说明

| 模块 | 说明 |
| --- | --- |
| `command` | G-code 解析、模态状态维护与圆弧离散 |
| `motion` | 运动请求校验、运动几何构造与 planner 提交 |
| `planner` | block 队列、前瞻计算和速度边界 |
| `stepper` | profile、segment 生成、DDA/AMASS 与 Timer/RMT 执行适配 |
| `driver` | UART、Timer、RMT、SD、限位和 I2C 等底层外设驱动 |
| `protocol` | 行组装、实时字节分流、统一输入门控、响应与状态报告 |
| `service` | 作业生命周期、安全状态机、暂停/继续、复位与输入源协调 |
| `sd` | SD 热插拔、文件读写、作业供料与进度统计 |
| `homing` | `$H` 系统运动与可信坐标建立 |
| `laser` | 激光输出及其与 planner 的工艺联动 |
| `accelerometer` | LIS2DH12 非实时异常加速度检测与安全中止请求 |
| `wifi` | NVS 配网数据、AP/STA 射频状态和连接事件 |
| `network` | 网络命令适配与 HTTP/WebSocket 生命周期管理 |
| `httpd` | 网页静态资源、命令和存储 HTTP 接口 |
| `websocket` | 独立 socket WebSocket 会话与异步状态推送 |
| `machine` | 机器行程、运动和工艺参数 |

各模块的详细约束见对应目录下的 `README.md`。涉及运动行为的修改必须保持
parser、planner、profile、prep、segment transport 和 Timer/RMT 的职责边界。

## 构建

在 ESP-IDF 5.5.1 环境中执行：

```powershell
idf.py build
```

首次烧录后，应先通过 `$H` 建立位置，再执行普通运动或启动 SD 作业。Wi-Fi 不可用时，
UART、SD 和运动执行链仍应独立工作。
