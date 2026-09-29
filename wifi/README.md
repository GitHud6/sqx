# Wi-Fi 模块

`wifi` 只管理 ESP-IDF 无线电、AP/STA 链路状态和持久化的无线设置。它不解析 G-code，
不直接启动 HTTP/WebSocket 服务，也不能访问 `planner`、`stepper`、Timer/RMT 或激光输出。

## 状态与所有权

| 所有者 | 职责 |
| --- | --- |
| `wifi_settings` | 初始化和读写 `laserctl_wifi` NVS 命名空间。 |
| `wifi_service` | 创建 ESP-NETIF、配置 ESP Wi-Fi、维护运行期 AP/STA 状态。 |
| `network_lifecycle` | 根据链路事件在普通任务上下文中串行停止或启动 HTTP 与 WebSocket 服务。 |

Wi-Fi 事件回调只更新轻量状态并发布 `StaDisconnected` 或 `StaGotIp` 通知。回调上下文
不得执行 socket 停止、socket 创建、文件系统访问、动态内存申请或运动控制。

## 持久化设置

首次使用时，`wifi_settings_init()` 在 NVS 中初始化以下键，并将目标无线模式设为 `STA`：

| 键 | 含义 |
| --- | --- |
| `sta_ssid` | STA 关联的网络名称；默认为空。 |
| `sta_password` | STA 密码；默认为空。 |
| `radio_mode` | 期望无线模式：`AP`、`STA` 或 `AP_STA`。 |

SSID 和密码以有界 ASCII 文本保存。它们用于本地设备配网，不是凭据保险库；公开仓库、
示例配置、串口日志和测试脚本中不得出现真实凭据。

## 启动行为

`wifi_start()` 读取已保存的设置，并只创建目标模式需要的 ESP-NETIF：

| 模式 | 无线接口 | 服务可用条件 |
| --- | --- | --- |
| `AP` | 仅 SoftAP | AP 启动后可提供本地网络服务。 |
| `STA` | 仅 STA | 获得 DHCP 地址后提供网络服务。 |
| `AP_STA` | SoftAP 与 STA | AP 可立即提供服务；STA 获取地址后继续使用该链路。 |

AP 标识由通用 `identity` 配置和 AP MAC 派生，不属于机器 profile。STA 仅在 SSID 非空时
发起关联；空凭据不会阻塞 Wi-Fi 服务启动。无线省电在启动后关闭，以减少本地控制和文件
传输期间的额外延迟。

链路断开时，`wifi` 清除 STA 已获取 IP 的运行期状态并通知 `network_lifecycle`。后者负责
销毁旧 HTTP/WebSocket 监听器、请求重新关联，并在新链路可用时重建传输服务。这样不会让
失效的 STA socket 跨 DHCP 地址变化继续存活。

## 修改步骤

1. 仅在 `wifi_settings` 中增加或修改持久化设置键；保持 NVS 初始化、读取和写入语义一致。
2. 仅在 `wifi_service` 中改动 ESP-IDF Wi-Fi 或 ESP-NETIF 配置。
3. 需要改变 HTTP/WebSocket 启停条件时，修改 `network_lifecycle`，不要在 Wi-Fi 事件回调
   中直接操作 socket。
4. 修改后清除测试设备的相关 NVS 键，分别验证首次启动和已有设置启动。

## 验收标准

- 首次启动时，空 STA 凭据不会阻塞 UART、SD 或核心运动服务。
- `AP`、`STA`、`AP_STA` 三种模式均只创建各自需要的接口。
- STA 获取地址、断开和重新获取地址期间，HTTP/WebSocket 服务由网络生命周期任务有序重建。
- Wi-Fi 回调不调用运动、SD 或 socket 生命周期代码。
- 应用新的无线设置后，页面断开是预期行为；重连后 HTTP 和唯一 WebSocket 门户会话可恢复。
