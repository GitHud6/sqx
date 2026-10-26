# Wi-Fi 管理模块

## 模块定位

`wifi_manager.c/.h` 管理 ESP-IDF Station 生命周期、凭据读取/保存、连接状态和断线重连。它不负责 MQTT Topic、BLE 外设 Profile 或 OTA 下载；网络联动由 `network/` 完成，本地凭据录入由 `provisioning/` 发起。

## 关键 API

| API | 用途 |
| --- | --- |
| `wifi_manager_init(handler, context)` | 初始化 netif/event loop/STA，并注册连接状态回调及上下文。 |
| `wifi_manager_is_connected()` | 查询 STA 是否处于已连接/已获得 IP 状态。 |
| `wifi_manager_set_provisioning_handler(handler, context)` | 注册配网服务的状态通知回调。 |
| `wifi_manager_apply_credentials(ssid, password)` | 尝试候选 WPA 凭据；连接成功后持久化，失败保留旧配置。 |

接口声明见 [`wifi_manager.h`](wifi_manager.h)。`network_manager` 注册连接 handler，配网服务也注册状态通知。

## 凭据与重连

启动时优先读取 NVS `wifi_cfg` namespace；没有有效配置时使用 Kconfig 默认值（`changlang` 仅供本地开发测试）。断连后由延迟重连 timer 调度 STA 重试，避免事件回调内立即高频重连。

候选配网凭据仅在成功连网后保存。不得将密码写入日志、错误消息或状态通知。NVS 擦除可能清除保存的凭据。

## 与其他模块的联动

```text
Kconfig/NVS ------------> wifi_manager
BLE provisioning -------> apply_credentials
wifi connected/IP ------> network_manager -> mqtt_gateway_start
wifi disconnected ------> network_manager -> mqtt_gateway_stop
```

BLE 与 Wi-Fi 共用 2.4 GHz radio/coexist 资源；拥塞、距离和干扰会影响时延。Wi-Fi 断开不等价于 BLE 设备断开。

## 故障定位

- `reason` 与 RSSI 需结合 AP、安全模式和现场信号判断，单一 reason 通常不足以断言密码错误。
- 反复 `auth -> init` 可检查凭据、路由器 WPA 模式及 AP 接入限制。
- Wi-Fi 已连但 MQTT 未连时转查 `mqtt/` Broker URI、端口和路由，不在 Wi-Fi 模块加入 MQTT 重试。
