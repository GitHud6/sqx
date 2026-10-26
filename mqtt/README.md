# MQTT 模块

`mqtt_gateway.c/.h` 封装 ESP-MQTT 连接、订阅、事件发布、配置接收和 OTA 命令接收。

| Topic | 方向 | QoS | 说明 |
| --- | --- | --- | --- |
| `netgate/devices/<device_id>/<topic>` | 发布 | 0 | BLE 统一事件；断线期间可能丢失 |
| `netgate/config/ble` | 订阅 | 1 | 完整设备 JSON 配置 |
| `netgate/config/ble/result` | 发布 | 1 | 配置事务结果 |
| `netgate/ota` | 订阅 | 1 | JSON `{ "url": "http://..." }` OTA 命令 |
| `netgate/ota/status` | 发布 | 1 | OTA 阶段、错误及字节进度 |

Topic 可通过 `main/Kconfig.projbuild` 的 NETGATE MQTT 选项修改。MQTT 当前使用 `mqtt://` 明文传输，无 TLS 和 Broker 身份验证；公开部署时使用者需按自己的 Broker 环境补上认证、ACL 和传输保护。
