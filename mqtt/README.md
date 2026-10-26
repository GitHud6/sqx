# MQTT 网关模块

## 模块定位

`mqtt_gateway.c/.h` 封装 ESP-MQTT，是设备数据、远程 BLE 配置和 OTA 命令共用的 MQTT 应用边界。它负责 Topic、消息分片重组、JSON 序列化和发布；不管理 Wi-Fi、不解析 BLE Profile schema，也不下载固件。

## Topic 契约

| Topic | 方向 | QoS | 数据/职责 |
| --- | --- | --- | --- |
| `netgate/devices/<device_id>/<topic>` | 发布 | 0 | BLE 统一遥测/设备事件，断线期间不保证缓存。 |
| `netgate/config/ble` | 订阅 | 1 | 完整 BLE Profile JSON 候选配置。 |
| `netgate/config/ble/result` | 发布 | 1 | 配置事务结果、错误码与 revision。 |
| `netgate/ota` | 订阅 | 1 | `{ "url": "http://..." }` OTA 命令。 |
| `netgate/ota/status` | 发布 | 1 | OTA 阶段、错误和接收字节进度。 |

Topic 可通过 [`../main/Kconfig.projbuild`](../main/Kconfig.projbuild) 修改。事件最终 Topic 由设备 ID 与 Profile 中的 topic 片段组成。

## 关键 API

| API | 用途 |
| --- | --- |
| `mqtt_gateway_init()` | 创建客户端并注册 ESP-MQTT 事件处理器。 |
| `mqtt_gateway_start()` / `mqtt_gateway_stop()` | 启动或停止 Broker 客户端。 |
| `mqtt_gateway_is_connected()` | 查询连接状态。 |
| `mqtt_gateway_set_config_handler(handler, context)` | 注册 BLE 配置消息消费者。 |
| `mqtt_gateway_set_ota_handler(handler, context)` | 注册 OTA 命令消费者。 |
| `mqtt_gateway_publish_event(event)` | 发布 `gateway_event_t`。 |
| `mqtt_gateway_publish_config_status(...)` | 发布配置事务结果。 |
| `mqtt_gateway_publish_ota_status(...)` | 发布 OTA 状态和进度。 |

完整类型和签名见 [`mqtt_gateway.h`](mqtt_gateway.h)。

## 与其他模块的协作

```text
gateway_service task --publish_event--> MQTT broker
MQTT config message --handler---------> ble_config_manager task
MQTT OTA command --handler------------> ota_manager task
network_manager --start/stop----------> mqtt_gateway
```

ESP-MQTT 事件回调不会执行 Profile 热切换或 HTTP 下载。配置/OTA 内容交由对应 Manager 排队处理，结果再由 MQTT 发布。

## 语义与限制

- telemetry 当前为 QoS 0；离线事件可能丢失，不提供持久队列或断线补发。
- 配置和 OTA 命令使用 QoS 1；QoS 1 不代替应用事务结果确认，也不意味着固件端持久化消息。
- Broker URI 当前为 `mqtt://`，没有 TLS、Broker 身份验证或内置 ACL。使用者需按自己的 Broker 环境补充认证和传输保护。
- 发布失败由上游记录；实时遥测消费者应接受可能丢弃。

本地 Mosquitto、配置下发和 OTA 示例见根目录 [`../README.md`](../README.md)。
