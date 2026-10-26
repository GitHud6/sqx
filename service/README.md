# 网关事件服务模块

## 模块定位

`gateway_service.c/.h` 是 BLE 事件生产者与 MQTT 网络消费者之间的任务边界。协议回调发生在 NimBLE 上下文时，只将固定大小 `gateway_event_t` 复制到 FreeRTOS 队列；独立服务任务再调用 MQTT 发布，避免在 BLE host 回调中编码 JSON 或执行网络 I/O。

## 关键 API

| API | 用途 |
| --- | --- |
| `gateway_service_init()` | 创建事件队列/发布任务，并将自身注册为 Protocol event sink。 |
| `gateway_event_set_sink(sink, context)` | 注册事件消费者，服务初始化内部使用。 |
| `gateway_event_publish(event)` | 协议层提交统一事件并进入服务队列。 |
| `mqtt_gateway_publish_event(event)` | 服务任务序列化并发布事件。 |

服务接口见 [`gateway_service.h`](gateway_service.h)，事件契约见 [`../protocol/gateway_event.h`](../protocol/gateway_event.h)。

## 事件路径

```text
NimBLE callback -> Driver -> Protocol -> gateway_event_publish
                -> bounded FreeRTOS queue -> gateway_service_task
                -> mqtt_gateway_publish_event
```

## 队列与错误语义

- 队列容量固定，事件按值复制；不会为每条遥测无界 malloc。
- 队列满时 sink 返回错误/超时；生产者可记录丢弃原因。这是有界背压，不是离线持久缓存。
- MQTT 未连接或发布失败时当前策略不保证重放。QoS 0 事件可能丢失，恢复后继续发布新事件。
- 调整队列容量应评估事件峰值、结构大小和排队延迟，不应简单改成无界缓冲。

本模块依赖 FreeRTOS queue/task、Protocol event 接口和 MQTT Publisher；不依赖具体 GATT UUID，也不解析厂商协议。
