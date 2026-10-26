# BLE 协议处理模块

## 模块定位

Protocol 层负责把 Driver 收到的特征字节解释为应用数据，并映射到统一 `gateway_event_t`。它隔离厂商私有帧格式和 MQTT/连接实现：不直接操作 NimBLE，不管理连接，也不执行网络 I/O。

## 组成与关键 API

- `generic_gatt_protocol.c/.h`：按 JSON decoder 元数据支持 raw、整数、字节序、偏移、比例和单位等简单解码。
- `ble_device0/ble_device0_protocol.c/.h`：设备 0 协议示例，公开 `ble_device0_protocol_on_telemetry(device_id, characteristic_id, data, length)`。
- `gateway_event.c/.h`：统一事件结构和 sink 注册/发布入口。

| API | 用途 |
| --- | --- |
| `generic_gatt_protocol_publish(...)` | 用通用 decoder 解释设备特征数据并生成事件。 |
| `ble_device0_protocol_on_telemetry(...)` | 执行设备 0 私有遥测处理。 |
| `gateway_event_set_sink(sink, context)` | 注册事件消费者，通常由 `gateway_service` 注册。 |
| `gateway_event_publish(event)` | 把协议结果交给已注册 sink。 |

参数和事件字段见 [`generic_gatt_protocol.h`](generic_gatt_protocol.h)、[`gateway_event.h`](gateway_event.h) 与 [`ble_device0/ble_device0_protocol.h`](ble_device0/ble_device0_protocol.h)。

## 数据路径

```text
BLE Notify -> ble_bus -> Driver callback_set -> Protocol decoder/parser
          -> gateway_event_publish -> service queue -> MQTT serializer
```

一个 Notify 不一定对应一个业务事件。厂商协议可能需要分包重组、CRC、压缩、加密或状态机；协议实现负责检查长度/边界和帧完整性，并决定是否发布有效事件。

## 添加厂商协议

1. 在 `protocol/<device_name>/` 新增 `.c/.h`，不要把设备 UUID 分支写入 BLE Bus。
2. 明确输入来自哪个特征、允许长度、字段偏移、字节序与校验规则。
3. 实现有边界检查的解析入口，并映射为 `gateway_event_t`。
4. 由设备 Driver 的 callback set 调用协议入口。
5. 添加 JSON Profile 示例，说明 Driver、callback set、decoder 和 Topic 的关系。

通用字段解码优先配置化；只有厂商专属算法才增加私有 C 实现。Protocol 不直接依赖 MQTT API，保持可测试和可替换。
