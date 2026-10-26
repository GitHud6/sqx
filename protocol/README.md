# 协议模块

Protocol 层把 BLE 特征的原始字节转换为统一 `gateway_event_t`，不直接调用 MQTT 或管理 BLE 连接。

## 当前实现

- `generic_gatt_protocol.c`：按配置支持 raw、uint8、uint16、int16、uint32、字节序、偏移、缩放和单位。
- `ble_device0/`：设备私有协议的扩展位置。
- `gateway_event.c/.h`：统一事件结构和事件 sink 接口。

复杂厂商帧应在独立子目录实现帧解析、长度检查、CRC、加密或状态机，再将结果映射为 gateway event。不要把设备私有数据规则塞入 BLE Bus 或 MQTT 模块。
