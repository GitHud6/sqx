# BLE 驱动模块

## 模块定位

Driver 层为 BLE Bus 提供统一设备行为接口。总线负责发现、连接和 GATT 生命周期；Driver 描述连接后对不同设备/特征应执行的行为，并把 GATT 数据交给协议层。JSON 选择的是固件中已有的 Driver 与 callback set，不会从 MQTT 动态下载可执行代码。

## 组成

- [`ble_driver.h`](ble_driver.h)：Driver vtable、能力声明、设备回调及特征 callback set 类型。
- `ble_driver_registry.c/.h`：按配置中的 `driver` 名称查找并注册 Driver。
- `generic_gatt_driver.c/.h`：声明式 GATT 读写和 Notify 分发入口。
- `ble_device0_driver.c/.h`：设备专用驱动示例。

## 关键 API

| API | 用途 |
| --- | --- |
| `ble_driver_registry_register(driver)` | 启动时注册静态 Driver vtable。 |
| `generic_gatt_read(characteristic_id)` | 对唯一匹配的特征执行读操作。 |
| `generic_gatt_read_device(device_id, characteristic_id)` | 指定设备和特征执行读操作，适用于多设备。 |
| `generic_gatt_write(characteristic_id, data, length, mode)` | 按特征 ID 写入数据。 |
| `generic_gatt_write_device(device_id, characteristic_id, data, length, mode)` | 指定设备执行写入，避免多设备 ID 歧义。 |
| `ble_device0_write(mode, data, length)` | 示例私有 Driver 的便捷写入封装。 |

总线操作与可选外设 OTA API 见 [`../ble/ble_bus.h`](../ble/ble_bus.h)；精确签名以对应头文件为准。

## 事件路径与协作关系

```text
ble_bus NimBLE event -> Profile-bound Driver -> characteristic callback_set
                    -> protocol decoder -> gateway_event -> service/MQTT
```

连接、断开、读完成、写完成和 Notify 是不同回调类别。BLE host 回调中不应执行长时间阻塞或网络 I/O；需要跨任务处理时交给 `service/` 队列。

## 何时新增专用 Driver

优先使用 `generic_gatt`：UUID、读写/Notify 属性、简单解码和 Topic 都可由 JSON 描述。设备握手、认证、强顺序命令、状态相关 GATT 操作或专有外设 OTA 才适合专用 C Driver。

新增 Driver：实现 `ble_driver_t`；定义独立 callback set；在启动路径注册；JSON 填入匹配的 `driver`/`callback_set`；由回调调用 `protocol/` 解析并映射统一事件。不要把扫描条件或 MQTT Topic 生成塞进 Driver。

Driver vtable 中的 OTA 是通过 BLE 给外设升级；网关自身固件升级由 [`../ota/README.md`](../ota/README.md) 负责。
