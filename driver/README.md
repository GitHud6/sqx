# BLE 驱动模块

驱动模块定义设备行为接口与驱动注册表，不负责扫描、连接调度或 MQTT 发布。

## 组成

- `ble_driver.h`：设备 Driver 接口及特征级回调集合。
- `ble_driver_registry.c/.h`：按 JSON 中的 `driver` 名称查找已注册 Driver。
- `generic_gatt_driver.c/.h`：通用读、写、Write Without Response 和 Notify 分发。
- `ble_device0_driver.c/.h`：设备专用 Driver 的示例。

## 扩展规则

普通 GATT 外设优先使用 `generic_gatt`，把 UUID、属性、callback_set 和解码元数据放进 JSON。只有需要厂商认证、复杂读写时序或 OTA 等特定行为时才增加专用 C Driver，并在启动时注册。

Driver 可选提供外设 OTA vtable；当前示例 Driver 未实现该能力。网关自身固件 OTA 由 `ota/` 模块负责。
